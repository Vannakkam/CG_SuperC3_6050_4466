/*
  ESP32-C3 SuperMini | MPU6050 @ 0x68 on SDA=GPIO8, SCL=GPIO9
  MAX4466 OUT -> GPIO0 (ADC1), powered at 3.3 V.

  Arduino-ESP32 3.x + NimBLE-Arduino 2.x + Adafruit MPU6050 +
  Adafruit Unified Sensor + Adafruit BusIO.

  BLE binary protocol, little endian:
    IMU: 'I', version=1, count, flags=0, firstSeq(u32), firstEpochUs(u64),
         count * {ax,ay,az,gx,gy,gz} (6 x int16)
         accel scale 16384 LSB/g; gyro scale 131 LSB/(deg/s).
    MIC: 'M', version=1, count, flags=0, firstSeq(u32), firstEpochUs(u64),
         count * ADC(u16)  (12 bits, 0..4095)
    STATUS: 'S', code (1=start,2=finished,3=error), MTU(u16),
            droppedMic(u32), droppedIMU(u32).
    CLIENT COMMANDS: 'T' + epoch_ms(u64), 'B'=begin, 'E'=end,
                     'R'=status query.

  Within each packet, sample j has nominal time firstEpochUs+j*periodUs
  (125us for MIC, 5000us for IMU). The first sample is timestamped
  at acquisition. MIC uses continuous ADC/DMA at 24 kHz then FIR decimation 3:1 to 8 kHz; IMU is best-effort 200 Hz.
  Sequence gaps reveal queue overflows, IMU missed slots, or transport losses.
  ADC DMA driver overruns may cause uncounted skipped conversions.
*/
#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_timer.h>
#include <esp_adc/adc_continuous.h>
#include <esp_adc/adc_cali.h>
#include <esp_err.h>

static constexpr uint8_t SDA_PIN = 8, SCL_PIN = 9, MIC_PIN = 0;
static constexpr uint32_t MIC_RATE_HZ = 8000, ADC_RATE_HZ = 24000, IMU_PERIOD_US = 5000;
static constexpr uint16_t MIC_Q_CAP = 8192, IMU_Q_CAP = 256;
static constexpr uint8_t MAX_MIC_BATCH = 108, MAX_IMU_BATCH = 10;
static constexpr uint32_t SEND_INTERVAL_US = 50000; // transmit queued samples every 50 ms
static constexpr char DEVICE_NAME[] = "C3-IMU-MIC";
static constexpr char SERVICE_UUID[] = "ee8e0001-7d9d-4f38-85bd-a703a196e710";
static constexpr char DATA_UUID[]    = "ee8e0002-7d9d-4f38-85bd-a703a196e710";
static constexpr char COMMAND_UUID[] = "ee8e0003-7d9d-4f38-85bd-a703a196e710";

struct MicSample { uint32_t seq; uint16_t adc; };
struct ImuSample { uint64_t timeUs; uint32_t seq; int16_t v[6]; };
static MicSample micQ[MIC_Q_CAP];
static ImuSample imuQ[IMU_Q_CAP];
static uint16_t micHead=0, micTail=0, micCount=0;
static uint16_t imuHead=0, imuTail=0, imuCount=0;
static uint32_t micSeq=0, imuSeq=0, dropMic=0, dropImu=0;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool sampling=false, stopRequested=false;
static volatile char pendingCommand=0;
static volatile bool connected=false;
static volatile uint16_t negotiatedMtu=23;
static uint64_t syncedEpochUs=0, syncedBootUs=0;
static bool hasClock=false, pendingStatus=false;
static bool sendingStopped=false;
static adc_continuous_handle_t adcHandle=nullptr;
static adc_unit_t adcUnit;
static adc_channel_t adcChannel;
static volatile bool micTaskActive=false, imuTaskActive=false;
static volatile bool adcError=false;
static uint64_t micStartBootUs=0;
static Adafruit_MPU6050 mpu;
static NimBLEServer* server=nullptr;
static NimBLECharacteristic* dataChar=nullptr;

static inline void put16(uint8_t* p, uint16_t v) { p[0]=v; p[1]=v>>8; }
static inline void put32(uint8_t* p, uint32_t v) {
  for (int i=0;i<4;i++) p[i]=(uint8_t)(v>>(8*i));
}
static inline void put64(uint8_t* p, uint64_t v) {
  for (int i=0;i<8;i++) p[i]=(uint8_t)(v>>(8*i));
}
static inline uint64_t get64(const uint8_t* p) {
  uint64_t v=0; for (int i=0;i<8;i++) v|=(uint64_t)p[i]<<(8*i); return v;
}
static inline int16_t clampI16(float x) {
  if(x>32767) return 32767;
  if(x< -32768) return -32768;
  return (int16_t)lroundf(x);
}
static uint64_t epochFromBoot(uint64_t bootUs) {
  // Signed delta avoids unsigned wrap when the sample predates clock sync.
  int64_t delta=(int64_t)bootUs-(int64_t)syncedBootUs;
  return (uint64_t)((int64_t)syncedEpochUs+delta);
}

class ServerCallbacks final : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer*, NimBLEConnInfo& ci) override {
    connected=true; negotiatedMtu=ci.getMTU();
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo&) override { negotiatedMtu=mtu; }
  void onDisconnect(NimBLEServer*, NimBLEConnInfo&, int) override {
    sampling=false; connected=false; stopRequested=false;
    pendingCommand=0; negotiatedMtu=23;
    NimBLEDevice::startAdvertising();
  }
};

class CommandCallbacks final : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* ch, NimBLEConnInfo&) override {
    const auto value=ch->getValue();
    if(value.size() == 0) return;
    const uint8_t* p=(const uint8_t*)value.data();
    if(p[0]=='T' && value.size()==9) {
      const uint64_t epochMs=get64(p+1);
      // The browser's clock time is applied when the BLE write arrives.
      portENTER_CRITICAL(&mux);
      syncedEpochUs=epochMs*1000ULL;
      syncedBootUs=(uint64_t)esp_timer_get_time();
      hasClock=true;
      portEXIT_CRITICAL(&mux);
    } else if (p[0]=='B' || p[0]=='E' || p[0]=='R') {
      pendingCommand=(char)p[0];
    }
  }
};

static void pushMic(const MicSample& s) {
  portENTER_CRITICAL(&mux);
  if(micCount<MIC_Q_CAP) {
    micQ[micHead]=s; micHead=(micHead+1)%MIC_Q_CAP; ++micCount;
  } else ++dropMic;
  portEXIT_CRITICAL(&mux);
}
static void pushImu(const ImuSample& s) {
  portENTER_CRITICAL(&mux);
  if(imuCount<IMU_Q_CAP) {
    imuQ[imuHead]=s; imuHead=(imuHead+1)%IMU_Q_CAP; ++imuCount;
  } else ++dropImu;
  portEXIT_CRITICAL(&mux);
}

// ADC continuous mode uses the ESP32-C3 hardware ADC controller + DMA.
// ADC sample sequence numbers represent uniformly spaced 8 kHz output slots.
// A DMA overrun (ESP_ERR_INVALID_STATE) cannot be reconstructed precisely;
// flag it as an error. Application queue overflows are counted separately.
// 31-tap Hamming-window low-pass, 3.25 kHz cutoff at 24 kHz input.
// Software FIR anti-alias filtering occurs BEFORE 3:1 decimation.
static float fir[31];
static void designFir() {
  constexpr int N=31; const double fc=3250.0/ADC_RATE_HZ;
  double sum=0;
  for(int i=0;i<N;i++) {
    int k=i-(N-1)/2;
    double sinc=k==0?2*fc:sin(2*PI*fc*k)/(PI*k);
    double ham=0.54-0.46*cos(2*PI*i/(N-1));
    fir[i]=(float)(sinc*ham); sum+=fir[i];
  }
  for(int i=0;i<N;i++)fir[i]/=(float)sum;
}
static void micTask(void*) {
  alignas(4) uint8_t bytes[512];
  float history[31]={}; int histPos=0, phase=0;
  bool running=false; uint32_t seq=0;
  for (;;) {
    if (!sampling) {
      if (running) {
        // Stop ADC before flushing completed DMA frames.
        adc_continuous_stop(adcHandle);
        uint32_t n=0;
        for (int frame=0;frame<24;frame++) {
          esp_err_t e=adc_continuous_read(adcHandle,bytes,sizeof(bytes),&n,0);
          if(e!=ESP_OK || !n)break;
          for(uint32_t off=0;off+SOC_ADC_DIGI_RESULT_BYTES<=n;off+=SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t raw;
            memcpy(&raw,bytes+off,SOC_ADC_DIGI_RESULT_BYTES);
            if(raw.type2.channel!=adcChannel)continue;
            history[histPos]=raw.type2.data;histPos=(histPos+1)%31;
            if(++phase==3) { phase=0; float v=0;
              for(int i=0;i<31;i++)v+=fir[i]*history[(histPos+30-i)%31];
              pushMic({seq++,(uint16_t)constrain((int)lroundf(v),0,4095)});
            }
          }
        }
        micTaskActive=false; running=false;
      }
      vTaskDelay(pdMS_TO_TICKS(1)); continue;
    }
    if (!running) {
      seq=0;phase=0;histPos=0;memset(history,0,sizeof(history));
      micStartBootUs=(uint64_t)esp_timer_get_time();
      if(adc_continuous_start(adcHandle)!=ESP_OK) {
        adcError=true;sampling=false;continue;
      }
      running=true;micTaskActive=true;
    }
    uint32_t n=0;
    esp_err_t err=adc_continuous_read(adcHandle,bytes,sizeof(bytes),&n,10);
    if(err==ESP_OK) {
      for(uint32_t off=0;off+SOC_ADC_DIGI_RESULT_BYTES<=n;off+=SOC_ADC_DIGI_RESULT_BYTES) {
        adc_digi_output_data_t raw;
        memcpy(&raw,bytes+off,SOC_ADC_DIGI_RESULT_BYTES);
        if(raw.type2.channel!=adcChannel)continue;
        history[histPos]=raw.type2.data;histPos=(histPos+1)%31;
        if(++phase==3) { phase=0; float v=0;
          for(int i=0;i<31;i++)v+=fir[i]*history[(histPos+30-i)%31];
          pushMic({seq++,(uint16_t)constrain((int)lroundf(v),0,4095)});
        }
      }
    } else if(err!=ESP_ERR_TIMEOUT) {
      adcError=true;
      Serial.printf("ADC DMA error: %s\n",esp_err_to_name(err));
      sampling=false;
    }
  }
}

// MPU6050 independently acquired at nominal 200 Hz; IMU scheduling jitter
// remains possible, and missed deadlines are represented as sequence gaps.
static void imuTask(void*) {
  uint64_t nextImu=0;
  bool active=false;
  for (;;) {
    if (!sampling) {
      active=false; imuTaskActive=false;
      vTaskDelay(pdMS_TO_TICKS(1)); continue;
    }
    uint64_t now=(uint64_t)esp_timer_get_time();
    if (!active) { nextImu=now; active=true; imuTaskActive=true; }
    if (now<nextImu) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
    uint32_t missed=(uint32_t)((now-nextImu)/IMU_PERIOD_US);
    imuSeq+=missed;
    nextImu+=(uint64_t)(missed+1)*IMU_PERIOD_US;
    sensors_event_t a,g,temp;
    mpu.getEvent(&a,&g,&temp);
    ImuSample s{};
    s.timeUs=(uint64_t)esp_timer_get_time();
    s.seq=imuSeq++;
    constexpr float ACC=16384.0f/9.80665f;
    constexpr float GYR=131.0f*180.0f/PI;
    s.v[0]=clampI16(a.acceleration.x*ACC);
    s.v[1]=clampI16(a.acceleration.y*ACC);
    s.v[2]=clampI16(a.acceleration.z*ACC);
    s.v[3]=clampI16(g.gyro.x*GYR);
    s.v[4]=clampI16(g.gyro.y*GYR);
    s.v[5]=clampI16(g.gyro.z*GYR);
    pushImu(s);
  }
}

static bool initializeAdc() {
  if (adc_continuous_io_to_channel(MIC_PIN,&adcUnit,&adcChannel)!=ESP_OK || adcUnit!=ADC_UNIT_1) return false;
  adc_continuous_handle_cfg_t hcfg={};
  hcfg.max_store_buf_size=4096;
  hcfg.conv_frame_size=256;
  if(adc_continuous_new_handle(&hcfg,&adcHandle)!=ESP_OK) return false;
  static adc_digi_pattern_config_t pattern[1]={};
  pattern[0].atten=ADC_ATTEN_DB_12;
  pattern[0].channel=(uint8_t)adcChannel;
  pattern[0].unit=(uint8_t)adcUnit;
  pattern[0].bit_width=ADC_BITWIDTH_12;
  adc_continuous_config_t cfg={};
  cfg.sample_freq_hz=ADC_RATE_HZ;
  cfg.conv_mode=ADC_CONV_SINGLE_UNIT_1;
  cfg.format=ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  cfg.pattern_num=1;
  cfg.adc_pattern=pattern;
  return adc_continuous_config(adcHandle,&cfg)==ESP_OK;
}

static bool sendStatus(uint8_t code) {
  uint8_t pkt[12]={'S',code};
  put16(pkt+2,negotiatedMtu);
  uint32_t m,i;
  portENTER_CRITICAL(&mux); m=dropMic; i=dropImu; portEXIT_CRITICAL(&mux);
  put32(pkt+4,m); put32(pkt+8,i);
  return connected && dataChar->notify(pkt,sizeof(pkt));
}

// Select contiguous samples only. This preserves accurate sequence-gap detection
// even after sample overruns or skipped schedule slots.
static bool sendMicBatch() {
  MicSample batch[MAX_MIC_BATCH];
  uint8_t n=0;
  uint16_t mtu=negotiatedMtu;
  int maxPayload=(int)mtu-3-16;
  if(maxPayload<2) return false;
  uint8_t maxN=min((int)MAX_MIC_BATCH,maxPayload/2);
  portENTER_CRITICAL(&mux);
  const uint16_t available=micCount;
  if(available) {
    uint16_t idx=micTail;
    for(uint16_t k=0;k<available && n<maxN;k++) {
      MicSample s=micQ[idx];
      if(n && s.seq!=batch[n-1].seq+1) break;
      batch[n++]=s; idx=(idx+1)%MIC_Q_CAP;
    }
  }
  portEXIT_CRITICAL(&mux);
  if(!n) return false;
  uint8_t pkt[16+2*MAX_MIC_BATCH];
  pkt[0]='M';pkt[1]=1;pkt[2]=n;pkt[3]=0;
  put32(pkt+4,batch[0].seq);
  put64(pkt+8,epochFromBoot(micStartBootUs + ((uint64_t)batch[0].seq*1000000ULL)/MIC_RATE_HZ));
  for(uint8_t j=0;j<n;j++) put16(pkt+16+2*j,batch[j].adc);
  if(!dataChar->notify(pkt,16+2*n)) return false;
  portENTER_CRITICAL(&mux);
  micTail=(micTail+n)%MIC_Q_CAP; micCount-=n;
  portEXIT_CRITICAL(&mux);
  return true;
}
static bool sendImuBatch() {
  ImuSample batch[MAX_IMU_BATCH];
  uint8_t n=0;
  int maxPayload=(int)negotiatedMtu-3-16;
  if(maxPayload<12) return false;
  uint8_t maxN=min((int)MAX_IMU_BATCH,maxPayload/12);
  portENTER_CRITICAL(&mux);
  uint16_t available=imuCount;
  if(available) {
    uint16_t idx=imuTail;
    for(uint16_t k=0;k<available && n<maxN;k++) {
      ImuSample s=imuQ[idx];
      if(n && s.seq!=batch[n-1].seq+1) break;
      batch[n++]=s; idx=(idx+1)%IMU_Q_CAP;
    }
  }
  portEXIT_CRITICAL(&mux);
  if(!n) return false;
  uint8_t pkt[16+12*MAX_IMU_BATCH];
  pkt[0]='I';pkt[1]=1;pkt[2]=n;pkt[3]=0;
  put32(pkt+4,batch[0].seq);
  put64(pkt+8,epochFromBoot(batch[0].timeUs));
  for(uint8_t j=0;j<n;j++) for(uint8_t k=0;k<6;k++)
    put16(pkt+16+j*12+k*2,(uint16_t)batch[j].v[k]);
  if(!dataChar->notify(pkt,16+12*n)) return false;
  portENTER_CRITICAL(&mux);
  imuTail=(imuTail+n)%IMU_Q_CAP; imuCount-=n;
  portEXIT_CRITICAL(&mux);
  return true;
}

void setup() {
  Serial.begin(115200);
  designFir();
  if(!initializeAdc()) {
    Serial.println("ADC DMA initialization failed. Check ESP32 Arduino core 3.x.");
    while(true) delay(2000);
  }
  Wire.begin(SDA_PIN,SCL_PIN,400000);
  if(!mpu.begin(0x68,&Wire)) {
    Serial.println("MPU6050 not found at 0x68. Check power/SDA/SCL.");
    while(true) delay(2000);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);

  NimBLEDevice::init(DEVICE_NAME);
  NimBLEDevice::setMTU(247);
  server=NimBLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  server->advertiseOnDisconnect(true);
  NimBLEService* svc=server->createService(SERVICE_UUID);
  dataChar=svc->createCharacteristic(DATA_UUID,NIMBLE_PROPERTY::NOTIFY);
  NimBLECharacteristic* cmd=svc->createCharacteristic(COMMAND_UUID,NIMBLE_PROPERTY::WRITE);
  cmd->setCallbacks(new CommandCallbacks());
  svc->start();
  NimBLEAdvertising* adv=NimBLEDevice::getAdvertising();
  adv->enableScanResponse(true);
  adv->addServiceUUID(SERVICE_UUID);
  adv->setName(DEVICE_NAME);
  bool advertisingOK=adv->start();
  Serial.printf("BLE advertising: %s\n",advertisingOK?"YES":"NO");
  xTaskCreate(micTask,"adc_dma",4096,nullptr,3,nullptr);
  xTaskCreate(imuTask,"imu_200hz",4096,nullptr,2,nullptr);
  Serial.println("BLE ready: C3-IMU-MIC | MIC 8000 Hz (24k DMA + FIR /3) | IMU 200 Hz | BLE 50ms batches");
}

void loop() {
  char cmd=pendingCommand;
  if(cmd) pendingCommand=0;
  if(cmd=='B') {
    sampling=false;
    while(micTaskActive || imuTaskActive) delay(1);
    uint16_t mtu=negotiatedMtu;
    if(!connected || !hasClock || mtu<100) {
      sendStatus(3);
    } else {
      portENTER_CRITICAL(&mux);
      micHead=micTail=micCount=imuHead=imuTail=imuCount=0;
      micSeq=imuSeq=dropMic=dropImu=0;
      portEXIT_CRITICAL(&mux);
      sendingStopped=false; stopRequested=false; adcError=false;
      sampling=true;
      sendStatus(1);
    }
  } else if(cmd=='E') {
    sampling=false; stopRequested=true;
    // Let ADC task drain the final DMA frame before declaring capture complete.
    while(micTaskActive || imuTaskActive) delay(1);
  } else if(cmd=='R') {
    pendingStatus=true;
  }
  if(!connected) { delay(20); return; }
  if(pendingStatus) { if(sendStatus(sampling?1:2)) pendingStatus=false; }
  // Send acquired data in bursts every 50 ms. BLE connection events still
  // continue between bursts. The sample clock remains independent of radio.
  static uint64_t nextBurstUs=0;
  static bool burstActive=false;
  static uint8_t packetsThisBurst=0;
  uint64_t now=(uint64_t)esp_timer_get_time();
  if(!(sampling || stopRequested)) {burstActive=false;nextBurstUs=0;}
  if((sampling || stopRequested) && (stopRequested || !nextBurstUs || now>=nextBurstUs)) {
    if(!burstActive) {burstActive=true;packetsThisBurst=0;}
    if(!nextBurstUs || now>=nextBurstUs)nextBurstUs=now+SEND_INTERVAL_US;
  }
  bool didWork=false;
  if(burstActive && packetsThisBurst<12) {
    // Prefer audio, but allocate one in four notifications to IMU.
    if((packetsThisBurst%4)==3) {
      didWork=sendImuBatch(); if(!didWork)didWork=sendMicBatch();
    } else {
      didWork=sendMicBatch(); if(!didWork)didWork=sendImuBatch();
    }
    if(didWork)++packetsThisBurst;
    else burstActive=false;
    if(packetsThisBurst>=12)burstActive=false;
  }
  if(stopRequested && !sendingStopped) {
    uint16_t m,i;
    portENTER_CRITICAL(&mux); m=micCount; i=imuCount; portEXIT_CRITICAL(&mux);
    if(!m && !i && sendStatus(adcError?3:2)) {
      stopRequested=false; sendingStopped=true;
    }
  }
  delay(didWork?2:1); // avoid zero-delay tight-loop flooding of notify()
}
