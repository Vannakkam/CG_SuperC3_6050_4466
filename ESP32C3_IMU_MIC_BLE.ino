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
  (500us for MIC, 5000us for IMU). The first sample is timestamped
  at acquisition. Sampling is best effort, not an ADC/I2C hardware-clocked system.
  Sequence gaps reveal skipped samples or transport losses.
*/
#include <Arduino.h>
#include <Wire.h>
#include <NimBLEDevice.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <esp_timer.h>

static constexpr uint8_t SDA_PIN = 8, SCL_PIN = 9, MIC_PIN = 0;
static constexpr uint32_t MIC_PERIOD_US = 500, IMU_PERIOD_US = 5000;
static constexpr uint16_t MIC_Q_CAP = 512, IMU_Q_CAP = 128;
static constexpr uint8_t MAX_MIC_BATCH = 40, MAX_IMU_BATCH = 10;
static constexpr char DEVICE_NAME[] = "C3-IMU-MIC";
static constexpr char SERVICE_UUID[] = "ee8e0001-7d9d-4f38-85bd-a703a196e710";
static constexpr char DATA_UUID[]    = "ee8e0002-7d9d-4f38-85bd-a703a196e710";
static constexpr char COMMAND_UUID[] = "ee8e0003-7d9d-4f38-85bd-a703a196e710";

struct MicSample { uint64_t timeUs; uint32_t seq; uint16_t adc; };
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
  // Written once from command callback; no overflow for normal session lengths.
  return syncedEpochUs + (bootUs - syncedBootUs);
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
    if(value.empty()) return;
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

static void sensorTask(void*) {
  uint64_t nextMic=0, nextImu=0;
  bool wasSampling=false;
  while(true) {
    if(!sampling) { wasSampling=false; vTaskDelay(pdMS_TO_TICKS(2)); continue; }
    uint64_t now=(uint64_t)esp_timer_get_time();
    if(!wasSampling) { nextMic=now; nextImu=now; wasSampling=true; }

    // IMU first: a 400kHz I2C burst can delay an ADC sample.
    if(now>=nextImu) {
      const uint32_t missed=(uint32_t)((now-nextImu)/IMU_PERIOD_US);
      imuSeq+=missed; nextImu+=(uint64_t)(missed+1)*IMU_PERIOD_US;
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
      now=(uint64_t)esp_timer_get_time();
    }
    if(now>=nextMic) {
      const uint32_t missed=(uint32_t)((now-nextMic)/MIC_PERIOD_US);
      micSeq+=missed; nextMic+=(uint64_t)(missed+1)*MIC_PERIOD_US;
      MicSample s{};
      s.adc=(uint16_t)analogRead(MIC_PIN);
      s.timeUs=(uint64_t)esp_timer_get_time();
      s.seq=micSeq++;
      pushMic(s);
    }
    // Keep ADC scheduling fine-grained while the BLE/OS higher-priority
    // tasks remain able to preempt this task. This is CPU-intensive.
    delayMicroseconds(40);
  }
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
  put64(pkt+8,epochFromBoot(batch[0].timeUs));
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
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN,ADC_11db);
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
  adv->addServiceUUID(SERVICE_UUID);
  adv->setName(DEVICE_NAME);
  adv->start();
  xTaskCreate(sensorTask,"acquisition",8192,nullptr,2,nullptr);
  Serial.println("BLE ready: C3-IMU-MIC");
}

void loop() {
  char cmd=pendingCommand;
  if(cmd) pendingCommand=0;
  if(cmd=='B') {
    sampling=false;
    // Allow prior acquisition iteration to finish before resetting its queues.
    delay(8);
    uint16_t mtu=negotiatedMtu;
    if(!connected || !hasClock || mtu<31) {
      sendStatus(3);
    } else {
      portENTER_CRITICAL(&mux);
      micHead=micTail=micCount=imuHead=imuTail=imuCount=0;
      micSeq=imuSeq=dropMic=dropImu=0;
      portEXIT_CRITICAL(&mux);
      sendingStopped=false; stopRequested=false;
      sampling=true;
      sendStatus(1);
    }
  } else if(cmd=='E') {
    sampling=false; stopRequested=true;
    // Sensor task may still be finishing its current cycle.
    delay(8);
  } else if(cmd=='R') {
    pendingStatus=true;
  }
  if(!connected) { delay(20); return; }
  if(pendingStatus) { if(sendStatus(sampling?1:2)) pendingStatus=false; }
  bool didWork=false;
  static uint8_t roundRobin=0;
  if(sampling || stopRequested) {
    if(roundRobin++ & 1) {
      didWork=sendMicBatch();
      if(!didWork) didWork=sendImuBatch();
    } else {
      didWork=sendImuBatch();
      if(!didWork) didWork=sendMicBatch();
    }
  }
  if(stopRequested && !sendingStopped) {
    uint16_t m,i;
    portENTER_CRITICAL(&mux); m=micCount; i=imuCount; portEXIT_CRITICAL(&mux);
    if(!m && !i && sendStatus(2)) {
      stopRequested=false; sendingStopped=true;
    }
  }
  delay(didWork?1:3);
}
