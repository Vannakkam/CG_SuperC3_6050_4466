/*
 * ESP32-C3 Super Mini — MPU6050 (I2C 8/9) + MAX4466 (GPIO 2)
 * Binary framed BLE streaming with audio DPCM compression.
 *
 * Protocol (all little-endian):
 *   Header (10 bytes):
 *     magic   : u8   = 0xA5
 *     type    : u8   = 0x01 (IMU) | 0x02 (AUDIO) | 0x03 (HELLO)
 *     seq     : u16  = per-stream monotonic counter (wraps)
 *     t0_us   : u32  = micros() of FIRST sample in packet
 *     len     : u16  = payload byte count (excluding header)
 *
 *   IMU payload (type 0x01):
 *     N * 12 bytes: int16 ax,ay,az,gx,gy,gz  (N = 10)
 *     (per-sample dt not sent; host uses 5000us spacing from t0_us)
 *
 *   AUDIO payload (type 0x02):
 *     u8  codec      = 0 = RAW16, 1 = DPCM8
 *     u8  first_lo   = low  8 bits of first sample (u16)
 *     u16 first_hi   = high 8 bits (only lower 4 used) of first sample
 *     u8  count      = number of DPCM deltas that follow
 *     u8  deltas[count]  // signed int8 differential, reconstructed by:
 *                        //   s[i] = s[i-1] + delta[i]   (clamped to 0..4095)
 *     If codec = 0 (fallback when DPCM deltas exceed int8):
 *     u16 raw[count]     // raw 12-bit values, count*2 bytes
 *
 *   HELLO payload (type 0x03):
 *     u8  version = 1
 *     u8  imu_hz  = 200
 *     u16 audio_hz= 2000
 */

#include <Wire.h>
#include <MPU6050.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------- Configuration ----------
#define I2C_SDA        8
#define I2C_SCL        9
#define MIC_PIN        2

#define IMU_HZ         200
#define AUDIO_HZ       2000
#define TIMER_TICK_US  500          // 2 kHz base tick
#define IMU_DIV        (AUDIO_HZ / IMU_HZ)   // = 10

#define IMU_BATCH      10           // 10 * 12 = 120 B payload
#define AUDIO_BATCH    90           // 90 deltas * 1 B = 90 B payload (+ 5 B audio hdr) = 95 B

#define PROTO_MAGIC    0xA5
#define TYPE_IMU       0x01
#define TYPE_AUDIO     0x02
#define TYPE_HELLO     0x03

// ---------- BLE UUIDs ----------
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID_IMU       "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_UUID_AUDIO     "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHAR_UUID_TIME      "beb5483e-36e1-4688-b7f5-ea07361b26b0"
#define CHAR_UUID_CTRL      "beb5483e-36e1-4688-b7f5-ea07361b26b1"

// ---------- BLE objects ----------
BLEServer*           pServer   = nullptr;
BLECharacteristic*   pImuChar  = nullptr;
BLECharacteristic*   pAudioChar= nullptr;
BLECharacteristic*   pTimeChar = nullptr;
BLECharacteristic*   pCtrlChar = nullptr;
volatile bool        deviceConnected = false;
volatile bool        recording       = false;

// ---------- Timing ----------
volatile uint32_t    baseEpochSec  = 0;    // set by host on pairing
volatile uint32_t    baseMicrosRef = 0;    // micros() when baseEpochSec was set

// ---------- Sampler state (ISR context) ----------
hw_timer_t*          timer    = nullptr;
portMUX_TYPE         timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool        tickFlag = false;
volatile uint8_t     imuDiv   = 0;

// ---------- Ring buffers (single-producer / single-consumer) ----------
// IMU: 32 packets × 10 samples × 12 B = 3840 B
struct ImuSample { int16_t ax, ay, az, gx, gy, gz; };
static_assert(sizeof(ImuSample) == 12, "ImuSample must be 12 B packed");

#define IMU_QUEUE_PKTS  32
static ImuSample     imuQueue[IMU_QUEUE_PKTS][IMU_BATCH];
static uint32_t      imuQueueT0[IMU_QUEUE_PKTS];
static volatile uint16_t imuHead = 0, imuTail = 0;

// Audio: 64 packets × 90 samples × 2 B = 11 520 B
#define AUD_QUEUE_PKTS  64
static uint16_t      audQueue[AUD_QUEUE_PKTS][AUDIO_BATCH];
static uint32_t      audQueueT0[AUD_QUEUE_PKTS];
static volatile uint16_t audHead = 0, audTail = 0;

// Local staging (ISR writes here, then commits a whole packet)
static ImuSample     imuStage[IMU_BATCH];
static uint8_t       imuStageCount = 0;
static uint32_t      imuStageT0    = 0;

static uint16_t      audStage[AUDIO_BATCH];
static uint8_t       audStageCount = 0;
static uint32_t      audStageT0    = 0;

// ---------- TX sequence ----------
static uint16_t      seqImu = 0, seqAud = 0;

// ---------- Timer ISR ----------
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  tickFlag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ---------- Helpers ----------
static inline uint16_t qNext(uint16_t h, uint16_t cap) {
  return (uint16_t)((h + 1) % cap);
}

// ---------- BLE callbacks ----------
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    deviceConnected = true;
    // Re-send HELLO once the client is up.
    sendHello();
  }
  void onDisconnect(BLEServer* s) override {
    deviceConnected = false;
    recording = false;
    s->startAdvertising();
  }
};

class TimeCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.size() == 4) {
      uint32_t sec;
      memcpy(&sec, v.data(), 4);
      baseEpochSec  = sec;
      baseMicrosRef = micros();
      Serial.printf("Time sync: epoch=%u s\n", sec);
    }
  }
};

class CtrlCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    std::string v = c->getValue();
    if (v.empty()) return;
    char cmd = v[0];
    if (cmd == '1') {
      // Reset sequence counters and drain queues on start.
      seqImu = seqAud = 0;
      imuHead = imuTail = 0;
      audHead = audTail = 0;
      imuStageCount = audStageCount = 0;
      recording = true;
      Serial.println("Recording START");
    } else if (cmd == '0') {
      recording = false;
      Serial.println("Recording STOP");
    }
  }
};

// ---------- HELLO ----------
static void sendHello() {
  if (!pImuChar) return;
  uint8_t pkt[10 + 4];
  uint16_t len = 4;
  pkt[0] = PROTO_MAGIC;
  pkt[1] = TYPE_HELLO;
  pkt[2] = 0; pkt[3] = 0;             // seq 0
  uint32_t t0 = micros();
  memcpy(&pkt[4], &t0, 4);
  memcpy(&pkt[8], &len, 2);
  pkt[10] = 1;                        // version
  pkt[11] = IMU_HZ & 0xFF;
  pkt[12] = AUDIO_HZ & 0xFF;
  pkt[13] = (AUDIO_HZ >> 8) & 0xFF;
  pImuChar->setValue(pkt, 10 + len);
  pImuChar->notify();
}

// ---------- Audio encoder ----------
// Returns payload length. Writes into out[] (needs >= 5 + AUDIO_BATCH*2).
static uint16_t encodeAudio(const uint16_t* samples, uint8_t n, uint8_t* out) {
  uint8_t  codec = 1;                 // try DPCM8 first
  int16_t  prev  = (int16_t)samples[0];

  // Trial encode as DPCM8
  uint8_t  deltas[AUDIO_BATCH];
  bool     fits = true;
  for (uint8_t i = 1; i < n; i++) {
    int32_t d = (int32_t)samples[i] - prev;
    if (d < -128 || d > 127) { fits = false; break; }
    deltas[i - 1] = (uint8_t)(int8_t)d;
    prev = (int16_t)samples[i];
  }

  out[0] = codec;
  out[1] = (uint8_t)(samples[0] & 0xFF);
  out[2] = (uint8_t)((samples[0] >> 8) & 0x0F);   // only 4 bits used (12-bit ADC)
  out[3] = (uint8_t)(n - 1);                       // count of deltas
  if (fits) {
    memcpy(&out[4], deltas, n - 1);
    return 4 + (n - 1);
  }

  // Fallback: RAW16
  out[0] = 0;
  out[1] = (uint8_t)(samples[0] & 0xFF);
  out[2] = (uint8_t)((samples[0] >> 8) & 0x0F);
  out[3] = (uint8_t)(n);                           // count of raw values
  memcpy(&out[4], samples, n * 2);
  return 4 + n * 2;
}

// ---------- FreeRTOS TX task ----------
static void txTask(void*) {
  static uint8_t frame[10 + IMU_BATCH * 12];
  static uint8_t aframe[10 + 5 + AUDIO_BATCH * 2];

  for (;;) {
    bool didWork = false;

    if (!deviceConnected) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }

    // ---- Drain IMU queue ----
    if (imuTail != imuHead) {
      uint16_t slot = imuTail;
      uint16_t len  = IMU_BATCH * sizeof(ImuSample);
      frame[0] = PROTO_MAGIC;
      frame[1] = TYPE_IMU;
      memcpy(&frame[2], &seqImu, 2);
      memcpy(&frame[4], &imuQueueT0[slot], 4);
      memcpy(&frame[8], &len, 2);
      memcpy(&frame[10], imuQueue[slot], len);

      pImuChar->setValue(frame, 10 + len);
      pImuChar->notify();
      seqImu++;
      imuTail = qNext(imuTail, IMU_QUEUE_PKTS);
      didWork = true;
    }

    // ---- Drain Audio queue ----
    if (audTail != audHead) {
      uint16_t slot = audTail;
      uint16_t plen = encodeAudio(audQueue[slot], AUDIO_BATCH, &aframe[10]);
      aframe[0] = PROTO_MAGIC;
      aframe[1] = TYPE_AUDIO;
      memcpy(&aframe[2], &seqAud, 2);
      memcpy(&aframe[4], &audQueueT0[slot], 4);
      memcpy(&aframe[8], &plen, 2);

      pAudioChar->setValue(aframe, 10 + plen);
      pAudioChar->notify();
      seqAud++;
      audTail = qNext(audTail, AUD_QUEUE_PKTS);
      didWork = true;
    }

    if (!didWork) vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  delay(200);

  // ADC
  analogReadResolution(12);
  analogSetPinAttenuation(MIC_PIN, ADC_11db);

  // I2C + MPU
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  static MPU6050 mpu(&Wire);
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 OK" : "MPU6050 FAIL");

  // BLE
  BLEDevice::init("ESP32_C3_Sensor");
  BLEDevice::setMTU(247);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCB());
  BLEService* svc = pServer->createService(SERVICE_UUID);

  pImuChar = svc->createCharacteristic(CHAR_UUID_IMU, BLECharacteristic::PROPERTY_NOTIFY);
  pImuChar->addDescriptor(new BLE2902());

  pAudioChar = svc->createCharacteristic(CHAR_UUID_AUDIO, BLECharacteristic::PROPERTY_NOTIFY);
  pAudioChar->addDescriptor(new BLE2902());

  pTimeChar = svc->createCharacteristic(CHAR_UUID_TIME, BLECharacteristic::PROPERTY_WRITE);
  pTimeChar->setCallbacks(new TimeCB());

  pCtrlChar = svc->createCharacteristic(CHAR_UUID_CTRL, BLECharacteristic::PROPERTY_WRITE);
  pCtrlChar->setCallbacks(new CtrlCB());

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  BLEDevice::startAdvertising();
  Serial.println("Advertising as ESP32_C3_Sensor");

  // FreeRTOS TX task on core 0 (BLE runs on core 0 too, but separate task OK)
  xTaskCreatePinnedToCore(txTask, "txTask", 4096, nullptr, 2, nullptr, 0);

  // 2 kHz hardware timer
  timer = timerBegin(0, 80, true);      // 1 MHz tick
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_TICK_US, true);
  timerAlarmEnable(timer);
}

// ---------- Main loop: sampling only, no BLE calls ----------
void loop() {
  if (!recording) { delay(5); return; }
  if (!tickFlag) { delayMicroseconds(50); return; }

  portENTER_CRITICAL(&timerMux);
  tickFlag = false;
  portEXIT_CRITICAL(&timerMux);

  uint32_t nowUs = micros();

  // ---- Audio (every tick = 2 kHz) ----
  if (audStageCount == 0) audStageT0 = nowUs;
  audStage[audStageCount++] = (uint16_t)analogRead(MIC_PIN);
  if (audStageCount >= AUDIO_BATCH) {
    uint16_t nh = qNext(audHead, AUD_QUEUE_PKTS);
    if (nh != audTail) {
      memcpy(audQueue[audHead], audStage, AUDIO_BATCH * 2);
      audQueueT0[audHead] = audStageT0;
      audHead = nh;
      audStageCount = 0;
    }
    // else: queue full, drop this batch (host will see seq gap)
  }

  // ---- IMU (every 10th tick = 200 Hz) ----
  if (++imuDiv >= IMU_DIV) {
    imuDiv = 0;
    if (imuStageCount == 0) imuStageT0 = nowUs;

    ImuSample s;
    mpu.getMotion6(&s.ax, &s.ay, &s.az, &s.gx, &s.gy, &s.gz);
    imuStage[imuStageCount++] = s;

    if (imuStageCount >= IMU_BATCH) {
      uint16_t nh = qNext(imuHead, IMU_QUEUE_PKTS);
      if (nh != imuTail) {
        memcpy(imuQueue[imuHead], imuStage, IMU_BATCH * sizeof(ImuSample));
        imuQueueT0[imuHead] = imuStageT0;
        imuHead = nh;
        imuStageCount = 0;
      }
    }
  }
}