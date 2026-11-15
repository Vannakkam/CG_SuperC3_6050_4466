#include <Wire.h>
#include <MPU6050.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// --- Hardware Configuration ---
#define I2C_SDA 8
#define I2C_SCL 9
#define MIC_PIN 2  // "A2" on many C3 SuperMini boards

// --- BLE UUIDs ---
#define SERVICE_UUID           "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHAR_UUID_IMU          "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHAR_UUID_AUDIO        "beb5483e-36e1-4688-b7f5-ea07361b26a9"
#define CHAR_UUID_TIME_SYNC    "beb5483e-36e1-4688-b7f5-ea07361b26b0"

// --- Objects ---
MPU6050 mpu;
BLEServer* pServer = NULL;
BLECharacteristic* pImuChar = NULL;
BLECharacteristic* pAudioChar = NULL;
BLECharacteristic* pTimeChar = NULL;

// --- State ---
bool deviceConnected = false;
bool recording = false;
uint32_t packetCounter = 0;
uint32_t baseTimeMs = 0;

// --- Timer Variables (Hardware Timer for precise sampling) ---
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool sampleFlag = false;

// --- Data Buffers ---
// We batch IMU data to save BLE bandwidth. 10 samples = 50ms at 200Hz.
struct ImuSample {
  int16_t ax, ay, az, gx, gy, gz;
  uint32_t dt; // delta time in microseconds from base
};
ImuSample imuBuffer[10];
uint8_t imuIdx = 0;

// Audio is sent in batches of 100 samples (50ms at 2kHz)
uint16_t audioBuffer[100];
uint8_t audioIdx = 0;

// --- Timer ISR ---
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  sampleFlag = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// --- BLE Callbacks ---
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      recording = false; // Stop recording on disconnect
      pServer->startAdvertising(); // Restart advertising
    }
};

class TimeCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() == 4) {
        // Receive 4-byte Unix timestamp from browser
        uint32_t ts = *((uint32_t*)value.data());
        baseTimeMs = ts; 
        Serial.print("Time synced: "); Serial.println(ts);
      }
    }
};

class CommandCallback: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) {
        char cmd = value[0];
        if (cmd == '1') { recording = true; baseTimeMs = millis(); } // Start with local time ref
        if (cmd == '0') { recording = false; }
      }
    }
};

void setup() {
  Serial.begin(115200);
  
  // 1. Init I2C & MPU6050
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
  } else {
    Serial.println("MPU6050 OK");
  }
  
  // 2. Init BLE
  BLEDevice::init("ESP32_C3_Sensor");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  // IMU Characteristic (Notify)
  pImuChar = pService->createCharacteristic(CHAR_UUID_IMU, BLECharacteristic::PROPERTY_NOTIFY);
  pImuChar->addDescriptor(new BLE2902());

  // Audio Characteristic (Notify)
  pAudioChar = pService->createCharacteristic(CHAR_UUID_AUDIO, BLECharacteristic::PROPERTY_NOTIFY);
  pAudioChar->addDescriptor(new BLE2902());

  // Time Sync Characteristic (Write only)
  pTimeChar = pService->createCharacteristic(CHAR_UUID_TIME_SYNC, BLECharacteristic::PROPERTY_WRITE);
  pTimeChar->setCallbacks(new TimeCallback());

  // Control Characteristic (Write only) - using same UUID for simplicity in this example
  BLECharacteristic *pCtrlChar = pService->createCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26b1", BLECharacteristic::PROPERTY_WRITE);
  pCtrlChar->setCallbacks(new CommandCallback());

  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); 
  BLEDevice::startAdvertising();
  Serial.println("Waiting for client...");

  // 3. Setup Timer for 2kHz (500us)
  // For 200Hz IMU we will divide by 10 in the loop
  timer = timerBegin(0, 80, true); // 80 prescaler = 1MHz (1us tick)
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 500, true); // 500us = 2kHz
  timerAlarmEnable(timer);
}

void loop() {
  if (!deviceConnected || !recording) {
    delay(10);
    return;
  }

  if (sampleFlag) {
    portENTER_CRITICAL(&timerMux);
    sampleFlag = false;
    portEXIT_CRITICAL(&timerMux);

    // --- AUDIO SAMPLING (2kHz) ---
    int val = analogRead(MIC_PIN);
    audioBuffer[audioIdx++] = (uint16_t)val;
    
    if (audioIdx >= 100) {
      // Send Audio Packet
      pAudioChar->setValue((uint8_t*)audioBuffer, sizeof(audioBuffer));
      pAudioChar->notify();
      audioIdx = 0;
    }

    // --- IMU SAMPLING (200Hz) ---
    static uint8_t imuDiv = 0;
    imuDiv++;
    if (imuDiv >= 10) {
      imuDiv = 0;
      
      int16_t ax, ay, az, gx, gy, gz;
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      
      imuBuffer[imuIdx].ax = ax;
      imuBuffer[imuIdx].ay = ay;
      imuBuffer[imuIdx].az = az;
      imuBuffer[imuIdx].gx = gx;
      imuBuffer[imuIdx].gy = gy;
      imuBuffer[imuIdx].gz = gz;
      imuBuffer[imuIdx].dt = micros(); // Timestamp
      imuIdx++;

      if (imuIdx >= 10) {
        // Send IMU Packet (10 samples = 60 bytes + headers, fits in standard MTU)
        pImuChar->setValue((uint8_t*)imuBuffer, sizeof(imuBuffer));
        pImuChar->notify();
        imuIdx = 0;
      }
    }
  }
}