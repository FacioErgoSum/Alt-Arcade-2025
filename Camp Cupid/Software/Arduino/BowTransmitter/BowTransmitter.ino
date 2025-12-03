#include <Adafruit_LSM6DSOX.h>
#include <Wire.h>

#include "ESP32_NOW_Serial.h"
#include "MacAddress.h"
#include "WiFi.h"
#include "esp_wifi.h"

Adafruit_LSM6DSOX lsm6dsox;
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL = 10; // Increased sampling rate for better detection

// IR Sensor variables
int IRsensorAddress = 0xB0;
int slaveAddress;
int ledPin = 13;
boolean ledState = false;
byte data_buf[16];
int i;
int Ix[4];
int Iy[4];
int s;

// Button variables
const int buttonPin = A5;

// Position buffer for shake compensation
const int BUFFER_SIZE = 10;  // Enough to store ~150ms of data
const unsigned long POSITION_DELAY = 100;  // Use position from 100ms ago
int xBuffer[BUFFER_SIZE];
int yBuffer[BUFFER_SIZE];
unsigned long timeBuffer[BUFFER_SIZE];
int bufferIndex = 0;
bool bufferFilled = false;

// Hit state tracking (0=none, 1=soft, 2=medium, 3=hard)
volatile int currentHitState = 0;
unsigned long hitStateSetTime = 0;
const unsigned long HIT_STATE_DURATION = 100; // How long to show hit state (ms)

// ===== ADJUSTABLE DETECTION PARAMETERS =====
const float HP_FILTER_ALPHA = 0.95;
const float SOFT_HIT_THRESHOLD = 5.5;
const float MEDIUM_HIT_THRESHOLD = 6.75;
const float HARD_HIT_THRESHOLD = 8.0;
const float BASELINE_ALPHA = 0.98;
const unsigned long DEBOUNCE_TIME = 200;
const float MIN_BASELINE = 0.5;
// ============================================

// High-pass filter state
float hpf_prev_input = 0;
float hpf_prev_output = 0;

// Baseline tracking
float baseline = 1.0;

// Peak detection state
float peak_value = 0;
unsigned long last_hit_time = 0;
bool in_hit_window = false;
const unsigned long HIT_WINDOW = 50; // Time window to capture peak (ms)
unsigned long hit_window_start = 0;

// ===== ESP-NOW SERIAL CONFIG (SIMPLE, HARD-CODED PEER) =====
#define ESPNOW_WIFI_MODE_STATION 1
#define ESPNOW_WIFI_CHANNEL 1

#if ESPNOW_WIFI_MODE_STATION
  #define ESPNOW_WIFI_MODE WIFI_STA
  #define ESPNOW_WIFI_IF   WIFI_IF_STA
#else
  #define ESPNOW_WIFI_MODE WIFI_AP
  #define ESPNOW_WIFI_IF   WIFI_IF_AP
#endif

// Hard-coded peer MAC: B0:81:84:9E:75:34 (receiver)
uint8_t peerMacAddress[6] = {0xB0, 0x81, 0x84, 0x9E, 0x75, 0x34};

ESP_NOW_Serial_Class* NowSerial = nullptr;
bool nowSerialActive = false;

bool initNowSerial() {
  Serial.println();
  Serial.println("Initializing ESP-NOW Serial (controller)...");

  WiFi.mode(ESPNOW_WIFI_MODE);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  while (!(WiFi.STA.started() || WiFi.AP.started())) {
    delay(10);
  }

  Serial.print("Controller MAC: ");
  Serial.println(ESPNOW_WIFI_MODE == WIFI_AP ? WiFi.softAPmacAddress() : WiFi.macAddress());

  // Create MacAddress object from the hard-coded peer MAC
  MacAddress peer_mac({
    peerMacAddress[0], peerMacAddress[1], peerMacAddress[2],
    peerMacAddress[3], peerMacAddress[4], peerMacAddress[5]
  });

  NowSerial = new ESP_NOW_Serial_Class(peer_mac, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IF);

  if (NowSerial->begin(115200)) {
    nowSerialActive = true;
    Serial.print("ESP-NOW Serial started. Peer: ");
    for (int i = 0; i < 6; i++) {
      if (peerMacAddress[i] < 16) Serial.print("0");
      Serial.print(peerMacAddress[i], HEX);
      if (i < 5) Serial.print(":");
    }
    Serial.println();
    return true;
  } else {
    Serial.println("ESP-NOW Serial failed to start");
    delete NowSerial;
    NowSerial = nullptr;
    nowSerialActive = false;
    return false;
  }
}
// ===========================================================

// IR sensor helper function
void Write_2bytes(byte d1, byte d2) {
  Wire.beginTransmission(slaveAddress);
  Wire.write(d1);
  Wire.write(d2);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(19200);  // USB serial for debug / local print
  delay(200);
  Serial.println();
  Serial.println("=== Controller: IMU+IR over ESP-NOW Serial ===");

  // ESP-NOW Serial
  initNowSerial();

  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize position buffer
  for (int i = 0; i < BUFFER_SIZE; i++) {
    xBuffer[i] = 0;
    yBuffer[i] = 0;
    timeBuffer[i] = 0;
  }

  // Initialize IMU-related pins
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

  // Initialize I2C with custom pins for IMU
  Wire.begin(3, 4);
  delay(100);

  // Initialize IR sensor
  slaveAddress = IRsensorAddress >> 1; // 0x21
  Write_2bytes(0x30,0x01); delay(10);
  Write_2bytes(0x30,0x08); delay(10);
  Write_2bytes(0x06,0x90); delay(10);
  Write_2bytes(0x08,0xC0); delay(10);
  Write_2bytes(0x1A,0x40); delay(10);
  Write_2bytes(0x33,0x33); delay(10);
  delay(100);

  Serial.println("Scanning for I2C devices...");
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.println(address, HEX);

      if (address == 0x6A || address == 0x6B) {
        Wire.beginTransmission(address);
        Wire.write(0x0F);
        Wire.endTransmission(false);
        Wire.requestFrom(address, 1);
        if (Wire.available()) {
          byte whoAmI = Wire.read();
          Serial.print("WHO_AM_I register value: 0x");
          Serial.println(whoAmI, HEX);
        }
      }
    }
  }

  Serial.println("Attempting to initialize LSM6DSOX...");
  if (!lsm6dsox.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX chip");
    Serial.println("Check your wiring!");
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSOX Found!");

  // Configure IMU settings
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_208_HZ);
  lsm6dsox.setGyroDataRate(LSM6DS_RATE_208_HZ);

  Serial.println("=== ARROW HIT DETECTOR + IR TRACKER ACTIVE ===");
  Serial.println("Thresholds: Soft=" + String(SOFT_HIT_THRESHOLD) + "x, Medium=" +
                 String(MEDIUM_HIT_THRESHOLD) + "x, Hard=" + String(HARD_HIT_THRESHOLD) + "x");
  Serial.println("Format: X1,Y1,X2,Y2,X3,Y3,X4,Y4,HitState,ButtonState");
  Serial.println("Position stabilization: Using 100ms delayed position during hits");
  
  delay(500);
}

// High-pass filter to remove gravity and slow movements
float highPassFilter(float input) {
  float output = HP_FILTER_ALPHA * (hpf_prev_output + input - hpf_prev_input);
  hpf_prev_input = input;
  hpf_prev_output = output;
  return output;
}

// Update baseline (moving average of signal magnitude)
void updateBaseline(float abs_value) {
  baseline = BASELINE_ALPHA * baseline + (1.0 - BASELINE_ALPHA) * abs_value;
  if (baseline < MIN_BASELINE) {
    baseline = MIN_BASELINE;
  }
}

// Classify hit intensity - returns 0=none, 1=soft, 2=medium, 3=hard
int classifyHit(float magnitude) {
  float ratio = magnitude / baseline;
  
  if (ratio >= HARD_HIT_THRESHOLD) {
    return 1; // HARD
  } else if (ratio >= MEDIUM_HIT_THRESHOLD) {
    return 1; // MEDIUM
  } else if (ratio >= SOFT_HIT_THRESHOLD) {
    return 1; // SOFT
  }
  return 0; // NONE
}

void detectHit(float filtered_x) {
  unsigned long currentTime = millis();
  float abs_filtered = abs(filtered_x);
  
  updateBaseline(abs_filtered);
  
  if (currentTime - last_hit_time < DEBOUNCE_TIME) {
    if (in_hit_window && abs_filtered > peak_value) {
      peak_value = abs_filtered;
    }
    return;
  }
  
  if (abs_filtered > baseline * SOFT_HIT_THRESHOLD && !in_hit_window) {
    in_hit_window = true;
    hit_window_start = currentTime;
    peak_value = abs_filtered;
  }
  
  if (in_hit_window) {
    if (abs_filtered > peak_value) {
      peak_value = abs_filtered;
    }
    
    if (currentTime - hit_window_start > HIT_WINDOW) {
      int hit_type = classifyHit(peak_value);
      
      if (hit_type > 0) {
        currentHitState = hit_type;
        hitStateSetTime = currentTime;
        last_hit_time = currentTime;
      }
      
      in_hit_window = false;
      peak_value = 0;
    }
  }
}

void sampleIMUData() {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6dsox.getEvent(&accel, &gyro, &temp);
  
  float filtered_x = highPassFilter(accel.acceleration.x);
  detectHit(filtered_x);
}

// Get position from buffer approximately POSITION_DELAY ms ago
void getDelayedPosition(int &outX, int &outY) {
  unsigned long currentTime = millis();
  unsigned long targetTime = currentTime - POSITION_DELAY;
  
  if (!bufferFilled) {
    return; // outX,outY already current
  }
  
  int closestIndex = bufferIndex;
  unsigned long closestDiff = 0xFFFFFFFF;
  
  for (int i = 0; i < BUFFER_SIZE; i++) {
    unsigned long diff;
    if (timeBuffer[i] <= targetTime) {
      diff = targetTime - timeBuffer[i];
    } else {
      diff = timeBuffer[i] - targetTime;
    }
    
    if (diff < closestDiff) {
      closestDiff = diff;
      closestIndex = i;
    }
  }
  
  outX = xBuffer[closestIndex];
  outY = yBuffer[closestIndex];
}

void readIRSensor() {
  // Toggle LED
  ledState = !ledState;
  digitalWrite(ledPin, ledState ? HIGH : LOW);
  
  // Read IR sensor
  Wire.beginTransmission(slaveAddress);
  Wire.write(0x36);
  Wire.endTransmission();
  Wire.requestFrom(slaveAddress, 16);
  
  for (i = 0; i < 16; i++) {
    data_buf[i] = 0;
  }
  
  i = 0;
  while (Wire.available() && i < 16) {
    data_buf[i] = Wire.read();
    i++;
  }
  
  // Parse IR data
  Ix[0] = data_buf[1];
  Iy[0] = data_buf[2];
  s = data_buf[3];
  Ix[0] += (s & 0x30) << 4;
  Iy[0] += (s & 0xC0) << 2;
  
  Ix[1] = data_buf[4];
  Iy[1] = data_buf[5];
  s = data_buf[6];
  Ix[1] += (s & 0x30) << 4;
  Iy[1] += (s & 0xC0) << 2;
  
  Ix[2] = data_buf[7];
  Iy[2] = data_buf[8];
  s = data_buf[9];
  Ix[2] += (s & 0x30) << 4;
  Iy[2] += (s & 0xC0) << 2;
  
  Ix[3] = data_buf[10];
  Iy[3] = data_buf[11];
  s = data_buf[12];
  Ix[3] += (s & 0x30) << 4;
  Iy[3] += (s & 0xC0) << 2;
  
  unsigned long currentTime = millis();
  
  // Store current X1, Y1 position in circular buffer
  xBuffer[bufferIndex] = Ix[0];
  yBuffer[bufferIndex] = Iy[0];
  timeBuffer[bufferIndex] = currentTime;
  bufferIndex++;
  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
    bufferFilled = true;
  }
  
  // Check if hit state should expire
  if (currentHitState > 0 && (currentTime - hitStateSetTime) > HIT_STATE_DURATION) {
    currentHitState = 0;
  }
  
  // If hit is active, use delayed position for X1, Y1
  if (currentHitState > 0) {
    getDelayedPosition(Ix[0], Iy[0]);
  }
  
  // Read button state
  int buttonState = (digitalRead(buttonPin) == LOW) ? 1 : 0;

  // ===== BUILD CSV PAYLOAD FOR RADIO (no padding) =====
  // Format: X1,Y1,X2,Y2,X3,Y3,X4,Y4,HitState,ButtonState
  String payload;
  payload.reserve(80);

  for (i = 0; i < 4; i++) {
    payload += String(int(Ix[i]));
    payload += ",";
    payload += String(int(Iy[i]));
    if (i < 3) payload += ",";
  }
  payload += ",";
  payload += String(currentHitState);
  payload += ",";
  payload += String(buttonState);
  // ===================================================

  // ===== ORIGINAL LOCAL SERIAL OUTPUT (with padding) =====
  /*for (i = 0; i < 4; i++) {
    if (Ix[i] < 1000) Serial.print(" ");
    if (Ix[i] < 100)  Serial.print(" ");
    if (Ix[i] < 10)   Serial.print(" ");
    Serial.print(int(Ix[i]));
    Serial.print(",");

    if (Iy[i] < 1000) Serial.print(" ");
    if (Iy[i] < 100)  Serial.print(" ");
    if (Iy[i] < 10)   Serial.print(" ");
    Serial.print(int(Iy[i]));

    if (i < 3) Serial.print(",");
  }*/
  
  //Serial.print(",");
  //Serial.print(currentHitState);
  //Serial.print(",");
  //Serial.println(buttonState);
  // =======================================================

  // ===== SEND THE SAME LINE OVER ESP-NOW SERIAL =====
  if (nowSerialActive && NowSerial != nullptr) {
    for (size_t idx = 0; idx < payload.length(); idx++) {
      NowSerial->write(payload.charAt(idx));
    }
    NowSerial->write('\n'); // end of line
  }
  // ==================================================
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    sampleIMUData();
    lastSampleTime = currentTime;
  }
  
  readIRSensor();
  
  delay(15); // Original IR sensor delay
}
