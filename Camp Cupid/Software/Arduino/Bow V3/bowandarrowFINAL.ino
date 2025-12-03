#include <Adafruit_LSM6DSOX.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>

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

// Position buffer for shake compensation - NEW
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
// High-pass filter coefficient (0.0-1.0, higher = more filtering of slow movements)
const float HP_FILTER_ALPHA = 0.95;

// Detection thresholds (multiples of local baseline noise)
const float SOFT_HIT_THRESHOLD = 5.5;    // 5.5x baseline = soft hit
const float MEDIUM_HIT_THRESHOLD = 6.75;  // 6.75x baseline = medium hit
const float HARD_HIT_THRESHOLD = 8.0;    // 8x baseline = hard hit

// Baseline tracking (how quickly baseline adapts, 0.0-1.0)
const float BASELINE_ALPHA = 0.98;  // Slower = more stable baseline

// Debounce time (ms) - minimum time between detections
const unsigned long DEBOUNCE_TIME = 200;

// Minimum baseline value to prevent false triggers when stationary
const float MIN_BASELINE = 0.5;
// ============================================

// High-pass filter state
float hpf_prev_input = 0;
float hpf_prev_output = 0;

// Baseline tracking (exponential moving average of absolute filtered values)
float baseline = 1.0;

// Peak detection state
float peak_value = 0;
unsigned long last_hit_time = 0;
bool in_hit_window = false;
const unsigned long HIT_WINDOW = 50; // Time window to capture peak (ms)
unsigned long hit_window_start = 0;

// IR sensor helper function
void Write_2bytes(byte d1, byte d2) {
  Wire.beginTransmission(slaveAddress);
  Wire.write(d1);
  Wire.write(d2);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(19200);  // Changed to 19200 to match IR sensor code
  pinMode(ledPin, OUTPUT);

  // Initialize button with internal pullup
  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize position buffer - NEW
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
  slaveAddress = IRsensorAddress >> 1; // This results in 0x21 as the address to pass to TWI
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
      if (address < 16) {
        Serial.print("0");
      }
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

  // Configure IMU settings - higher sample rate for better impact detection
  lsm6dsox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);  // Increased range for impacts
  lsm6dsox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm6dsox.setAccelDataRate(LSM6DS_RATE_208_HZ);   // Higher rate
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
  // First-order IIR high-pass filter
  float output = HP_FILTER_ALPHA * (hpf_prev_output + input - hpf_prev_input);
  hpf_prev_input = input;
  hpf_prev_output = output;
  return output;
}

// Update baseline (moving average of signal magnitude)
void updateBaseline(float abs_value) {
  baseline = BASELINE_ALPHA * baseline + (1.0 - BASELINE_ALPHA) * abs_value;
  // Ensure minimum baseline to prevent false triggers
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
  
  // Update baseline with current signal level
  updateBaseline(abs_filtered);
  
  // Check if we're in debounce period
  if (currentTime - last_hit_time < DEBOUNCE_TIME) {
    // Still in debounce, but track peak if in hit window
    if (in_hit_window && abs_filtered > peak_value) {
      peak_value = abs_filtered;
    }
    return;
  }
  
  // Detect threshold crossing
  if (abs_filtered > baseline * SOFT_HIT_THRESHOLD && !in_hit_window) {
    // Start of potential hit
    in_hit_window = true;
    hit_window_start = currentTime;
    peak_value = abs_filtered;
  }
  
  // Track peak during hit window
  if (in_hit_window) {
    if (abs_filtered > peak_value) {
      peak_value = abs_filtered;
    }
    
    // Check if hit window has expired
    if (currentTime - hit_window_start > HIT_WINDOW) {
      // Window closed, evaluate the peak
      int hit_type = classifyHit(peak_value);
      
      if (hit_type > 0) {
        // Valid hit detected!
        currentHitState = hit_type;
        hitStateSetTime = currentTime;
        
        String hit_name[] = {"NONE", "SOFT", "MEDIUM", "HARD"};
        /*Serial.println();
        Serial.println("*** HIT DETECTED: " + hit_name[hit_type] + " ***");
        Serial.println("Peak: " + String(peak_value, 2) + " m/s² | Baseline: " + 
                       String(baseline, 2) + " m/s² | Ratio: " + String(peak_value/baseline, 1) + "x");
        Serial.println();
        */
        last_hit_time = currentTime;
      }
      
      // Reset for next detection
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
  
  // Apply high-pass filter to X acceleration
  float filtered_x = highPassFilter(accel.acceleration.x);
  
  // Detect hits
  detectHit(filtered_x);
}

// Get position from buffer approximately POSITION_DELAY ms ago - NEW
void getDelayedPosition(int &outX, int &outY) {
  unsigned long currentTime = millis();
  unsigned long targetTime = currentTime - POSITION_DELAY;
  
  // If buffer hasn't been filled yet, just use current position
  if (!bufferFilled) {
    return; // outX and outY already contain current position
  }
  
  // Search buffer for closest timestamp to targetTime
  int closestIndex = bufferIndex;
  unsigned long closestDiff = 0xFFFFFFFF; // Max unsigned long
  
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
  
  // Return the delayed position
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
  
  for (i=0; i<16; i++) {
    data_buf[i] = 0;
  }
  
  i = 0;
  while(Wire.available() && i < 16) {
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
  
  // Store current X1, Y1 position in circular buffer - NEW
  xBuffer[bufferIndex] = Ix[0];
  yBuffer[bufferIndex] = Iy[0];
  timeBuffer[bufferIndex] = currentTime;
  bufferIndex++;
  if (bufferIndex >= BUFFER_SIZE) {
    bufferIndex = 0;
    bufferFilled = true;  // Buffer has wrapped around at least once
  }
  
  // Check if hit state should expire
  if (currentHitState > 0 && (currentTime - hitStateSetTime) > HIT_STATE_DURATION) {
    currentHitState = 0; // Reset hit state after duration expires
  }
  
  // If hit is active, use delayed position for X1, Y1 - NEW
  if (currentHitState > 0) {
    getDelayedPosition(Ix[0], Iy[0]);
  }
  
  // Read button state
  // With INPUT_PULLUP: HIGH when open, LOW when pressed
  // We want: 0 when open, 1 when pressed, so invert the reading
  int buttonState = (digitalRead(buttonPin) == LOW) ? 1 : 0;
  
  // Output IR coordinates with hit state
  for(i=0; i<4; i++) {
    if (Ix[i] < 1000) Serial.print(" ");
    if (Ix[i] < 100) Serial.print(" ");
    if (Ix[i] < 10) Serial.print(" ");
    Serial.print(int(Ix[i]));
    Serial.print(",");
    
    if (Iy[i] < 1000) Serial.print(" ");
    if (Iy[i] < 100) Serial.print(" ");
    if (Iy[i] < 10) Serial.print(" ");
    Serial.print(int(Iy[i]));
    
    if (i < 3) Serial.print(",");
  }
  
  // Append hit state (0, 1, 2, or 3)
  Serial.print(",");
  Serial.print(currentHitState);
  
  // Append button state (0 or 1)
  Serial.print(",");
  Serial.println(buttonState);
}

void loop() {
  unsigned long currentTime = millis();

  // Sample IMU data at specified interval
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    sampleIMUData();
    lastSampleTime = currentTime;
  }
  
  // Read and output IR sensor data
  readIRSensor();
  
  delay(15); // Original IR sensor delay
}