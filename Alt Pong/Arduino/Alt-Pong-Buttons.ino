#include "USB.h"
#include "USBHIDKeyboard.h"

// Debug mode - set to 1 to enable, 0 to disable
#define DEBUG_MODE 1

USBHIDKeyboard Keyboard;

// Configuration
const int NUM_BUTTONS = 5;
const int buttonPins[NUM_BUTTONS] = {8, A2, A1, A0, 9};
const char buttonKeys[NUM_BUTTONS] = {'r', 'y', 'w', 'b', 'g'};

// Button state tracking
bool buttonStates[NUM_BUTTONS] = {false};
bool lastButtonStates[NUM_BUTTONS] = {false};
unsigned long lastDebounceTime[NUM_BUTTONS] = {0};
unsigned long buttonPressTime[NUM_BUTTONS] = {0};

// Timing constants
const unsigned long DEBOUNCE_DELAY = 50;    // 50ms debounce
const unsigned long HOLD_THRESHOLD = 500;   // 500ms for hold detection

void setup() {
  #if DEBUG_MODE
  Serial.begin(115200);
  Serial.println("ESP32 Game Controller Ready!");
  Serial.println("Buttons mapped to keys: a, s, d, f, g");
  Serial.println("Debug mode enabled - will show button state changes");
  #endif
  
  // Initialize button pins with internal pullups
  for (int i = 0; i < NUM_BUTTONS; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  
  // Initialize USB HID
  Keyboard.begin();
  USB.begin();
}

void loop() {
  readButtons();
  handleHIDOutput();
  delay(1); // Small delay for stability
}

void readButtons() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    // Read current pin state (inverted because we're using pullup)
    bool reading = !digitalRead(buttonPins[i]);
    
    // Check if button state changed (for debouncing)
    if (reading != lastButtonStates[i]) {
      lastDebounceTime[i] = millis();
    }
    
    // If enough time has passed since last change, accept the reading
    if ((millis() - lastDebounceTime[i]) > DEBOUNCE_DELAY) {
      // If the button state has actually changed
      if (reading != buttonStates[i]) {
        buttonStates[i] = reading;
        
        // Record press time for hold detection
        if (buttonStates[i]) {
          buttonPressTime[i] = millis();
          #if DEBUG_MODE
          Serial.print("Button ");
          Serial.print(i + 1);
          Serial.print(" (");
          Serial.print(buttonKeys[i]);
          Serial.println(") PRESSED");
          printButtonStates();
          #endif
        } else {
          #if DEBUG_MODE
          Serial.print("Button ");
          Serial.print(i + 1);
          Serial.print(" (");
          Serial.print(buttonKeys[i]);
          Serial.println(") RELEASED");
          printButtonStates();
          #endif
        }
      }
    }
    
    lastButtonStates[i] = reading;
  }
}

void handleHIDOutput() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    static bool keyPressed[NUM_BUTTONS] = {false};
    
    if (buttonStates[i] && !keyPressed[i]) {
      // Button is pressed and key not yet sent
      Keyboard.press(buttonKeys[i]);
      keyPressed[i] = true;
      
    } else if (!buttonStates[i] && keyPressed[i]) {
      // Button released and key was pressed
      Keyboard.release(buttonKeys[i]);
      keyPressed[i] = false;
    }
  }
}

// Optional: Function to check if button is being held
bool isButtonHeld(int buttonIndex) {
  if (!buttonStates[buttonIndex]) return false;
  return (millis() - buttonPressTime[buttonIndex]) >= HOLD_THRESHOLD;
}

// Optional: Get hold duration for a button
unsigned long getHoldDuration(int buttonIndex) {
  if (!buttonStates[buttonIndex]) return 0;
  return millis() - buttonPressTime[buttonIndex];
}

// Optional: Check if any buttons are currently pressed
bool anyButtonPressed() {
  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (buttonStates[i]) return true;
  }
  return false;
}

// Optional: Get bitmask of all button states
uint8_t getButtonMask() {
  uint8_t mask = 0;
  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (buttonStates[i]) {
      mask |= (1 << i);
    }
  }
  return mask;
}

#if DEBUG_MODE
// Debug function to print current state of all buttons
void printButtonStates() {
  Serial.print("Button States: [");
  for (int i = 0; i < NUM_BUTTONS; i++) {
    Serial.print(buttonStates[i] ? "1" : "0");
    if (i < NUM_BUTTONS - 1) Serial.print(",");
  }
  Serial.print("] Keys: [");
  for (int i = 0; i < NUM_BUTTONS; i++) {
    if (buttonStates[i]) {
      Serial.print(buttonKeys[i]);
    } else {
      Serial.print("-");
    }
    if (i < NUM_BUTTONS - 1) Serial.print(",");
  }
  Serial.println("]");
}
#endif