#include "ESP32_NOW_Serial.h"
#include "MacAddress.h"
#include "WiFi.h"
#include "esp_wifi.h"

// Simple one-way receiver: it never sends anything back

#define ESPNOW_WIFI_MODE_STATION 1
#define ESPNOW_WIFI_CHANNEL 1

#if ESPNOW_WIFI_MODE_STATION
  #define ESPNOW_WIFI_MODE WIFI_STA
  #define ESPNOW_WIFI_IF   WIFI_IF_STA
#else
  #define ESPNOW_WIFI_MODE WIFI_AP
  #define ESPNOW_WIFI_IF   WIFI_IF_AP
#endif

ESP_NOW_Serial_Class* NowSerial = nullptr;
bool nowSerialActive = false;

bool initNowSerial() {
  Serial.println();
  Serial.println("Initializing ESP-NOW Serial (receiver)...");

  WiFi.mode(ESPNOW_WIFI_MODE);
  WiFi.setChannel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  while (!(WiFi.STA.started() || WiFi.AP.started())) {
    delay(10);
  }

  Serial.print("Receiver MAC: ");
  Serial.println(ESPNOW_WIFI_MODE == WIFI_AP ? WiFi.softAPmacAddress() : WiFi.macAddress());

  //Peer MAC (we don't send, only receive)
  MacAddress peer({
    0xB0, 0x81, 0x84,
    0x99, 0xC8, 0x14
  });

  NowSerial = new ESP_NOW_Serial_Class(peer, ESPNOW_WIFI_CHANNEL, ESPNOW_WIFI_IF);

  if (NowSerial->begin(115200)) {
    nowSerialActive = true;
    Serial.println("ESP-NOW Serial receiver ready.");
    return true;
  } else {
    Serial.println("ESP-NOW Serial failed to start");
    delete NowSerial;
    NowSerial = nullptr;
    nowSerialActive = false;
    return false;
  }
}

String receivedLine = "";

void setup() {
  Serial.begin(115200);  // USB serial to PC
  delay(500);
  Serial.println();
  Serial.println("=== ESP-NOW Serial Receiver ===");

  initNowSerial();
}

void loop() {
  if (!nowSerialActive || NowSerial == nullptr) {
    delay(10);
    return;
  }

  while (NowSerial->available()) {
    char c = NowSerial->read();

    if (c == '\n') {
      if (receivedLine.length() > 0) {
        // Print full line from controller
        Serial.println(receivedLine);
        receivedLine = "";
      }
    } else if (c != '\r') {
      receivedLine += c;
    }
  }

  delay(1);
}
