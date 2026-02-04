#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);

  WiFi.mode(WIFI_STA);   // 🔴 REQUIRED
  delay(100);

  Serial.print("ESP32 STA MAC: ");
  Serial.println(WiFi.macAddress());
}

void loop() {}
