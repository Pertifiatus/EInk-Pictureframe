#include <Wire.h>
#include "Arduino.h"

#define PIN_I2C_SDA 4
#define PIN_I2C_SCL 5

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(10000);
  Serial.println("I2C Scanner gestartet...");
}

void loop() {
  Serial.println("\n--- Scan ---");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.printf("Gerät gefunden: 0x%02X", addr);
      if (addr == 0x68) Serial.print(" → DS3231 RTC");
      if (addr == 0x69) Serial.print(" → MPU-9250");
      Serial.println();
      found++;
    }
  }
  if (found == 0) Serial.println("Keine Geräte gefunden.");
  else Serial.printf("%d Gerät(e) gefunden.\n", found);
  delay(3000);
}
