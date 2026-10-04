// eink_basic_test.ino
// Konzentrisches Rechteck-Muster – alle 6 Spectra-Farben
// GDEP073E01  800×480  Spectra 6
//
// Pins (ESP32-S3): MOSI=11  SCK=12  CS=10  DC=9  RST=8  BUSY=7  MISO=13

#include <Arduino.h>
#include <SPI.h>
#include <GxEPD2_7C.h>
#include <epd7c/GxEPD2_730c_GDEP073E01.h>

GxEPD2_7C<GxEPD2_730c_GDEP073E01, GxEPD2_730c_GDEP073E01::HEIGHT / 8> display(
  GxEPD2_730c_GDEP073E01(/*CS=*/10, /*DC=*/9, /*RST=*/8, /*BUSY=*/7)
);

// Spectra-6-Palette — kein Orange
static const uint16_t COLORS[6] = {
  GxEPD_GREEN,
  GxEPD_BLUE,
  GxEPD_RED,
  GxEPD_YELLOW,
  GxEPD_BLACK,
  GxEPD_WHITE,
};

void setup() {
  Serial.begin(115200);
  SPI.begin(12, 13, 11, 10);
  display.init(115200, true, 2, false);
  display.setRotation(0);
  drawPattern();
  display.hibernate();
}

void loop() {}

void drawPattern() {
  const int16_t W    = display.width();   // 800
  const int16_t H    = display.height();  // 480
  const int16_t RING = 40; // Bandbreite in Pixeln — 480/2/40 = 6 Ringe = genau 1 Farbzyklus

  display.setFullWindow();
  display.firstPage();
  do {
    // Von außen nach innen — jedes fillRect übermalt die Mitte des vorherigen
    for (int i = 0; i < 6; i++) {
      int16_t x = i * RING;
      int16_t y = i * RING;
      int16_t w = W - 2 * x;
      int16_t h = H - 2 * y;
      if (w <= 0 || h <= 0) break;
      display.fillRect(x, y, w, h, COLORS[i]);
    }
  } while (display.nextPage());
}
