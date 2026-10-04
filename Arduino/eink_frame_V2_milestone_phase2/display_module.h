// display_module.h
// E-Ink Display Modul – GDEP073E01 (Spectra 6, 800×480, 7 Farben) via GxEPD2
//
// Benötigte Bibliothek:
//   - GxEPD2  (ZinggJM/GxEPD2)
//
// Pinbelegung (aus config.h):
//   VSPI  MOSI=11  SCK=12  CS=10  DC=9  RST=8  BUSY=7
//
// Unterstützte BMP-Formate:
//   24-Bit, top-down (negativer Height-Wert) oder bottom-up
//   Landscape: 800×480  |  Portrait: 480×800

#pragma once
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <GxEPD2_7C.h>
#include <epd7c/GxEPD2_730c_GDEP073E01.h>
#include "esp_task_wdt.h"
#include "config.h"

// ─── Display-Instanz ────────────────────────────────────────────────────────
// Zweiter Template-Parameter: Zeilenzahl pro Seite (60 = ~24 KB Puffer)
GxEPD2_7C<GxEPD2_730c_GDEP073E01, GxEPD2_730c_GDEP073E01::HEIGHT / 8> display(
  GxEPD2_730c_GDEP073E01(PIN_EPD_CS, PIN_EPD_DC, PIN_EPD_RST, PIN_EPD_BUSY)
);

// ─── Spectra 6 Farbpalette ───────────────────────────────────────────────────
// RGB-Referenzwerte für die Abstandsberechnung + passender GxEPD2-Farbwert
struct PaletteEntry {
  uint8_t  r, g, b;
  uint16_t gfxColor;
};

static const PaletteEntry PALETTE[7] = {
  {   0,   0,   0, GxEPD_BLACK  },
  { 255, 255, 255, GxEPD_WHITE  },
  {   0, 180,  60, GxEPD_GREEN  },  // Spectra-Grün (kräftiger Olivton)
  {  30,  60, 180, GxEPD_BLUE   },  // Spectra-Blau
  { 200,  40,  30, GxEPD_RED    },  // Spectra-Rot
  { 220, 190,  40, GxEPD_YELLOW },  // Spectra-Gelb
  { 230, 100,  20, GxEPD_ORANGE },  // Spectra-Orange
};

// Nächste Palettenfarbe per minimaler euklidischer RGB-Distanz
static uint16_t nearestColor(uint8_t r, uint8_t g, uint8_t b) {
  int32_t bestDist = INT32_MAX;
  int     bestIdx  = 0;
  for (int i = 0; i < 7; i++) {
    int32_t dr = (int32_t)r - PALETTE[i].r;
    int32_t dg = (int32_t)g - PALETTE[i].g;
    int32_t db = (int32_t)b - PALETTE[i].b;
    int32_t d  = dr*dr + dg*dg + db*db;
    if (d < bestDist) { bestDist = d; bestIdx = i; }
  }
  return PALETTE[bestIdx].gfxColor;
}

// ─── Display initialisieren ──────────────────────────────────────────────────
void displayInit() {
  // VSPI-Bus: MOSI=11, SCK=12, MISO wird vom EPD nicht genutzt
  SPI.begin(PIN_EPD_SCK, /*MISO=*/-1, PIN_EPD_MOSI, PIN_EPD_CS);
  display.init(115200, /*initial=*/true, /*reset_duration_ms=*/2, /*pulldown_rst_mode=*/false);
  display.setRotation(0);
  Serial.println("[EPD] Display initialisiert");
}

// ─── BMP auf Display anzeigen ────────────────────────────────────────────────
// Landscape (800×480): Single-Pass — kein PSRAM-Puffer, SD und Display parallel
//   → Display startet ~3s früher, da BMP nicht komplett vorgeladen wird
// Portrait  (480×800): PSRAM-Puffer nötig (Koordinaten-Transformation braucht
//   zufälligen Zeilenzugriff)
//
// Seitenbreite: HEIGHT/8 = 60 Zeilen/Seite (8 Seiten für 480 Zeilen)
static const int16_t EPD_PAGE_H = GxEPD2_730c_GDEP073E01::HEIGHT / 8;

bool displayShowBMP(const char* path) {
  Serial.printf("[EPD] Lade: %s\n", path);

  // ── BMP-Header lesen ────────────────────────────────────────────────────
  File f = SD.open(path);
  if (!f) { Serial.printf("[EPD] Nicht gefunden: %s\n", path); return false; }

  uint8_t hdr[54];
  if (f.read(hdr, 54) < 54 || hdr[0] != 'B' || hdr[1] != 'M') {
    Serial.println("[EPD] Kein gültiges BMP");
    f.close(); return false;
  }

  uint32_t dataOffset = *(uint32_t*)&hdr[10];
  int32_t  bmpW       = *(int32_t* )&hdr[18];
  int32_t  bmpH       = *(int32_t* )&hdr[22];
  uint16_t bpp        = *(uint16_t*)&hdr[28];

  if (bpp != 24) {
    Serial.printf("[EPD] Nur 24-Bit BMP (ist %u bpp)\n", bpp);
    f.close(); return false;
  }

  int32_t w        = abs(bmpW);
  int32_t h        = abs(bmpH);
  bool    topDown  = (bmpH < 0);
  bool    portrait = (h > w);
  int32_t stride   = (w * 3 + 3) & ~3;

  Serial.printf("[EPD] %dx%d, %s, %s\n",
    w, h, topDown ? "top-down" : "bottom-up", portrait ? "Portrait" : "Landscape");

  display.setRotation(0);
  display.setFullWindow();

  if (!portrait) {
    // ── Landscape: In PSRAM laden, Datei schließen, dann rendern ────────────
    // Die Datei darf NICHT während des 30s Display-Refresh offen bleiben —
    // andere Requests würden dann keine File-Handles mehr bekommen.
    size_t   pixelBytes = (size_t)w * h * 3;
    uint8_t* pixels     = (uint8_t*)ps_malloc(pixelBytes);
    if (!pixels) { Serial.println("[EPD] PSRAM-Fehler (Landscape)"); f.close(); return false; }

    uint8_t* rowBuf = (uint8_t*)malloc(stride);
    if (!rowBuf) { free(pixels); f.close(); return false; }

    if (topDown) f.seek(dataOffset);
    for (int32_t row = 0; row < h; row++) {
      if (!topDown) f.seek(dataOffset + (uint32_t)(h - 1 - row) * stride);
      f.read(rowBuf, stride);
      uint8_t* dst = pixels + (size_t)row * w * 3;
      for (int32_t x = 0; x < w; x++) {
        dst[x*3+0] = rowBuf[x*3+2];  // R (BMP = BGR)
        dst[x*3+1] = rowBuf[x*3+1];  // G
        dst[x*3+2] = rowBuf[x*3+0];  // B
      }
    }
    free(rowBuf);
    f.close();  // ← Datei ZU bevor Display-Rendering startet
    Serial.printf("[EPD] %u Bytes geladen, Datei geschlossen\n", (unsigned)pixelBytes);

    display.firstPage();
    int pageNum = 0;
    do {
      int16_t startY = pageNum * EPD_PAGE_H;
      int16_t endY   = min((int16_t)(startY + EPD_PAGE_H), (int16_t)DISPLAY_HEIGHT);
      for (int16_t sy = startY; sy < endY; sy++) {
        uint8_t* row = pixels + (size_t)sy * w * 3;
        for (int16_t sx = 0; sx < w; sx++) {
          uint8_t* p = row + sx * 3;
          display.drawPixel(sx, sy, nearestColor(p[0], p[1], p[2]));
        }
        esp_task_wdt_reset();
      }
      pageNum++;
    } while (display.nextPage());

    free(pixels);

  } else {
    // ── Portrait: PSRAM-Puffer (Koordinaten-Transformation benötigt alle Zeilen) ──
    size_t   pixelBytes = (size_t)w * h * 3;
    uint8_t* pixels     = (uint8_t*)ps_malloc(pixelBytes);
    if (!pixels) { Serial.println("[EPD] PSRAM-Fehler"); f.close(); return false; }

    uint8_t* rowBuf = (uint8_t*)malloc(stride);
    if (!rowBuf) { free(pixels); f.close(); return false; }

    if (topDown) f.seek(dataOffset);
    for (int32_t row = 0; row < h; row++) {
      if (!topDown) f.seek(dataOffset + (uint32_t)(h - 1 - row) * stride);
      f.read(rowBuf, stride);
      uint8_t* dst = pixels + (size_t)row * w * 3;
      for (int32_t x = 0; x < w; x++) {
        dst[x*3+0] = rowBuf[x*3+2];  // R
        dst[x*3+1] = rowBuf[x*3+1];  // G
        dst[x*3+2] = rowBuf[x*3+0];  // B
      }
    }
    free(rowBuf);
    f.close();
    Serial.printf("[EPD] %u Bytes geladen\n", (unsigned)pixelBytes);

    display.firstPage();
    do {
      for (int16_t sy = 0; sy < DISPLAY_HEIGHT; sy++) {
        for (int16_t sx = 0; sx < DISPLAY_WIDTH; sx++) {
          // 90° CCW: Display(sx,sy) → BMP(sy, DISPLAY_WIDTH-1-sx)
          int32_t bx = sy;
          int32_t by = DISPLAY_WIDTH - 1 - sx;
          uint8_t* p = pixels + ((size_t)by * w + bx) * 3;
          display.drawPixel(sx, sy, nearestColor(p[0], p[1], p[2]));
        }
        if (sy % 48 == 0) esp_task_wdt_reset();
      }
    } while (display.nextPage());

    free(pixels);
  }

  Serial.println("[EPD] Anzeige fertig");
  return true;
}
