// display_module.h
// E-Ink Display Modul – GDEP073E01 (Spectra 6, 800×480, 6 Farben) via GxEPD2
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

static const PaletteEntry PALETTE[] = {
  {   0,   0,   0, GxEPD_BLACK  },
  { 255, 255, 255, GxEPD_WHITE  },
  {   0, 180,  60, GxEPD_GREEN  },  // Spectra-Grün (kräftiger Olivton)
  {  30,  60, 180, GxEPD_BLUE   },  // Spectra-Blau
  { 200,  40,  30, GxEPD_RED    },  // Spectra-Rot
  { 220, 190,  40, GxEPD_YELLOW },  // Spectra-Gelb
};
static const int PALETTE_N = sizeof(PALETTE) / sizeof(PALETTE[0]);

// Nächste Palettenfarbe per minimaler euklidischer RGB-Distanz
static uint16_t nearestColor(uint8_t r, uint8_t g, uint8_t b) {
  int32_t bestDist = INT32_MAX;
  int     bestIdx  = 0;
  for (int i = 0; i < PALETTE_N; i++) {
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
  SPI.begin(PIN_EPD_SCK, PIN_SD_MISO, PIN_EPD_MOSI, PIN_EPD_CS);
  display.init(115200, /*initial=*/true, /*reset_duration_ms=*/2, /*pulldown_rst_mode=*/false);
  display.setRotation(0);
  Serial.println("[EPD] Display initialisiert");
}

// ─── BMP öffnen + Header prüfen ──────────────────────────────────────────────
// Nur 24-Bit in exakter Display-Größe: die Koordinaten-Transformationen lesen sonst
// außerhalb des Puffers. Bei Erfolg bleibt f offen.
struct BmpInfo { uint32_t dataOffset; int32_t w, h, stride; bool topDown, portrait; };

static bool openBmp(const char* path, File& f, BmpInfo& b) {
  f = SD.open(path);
  if (!f) { Serial.printf("[EPD] Nicht gefunden: %s\n", path); return false; }

  uint8_t hdr[54];
  int32_t bmpW, bmpH; uint16_t bpp;
  bool ok = f.read(hdr, 54) == 54 && hdr[0] == 'B' && hdr[1] == 'M';
  memcpy(&b.dataOffset, hdr + 10, 4);
  memcpy(&bmpW,         hdr + 18, 4);
  memcpy(&bmpH,         hdr + 22, 4);
  memcpy(&bpp,          hdr + 28, 2);

  b.w        = abs(bmpW);
  b.h        = abs(bmpH);
  b.topDown  = (bmpH < 0);
  b.portrait = (b.h > b.w);
  b.stride   = (b.w * 3 + 3) & ~3;

  ok = ok && bpp == 24 && (b.portrait ? (b.w == DISPLAY_HEIGHT && b.h == DISPLAY_WIDTH)
                                      : (b.w == DISPLAY_WIDTH  && b.h == DISPLAY_HEIGHT));
  if (!ok) {
    Serial.printf("[EPD] Nur 24-Bit BMP 800x480/480x800 (%s)\n", path);
    f.close();
  }
  return ok;
}

// Datei-Offset einer Bildzeile (Zeile 0 = oben)
static uint32_t bmpRowOffset(const BmpInfo& b, int32_t row) {
  return b.dataOffset + (uint32_t)(b.topDown ? row : b.h - 1 - row) * b.stride;
}

// ─── Akku-leer-Symbol ────────────────────────────────────────────────────────
// Roter Blitz direkt über dem Bild, unten rechts im BILD (nicht im Panel): die
// Rotation passend zur Bildlage setzen, dann liegen die Koordinaten wie im Bild.
//   Landscape-Bild wird 180° gedreht angezeigt → Rotation 2
//   Portrait-Bild wird 90° CW gedreht angezeigt  → Rotation 3
// Schwarzer Schatten (2 px versetzt) hält ihn auch auf roten Bildbereichen sichtbar.
#define BAT_ICON_W       48
#define BAT_ICON_H       72
#define BAT_ICON_MARGIN  16

static void setImageRotation(bool portrait) { display.setRotation(portrait ? 3 : 2); }

static void drawBolt(int16_t x, int16_t y, uint16_t color) {
  // Blitz-Polygon (30,4)(10,40)(22,40)(16,68)(38,30)(26,30) als 4 Dreiecke
  display.fillTriangle(x+30, y+4,  x+10, y+40, x+22, y+40, color);
  display.fillTriangle(x+30, y+4,  x+22, y+40, x+26, y+30, color);
  display.fillTriangle(x+26, y+30, x+22, y+40, x+38, y+30, color);
  display.fillTriangle(x+22, y+40, x+16, y+68, x+38, y+30, color);
}

static void drawLowBatteryIcon() {
  int16_t x = display.width()  - BAT_ICON_MARGIN - BAT_ICON_W;
  int16_t y = display.height() - BAT_ICON_MARGIN - BAT_ICON_H;
  drawBolt(x + 2, y + 2, GxEPD_BLACK);  // Schatten
  drawBolt(x, y, GxEPD_RED);
}

// ─── BMP auf Display anzeigen ────────────────────────────────────────────────
// BMP komplett in PSRAM laden und die Datei schließen, BEVOR der ~30s Refresh
// startet — sonst blockiert der offene Handle andere Requests.
// Landscape (800×480): 180° gedreht. Portrait (480×800): 90° CW gedreht.
// Gezeichnet werden pro Display-Seite nur deren Zeilen (GxEPD2-Paging).
//
// Seitenhöhe: HEIGHT/8 = 60 Zeilen/Seite (8 Seiten für 480 Zeilen)
static const int16_t EPD_PAGE_H = GxEPD2_730c_GDEP073E01::HEIGHT / 8;

// lowBattery: Akku-leer-Symbol gleich mit in den (ohnehin nötigen) vollen Refresh zeichnen
bool displayShowBMP(const char* path, bool lowBattery = false) {
  // Nach hibernate() muss das Display neu initialisiert werden (initial=false = kein Hard-Reset)
  display.init(115200, /*initial=*/false, /*reset_duration_ms=*/2, /*pulldown_rst_mode=*/false);
  display.setRotation(0);
  Serial.printf("[EPD] Lade: %s\n", path);

  File f; BmpInfo b;
  if (!openBmp(path, f, b)) return false;
  const int32_t w = b.w, h = b.h;
  const bool portrait = b.portrait;

  Serial.printf("[EPD] %dx%d, %s, %s\n",
    w, h, b.topDown ? "top-down" : "bottom-up", portrait ? "Portrait" : "Landscape");
  // ── In PSRAM laden (BGR → RGB, Zeile 0 = oben) ──────────────────────────
  size_t   pixelBytes = (size_t)w * h * 3;
  uint8_t* pixels     = (uint8_t*)ps_malloc(pixelBytes);
  if (!pixels) { Serial.println("[EPD] PSRAM-Fehler"); f.close(); return false; }

  uint8_t* rowBuf = (uint8_t*)malloc(b.stride);
  if (!rowBuf) { free(pixels); f.close(); return false; }

  for (int32_t row = 0; row < h; row++) {
    f.seek(bmpRowOffset(b, row));
    f.read(rowBuf, b.stride);
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

  // ── Rendern ─────────────────────────────────────────────────────────────
  display.setFullWindow();
  display.firstPage();
  int16_t page = 0;
  do {
    int16_t startY = page * EPD_PAGE_H;
    int16_t endY   = min((int16_t)(startY + EPD_PAGE_H), (int16_t)DISPLAY_HEIGHT);
    for (int16_t sy = startY; sy < endY; sy++) {
      for (int16_t sx = 0; sx < DISPLAY_WIDTH; sx++) {
        // Landscape 180°: BMP(w-1-sx, h-1-sy) | Portrait 90° CW: BMP(DISPLAY_HEIGHT-1-sy, sx)
        int32_t bx = portrait ? DISPLAY_HEIGHT - 1 - sy : w - 1 - sx;
        int32_t by = portrait ? sx                      : h - 1 - sy;
        uint8_t* p = pixels + ((size_t)by * w + bx) * 3;
        display.drawPixel(sx, sy, nearestColor(p[0], p[1], p[2]));
      }
      esp_task_wdt_reset();
    }
    if (lowBattery) { setImageRotation(portrait); drawLowBatteryIcon(); display.setRotation(0); }
    page++;
    esp_task_wdt_reset();  // WDT-Reset direkt vor nextPage() – blockiert während BUSY-Wartezeit
  } while (display.nextPage());

  free(pixels);

  esp_task_wdt_reset();
  display.hibernate();  // Wartet auf BUSY-Ende, fährt Display-Controller herunter
  Serial.println("[EPD] Anzeige fertig");
  return true;
}