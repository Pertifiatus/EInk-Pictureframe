// eink_frame_V10.ino
//
// Benötigte Bibliotheken:
//   - ESPAsyncWebServer  (mathieucarbou/ESPAsyncWebServer)
//   - AsyncTCP           (mathieucarbou/AsyncTCP)
//   - GxEPD2             (ZinggJM/GxEPD2)
//   - SD (Arduino built-in)
//   - RTClib             (Adafruit)

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h> 
#include <RTClib.h>
#include "esp_task_wdt.h"
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <math.h>
#include "config.h"
#include "display_module.h"

// ─── WebServer ────────────────────────────────────────────────────────────────
AsyncWebServer server(80);

// ─── RTC ──────────────────────────────────────────────────────────────────────
RTC_DS3231 rtc;

// ─── index.html im PSRAM ──────────────────────────────────────────────────────
static uint8_t* htmlBuffer = nullptr;
static size_t   htmlSize   = 0;

// ─── Image-Liste Cache im PSRAM ───────────────────────────────────────────────
static char*  g_imageListJson    = nullptr;
static size_t g_imageListJsonLen = 0;

void loadIndexHtml() {
  File f = SD.open("/index.html", FILE_READ);
  if (!f) { Serial.println("[!] index.html nicht gefunden"); return; }
  htmlSize   = f.size();
  htmlBuffer = (uint8_t*)ps_malloc(htmlSize);
  if (!htmlBuffer) { Serial.println("[!] Kein PSRAM für index.html"); f.close(); return; }
  f.read(htmlBuffer, htmlSize);
  f.close();
  Serial.printf("index.html: %u Bytes im PSRAM\n", (unsigned)htmlSize);
}

void buildImageListCache() {
  String json = "[";
  bool first = true;
  File dir = SD.open("/images");
  if (dir) {
    while (true) {
      File entry = dir.openNextFile();
      if (!entry) break;
      if (entry.isDirectory()) { entry.close(); continue; }
      String fname = entry.name();
      entry.close();
      if (!fname.endsWith(".bmp") && !fname.endsWith(".BMP")) continue;
      int slash = fname.lastIndexOf('/');
      if (slash >= 0) fname = fname.substring(slash + 1);

      char path[32];
      snprintf(path, sizeof(path), "/images/%s", fname.c_str());
      int32_t w = 0, h = 0;
      if (!getBmpSize(path, w, h)) continue;
      const char* orient = (abs(h) > abs(w)) ? "portrait" : "landscape";

      json += first ? "" : ",";
      json += "{\"name\":\"" + fname + "\",\"orientation\":\"" + String(orient) + "\"}";
      first = false;
    }
    dir.close();
  }
  json += "]";

  if (g_imageListJson) { free(g_imageListJson); g_imageListJson = nullptr; }
  g_imageListJsonLen = json.length();
  g_imageListJson = (char*)ps_malloc(g_imageListJsonLen + 1);
  if (g_imageListJson) {
    memcpy(g_imageListJson, json.c_str(), g_imageListJsonLen + 1);
    Serial.printf("[cache] Image-Liste: %u Bytes gecacht\n", (unsigned)g_imageListJsonLen);
  } else {
    Serial.println("[!] Kein PSRAM für Image-Liste");
  }
}

// ─── Upload-State ─────────────────────────────────────────────────────────────
static File uploadFile;
static int  uploadImageNum = -1;
static File thumbFile;

// ─── Globale State-Variablen (vor loadConfig, da dort verwendet) ──────────────
static volatile bool  g_displayPending = false;
static volatile char  g_displayPath[32] = "";
static volatile bool  g_displayBusy    = false;
static volatile char  g_displayPhase[12] = ""; // "rotating" | "rendering" | ""
static char           g_currentImage[32] = "";
static char           g_activePlId[64]   = "";   // ID der aktiven Playlist
static bool           g_shuffle          = false;
static uint8_t        g_wifiTimeoutMin   = 5;    // WLAN AP-Timeout in Minuten
static uint8_t        g_imgPerDay        = 1;    // Bildwechsel pro Tag (0 = Custom)
static uint8_t        g_startHour        = IMAGE_CHANGE_HOUR;  // Erste Wechsel-Uhrzeit
static char           g_customTimes[64]  = "";   // Custom-Modus: "HH:MM,HH:MM,..." (kommagetrennt)
static char           g_lastChange[20]   = "";   // Zeitpunkt letzter Bildwechsel "YYYY-MM-DD HH:MM"
static char           g_orientFilter[12] = "";   // "landscape", "portrait", "" = alle
static char           g_plBuf[8192]      = "";   // Puffer für POST /api/playlists (formatiertes JSON)
static size_t         g_plBufLen         = 0;
static char           g_libSettingsBuf[2048] = ""; // Puffer für POST /api/lib_settings
static size_t         g_libSettingsBufLen    = 0;
static volatile uint32_t g_lastActivityMs    = 0;    // Zeitstempel letzter Browser-Ping (für Inaktivitäts-Timeout)
static volatile bool     g_shutdownPending   = false; // POST /api/shutdown → sofort schlafen

// ─── RGB LED ──────────────────────────────────────────────────────────────────
// LEDC-PWM: 8 Bit, 5 kHz – reicht für flimmerfreies Dimmen der LED
#define LED_FREQ      5000
#define LED_RES       8      // 8 Bit → Werte 0–255

void ledInit() {
  ledcAttach(PIN_LED_R, LED_FREQ, LED_RES);
  ledcAttach(PIN_LED_G, LED_FREQ, LED_RES);
  ledcAttach(PIN_LED_B, LED_FREQ, LED_RES);
  ledcWrite(PIN_LED_R, 0);
  ledcWrite(PIN_LED_G, 0);
  ledcWrite(PIN_LED_B, 0);
}

// Setzt eine feste Farbe (0–255 je Kanal)
void ledSet(uint8_t r, uint8_t g, uint8_t b) {
  ledcWrite(PIN_LED_R, r);
  ledcWrite(PIN_LED_G, g);
  ledcWrite(PIN_LED_B, b);
}

void ledOff() { ledSet(0, 0, 0); }

// Zwei weiche rote Sinuspulse — signalisiert WLAN-AP deaktiviert
void ledDoubleRedPulse() {
  const uint32_t PULSE_MS  = 600;  // Dauer eines Pulses (Auf + Ab)
  const uint32_t PAUSE_MS  = 250;  // Pause zwischen den Pulsen
  const uint8_t  MAX_BRIGHT = 180; // Spitzenhelligkeit (gedämpftes Rot)

  for (int i = 0; i < 2; i++) {
    uint32_t start = millis();
    while (millis() - start < PULSE_MS) {
      float t      = (float)(millis() - start) / PULSE_MS * (float)M_PI;
      float bright = sinf(t);
      bright       = bright * bright; // Gamma → weicherer Abfall
      ledSet((uint8_t)(bright * MAX_BRIGHT), 0, 0);
      delay(16); // ~60 Hz
    }
    ledOff();
    if (i < 1) delay(PAUSE_MS);
  }
}

// Kurzes Aufblitzen einer Farbe (blockierend, nur beim Boot)
void ledFlash(uint8_t r, uint8_t g, uint8_t b, uint16_t ms = 300) {
  ledSet(r, g, b);
  delay(ms);
  ledOff();
}

// Blaues Pulsieren für AP-Modus — kurzer Puls, dann lange Pause (Stromsparen)
// Zyklus: 1,5 s Puls → 5,5 s aus → repeat (LED an nur ~21% der Zeit)
void ledBreathing() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < 20) return; // 50 Hz Update
  lastMs = now;

  const uint32_t CYCLE_MS = 7000; // Gesamtzyklus 7 s
  const uint32_t PULSE_MS = 1500; // davon 1,5 s Pulsierphase

  uint32_t pos = now % CYCLE_MS;
  if (pos >= PULSE_MS) { ledOff(); return; } // lange Pause → aus

  // Halbe Sinuskurve (0→max→0) innerhalb der Pulsierphase
  float t      = (float)pos / PULSE_MS * (float)M_PI;
  float bright = sinf(t);
  bright       = bright * bright; // Gamma-Korrektur → weicherer Abfall
  ledSet(0, 0, (uint8_t)(bright * 70.0f));
}

// ─── Konfig-Datei /config.json ────────────────────────────────────────────────
// Format: {"current_image":"/images/001.bmp","active_playlist":"12345"}

// Einfacher Bool-Extraktor für JSON-Boolean-Felder
static bool jsonExtractBool(const String& json, const char* key) {
  int ki = json.indexOf(key);
  if (ki < 0) return false;
  int colon = json.indexOf(':', ki + strlen(key));
  if (colon < 0) return false;
  int vs = colon + 1;
  while (vs < (int)json.length() && json[vs] == ' ') vs++;
  return json.substring(vs, vs + 4) == "true";
}

// Einfacher String-Extraktor für JSON-String-Felder (kein ArduinoJson nötig)
static bool jsonExtractStr(const String& json, const char* key, char* out, size_t outSize) {
  int ki = json.indexOf(key);
  if (ki < 0) return false;
  int colon = json.indexOf(':', ki + strlen(key));
  if (colon < 0) return false;
  int vs = colon + 1;
  while (vs < (int)json.length() && json[vs] == ' ') vs++;
  if (json[vs] == '"') {
    int s = vs + 1;
    int e = json.indexOf('"', s);
    if (e <= s || (e - s) >= (int)outSize) return false;
    json.substring(s, e).toCharArray(out, outSize);
  } else if (isdigit(json[vs])) {
    int s = vs, e = vs;
    while (e < (int)json.length() && isdigit(json[e])) e++;
    if ((e - s) <= 0 || (e - s) >= (int)outSize) return false;
    json.substring(s, e).toCharArray(out, outSize);
  } else {
    return false;  // null oder anderer Nicht-String-Wert
  }
  return true;
}

void loadConfig() {
  File f = SD.open("/config.json", FILE_READ);
  if (!f) return;
  String json = f.readString();
  f.close();
  if (jsonExtractStr(json, "\"current_image\"",  g_currentImage, sizeof(g_currentImage)))
    Serial.printf("[config] current_image: %s\n", g_currentImage);
  if (jsonExtractStr(json, "\"active_playlist\"", g_activePlId,   sizeof(g_activePlId)))
    Serial.printf("[config] active_playlist: %s\n", g_activePlId);
  g_shuffle = jsonExtractBool(json, "\"shuffle\"");
  Serial.printf("[config] shuffle: %s\n", g_shuffle ? "true" : "false");

  char tmp[8];
  if (jsonExtractStr(json, "\"wifi_timeout\"", tmp, sizeof(tmp))) g_wifiTimeoutMin = atoi(tmp);
  if (jsonExtractStr(json, "\"img_per_day\"",  tmp, sizeof(tmp))) g_imgPerDay      = atoi(tmp);
  if (jsonExtractStr(json, "\"start_hour\"",   tmp, sizeof(tmp))) g_startHour      = atoi(tmp);
  jsonExtractStr(json, "\"custom_times\"",  g_customTimes,  sizeof(g_customTimes));
  jsonExtractStr(json, "\"last_change\"",   g_lastChange,   sizeof(g_lastChange));
  jsonExtractStr(json, "\"orient_filter\"", g_orientFilter, sizeof(g_orientFilter));
  Serial.printf("[config] wifi_timeout: %u min, img_per_day: %u, start_hour: %u\n",
                g_wifiTimeoutMin, g_imgPerDay, g_startHour);
}

void saveConfig() {
  File f = SD.open("/config.json", FILE_WRITE);
  if (!f) { Serial.println("[config] Schreiben fehlgeschlagen"); return; }
  f.printf("{\n"
           "  \"current_image\": \"%s\",\n"
           "  \"active_playlist\": %s%s%s,\n"
           "  \"shuffle\": %s,\n"
           "  \"wifi_timeout\": %u,\n"
           "  \"img_per_day\": %u,\n"
           "  \"start_hour\": %u,\n"
           "  \"custom_times\": \"%s\",\n"
           "  \"last_change\": \"%s\",\n"
           "  \"orient_filter\": \"%s\"\n"
           "}\n",
           g_currentImage,
           g_activePlId[0] ? "\"" : "", g_activePlId[0] ? g_activePlId : "null", g_activePlId[0] ? "\"" : "",
           g_shuffle ? "true" : "false",
           g_wifiTimeoutMin, g_imgPerDay, g_startHour, g_customTimes, g_lastChange,
           g_orientFilter);
  f.close();
}

// Aktuellen RTC-Zeitstempel in g_lastChange schreiben ("YYYY-MM-DD HH:MM")
void stampLastChange() {
  if (rtc.lostPower()) return;
  DateTime now = rtc.now();
  snprintf(g_lastChange, sizeof(g_lastChange), "%04u-%02u-%02u %02u:%02u",
           now.year(), now.month(), now.day(), now.hour(), now.minute());
}

// ─── Deep Sleep ───────────────────────────────────────────────────────────────
// sleepSecs: Sekunden bis zum Timer-Wakeup (automatischer Bildwechsel)
// Wakeup-Quellen: Timer (Bildwechsel) + EXT0 GPIO 2 LOW (Taster → AP-Modus)
void goToDeepSleep(uint32_t sleepSecs) {
  ledOff(); // LED vor dem Schlafen ausschalten
  Serial.printf("[sleep] Schlafe %lu s (naechster Wechsel in %lu min)\n",
                sleepSecs, sleepSecs / 60);
  Serial.flush();

  WiFi.softAPdisconnect(true);
  delay(100);            // AP-Clients Zeit zum Trennen geben
  WiFi.mode(WIFI_OFF);
  delay(100);            // WiFi-Treiber vollständig herunterfahren lassen

  // Taster GPIO 2: RTC-Pullup aktivieren damit Pin definiert HIGH liegt
  rtc_gpio_pullup_en((gpio_num_t)PIN_WAKEUP_BTN);
  rtc_gpio_pulldown_dis((gpio_num_t)PIN_WAKEUP_BTN);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_WAKEUP_BTN, 0); // LOW = gedrückt

  // Timer für automatischen Bildwechsel
  esp_sleep_enable_timer_wakeup((uint64_t)sleepSecs * 1000000ULL);

  esp_deep_sleep_start(); // kehrt nicht zurück
}

// ─── Schlafzeit berechnen ─────────────────────────────────────────────────────
// Gibt Sekunden bis zum nächsten geplanten Bildwechsel zurück.
// Regular-Modus: g_imgPerDay gleichmäßige Slots ab g_startHour.
// Custom-Modus:  g_customTimes = "HH:MM,HH:MM,..." (kommagetrennt).
// Fallback bei defekter/nicht gesetzter RTC: 3600 s (1 Stunde).
uint32_t calcSleepSeconds() {
  if (rtc.lostPower()) {
    Serial.println("[sleep] RTC ungueltig – Fallback 1h");
    return 3600;
  }
  DateTime now = rtc.now();
  if (now.year() < 2024) {
    Serial.println("[sleep] RTC nicht gesetzt – Fallback 1h");
    return 3600;
  }

  uint32_t nowSecs = (uint32_t)now.hour() * 3600 + now.minute() * 60 + now.second();

  // ── Slots aufbauen ────────────────────────────────────────────────────────
  uint32_t slots[24];
  int slotCount = 0;

  if (g_imgPerDay == 0 && g_customTimes[0] != '\0') {
    // Custom-Modus: "HH:MM,HH:MM,..." parsen
    char buf[64];
    strncpy(buf, g_customTimes, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    char* tok = strtok(buf, ",");
    while (tok && slotCount < 24) {
      int h = 0, m = 0;
      if (sscanf(tok, "%d:%d", &h, &m) == 2 && h >= 0 && h < 24 && m >= 0 && m < 60)
        slots[slotCount++] = (uint32_t)h * 3600 + (uint32_t)m * 60;
      tok = strtok(nullptr, ",");
    }
  } else if (g_imgPerDay > 0) {
    // Regular-Modus: gleichmäßige Intervalle ab g_startHour
    uint32_t intervalSecs = 86400UL / g_imgPerDay;
    uint32_t startSecs    = (uint32_t)g_startHour * 3600;
    for (int i = 0; i < g_imgPerDay && slotCount < 24; i++)
      slots[slotCount++] = (startSecs + (uint32_t)i * intervalSecs) % 86400UL;
  }

  if (slotCount == 0) {
    Serial.println("[sleep] Keine Slots – Fallback 1h");
    return 3600;
  }

  // ── Nächsten Slot finden (mind. 60 s in der Zukunft) ─────────────────────
  uint32_t minDelta = 86400UL;
  for (int i = 0; i < slotCount; i++) {
    uint32_t delta = (slots[i] > nowSecs)
                     ? slots[i] - nowSecs
                     : 86400UL - nowSecs + slots[i];
    if (delta < 60) delta += 86400UL; // Slot gerade verpasst → nächsten Tag
    if (delta < minDelta) minDelta = delta;
  }

  Serial.printf("[sleep] Naechster Slot in %lu s (%lu min)\n", minDelta, minDelta / 60);
  return minDelta;
}

// ─── Nächstes Bild wählen ────────────────────────────────────────────────────
// Baut Bildliste aus aktiver Playlist (playlists.json) oder allen /images/*.bmp.
// Wählt sequenziell oder per Shuffle das nächste Bild (nicht das aktuelle).
// Schreibt vollen Pfad ("/images/001.bmp") nach outPath.
// Gibt false zurück wenn keine Bilder vorhanden.
#define MAX_IMG 64

bool pickNextImage(char* outPath, size_t outSize) {
  char names[MAX_IMG][12]; // "001.bmp\0" passt in 12 Bytes
  int  count = 0;

  // ── Bildliste aufbauen ───────────────────────────────────────────────────
  if (g_activePlId[0] != '\0') {
    // Aktive Playlist aus playlists.json laden
    File pf = SD.open("/playlists.json", FILE_READ);
    if (pf) {
      String js = pf.readString();
      pf.close();

      // Playlist-Eintrag mit passender ID finden: "id": ACTIVEPLID oder "id":ACTIVEPLID
      char idSearch[72];
      snprintf(idSearch, sizeof(idSearch), "\"id\":%s", g_activePlId);
      int plPos = js.indexOf(idSearch);
      if (plPos < 0) {
        // Kompaktes JSON nicht gefunden → Pretty-Print mit Leerzeichen versuchen
        snprintf(idSearch, sizeof(idSearch), "\"id\": %s", g_activePlId);
        plPos = js.indexOf(idSearch);
      }
      if (plPos >= 0) {
        int imPos    = js.indexOf("\"images\"", plPos);
        int arrStart = imPos >= 0 ? js.indexOf('[', imPos) : -1;
        int arrEnd   = arrStart >= 0 ? js.indexOf(']', arrStart) : -1;
        if (arrEnd > arrStart) {
          String arr = js.substring(arrStart + 1, arrEnd);
          int pos = 0;
          while (count < MAX_IMG) {
            int q1 = arr.indexOf('"', pos);
            if (q1 < 0) break;
            int q2 = arr.indexOf('"', q1 + 1);
            if (q2 < 0) break;
            String n = arr.substring(q1 + 1, q2);
            if (n.length() > 0 && n.length() < 12)
              n.toCharArray(names[count++], 12);
            pos = q2 + 1;
          }
        }
      }
    }
  }

  if (count == 0) {
    // Globale Reihenfolge aus playlists.json "order"-Feld lesen
    File pf = SD.open("/playlists.json", FILE_READ);
    if (pf) {
      String js = pf.readString();
      pf.close();
      int oPos     = js.indexOf("\"order\"");
      int arrStart = oPos >= 0 ? js.indexOf('[', oPos) : -1;
      int arrEnd   = arrStart >= 0 ? js.indexOf(']', arrStart) : -1;
      if (arrEnd > arrStart) {
        String arr = js.substring(arrStart + 1, arrEnd);
        int pos = 0;
        while (count < MAX_IMG) {
          int q1 = arr.indexOf('"', pos);
          if (q1 < 0) break;
          int q2 = arr.indexOf('"', q1 + 1);
          if (q2 < 0) break;
          String n = arr.substring(q1 + 1, q2);
          if (n.length() > 0 && n.length() < 12)
            n.toCharArray(names[count++], 12);
          pos = q2 + 1;
        }
      }
    }
  }

  if (count == 0) {
    // Fallback: alle .bmp-Dateien in /images/ (Dateisystemreihenfolge)
    File dir = SD.open("/images");
    if (dir) {
      while (count < MAX_IMG) {
        File f = dir.openNextFile();
        if (!f) break;
        if (!f.isDirectory()) {
          String n = String(f.name());
          int slash = n.lastIndexOf('/');
          if (slash >= 0) n = n.substring(slash + 1);
          if ((n.endsWith(".bmp") || n.endsWith(".BMP")) && n.length() < 12)
            n.toCharArray(names[count++], 12);
        }
        f.close();
      }
      dir.close();
    }
  }

  if (count == 0) { Serial.println("[pick] Keine Bilder gefunden"); return false; }

  // ── Orientierungsfilter anwenden ─────────────────────────────────────────
  if (g_orientFilter[0] != '\0') {
    bool wantPortrait = strcmp(g_orientFilter, "portrait") == 0;
    int filtered = 0;
    for (int i = 0; i < count; i++) {
      char path[32];
      snprintf(path, sizeof(path), "/images/%s", names[i]);
      int32_t fw = 0, fh = 0;
      if (!getBmpSize(path, fw, fh)) continue;
      if ((abs(fh) > abs(fw)) == wantPortrait) {
        if (filtered != i) memcpy(names[filtered], names[i], 12);
        filtered++;
      }
    }
    if (filtered > 0) {
      count = filtered;
      Serial.printf("[pick] Orientierungsfilter '%s': %d Bilder\n", g_orientFilter, count);
    } else {
      Serial.printf("[pick] Kein Bild fuer Filter '%s' – Filter ignoriert\n", g_orientFilter);
    }
  }

  if (count == 1) { snprintf(outPath, outSize, "/images/%s", names[0]); return true; }

  // ── Aktuellen Index finden ───────────────────────────────────────────────
  const char* curName = strrchr(g_currentImage, '/');
  curName = curName ? curName + 1 : g_currentImage;
  int curIdx = -1;
  for (int i = 0; i < count; i++) {
    if (strcmp(names[i], curName) == 0) { curIdx = i; break; }
  }

  // ── Nächstes Bild wählen ────────────────────────────────────────────────
  int nextIdx;
  if (g_shuffle) {
    // Zufällig, aktuelles Bild ausschließen
    nextIdx = random(count - 1);
    if (curIdx >= 0 && nextIdx >= curIdx) nextIdx++;
  } else {
    // Sequenziell (Wrap-around)
    nextIdx = (curIdx >= 0) ? (curIdx + 1) % count : 0;
  }

  snprintf(outPath, outSize, "/images/%s", names[nextIdx]);
  Serial.printf("[pick] %s → %s\n", curName[0] ? curName : "(leer)", names[nextIdx]);
  return true;
}

// ─── Display-Queue (Inter-Core-Kommunikation) ─────────────────────────────────
// WebServer (Core 1) schreibt → loop() (Core 0) liest und rendert
// (Variablen oben vor loadConfig deklariert)

// ─── Hilfsfunktionen ──────────────────────────────────────────────────────────

int nextImageNumber() {
  for (int i = 1; i <= 999; i++) {
    char path[24];
    snprintf(path, sizeof(path), "/images/%03d.bmp", i);
    File f = SD.open(path, FILE_READ);
    if (!f) return i;   // Datei existiert nicht → diese Nummer ist frei
    f.close();
  }
  return -1;
}

bool getBmpSize(const char* path, int32_t& w, int32_t& h) {
  File f = SD.open(path);
  if (!f) return false;
  uint8_t hdr[26];
  bool ok = (f.read(hdr, 26) == 26) && (hdr[0] == 'B') && (hdr[1] == 'M');
  if (ok) {
    w = *(int32_t*)&hdr[18];
    h = *(int32_t*)&hdr[22];
  }
  f.close();
  return ok;
}

// Exponential Moving Average über alle /api/status-Aufrufe hinweg.
// Erstmessung (< 0) initialisiert direkt; danach EMA mit α=0,15
// → Zeitkonstante ≈ 7 Aufrufe ≈ 35 s bei 5-s-Polling → glatte Anzeige.
static float g_batteryEma = -1.0f;
#define BATTERY_EMA_ALPHA 0.15f

float readBatteryVoltage() {
  // 64 Samples statt 16 → 4× mehr Rauschunterdrückung durch Mittelung
  uint32_t sum = 0;
  for (int i = 0; i < 64; i++) sum += analogReadMilliVolts(PIN_BATTERY_ADC);
  float v = (sum / 64.0f / 1000.0f) * BATTERY_DIVIDER_RATIO;

  // EMA: erste Messung direkt übernehmen, danach gleitend mitteln
  if (g_batteryEma < 0.0f) g_batteryEma = v;
  else                      g_batteryEma += BATTERY_EMA_ALPHA * (v - g_batteryEma);
  return g_batteryEma;
}

// Li-Ion Entladekurve: Spannung → Kapazität in Prozent
// Stützstellen empirisch für Standard-Li-Ion (z.B. 18650, LiPo)
float batteryVoltageToPercent(float v) {
  static const float volts[] = { 2.50f, 2.60f, 2.70f, 2.80f, 2.90f, 3.00f,
                                  3.10f, 3.20f, 3.30f, 3.40f, 3.50f,
                                  3.60f, 3.70f, 3.80f, 3.90f, 4.00f, 4.10f, 4.20f };
  static const float pcts[]  = {  0.0f,  0.0f,  0.0f,  0.0f,  1.0f,  2.0f,
                                   5.0f, 10.0f, 18.0f, 30.0f, 45.0f,
                                  60.0f, 72.0f, 82.0f, 90.0f, 95.0f, 98.0f,100.0f };
  const int N = sizeof(volts) / sizeof(volts[0]);

  if (v <= volts[0])     return 0.0f;
  if (v >= volts[N - 1]) return 100.0f;

  // Lineares Interpolieren zwischen den zwei umschließenden Stützstellen
  for (int i = 1; i < N; i++) {
    if (v <= volts[i]) {
      float t = (v - volts[i - 1]) / (volts[i] - volts[i - 1]);
      return pcts[i - 1] + t * (pcts[i] - pcts[i - 1]);
    }
  }
  return 100.0f;
}

// Setzt g_lastActivityMs bei jedem eingehenden Request → Inaktivitäts-Timer
// canHandle() gibt immer false zurück → leitet an den echten Handler weiter
class ActivityHandler : public AsyncWebHandler {
public:
  bool canHandle(AsyncWebServerRequest* req) const override {
    g_lastActivityMs = millis();
    return false;
  }
};

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1000);

  // LED + ADC initialisieren
  ledInit();
  analogSetPinAttenuation(PIN_BATTERY_ADC, ADC_11db); // voller 0–3,3 V Messbereich
  pinMode(PIN_WAKEUP_BTN, INPUT_PULLUP); // Taster: LOW = gedrückt

  // I2C (DS3231 RTC)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  if (!rtc.begin(&Wire)) Serial.println("[!] DS3231 nicht gefunden");
  else {
    if (rtc.lostPower()) Serial.println("[RTC] Batterie leer oder erste Inbetriebnahme – Zeit nicht gesetzt");
    else { DateTime now = rtc.now(); Serial.printf("[RTC] Zeit: %04d-%02d-%02d %02d:%02d:%02d\n", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second()); }
  }

  // SD-Karte (shared SPI-Bus mit Display, CS=PIN_SD_CS, MISO=PIN_SD_MISO)
  SPI.begin(PIN_EPD_SCK, PIN_SD_MISO, PIN_EPD_MOSI, PIN_EPD_CS);
  if (!SD.begin(PIN_SD_CS, SPI)) {
    Serial.println("[!] SD nicht gefunden");
    for (int i = 0; i < 6; i++) { ledSet(255, 0, 0); delay(200); ledOff(); delay(200); }
    return;
  }
  Serial.println("SD OK");
  SD.mkdir("/images");      // harmlos wenn schon vorhanden
  SD.mkdir("/thumbnails");
  loadConfig();

  // Playlist-Name zur aktiven ID nachschlagen (nur für Serial-Ausgabe)
  if (g_activePlId[0]) {
    File pf = SD.open("/playlists.json", FILE_READ);
    if (pf) {
      String js = pf.readString();
      pf.close();
      char plName[64] = "?";
      int idPos = js.indexOf("\"id\"");
      while (idPos >= 0) {
        int nc = js.indexOf(':', idPos + 4);
        if (nc < 0) break;
        int nv = nc + 1;
        while (nv < (int)js.length() && js[nv] == ' ') nv++;
        if (js.substring(nv, nv + strlen(g_activePlId)) == String(g_activePlId)) {
          int namePos = js.indexOf("\"name\"", nv);
          if (namePos >= 0) {
            int nc2 = js.indexOf(':', namePos + 6);
            if (nc2 >= 0) {
              int nv2 = nc2 + 1;
              while (nv2 < (int)js.length() && js[nv2] == ' ') nv2++;
              if (js[nv2] == '"') {
                int ns = nv2 + 1, ne = js.indexOf('"', ns);
                if (ne > ns && (ne - ns) < (int)sizeof(plName))
                  js.substring(ns, ne).toCharArray(plName, sizeof(plName));
              }
            }
          }
          break;
        }
        idPos = js.indexOf("\"id\"", idPos + 4);
      }
      Serial.printf("[config] active_playlist Name: \"%s\"\n", plName);
    }
  }

  // WDT-Timeout auf 60s erhöhen (E-Ink-Refresh ~30s + Puffer)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  esp_task_wdt_config_t wdt_config = { .timeout_ms = 60000, .idle_core_mask = 0, .trigger_panic = false };
  esp_task_wdt_reconfigure(&wdt_config);
#else
  esp_task_wdt_init(60, false);
#endif
  Serial.println("WDT-Timeout: 60s");

  // ── Wakeup-Ursache prüfen ────────────────────────────────────────────────
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    // Timer-Wakeup: automatischer Bildwechsel, kein WiFi/Webserver nötig
    Serial.println("[boot] Timer-Wakeup → automatischer Bildwechsel");
    displayInit();
    char nextPath[32];
    if (pickNextImage(nextPath, sizeof(nextPath))) {
      esp_task_wdt_add(NULL);
      bool ok = displayShowBMP(nextPath);
      esp_task_wdt_delete(NULL);
      if (ok) { strncpy(g_currentImage, nextPath, sizeof(g_currentImage)); stampLastChange(); saveConfig(); }
    } else {
      Serial.println("[boot] Keine Bilder – Sleep ohne Wechsel");
    }
    goToDeepSleep(calcSleepSeconds());
    return; // wird nie erreicht
  }

  if (cause == ESP_SLEEP_WAKEUP_EXT0) Serial.println("[boot] Taster-Wakeup → AP-Modus");
  else                                 Serial.println("[boot] Kaltstart → AP-Modus");

  ledFlash(180, 180, 180); // kurzes weißes Aufblitzen: Gerät gestartet

  // ── AP-Modus ─────────────────────────────────────────────────────────────
  loadIndexHtml();
  buildImageListCache();
  displayInit();

  // WiFi Access Point
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);
  Serial.printf("AP: %s  IP: %s\n", WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());

  server.addHandler(new ActivityHandler()); // Inaktivitäts-Timer bei jedem Request

  // ── API-Routen (müssen vor serveStatic registriert werden) ───────────────

  // GET /api/status
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    float v   = readBatteryVoltage();
    float pct = batteryVoltageToPercent(v);
    uint64_t total = SD.totalBytes();
    uint64_t free_ = total - SD.usedBytes();

    int count = 0;
    File dir = SD.open("/images");
    if (dir) {
      while (true) {
        File f = dir.openNextFile();
        if (!f) break;
        if (!f.isDirectory()) count++;
        f.close();   // jedes Handle explizit schließen → kein Descriptor-Leak
      }
      dir.close();
    }

    DateTime rtcNow = rtc.now();
    bool rtcOk = !rtc.lostPower() && rtcNow.year() >= 2024;

    char json[600];
    snprintf(json, sizeof(json),
      "{\"battery_pct\":%.1f,\"battery_v\":%.2f"
      ",\"sd_total_mb\":%.0f,\"sd_free_mb\":%.0f"
      ",\"image_count\":%d"
      ",\"display_busy\":%s"
      ",\"current_image\":\"%s\""
      ",\"rtc_ok\":%s"
      ",\"rtc_y\":%u,\"rtc_mo\":%u,\"rtc_d\":%u"
      ",\"rtc_h\":%u,\"rtc_m\":%u,\"rtc_s\":%u"
      ",\"shuffle\":%s"
      ",\"wifi_timeout\":%u"
      ",\"img_per_day\":%u"
      ",\"start_hour\":%u"
      ",\"custom_times\":\"%s\""
      ",\"last_change\":\"%s\""
      ",\"orient_filter\":\"%s\""
      ",\"display_phase\":\"%s\"}",
      pct, v,
      total / 1048576.0f, free_ / 1048576.0f,
      count,
      g_displayBusy ? "true" : "false",
      g_currentImage,
      rtcOk ? "true" : "false",
      rtcNow.year(), rtcNow.month(), rtcNow.day(),
      rtcNow.hour(), rtcNow.minute(), rtcNow.second(),
      g_shuffle ? "true" : "false",
      g_wifiTimeoutMin, g_imgPerDay, g_startHour, g_customTimes, g_lastChange, g_orientFilter,
      (const char*)g_displayPhase
    );
    req->send(200, "application/json", json);
  });

  // GET /api/images – aus PSRAM-Cache bedienen (kein SD-Scan)
  server.on("/api/images", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (g_imageListJson) {
      req->send(200, "application/json", g_imageListJson);
    } else {
      req->send(200, "application/json", "[]");
    }
  });

  // POST /api/upload/image  (rohe BMP-Bytes)
  server.on("/api/upload/image", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      if (uploadImageNum > 0) {
        Serial.printf("[upload] Bild gespeichert: /images/%03d.bmp\n", uploadImageNum);
        buildImageListCache();
        char json[32];
        snprintf(json, sizeof(json), "{\"name\":\"%03d\"}", uploadImageNum);
        req->send(200, "application/json", json);
      } else {
        Serial.println("[upload] Fehler: Bild konnte nicht gespeichert werden");
        req->send(500, "application/json", "{\"error\":\"upload failed\"}");
      }
      uploadImageNum = -1;
    },
    NULL,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) {
        if (uploadFile) uploadFile.close();  // vorherigen Handle schließen falls Upload abgebrochen
        uploadImageNum = nextImageNumber();
        if (uploadImageNum < 0) return;
        char path[24];
        snprintf(path, sizeof(path), "/images/%03d.bmp", uploadImageNum);
        uploadFile = SD.open(path, FILE_WRITE);
      }
      if (uploadFile) uploadFile.write(data, len);
      if ((index + len) >= total && uploadFile) uploadFile.close();
    }
  );

  // POST /api/upload/thumb?name=NNN  (JPEG-Bytes)
  server.on("/api/upload/thumb", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      req->send(200, "application/json", "{\"ok\":true}");
    },
    NULL,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) {
        if (thumbFile) thumbFile.close();  // vorherigen Handle schließen falls Upload abgebrochen
        String name = req->hasParam("name") ? req->getParam("name")->value() : "";
        if (name.length() > 0) {
          char path[32];
          snprintf(path, sizeof(path), "/thumbnails/%s.jpg", name.c_str());
          thumbFile = SD.open(path, FILE_WRITE);
        }
      }
      if (thumbFile) thumbFile.write(data, len);
      if ((index + len) >= total && thumbFile) thumbFile.close();
    }
  );

  // DELETE /api/image?name=001.bmp
  server.on("/api/image", HTTP_DELETE, [](AsyncWebServerRequest* req) {
    if (!req->hasParam("name")) { req->send(400); return; }
    String name = req->getParam("name")->value();
    String base = name.endsWith(".bmp") ? name.substring(0, name.length() - 4) : name;

    char imgPath[32], thumbPath[36];
    snprintf(imgPath,   sizeof(imgPath),   "/images/%s.bmp",     base.c_str());
    snprintf(thumbPath, sizeof(thumbPath), "/thumbnails/%s.jpg", base.c_str());

    bool ok = SD.remove(imgPath);    // false wenn nicht vorhanden, kein SD.exists() nötig
    SD.remove(thumbPath);            // ignoriert Fehler falls kein Thumbnail

    // Aktuelles Bild löschen → Anzeige zurücksetzen
    if (ok && strcmp(g_currentImage, imgPath) == 0) {
      g_currentImage[0] = '\0';
      Serial.printf("[delete] Aktuelles Bild gelöscht, Anzeige zurückgesetzt\n");
    }
    if (ok) buildImageListCache();
    Serial.printf("[delete] %s: %s\n", imgPath, ok ? "OK" : "fehlgeschlagen");

    req->send(ok ? 200 : 500, "application/json",
              ok ? "{\"ok\":true}" : "{\"error\":\"delete failed\"}");
  });

  // POST /api/display?name=001.bmp  → Bild auf E-Ink anzeigen
  server.on("/api/display", HTTP_POST, [](AsyncWebServerRequest* req) {
    if (!req->hasParam("name")) { req->send(400); return; }
    if (g_displayBusy) {
      req->send(503, "application/json", "{\"error\":\"display busy\"}");
      return;
    }
    String name = req->getParam("name")->value();
    char path[32];
    snprintf(path, sizeof(path), "/images/%s", name.c_str());
    // SD.exists() vermeiden → getBmpSize öffnet+schließt selbst
    int32_t chkW = 0, chkH = 0;
    if (!getBmpSize(path, chkW, chkH)) { Serial.printf("[display] Datei nicht gefunden: %s\n", path); req->send(404); return; }

    // Pfad in Queue schreiben → loop() übernimmt das Rendering
    strncpy((char*)g_displayPath, path, sizeof(g_displayPath) - 1);
    g_displayPending = true;
    Serial.printf("[display] Auftrag: %s (%dx%d)\n", path, chkW, abs(chkH));

    req->send(200, "application/json", "{\"ok\":true}");
  });

  // GET /  →  index.html aus PSRAM
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
    if (!htmlBuffer || htmlSize == 0) {
      req->send(503, "text/plain", "index.html nicht im Speicher");
      return;
    }
    AsyncWebServerResponse* resp = req->beginChunkedResponse("text/html",
      [](uint8_t* buf, size_t maxLen, size_t index) -> size_t {
        if (index >= htmlSize) return 0;
        size_t len = min(maxLen, htmlSize - index);
        memcpy(buf, htmlBuffer + index, len);
        return len;
      });
    resp->addHeader("Cache-Control", "max-age=3600");
    resp->addHeader("Connection", "close");
    req->send(resp);
  });

  // GET /api/playlists → liefert /playlists.json (oder leeres Objekt)
  server.on("/api/playlists", HTTP_GET, [](AsyncWebServerRequest* req) {
    File f = SD.open("/playlists.json", FILE_READ);
    if (!f) {
      req->send(200, "application/json", "{\"active\":null,\"playlists\":[]}");
      return;
    }
    String body = f.readString();
    f.close();
    req->send(200, "application/json", body);
  });

  // POST /api/playlists → speichert Playlists + aktive Playlist
  server.on("/api/playlists", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      if (g_plBufLen > 0) {
        // Komplette JSON-Datei speichern
        File f = SD.open("/playlists.json", FILE_WRITE);
        if (f) { f.write((uint8_t*)g_plBuf, g_plBufLen); f.close(); }

        // "active"-Feld extrahieren und in config.json speichern
        String js(g_plBuf, g_plBufLen);
        int ai = js.indexOf("\"active\"");
        if (ai >= 0) {
          int colon = js.indexOf(':', ai + 8);
          if (colon >= 0) {
            int v = colon + 1;
            while (v < (int)js.length() && js[v] == ' ') v++;
            if (js[v] == '"') {
              int s = v + 1, e = js.indexOf('"', s);
              if (e > s && (e - s) < (int)sizeof(g_activePlId))
                js.substring(s, e).toCharArray(g_activePlId, sizeof(g_activePlId));
            } else if (isdigit(js[v])) {
              int s = v, e = v;
              while (e < (int)js.length() && isdigit(js[e])) e++;
              if ((e - s) < (int)sizeof(g_activePlId))
                js.substring(s, e).toCharArray(g_activePlId, sizeof(g_activePlId));
            } else {
              g_activePlId[0] = '\0';  // null → keine aktive Playlist
            }
            // Playlist-Namen zur ID suchen — "id": <activePlId> im Array finden
            if (g_activePlId[0]) {
              char plName[64] = "?";
              int idPos = js.indexOf("\"id\"");
              while (idPos >= 0) {
                int nc = js.indexOf(':', idPos + 4);
                if (nc < 0) break;
                int nv = nc + 1;
                while (nv < (int)js.length() && js[nv] == ' ') nv++;
                if (js.substring(nv, nv + strlen(g_activePlId)) == String(g_activePlId)) {
                  int namePos = js.indexOf("\"name\"", nv);
                  if (namePos >= 0) {
                    int nc2 = js.indexOf(':', namePos + 6);
                    if (nc2 >= 0) {
                      int nv2 = nc2 + 1;
                      while (nv2 < (int)js.length() && js[nv2] == ' ') nv2++;
                      if (js[nv2] == '"') {
                        int ns = nv2 + 1, ne = js.indexOf('"', ns);
                        if (ne > ns && (ne - ns) < (int)sizeof(plName))
                          js.substring(ns, ne).toCharArray(plName, sizeof(plName));
                      }
                    }
                  }
                  break;
                }
                idPos = js.indexOf("\"id\"", idPos + 4);
              }
              Serial.printf("[playlist] Aktiv: \"%s\"\n", plName);
            } else {
              Serial.println("[playlist] Keine aktive Playlist");
            }
          }
        }
        saveConfig();
        g_plBufLen = 0;
      }
      req->send(200, "application/json", "{\"ok\":true}");
    },
    NULL,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) g_plBufLen = 0;
      size_t space = sizeof(g_plBuf) - 1 - g_plBufLen;
      size_t copy  = min(len, space);
      memcpy(g_plBuf + g_plBufLen, data, copy);
      g_plBufLen += copy;
      g_plBuf[g_plBufLen] = '\0';
    }
  );

  // POST /api/rtc?y=&mo=&d=&h=&m=&s=  → DS3231 lokale Zeit setzen (keine Zeitzone)
  server.on("/api/rtc", HTTP_POST, [](AsyncWebServerRequest* req) {
    if (!req->hasParam("y") || !req->hasParam("mo") || !req->hasParam("d") ||
        !req->hasParam("h") || !req->hasParam("m")) {
      req->send(400, "application/json", "{\"error\":\"params missing\"}"); return;
    }
    uint16_t y  = req->getParam("y")->value().toInt();
    uint8_t  mo = req->getParam("mo")->value().toInt();
    uint8_t  d  = req->getParam("d")->value().toInt();
    uint8_t  h  = req->getParam("h")->value().toInt();
    uint8_t  m  = req->getParam("m")->value().toInt();
    uint8_t  s  = req->hasParam("s") ? req->getParam("s")->value().toInt() : 0;
    rtc.adjust(DateTime(y, mo, d, h, m, s));
    delay(5);
    DateTime verify = rtc.now();
    Serial.printf("[RTC] Zeit: %04d-%02d-%02d %02d:%02d:%02d\n",
      verify.year(), verify.month(), verify.day(),
      verify.hour(), verify.minute(), verify.second());
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // POST /api/settings  → Einstellungen speichern
  // Parameter: shuffle=1|0, wifi_timeout=N, img_per_day=N, start_hour=N, custom_times=HH:MM,...
  server.on("/api/settings", HTTP_POST, [](AsyncWebServerRequest* req) {
    bool changed = false;
    if (req->hasParam("shuffle")) {
      g_shuffle = req->getParam("shuffle")->value() == "1";
      Serial.printf("[settings] shuffle: %s\n", g_shuffle ? "true" : "false");
      changed = true;
    }
    if (req->hasParam("wifi_timeout")) {
      g_wifiTimeoutMin = (uint8_t)constrain(req->getParam("wifi_timeout")->value().toInt(), 1, 60);
      Serial.printf("[settings] wifi_timeout: %u min\n", g_wifiTimeoutMin);
      changed = true;
    }
    if (req->hasParam("img_per_day")) {
      g_imgPerDay = (uint8_t)constrain(req->getParam("img_per_day")->value().toInt(), 0, 24);
      Serial.printf("[settings] img_per_day: %u\n", g_imgPerDay);
      changed = true;
    }
    if (req->hasParam("start_hour")) {
      g_startHour = (uint8_t)constrain(req->getParam("start_hour")->value().toInt(), 0, 23);
      Serial.printf("[settings] start_hour: %u\n", g_startHour);
      changed = true;
    }
    if (req->hasParam("orient_filter")) {
      String of = req->getParam("orient_filter")->value();
      of.toCharArray(g_orientFilter, sizeof(g_orientFilter));
      Serial.printf("[settings] orient_filter: %s\n", g_orientFilter);
      changed = true;
    }
    if (req->hasParam("custom_times")) {
      String ct = req->getParam("custom_times")->value();
      ct.toCharArray(g_customTimes, sizeof(g_customTimes));
      Serial.printf("[settings] custom_times: %s\n", g_customTimes);
      changed = true;
    }
    if (changed) saveConfig();
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // POST /api/shutdown → Antwort senden, dann sofort Deep Sleep
  server.on("/api/shutdown", HTTP_POST, [](AsyncWebServerRequest* req) {
    req->send(200, "application/json", "{\"ok\":true}");
    g_shutdownPending = true;
  });

  // Thumbnails: komplett in PSRAM laden, Datei schließen, DANN asynchron senden.
  // serveStatic würde die Datei für die gesamte Übertragung offen halten →
  // bei gleichzeitigen Galerie-Anfragen des Browsers = Descriptor-Erschöpfung.
  // Wildcard * funktioniert ohne ASYNCWEBSERVER_REGEX, req->url() liefert den vollen Pfad
  server.on("/thumbnails/*", HTTP_GET, [](AsyncWebServerRequest* req) {
    String url = req->url();  // z.B. "/thumbnails/001.jpg"
    if (url.indexOf("..") >= 0) { req->send(403); return; }
    char path[56];
    url.toCharArray(path, sizeof(path));

    File f = SD.open(path);
    if (!f || f.isDirectory()) { if (f) f.close(); req->send(404); return; }

    size_t sz = f.size();
    uint8_t* buf = (uint8_t*)ps_malloc(sz);
    if (!buf) { f.close(); req->send(503, "text/plain", "OOM"); return; }

    f.read(buf, sz);
    f.close();   // ← Datei ZU bevor Response gesendet wird

    AsyncWebServerResponse* resp = req->beginChunkedResponse("image/jpeg",
      [buf, sz](uint8_t* chunk, size_t maxLen, size_t index) -> size_t {
        if (index >= sz) { free(buf); return 0; }
        size_t len = min(maxLen, sz - index);
        memcpy(chunk, buf + index, len);
        return len;
      });
    resp->addHeader("Cache-Control", "max-age=3600");
    req->send(resp);
  });

  // GET /api/lib_settings → liefert /lib_settings.json (Bibliotheks-Einstellungen)
  server.on("/api/lib_settings", HTTP_GET, [](AsyncWebServerRequest* req) {
    File f = SD.open("/lib_settings.json", FILE_READ);
    if (!f) {
      req->send(200, "application/json", "{\"photo_tilt\":true,\"photo_size\":\"medium\",\"book_colors\":{},\"book_order\":[]}");
      return;
    }
    String body = f.readString();
    f.close();
    req->send(200, "application/json", body);
  });

  // POST /api/lib_settings → speichert /lib_settings.json
  server.on("/api/lib_settings", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      if (g_libSettingsBufLen > 0) {
        File f = SD.open("/lib_settings.json", FILE_WRITE);
        if (f) { f.write((uint8_t*)g_libSettingsBuf, g_libSettingsBufLen); f.close(); }
        g_libSettingsBufLen = 0;
      }
      req->send(200, "application/json", "{\"ok\":true}");
    },
    NULL,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) g_libSettingsBufLen = 0;
      size_t space = sizeof(g_libSettingsBuf) - 1 - g_libSettingsBufLen;
      size_t copy  = min(len, space);
      memcpy(g_libSettingsBuf + g_libSettingsBufLen, data, copy);
      g_libSettingsBufLen += copy;
      g_libSettingsBuf[g_libSettingsBufLen] = '\0';
    }
  );

  // GET /api/ping → Browser-Heartbeat, setzt Inaktivitäts-Timer zurück
  server.on("/api/ping", HTTP_GET, [](AsyncWebServerRequest* req) {
    g_lastActivityMs = millis();
    req->send(200, "application/json", "{\"ok\":true}");
  });

  server.serveStatic("/images/", SD, "/images/");

  server.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Not found");
  });

  server.begin();
  g_lastActivityMs = millis(); // Timer startet mit AP-Start
  Serial.println("Server laeuft auf http://192.168.4.1");
}

// ─── Loop ─────────────────────────────────────────────────────────────────────
// Läuft auf Core 0 — verarbeitet ausstehende Display-Anfragen aus der Queue
void loop() {
  if (g_displayPending && !g_displayBusy) {
    g_displayPending = false;
    g_displayBusy    = true;

    char path[32];
    strncpy(path, (const char*)g_displayPath, sizeof(path));
    Serial.printf("[loop] Display-Auftrag: %s\n", path);

    esp_task_wdt_add(NULL);

    // ── Bild anzeigen ────────────────────────────────────────────────────────
    strcpy((char*)g_displayPhase, "rendering");
    ledSet(255, 80, 0); // Orange: Display-Rendering
    bool ok = displayShowBMP(path);
    esp_task_wdt_delete(NULL);
    ledOff();

    if (ok) {
      strncpy(g_currentImage, path, sizeof(g_currentImage));
      stampLastChange();
      saveConfig();
    }
    g_displayBusy = false;
    strcpy((char*)g_displayPhase, "");
  }

  // ── Manueller Shutdown via /api/shutdown ─────────────────────────────────
  if (g_shutdownPending && !g_displayBusy) {
    Serial.println("[shutdown] Manueller Shutdown – gehe sofort schlafen");
    server.end();
    delay(100);
    goToDeepSleep(calcSleepSeconds());
  }

  // Blaues Pulsieren im AP-Modus (nur wenn kein Rendering läuft)
  if (!g_displayBusy) ledBreathing();

  // ── Taster: WLAN manuell abschalten ────────────────────────────────────────
  if (!g_displayBusy && digitalRead(PIN_WAKEUP_BTN) == LOW) {
    delay(50); // Entprellen
    if (digitalRead(PIN_WAKEUP_BTN) == LOW) {
      Serial.println("[AP] Taster gedrückt – WLAN wird deaktiviert");
      server.end();
      ledDoubleRedPulse();
      goToDeepSleep(calcSleepSeconds());
    }
  }

  // ── Inaktivitäts-Timeout: kein Browser-Ping → Deep Sleep ─────────────────
  // g_lastActivityMs einmal lesen (volatile) → lokale Variable, dann konsistent verwenden
  uint32_t _last = (uint32_t)g_lastActivityMs;
  if (!g_displayBusy && _last > 0) {
    uint32_t timeoutMs = (uint32_t)g_wifiTimeoutMin * 60000UL;
    uint32_t _now  = millis();
    uint32_t _diff = _now - _last;
    if (_diff >= timeoutMs) {
      Serial.printf("[AP] Inaktivitaet > %u min – gehe schlafen\n", g_wifiTimeoutMin);
      server.end();
      ledDoubleRedPulse();
      goToDeepSleep(calcSleepSeconds());
    }
  }
}
