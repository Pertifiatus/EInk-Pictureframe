// eink_frame_V11.ino
//
// Benötigte Bibliotheken:
//   - ESPAsyncWebServer  (mathieucarbou/ESPAsyncWebServer)
//   - AsyncTCP           (mathieucarbou/AsyncTCP)
//   - GxEPD2             (ZinggJM/GxEPD2)
//   - SD (Arduino built-in)
//   - RTClib             (Adafruit)
//   - AccelStepper       (Mike McCauley)

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
#include "motor_module.h"

// ─── WebServer ────────────────────────────────────────────────────────────────
AsyncWebServer server(80);

// ─── RTC ──────────────────────────────────────────────────────────────────────
RTC_DS3231 rtc;

// JSON-Body (Playlists, Bibliotheks-Einstellungen) direkt in eine Temp-Datei streamen:
// keine Größengrenze durch RAM-Puffer, und das Original wird erst ersetzt wenn alle
// Bytes geschrieben sind → nie eine abgeschnittene/kaputte JSON-Datei.
struct BodyFile { File f; bool ok = false; const char* tmp; const char* path; };
static BodyFile g_plBody  = { File(), false, "/playlists.tmp",    "/playlists.json"    };
static BodyFile g_libBody = { File(), false, "/lib_settings.tmp", "/lib_settings.json" };

static void bodyChunk(BodyFile& b, uint8_t* data, size_t len, size_t index, size_t total) {
  if (index == 0) {
    if (b.f) b.f.close();
    b.f  = SD.open(b.tmp, FILE_WRITE);
    b.ok = (bool)b.f;
  }
  if (b.f && b.f.write(data, len) != len) b.ok = false;  // SD voll / Schreibfehler
  if (index + len >= total && b.f) b.f.close();
}

static bool bodyCommit(BodyFile& b) {
  bool ok = b.ok;
  b.ok = false;
  if (!ok) { SD.remove(b.tmp); return false; }
  SD.remove(b.path);
  return SD.rename(b.tmp, b.path);
}

// Dateinamen aus Requests: nur einfache Namen, kein Pfad (verhindert "../config.json")
static bool safeName(const String& n) {
  return n.length() > 0 && n.length() < 20 &&
         n.indexOf('/') < 0 && n.indexOf('\\') < 0 && n.indexOf("..") < 0;
}


// ─── index.html im PSRAM ──────────────────────────────────────────────────────
static uint8_t* htmlBuffer = nullptr;
static size_t   htmlSize   = 0;

// ─── Image-Liste Cache im PSRAM ───────────────────────────────────────────────
// Beim AP-Start komplett aufgebaut, bei Upload/Löschen gezielt angepasst.
// /api/status liest nur diese Werte → das Status-Polling greift nie auf die SD zu.
#define MAX_IMG 999                       // Bildnummern 001..999
static char*    g_imageListJson    = nullptr;
static size_t   g_imageListJsonLen = 0;
static int      g_imageCount       = 0;
static float    g_sdTotalMb        = 0;
static float    g_sdFreeMb         = 0;
static bool     g_imgNumUsed[MAX_IMG + 1]; // belegte Nummern NNN.bmp → freie Nummer ohne SD-Suche

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
  g_imageCount = 0;
  memset(g_imgNumUsed, 0, sizeof(g_imgNumUsed));
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

      int num = atoi(fname.c_str());
      if (num > 0 && num <= MAX_IMG) g_imgNumUsed[num] = true;
      g_imageCount++;

      json += first ? "" : ",";
      json += "{\"name\":\"" + fname + "\",\"orientation\":\"" + String(orient) + "\"}";
      first = false;
    }
    dir.close();
  }
  json += "]";

  // usedBytes() durchsucht die ganze FAT → nur hier, nicht bei jedem Status-Poll
  uint64_t total = SD.totalBytes();
  g_sdTotalMb = total / 1048576.0f;
  g_sdFreeMb  = (total - SD.usedBytes()) / 1048576.0f;

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
static bool uploadOk       = false;
static bool uploadPortrait = false;
static size_t uploadBytes  = 0;
static const char* uploadErr = "";
static File thumbFile;

// Upload/Löschen: Cache gezielt anpassen statt alle Bilder neu zu scannen
// (Neuaufbau öffnet jede Datei + durchsucht die FAT → bei hunderten Bildern Sekunden pro Upload)
static void imageCacheAdd(int num, bool portrait, size_t bytes) {
  char entry[64];
  int n = snprintf(entry, sizeof(entry), "%s{\"name\":\"%03d.bmp\",\"orientation\":\"%s\"}]",
                   g_imageCount ? "," : "", num, portrait ? "portrait" : "landscape");
  char* p = g_imageListJson ? (char*)ps_realloc(g_imageListJson, g_imageListJsonLen + n + 1) : nullptr;
  if (!p) { buildImageListCache(); return; }
  memcpy(p + g_imageListJsonLen - 1, entry, n + 1);  // schließendes ']' überschreiben
  g_imageListJson    = p;
  g_imageListJsonLen += n - 1;
  g_imageCount++;
  g_imgNumUsed[num] = true;
  g_sdFreeMb -= bytes / 1048576.0f;
}

static void imageCacheRemove(const char* fname, size_t bytes) {
  char key[32];
  snprintf(key, sizeof(key), "{\"name\":\"%s\"", fname);
  char* s = g_imageListJson ? strstr(g_imageListJson, key) : nullptr;
  char* e = s ? strchr(s, '}') : nullptr;
  if (!e) { buildImageListCache(); return; }
  e++;
  if (*e == ',') e++;                         // folgendes Komma mitnehmen
  else if (s > g_imageListJson + 1) s--;      // letztes Element: vorheriges Komma mitnehmen
  memmove(s, e, strlen(e) + 1);
  g_imageListJsonLen = strlen(g_imageListJson);
  g_imageCount--;
  int num = atoi(fname);
  if (num > 0 && num <= MAX_IMG) g_imgNumUsed[num] = false;
  g_sdFreeMb += bytes / 1048576.0f;
}

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
static char           g_frameOrient[12]  = "";   // aktuelle Rahmen-Orientierung ("portrait"/"landscape"/"")
static int32_t        g_motorSteps90     = 0;    // kalibrierte Schritte für 90° (0 = noch nicht kalibriert)
static bool           g_autoRotate       = true; // Auto-Rotate Feature aktiv
static volatile uint32_t g_lastActivityMs    = 0;    // Zeitstempel letzter Browser-Ping (für Inaktivitäts-Timeout)
static volatile bool     g_motorCalibPending = false; // POST /api/motor/calibrate → loop() führt aus
static volatile bool     g_motorBusy         = false; // Motor läuft gerade
static volatile char     g_motorCalibPhase[32] = ""; // Aktueller Kalibrierungsschritt für /api/status
static volatile bool     g_motorMovePending  = false; // POST /api/motor/move → loop() führt aus
static volatile int32_t  g_motorMoveSteps    = 0;    // Schritte für manuellen Motorlauf
static volatile bool     g_shutdownPending   = false; // POST /api/shutdown → sofort schlafen
RTC_DATA_ATTR static bool g_lowBatIconShown  = false; // Blitz ist auf dem Display (überlebt Deep Sleep)

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

  // Onboard-WS2812 (GPIO 48) wird nicht verwendet, bleibt aber ohne Init
  // in einem undefinierten Zustand und leuchtet sonst dauerhaft weiß.
  // Einmalig ein "Aus"-Frame senden, um sie zuverlässig auszuschalten.
  neopixelWrite(48, 0, 0, 0);
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
  jsonExtractStr(json, "\"frame_orient\"",  g_frameOrient,  sizeof(g_frameOrient));
  if (json.indexOf("\"auto_rotate\"") >= 0)
    g_autoRotate = jsonExtractBool(json, "\"auto_rotate\"");
  char stepsStr[12] = "";
  if (jsonExtractStr(json, "\"motor_steps_90\"", stepsStr, sizeof(stepsStr)))
    g_motorSteps90 = atol(stepsStr);
  Serial.printf("[config] wifi_timeout: %u min, img_per_day: %u, start_hour: %u\n",
                g_wifiTimeoutMin, g_imgPerDay, g_startHour);
  Serial.printf("[config] frame_orient: %s, motor_steps_90: %ld\n",
                g_frameOrient[0] ? g_frameOrient : "(unbekannt)", (long)g_motorSteps90);
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
           "  \"orient_filter\": \"%s\",\n"
           "  \"frame_orient\": \"%s\",\n"
           "  \"auto_rotate\": %s,\n"
           "  \"motor_steps_90\": %ld\n"
           "}\n",
           g_currentImage,
           g_activePlId[0] ? "\"" : "", g_activePlId[0] ? g_activePlId : "null", g_activePlId[0] ? "\"" : "",
           g_shuffle ? "true" : "false",
           g_wifiTimeoutMin, g_imgPerDay, g_startHour, g_customTimes, g_lastChange,
           g_orientFilter, g_frameOrient, g_autoRotate ? "true" : "false", (long)g_motorSteps90);
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
  mpuSleep(); // MPU zieht sonst ~3,5 mA durch den gesamten Deep Sleep
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

// RTC-Sekunden seit Mitternacht, -1 wenn RTC ungültig / nicht gesetzt
static int32_t rtcDaySeconds() {
  if (rtc.lostPower()) { Serial.println("[sleep] RTC ungueltig"); return -1; }
  DateTime now = rtc.now();
  if (now.year() < 2024) { Serial.println("[sleep] RTC nicht gesetzt"); return -1; }
  return (int32_t)now.hour() * 3600 + now.minute() * 60 + now.second();
}

// Wechsel-Slots des Tages (Sekunden ab 0:00). Gibt Anzahl zurück.
static int buildSlots(uint32_t slots[24]) {
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

  return slotCount;
}

uint32_t calcSleepSeconds() {
  int32_t nowSecs = rtcDaySeconds();
  uint32_t slots[24];
  int slotCount = buildSlots(slots);
  if (nowSecs < 0 || slotCount == 0) {
    Serial.println("[sleep] Keine gueltige Zeit/Slots – Fallback 1h");
    return 3600;
  }

  // ── Nächsten Slot finden (mind. 60 s in der Zukunft) ─────────────────────
  uint32_t minDelta = 86400UL;
  for (int i = 0; i < slotCount; i++) {
    uint32_t delta = (slots[i] + 86400UL - (uint32_t)nowSecs) % 86400UL;
    if (delta < 60) delta += 86400UL; // Slot gerade verpasst / jetzt → nächsten Tag
    if (delta < minDelta) minDelta = delta;
  }

  Serial.printf("[sleep] Naechster Slot in %lu s (%lu min)\n", minDelta, minDelta / 60);
  return minDelta;
}

// Timer-Wakeup zu früh? Der Schlaf-Timer des ESP32 läuft auf einem RC-Oszillator mit
// einigen Prozent Abweichung (bei 24 h Schlaf bis ~1 h). Liegt der nächste Slot näher
// als der letzte, ist der Wechsel noch nicht dran.
// Gibt die Sekunden bis zu diesem Slot zurück, 0 = Wechsel ist fällig.
static uint32_t secondsUntilDueSlot() {
  int32_t nowSecs = rtcDaySeconds();
  uint32_t slots[24];
  int slotCount = buildSlots(slots);
  if (nowSecs < 0 || slotCount == 0) return 0;  // ohne gültige Zeit: wie bisher sofort wechseln

  uint32_t toNext = 86400UL, sincePrev = 86400UL;
  for (int i = 0; i < slotCount; i++) {
    uint32_t ahead  = (slots[i] + 86400UL - (uint32_t)nowSecs) % 86400UL;
    uint32_t behind = ((uint32_t)nowSecs + 86400UL - slots[i]) % 86400UL;
    if (ahead  < toNext)    toNext    = ahead;
    if (behind < sincePrev) sincePrev = behind;
  }
  return (toNext < sincePrev) ? toNext : 0;
}

// ─── Nächstes Bild wählen ────────────────────────────────────────────────────
// Bildliste: aktive Playlist → globale "order" aus playlists.json → alle /images/*.bmp.
// Wählt sequenziell oder per Shuffle das nächste Bild (nicht das aktuelle).
// Schreibt vollen Pfad ("/images/001.bmp") nach outPath.
// Gibt false zurück wenn keine Bilder vorhanden.
// Strings aus dem JSON-Array ab Position `from` (z.B. "images": [...]) nach names übernehmen
static int parseNameArray(const String& js, int from, char (*names)[12]) {
  int arrStart = from >= 0 ? js.indexOf('[', from) : -1;
  int arrEnd   = arrStart >= 0 ? js.indexOf(']', arrStart) : -1;
  int count = 0, pos = arrStart + 1;
  while (arrEnd > arrStart && count < MAX_IMG) {
    int q1 = js.indexOf('"', pos);
    if (q1 < 0 || q1 > arrEnd) break;
    int q2 = js.indexOf('"', q1 + 1);
    if (q2 < 0 || q2 > arrEnd) break;
    if (q2 - q1 - 1 > 0 && q2 - q1 - 1 < 12)
      js.substring(q1 + 1, q2).toCharArray(names[count++], 12);
    pos = q2 + 1;
  }
  return count;
}

static bool matchesOrientFilter(const char* name) {
  if (g_orientFilter[0] == '\0') return true;
  char path[32];
  snprintf(path, sizeof(path), "/images/%s", name);
  int32_t w = 0, h = 0;
  if (!getBmpSize(path, w, h)) return false;
  return (abs(h) > abs(w)) == (strcmp(g_orientFilter, "portrait") == 0);
}

bool pickNextImage(char* outPath, size_t outSize) {
  // 999 × 12 Bytes passen nicht auf den Stack → einmalig im PSRAM
  static char (*names)[12] = (char (*)[12])ps_malloc(MAX_IMG * 12);  // "001.bmp\0" passt in 12 Bytes
  if (!names) { Serial.println("[pick] Kein PSRAM"); return false; }
  int count = 0;

  // ── Bildliste aufbauen ───────────────────────────────────────────────────
  File pf = SD.open("/playlists.json", FILE_READ);
  if (pf) {
    String js = pf.readString();
    pf.close();
    if (g_activePlId[0] != '\0') {
      // Playlist-Eintrag mit passender ID: kompakt ("id":X) oder Pretty-Print ("id": X)
      char idSearch[72];
      snprintf(idSearch, sizeof(idSearch), "\"id\":%s", g_activePlId);
      int plPos = js.indexOf(idSearch);
      if (plPos < 0) {
        snprintf(idSearch, sizeof(idSearch), "\"id\": %s", g_activePlId);
        plPos = js.indexOf(idSearch);
      }
      if (plPos >= 0) count = parseNameArray(js, js.indexOf("\"images\"", plPos), names);
    }
    if (count == 0) count = parseNameArray(js, js.indexOf("\"order\""), names);
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

  // ── Aktuellen Index finden ───────────────────────────────────────────────
  const char* curName = strrchr(g_currentImage, '/');
  curName = curName ? curName + 1 : g_currentImage;
  int curIdx = -1;
  for (int i = 0; i < count; i++) {
    if (strcmp(names[i], curName) == 0) { curIdx = i; break; }
  }

  // ── Startpunkt: sequenziell nach dem aktuellen, bzw. zufällig ohne das aktuelle ──
  int start;
  if (g_shuffle && count > 1) {
    start = random(count - 1);
    if (curIdx >= 0 && start >= curIdx) start++;
  } else {
    start = (curIdx + 1) % count;  // curIdx = -1 → 0
  }

  // Ab Startpunkt das erste Bild, das zum Orientierungsfilter passt. Nur Kandidaten
  // werden geöffnet (statt aller Dateien) → schnell auch bei hunderten Bildern.
  int nextIdx = -1;
  for (int k = 0; k < count && nextIdx < 0; k++) {
    int i = (start + k) % count;
    if (i == curIdx && count > 1) continue;
    if (matchesOrientFilter(names[i])) nextIdx = i;
  }
  if (nextIdx < 0) {
    Serial.printf("[pick] Kein Bild fuer Filter '%s' – Filter ignoriert\n", g_orientFilter);
    nextIdx = start;
  }

  snprintf(outPath, outSize, "/images/%s", names[nextIdx]);
  Serial.printf("[pick] %s → %s (%d Bilder)\n", curName[0] ? curName : "(leer)", names[nextIdx], count);
  return true;
}
// ─── Hilfsfunktionen ──────────────────────────────────────────────────────────

// Erste freie Bildnummer aus dem Cache (statt bis zu 999 SD.open-Versuchen)
int nextImageNumber() {
  for (int i = 1; i <= MAX_IMG; i++)
    if (!g_imgNumUsed[i]) return i;
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

// ─── Rahmen drehen + Bild anzeigen ───────────────────────────────────────────
// Gemeinsam für Timer-Wakeup und Web-Auftrag. Kalibriert bei Bedarf zuerst.
// Schlägt Kalibrierung oder Drehung fehl, wird g_frameOrient geleert → der nächste
// Lauf kalibriert neu, statt mit einer falschen gespeicherten Lage weiterzuarbeiten.
static void ensureOrientation(bool imgIsLandscape) {
  if (!g_autoRotate) { Serial.println("[Motor] Auto-Rotate deaktiviert – kein Motorlauf"); return; }
  const char* target = imgIsLandscape ? "landscape" : "portrait";
  motorInit();

  if (g_motorSteps90 == 0 || g_frameOrient[0] == '\0') {
    Serial.println("[Motor] Nicht kalibriert / Lage unbekannt → Vollkalibrierung");
    bool endedPortrait = false;
    int32_t measured = motorCalibrateFull(g_motorCalibPhase, &endedPortrait);
    strcpy((char*)g_motorCalibPhase, "");
    if (measured <= 0) { Serial.println("[Motor] Kalibrierung fehlgeschlagen – kein Motorlauf"); return; }
    g_motorSteps90 = measured;
    strcpy(g_frameOrient, endedPortrait ? "portrait" : "landscape");
    saveConfig();
  }

  if (strcmp(g_frameOrient, target) == 0) {
    Serial.printf("[Motor] Orientierung unveraendert (%s) – kein Motorlauf\n", target);
    return;
  }
  ledSet(0, 120, 255); // Blau: Motor dreht
  bool ok = motorRotate(g_motorSteps90, imgIsLandscape);
  strcpy(g_frameOrient, ok ? target : "");
  saveConfig();
}

// Neustart-Grund ins Motor-Log: zeigt z.B. ob Watchdog oder Spannungseinbruch eine
// Kalibrierung abgebrochen haben. Log wird ab 100 KB neu begonnen.
static void logBootReason() {
  File f = SD.open("/motor_log.txt");
  bool tooBig = f && f.size() > 100000;
  if (f) f.close();
  if (tooBig) SD.remove("/motor_log.txt");

  const char* why;
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:   why = "Einschalten";               break;
    case ESP_RST_SW:        why = "Software-Neustart";         break;
    case ESP_RST_PANIC:     why = "ABSTURZ (Panic)";           break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:       why = "WATCHDOG";                  break;
    case ESP_RST_DEEPSLEEP: why = "Deep-Sleep-Wakeup";         break;
    case ESP_RST_BROWNOUT:  why = "SPANNUNGSEINBRUCH (Akku?)"; break;
    default:                why = "anderer";                   break;
  }
  DateTime now = rtc.now();
  mlog("\n=== Boot %04u-%02u-%02u %02u:%02u:%02u, Grund: %s ===\n",
       now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second(), why);
  mlogFlush();
}

static bool batteryLow() {
  float pct = batteryVoltageToPercent(readBatteryVoltage());
  Serial.printf("[Akku] %.0f %%\n", pct);
  return pct <= BATTERY_LOW_PCT;
}

// Wakeup ohne neues Bild: bei leerem Akku das aktuelle Bild einmal mit Blitz neu zeichnen.
// Ein Teil-Refresh geht beim Spectra 6 nicht: der Controller verliert im hibernate() seinen
// Bildspeicher, der Refresh treibt aber das ganze Panel → Rest des Bildes wird gelb.
// Verschwindet beim nächsten Bildwechsel mit vollem Akku.
static void lowBatteryCheck() {
  if (g_lowBatIconShown || !g_currentImage[0] || !batteryLow()) return;
  displayInit();
  esp_task_wdt_add(NULL);
  if (displayShowBMP(g_currentImage, true)) g_lowBatIconShown = true;
  esp_task_wdt_delete(NULL);
}

static bool rotateAndShow(const char* path) {
  bool low = batteryLow();  // vor dem Motorlauf messen (ohne Last)
  int32_t w = 0, h = 0;
  bool imgIsLandscape = getBmpSize(path, w, h) && abs(w) >= abs(h);

  esp_task_wdt_add(NULL); // WDT überwacht Motor + Display
  strcpy((char*)g_displayPhase, "rotating");
  ensureOrientation(imgIsLandscape);

  strcpy((char*)g_displayPhase, "rendering");
  ledSet(255, 80, 0); // Orange: Display-Rendering
  bool ok = displayShowBMP(path, low);
  esp_task_wdt_delete(NULL);
  ledOff();
  strcpy((char*)g_displayPhase, "");

  if (ok) {
    g_lowBatIconShown = low;
    strncpy(g_currentImage, path, sizeof(g_currentImage) - 1);
    stampLastChange();
    saveConfig();
  }
  return ok;
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_CDC_ON_BOOT && ARDUINO_USB_MODE
  // USB-CDC: ohne lesenden Monitor blockiert jedes printf bis zum Timeout →
  // Schrittmotor stockt während der Kalibrierlogs. 0 = nie blockieren.
  Serial.setTxTimeoutMs(0);
#endif
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
  logBootReason();

  // WDT-Timeout auf 210s erhöhen:
  // Motor 90° Schneckengetriebe ~82s + E-Ink-Refresh ~30s + Puffer = ~210s
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  esp_task_wdt_config_t wdt_config = { .timeout_ms = 210000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_reconfigure(&wdt_config);
#else
  esp_task_wdt_init(210, true);
#endif
  Serial.println("WDT-Timeout: 210s (Neustart bei Haenger)");

  // ── Wakeup-Ursache prüfen ────────────────────────────────────────────────
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_TIMER) {
    // Timer-Wakeup: automatischer Bildwechsel, kein WiFi/Webserver nötig
    Serial.println("[boot] Timer-Wakeup → automatischer Bildwechsel");

    // Zu früh aufgewacht (Timer-Drift)? → bis zum Slot weiterschlafen statt doppelt zu wechseln
    uint32_t early = secondsUntilDueSlot();
    if (early > 60) {
      Serial.printf("[boot] %lu s zu frueh aufgewacht – schlafe bis zum Slot\n", (unsigned long)early);
      lowBatteryCheck();
      goToDeepSleep(early);
    }
    if (early > 0) delay(early * 1000UL);  // < 1 min: kurz warten statt erneut schlafen

    displayInit();
    char nextPath[32];
    if (pickNextImage(nextPath, sizeof(nextPath))) rotateAndShow(nextPath);
    else { Serial.println("[boot] Keine Bilder – Sleep ohne Wechsel"); lowBatteryCheck(); }
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
  lowBatteryCheck();

  // WiFi Access Point
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);
  Serial.printf("AP: %s  IP: %s\n", WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());

  server.addHandler(new ActivityHandler()); // Inaktivitäts-Timer bei jedem Request

  // ── API-Routen (müssen vor serveStatic registriert werden) ───────────────

  // GET /api/status – nur Akku (ADC) + RTC werden live gelesen, SD-Werte kommen aus dem Cache
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    float v   = readBatteryVoltage();
    float pct = batteryVoltageToPercent(v);

    DateTime rtcNow = rtc.now();
    bool rtcOk = !rtc.lostPower() && rtcNow.year() >= 2024;

    char json[700];
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
      ",\"motor_steps\":%ld"
      ",\"motor_busy\":%s"
      ",\"frame_orient\":\"%s\""
      ",\"auto_rotate\":%s"
      ",\"display_phase\":\"%s\""
      ",\"calib_phase\":\"%s\"}",
      pct, v,
      g_sdTotalMb, g_sdFreeMb,
      g_imageCount,
      g_displayBusy ? "true" : "false",
      g_currentImage,
      rtcOk ? "true" : "false",
      rtcNow.year(), rtcNow.month(), rtcNow.day(),
      rtcNow.hour(), rtcNow.minute(), rtcNow.second(),
      g_shuffle ? "true" : "false",
      g_wifiTimeoutMin, g_imgPerDay, g_startHour, g_customTimes, g_lastChange, g_orientFilter,
      (long)g_motorSteps90,
      g_motorBusy ? "true" : "false",
      g_frameOrient,
      g_autoRotate ? "true" : "false",
      (const char*)g_displayPhase,
      (const char*)g_motorCalibPhase
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

  // POST /api/upload/image  (rohe BMP-Bytes, 24 Bit, 800×480 oder 480×800)
  server.on("/api/upload/image", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      char path[24];
      snprintf(path, sizeof(path), "/images/%03d.bmp", uploadImageNum);
      if (uploadImageNum > 0 && uploadOk) {
        Serial.printf("[upload] Bild gespeichert: %s\n", path);
        imageCacheAdd(uploadImageNum, uploadPortrait, uploadBytes);
        char json[32];
        snprintf(json, sizeof(json), "{\"name\":\"%03d\"}", uploadImageNum);
        req->send(200, "application/json", json);
      } else {
        if (uploadImageNum > 0) SD.remove(path);  // halbe Datei nicht liegen lassen
        Serial.printf("[upload] Fehler: %s\n", uploadErr);
        char json[80];
        snprintf(json, sizeof(json), "{\"error\":\"%s\"}", uploadErr);
        req->send(uploadImageNum > 0 ? 500 : 400, "application/json", json);
      }
      uploadImageNum = -1;
      uploadOk = false;
    },
    NULL,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) {
        if (uploadFile) uploadFile.close();  // vorherigen Handle schließen falls Upload abgebrochen
        uploadImageNum = -1;
        uploadOk       = false;
        uploadBytes    = total;
        // Header prüfen, bevor etwas auf die SD geschrieben wird
        int32_t w = 0, h = 0; uint16_t bpp = 0;
        if (len >= 30) { memcpy(&w, data + 18, 4); memcpy(&h, data + 22, 4); memcpy(&bpp, data + 28, 2); }
        h = abs(h);  // negativ = top-down
        if (len < 30 || data[0] != 'B' || data[1] != 'M' || bpp != 24 ||
            !((w == DISPLAY_WIDTH && h == DISPLAY_HEIGHT) || (w == DISPLAY_HEIGHT && h == DISPLAY_WIDTH))) {
          uploadErr = "kein 24-Bit BMP 800x480/480x800";
          return;
        }
        uploadPortrait = h > w;
        uploadImageNum = nextImageNumber();
        if (uploadImageNum < 0) { uploadErr = "max. 999 Bilder"; return; }
        char path[24];
        snprintf(path, sizeof(path), "/images/%03d.bmp", uploadImageNum);
        uploadFile = SD.open(path, FILE_WRITE);
        uploadOk   = (bool)uploadFile;
        uploadErr  = "SD-Fehler";
      }
      if (uploadFile && uploadFile.write(data, len) != len) { uploadOk = false; uploadErr = "SD voll / Schreibfehler"; }
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
        if (safeName(name)) {
          char path[40];
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
    if (!safeName(name)) { req->send(400); return; }
    String base = name.endsWith(".bmp") ? name.substring(0, name.length() - 4) : name;

    char imgPath[40], thumbPath[44];
    snprintf(imgPath,   sizeof(imgPath),   "/images/%s.bmp",     base.c_str());
    snprintf(thumbPath, sizeof(thumbPath), "/thumbnails/%s.jpg", base.c_str());

    File f = SD.open(imgPath);
    size_t bytes = f ? f.size() : 0;
    if (f) f.close();

    bool ok = SD.remove(imgPath);    // false wenn nicht vorhanden, kein SD.exists() nötig
    SD.remove(thumbPath);            // ignoriert Fehler falls kein Thumbnail

    // Aktuelles Bild löschen → Anzeige zurücksetzen
    if (ok && strcmp(g_currentImage, imgPath) == 0) {
      g_currentImage[0] = '\0';
      Serial.printf("[delete] Aktuelles Bild gelöscht, Anzeige zurückgesetzt\n");
    }
    if (ok) imageCacheRemove(strrchr(imgPath, '/') + 1, bytes);
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
    if (!safeName(name)) { req->send(400); return; }
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
      if (!bodyCommit(g_plBody)) {
        Serial.println("[playlist] Speichern fehlgeschlagen – alte Datei bleibt erhalten");
        req->send(500, "application/json", "{\"error\":\"save failed\"}");
        return;
      }
      // "active"-Feld in config.json übernehmen (String-ID, Zahl oder null)
      File f = SD.open("/playlists.json", FILE_READ);
      String js = f ? f.readString() : String();
      if (f) f.close();
      if (js.indexOf("\"active\"") >= 0 &&
          !jsonExtractStr(js, "\"active\"", g_activePlId, sizeof(g_activePlId)))
        g_activePlId[0] = '\0';  // null → keine aktive Playlist
      Serial.printf("[playlist] Gespeichert (%u Bytes), aktiv: %s\n",
                    (unsigned)js.length(), g_activePlId[0] ? g_activePlId : "(keine)");
      saveConfig();
      req->send(200, "application/json", "{\"ok\":true}");
    },
    NULL,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      bodyChunk(g_plBody, data, len, index, total);
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

  // POST /api/motor/calibrate → Vollkalibrierung anstoßen (läuft in loop())
  // Antwortet sofort; Fortschritt per /api/status pollen (motor_busy + motor_steps)
  server.on("/api/motor/calibrate", HTTP_POST, [](AsyncWebServerRequest* req) {
    if (g_motorBusy || g_displayBusy) {
      req->send(503, "application/json", "{\"error\":\"busy\"}");
      return;
    }
    g_motorCalibPending = true;
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // POST /api/motor/autorotate?enabled=1|0 → Auto-Rotate ein/ausschalten
  server.on("/api/motor/autorotate", HTTP_POST, [](AsyncWebServerRequest* req) {
    if (req->hasParam("enabled")) {
      g_autoRotate = req->getParam("enabled")->value() != "0";
      saveConfig();
    }
    req->send(200, "application/json", "{\"ok\":true}");
  });

  // POST /api/motor/move?steps=N&dir=1|-1 → manuelle Motorsteuerung (3D-Drucker-Jog)
  server.on("/api/motor/move", HTTP_POST, [](AsyncWebServerRequest* req) {
    if (g_motorBusy || g_displayBusy) {
      req->send(503, "application/json", "{\"error\":\"busy\"}");
      return;
    }
    if (!req->hasParam("steps") || !req->hasParam("dir")) {
      req->send(400, "application/json", "{\"error\":\"params missing\"}");
      return;
    }
    int32_t steps = req->getParam("steps")->value().toInt();
    int8_t  dir   = (int8_t)req->getParam("dir")->value().toInt();
    if (dir != 1 && dir != -1) dir = 1;
    if (steps <= 0 || steps > 200000) {
      req->send(400, "application/json", "{\"error\":\"steps out of range\"}");
      return;
    }
    g_motorMoveSteps  = (int32_t)dir * steps;
    g_motorMovePending = true;
    req->send(200, "application/json", "{\"ok\":true}");
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
      bool ok = bodyCommit(g_libBody);
      req->send(ok ? 200 : 500, "application/json", ok ? "{\"ok\":true}" : "{\"error\":\"save failed\"}");
    },
    NULL,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      bodyChunk(g_libBody, data, len, index, total);
    }
  );
  // GET /api/log → Motor-Log von der SD (Kalibrierung/Drehungen ohne USB nachlesen)
  // (kein mlogFlush hier: liefe parallel zum Motor-Loop → Stand = letzter Motorstopp)
  server.on("/api/log", HTTP_GET, [](AsyncWebServerRequest* req) {
    req->send(SD, "/motor_log.txt", "text/plain; charset=utf-8");
  });

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
  if (g_displayPending && !g_displayBusy && !g_motorBusy) {
    g_displayPending = false;
    g_displayBusy    = true;

    char path[32];
    strncpy(path, (const char*)g_displayPath, sizeof(path));
    Serial.printf("[loop] Display-Auftrag: %s\n", path);

    rotateAndShow(path);
    g_displayBusy = false;
  }

  // ── Motor-Kalibrierung via /api/motor/calibrate ──────────────────────────
  if (g_motorCalibPending && !g_motorBusy && !g_displayBusy) {
    g_motorCalibPending = false;
    g_motorBusy         = true;
    Serial.println("[loop] Motor-Kalibrierung gestartet");

    motorInit();
    esp_task_wdt_add(NULL);
    bool calibEndedPortrait = false;
    int32_t measured = motorCalibrateFull(g_motorCalibPhase, &calibEndedPortrait);
    esp_task_wdt_delete(NULL);
    strcpy((char*)g_motorCalibPhase, "");

    if (measured > 0) {
      g_motorSteps90 = measured;
      strcpy(g_frameOrient, calibEndedPortrait ? "portrait" : "landscape");
      saveConfig();
      Serial.printf("[loop] Kalibrierung OK: %ld Schritte, Endlage: %s\n",
                    (long)measured, calibEndedPortrait ? "portrait" : "landscape");
      ledFlash(0, 255, 0, 400); // Grün: Erfolg
    } else {
      Serial.println("[loop] Kalibrierung fehlgeschlagen");
      g_frameOrient[0] = '\0';  // Lage unbekannt → nächster Bildwechsel kalibriert neu
      saveConfig();
      ledFlash(255, 0, 0, 400); // Rot: Fehler
    }
    g_motorBusy = false;
  }

  // ── Manueller Motorlauf via /api/motor/move ───────────────────────────────
  if (g_motorMovePending && !g_motorBusy && !g_displayBusy) {
    g_motorMovePending = false;
    g_motorBusy        = true;
    motorInit();
    esp_task_wdt_add(NULL);
    motorManualMove((int32_t)g_motorMoveSteps);
    esp_task_wdt_delete(NULL);
    g_motorBusy = false;
  }

  // ── Manueller Shutdown via /api/shutdown ─────────────────────────────────
  if (g_shutdownPending && !g_displayBusy) {
    Serial.println("[shutdown] Manueller Shutdown – gehe sofort schlafen");
    server.end();
    delay(100);
    goToDeepSleep(calcSleepSeconds());
  }

  // Blaues Pulsieren im AP-Modus (nur wenn kein Rendering/Motor läuft)
  if (!g_displayBusy && !g_motorBusy) ledBreathing();

  // ── Taster: WLAN manuell abschalten ────────────────────────────────────────
  if (!g_displayBusy && !g_motorBusy && digitalRead(PIN_WAKEUP_BTN) == LOW) {
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
