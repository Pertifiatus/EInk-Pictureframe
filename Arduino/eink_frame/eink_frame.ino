// eink_frame.ino – Phase 1: WiFi AP + WebServer + SD + Gallery + Upload
//
// Benötigte Bibliotheken:
//   - ESPAsyncWebServer  (me-no-dev/ESPAsyncWebServer)
//   - AsyncTCP           (me-no-dev/AsyncTCP)
//   - SD (Arduino built-in)

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "esp_task_wdt.h"
#include "config.h"

// ─── SD-Karte (Test-Aufbau, separater HSPI) ─────────────────────────────────
#define SD_MOSI  39
#define SD_CLK   40
#define SD_MISO  41
#define SD_CS    42
static SPIClass sdSPI(HSPI);

// ─── WebServer ───────────────────────────────────────────────────────────────
AsyncWebServer server(80);

// ─── index.html im PSRAM ─────────────────────────────────────────────────────
// Einmalig beim Start geladen → keine SD-Reads während des Servierens,
// damit der async_tcp-Task nie lange genug blockiert um den Watchdog auszulösen.
static uint8_t* htmlBuffer = nullptr;
static size_t   htmlSize   = 0;

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

// ─── Upload-State ────────────────────────────────────────────────────────────
static File uploadFile;
static int  uploadImageNum = -1;
static File thumbFile;

// ─── Hilfsfunktionen ─────────────────────────────────────────────────────────

// Nächste freie Bildnummer 001–999
int nextImageNumber() {
  for (int i = 1; i <= 999; i++) {
    char path[24];
    snprintf(path, sizeof(path), "/images/%03d.bmp", i);
    if (!SD.exists(path)) return i;
  }
  return -1;
}

// BMP-Header lesen → Breite/Höhe (für Orientierungserkennung)
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

// Akkuspannung messen
float readBatteryVoltage() {
  int raw = analogRead(PIN_BATTERY_ADC);
  return (raw / ADC_RESOLUTION) * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;
}

// ─── Setup ───────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  // SD-Karte initialisieren
  sdSPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSPI)) {
    Serial.println("[!] SD nicht gefunden");
    return;
  }
  Serial.println("SD OK");
  loadIndexHtml();

  // Ordner sicherstellen
  if (!SD.exists("/images"))     SD.mkdir("/images");
  if (!SD.exists("/thumbnails")) SD.mkdir("/thumbnails");

  // WiFi Access Point starten
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);
  Serial.printf("AP: %s  IP: %s\n", WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());

  // ── Routen ───────────────────────────────────────────────────────────────
  // Hinweis: API-Routen müssen VOR serveStatic registriert werden,
  // da der erste passende Handler gewinnt.

  // ── GET /api/status ────────────────────────────────────────────────────
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    float v   = readBatteryVoltage();
    float pct = constrain(
      (v - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100.0f,
      0.0f, 100.0f
    );
    uint64_t total = SD.totalBytes();
    uint64_t free_ = total - SD.usedBytes();

    // Bilder in /images zählen
    int count = 0;
    File dir = SD.open("/images");
    if (dir) {
      File f = dir.openNextFile();
      while (f) { if (!f.isDirectory()) count++; f = dir.openNextFile(); }
      dir.close();
    }

    char json[160];
    snprintf(json, sizeof(json),
      "{\"battery_pct\":%.1f,\"battery_v\":%.2f"
      ",\"sd_total_mb\":%.0f,\"sd_free_mb\":%.0f"
      ",\"image_count\":%d}",
      pct, v,
      total / 1048576.0f, free_ / 1048576.0f,
      count
    );
    req->send(200, "application/json", json);
  });

  // ── GET /api/images ────────────────────────────────────────────────────
  // Gibt alle Bilder in /images als JSON-Array zurück, sortiert nach Name
  server.on("/api/images", HTTP_GET, [](AsyncWebServerRequest* req) {
    String json = "[";
    bool first  = true;
    for (int i = 1; i <= 999; i++) {
      char path[24];
      snprintf(path, sizeof(path), "/images/%03d.bmp", i);
      if (!SD.exists(path)) continue;

      int32_t w = 0, h = 0;
      getBmpSize(path, w, h);
      const char* orient = (abs(h) > abs(w)) ? "portrait" : "landscape";

      char buf[72];
      snprintf(buf, sizeof(buf),
        "%s{\"name\":\"%03d.bmp\",\"orientation\":\"%s\"}",
        first ? "" : ",", i, orient);
      json += buf;
      first = false;
    }
    json += "]";
    req->send(200, "application/json", json);
  });

  // ── POST /api/upload/image ─────────────────────────────────────────────
  // Body: rohe BMP-Bytes (Content-Type: image/bmp)
  // Response: {"name":"001"}  → Basisname ohne Extension
  server.on("/api/upload/image", HTTP_POST,
    // Completion-Handler (nach vollständigem Empfang)
    [](AsyncWebServerRequest* req) {
      if (uploadImageNum > 0) {
        char json[32];
        snprintf(json, sizeof(json), "{\"name\":\"%03d\"}", uploadImageNum);
        req->send(200, "application/json", json);
      } else {
        req->send(500, "application/json", "{\"error\":\"upload failed\"}");
      }
      uploadImageNum = -1;
    },
    NULL,  // Multipart-File-Handler nicht genutzt
    // Body-Handler: chunkweise in Datei schreiben
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) {
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

  // ── POST /api/upload/thumb?name=NNN ───────────────────────────────────
  // Body: JPEG-Bytes (Content-Type: image/jpeg)
  // Query-Param name: Basisname (z.B. "001")
  server.on("/api/upload/thumb", HTTP_POST,
    [](AsyncWebServerRequest* req) {
      req->send(200, "application/json", "{\"ok\":true}");
    },
    NULL,
    [](AsyncWebServerRequest* req, uint8_t* data, size_t len, size_t index, size_t total) {
      if (index == 0) {
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

  // ── DELETE /api/image?name=001.bmp ────────────────────────────────────
  // Löscht Bild und zugehörigen Thumbnail von der SD-Karte
  server.on("/api/image", HTTP_DELETE, [](AsyncWebServerRequest* req) {
    if (!req->hasParam("name")) { req->send(400); return; }
    String name = req->getParam("name")->value();
    String base = name.endsWith(".bmp") ? name.substring(0, name.length() - 4) : name;

    char imgPath[32], thumbPath[36];
    snprintf(imgPath,   sizeof(imgPath),   "/images/%s.bmp",     base.c_str());
    snprintf(thumbPath, sizeof(thumbPath), "/thumbnails/%s.jpg", base.c_str());

    bool ok = true;
    if (SD.exists(imgPath))   ok = SD.remove(imgPath);
    if (SD.exists(thumbPath)) SD.remove(thumbPath);

    req->send(ok ? 200 : 500, "application/json",
              ok ? "{\"ok\":true}" : "{\"error\":\"delete failed\"}");
  });

  // index.html aus PSRAM ausliefern (kein SD-Read, kein Watchdog-Risiko)
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
    req->send(resp);
  });

  // Thumbnails + Bilder aus SD ausliefern
  server.serveStatic("/thumbnails/", SD, "/thumbnails/");
  server.serveStatic("/images/",     SD, "/images/");

  // 404 für alles andere
  server.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("Server laeuft auf http://192.168.4.1");

  // WDT-Timeout auf 30s erhöhen.
  // Der async_tcp-Task blockiert kurz bei SD-Reads (blocking SPI);
  // der Standard-Timeout von 5s ist dafür zu kurz.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // ESP32-Core 3.x / IDF 5.x
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms     = 30000,
    .idle_core_mask = 0,
    .trigger_panic  = false,
  };
  esp_task_wdt_reconfigure(&wdt_config);
#else
  // ESP32-Core 2.x / IDF 4.x
  esp_task_wdt_init(30, false);
#endif
  Serial.println("WDT-Timeout: 30s");
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
  // ESPAsyncWebServer arbeitet vollständig interrupt-basiert im Hintergrund.
  // Loop bleibt frei für spätere Display-/Motor-Logik (Phase 2+).
}
