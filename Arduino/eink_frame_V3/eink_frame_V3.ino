// eink_frame_V2.ino – Phase 2: WebServer + SD + Gallery + Upload + E-Ink Display
//
// Benötigte Bibliotheken:
//   - ESPAsyncWebServer  (mathieucarbou/ESPAsyncWebServer)
//   - AsyncTCP           (mathieucarbou/AsyncTCP)
//   - GxEPD2             (ZinggJM/GxEPD2)
//   - SD (Arduino built-in)

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "esp_task_wdt.h"
#include "config.h"
#include "display_module.h"

// ─── SD-Karte (Test-Aufbau: separater HSPI) ──────────────────────────────────
#define SD_MOSI  39
#define SD_CLK   40
#define SD_MISO  41
#define SD_CS    42
static SPIClass sdSPI(HSPI);

// ─── WebServer ────────────────────────────────────────────────────────────────
AsyncWebServer server(80);

// ─── index.html im PSRAM ──────────────────────────────────────────────────────
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

// ─── Upload-State ─────────────────────────────────────────────────────────────
static File uploadFile;
static int  uploadImageNum = -1;
static File thumbFile;

// ─── Globale State-Variablen (vor loadConfig, da dort verwendet) ──────────────
static volatile bool  g_displayPending = false;
static volatile char  g_displayPath[32] = "";
static volatile bool  g_displayBusy    = false;
static char           g_currentImage[32] = "";
static char           g_activePlId[64]   = "";   // ID der aktiven Playlist
static char           g_plBuf[8192]      = "";   // Puffer für POST /api/playlists (formatiertes JSON)
static size_t         g_plBufLen         = 0;

// ─── Konfig-Datei /config.json ────────────────────────────────────────────────
// Format: {"current_image":"/images/001.bmp","active_playlist":"12345"}

// Einfacher String-Extraktor für JSON-String-Felder (kein ArduinoJson nötig)
static bool jsonExtractStr(const String& json, const char* key, char* out, size_t outSize) {
  int ki = json.indexOf(key);
  if (ki < 0) return false;
  int colon = json.indexOf(':', ki + strlen(key));
  if (colon < 0) return false;
  int vs = colon + 1;
  while (vs < (int)json.length() && json[vs] == ' ') vs++;
  if (json[vs] != '"') return false;  // null oder kein String → kein Treffer
  int s = vs + 1;
  int e = json.indexOf('"', s);
  if (e <= s || (e - s) >= (int)outSize) return false;
  json.substring(s, e).toCharArray(out, outSize);
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
}

void saveConfig() {
  File f = SD.open("/config.json", FILE_WRITE);
  if (!f) { Serial.println("[config] Schreiben fehlgeschlagen"); return; }
  if (g_activePlId[0])
    f.printf("{\n  \"current_image\": \"%s\",\n  \"active_playlist\": \"%s\"\n}\n",
             g_currentImage, g_activePlId);
  else
    f.printf("{\n  \"current_image\": \"%s\",\n  \"active_playlist\": null\n}\n",
             g_currentImage);
  f.close();
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

float readBatteryVoltage() {
  int raw = analogRead(PIN_BATTERY_ADC);
  return (raw / ADC_RESOLUTION) * ADC_REF_VOLTAGE * BATTERY_DIVIDER_RATIO;
}

// ─── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  // SD-Karte (HSPI Test-Pins)
  sdSPI.begin(SD_CLK, SD_MISO, SD_MOSI, SD_CS);
  if (!SD.begin(SD_CS, sdSPI)) {
    Serial.println("[!] SD nicht gefunden");
    return;
  }
  Serial.println("SD OK");
  loadIndexHtml();

  SD.mkdir("/images");      // harmlos wenn schon vorhanden
  SD.mkdir("/thumbnails");
  loadConfig();

  // E-Ink Display initialisieren
  displayInit();

  // WiFi Access Point
  WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD, WIFI_AP_CHANNEL, 0, WIFI_AP_MAX_CONN);
  Serial.printf("AP: %s  IP: %s\n", WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());

  // ── API-Routen (müssen vor serveStatic registriert werden) ───────────────

  // GET /api/status
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* req) {
    float v   = readBatteryVoltage();
    float pct = constrain(
      (v - BATTERY_MIN_VOLTAGE) / (BATTERY_MAX_VOLTAGE - BATTERY_MIN_VOLTAGE) * 100.0f,
      0.0f, 100.0f
    );
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

    char json[200];
    snprintf(json, sizeof(json),
      "{\"battery_pct\":%.1f,\"battery_v\":%.2f"
      ",\"sd_total_mb\":%.0f,\"sd_free_mb\":%.0f"
      ",\"image_count\":%d"
      ",\"display_busy\":%s"
      ",\"current_image\":\"%s\"}",
      pct, v,
      total / 1048576.0f, free_ / 1048576.0f,
      count,
      g_displayBusy ? "true" : "false",
      g_currentImage
    );
    req->send(200, "application/json", json);
  });

  // GET /api/images
  server.on("/api/images", HTTP_GET, [](AsyncWebServerRequest* req) {
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
        // Nur .bmp / .BMP
        if (!fname.endsWith(".bmp") && !fname.endsWith(".BMP")) continue;
        // SD gibt manchmal den vollen Pfad zurück – nur Dateiname behalten
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
    req->send(200, "application/json", json);
  });

  // POST /api/upload/image  (rohe BMP-Bytes)
  server.on("/api/upload/image", HTTP_POST,
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
    }

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
    if (!getBmpSize(path, chkW, chkH)) { req->send(404); return; }

    // Pfad in Queue schreiben → loop() übernimmt das Rendering
    strncpy((char*)g_displayPath, path, sizeof(g_displayPath) - 1);
    g_displayPending = true;

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
            } else {
              g_activePlId[0] = '\0';  // null → keine aktive Playlist
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

  server.serveStatic("/images/", SD, "/images/");

  server.onNotFound([](AsyncWebServerRequest* req) {
    req->send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("Server laeuft auf http://192.168.4.1");

  // WDT-Timeout auf 60s erhöhen (E-Ink-Refresh dauert ~30s)
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms     = 90000,
    .idle_core_mask = 0,
    .trigger_panic  = false,
  };
  esp_task_wdt_reconfigure(&wdt_config);
#else
  esp_task_wdt_init(90, false);
#endif
  Serial.println("WDT-Timeout: 90s");
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

    // Aktuellen Task beim WDT anmelden, damit esp_task_wdt_reset() funktioniert
    esp_task_wdt_add(NULL);
    bool ok = displayShowBMP(path);
    esp_task_wdt_delete(NULL);

    if (ok) {
      strncpy(g_currentImage, path, sizeof(g_currentImage));
      saveConfig();
    }
    g_displayBusy = false;
  }
}
