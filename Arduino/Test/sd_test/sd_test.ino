#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// Pins aus config.h
#define PIN_SD_CS    6
#define PIN_SD_MISO 13
#define PIN_SD_MOSI 11
#define PIN_SD_SCK  12

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n=== SD Karten Test ===");

  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  Serial.println("Initialisiere SD...");
  if (!SD.begin(PIN_SD_CS, SPI, 400000)) {  // 400 kHz statt ~4 MHz    Serial.println("[FEHLER] SD.begin() fehlgeschlagen!");
    Serial.println("  → Karte nicht eingelegt?");
    Serial.println("  → Slot-Kontakte defekt?");
    Serial.println("  → Lötung prüfen");
    return;
  }

  Serial.println("[OK] SD Karte erkannt!");

  // Kartentyp
  uint8_t cardType = SD.cardType();
  Serial.print("Kartentyp: ");
  switch (cardType) {
    case CARD_MMC:  Serial.println("MMC");   break;
    case CARD_SD:   Serial.println("SD");    break;
    case CARD_SDHC: Serial.println("SDHC");  break;
    default:        Serial.println("Unbekannt"); break;
  }

  // Kartengröße
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("Kartengröße: %llu MB\n", cardSize);
  Serial.printf("Gesamt: %llu MB  |  Frei: %llu MB\n",
    SD.totalBytes() / (1024 * 1024),
    (SD.totalBytes() - SD.usedBytes()) / (1024 * 1024));

  // Root-Verzeichnis auflisten
  Serial.println("\nDateien auf /:");
  File root = SD.open("/");
  File entry = root.openNextFile();
  int count = 0;
  while (entry) {
    Serial.printf("  %s  (%s, %lu Bytes)\n",
      entry.name(),
      entry.isDirectory() ? "DIR" : "FILE",
      entry.isDirectory() ? 0UL : entry.size());
    entry.close();
    entry = root.openNextFile();
    count++;
  }
  root.close();
  if (count == 0) Serial.println("  (leer)");

  Serial.println("\n=== Test abgeschlossen ===");
}

void loop() {}
