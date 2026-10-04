// config.h
// Zentrale Konfigurationsdatei — alle Pins, Konstanten und Pfade an einem Ort.
#pragma once

// ─── Display (DESPI-C02 via SPI) ────────────────────────────────────────────
#define PIN_EPD_MOSI    11
#define PIN_EPD_SCK     12
#define PIN_EPD_CS      10
#define PIN_EPD_DC       9  // LOW = Befehl, HIGH = Bilddaten
#define PIN_EPD_RST      8
#define PIN_EPD_BUSY     7  // HIGH solange das Display noch arbeitet

// ─── SD-Karte (SPI, shared bus) ─────────────────────────────────────────────
// MOSI + SCK werden mit dem Display geteilt, nur CS ist getrennt
#define PIN_SD_CS        6
#define PIN_SD_MISO     13  // Nur SD-Karte sendet Daten zurück, Display nicht

// ─── I2C (DS3231 RTC + MPU-9250, shared bus) ────────────────────────────────
// DS3231: Adresse 0x68 | MPU-9250: AD0 auf HIGH → Adresse 0x69 (kein Konflikt)
// GPIO 19/20 NICHT verwenden — das sind USB D-/D+ am ESP32-S3!
#define PIN_I2C_SDA      4
#define PIN_I2C_SCL      5

// ─── Schrittmotor 28BYJ-48 via ULN2003 ─────────────────────────────────────
// Direkt an Akkuspannung (3,7–4,2V) betreiben — kein Boost-Converter nötig
#define PIN_MOTOR_IN1   14
#define PIN_MOTOR_IN2   17
#define PIN_MOTOR_IN3   16
#define PIN_MOTOR_IN4   15

// Schrittmotor-Konfiguration (AccelStepper)
#define MOTOR_MAX_SPEED       500.0f   // Schritte/Sek — langsam für elegante Bewegung
#define MOTOR_ACCELERATION    200.0f   // Schritte/Sek² — sanftes Anfahren/Abbremsen
#define MOTOR_STEPS_DEFAULT  2048      // Fallback-Wert für 90° (überschrieben durch Kalibrierung)

// ─── Orientierungssensor ────────────────────────────────────────────────────
// Kugelschalter SW-520D in 45° Diagonale montiert
// Ändert Zustand bei jeder 90° Drehung des Rahmens
// RTC-GPIO → reagiert auf beide Flanken im Deep Sleep
#define PIN_BALL_SWITCH  1   // RTC-GPIO, EXT1 Wakeup

// ─── Sonstiges ──────────────────────────────────────────────────────────────
#define PIN_BATTERY_ADC  3   // Spannungsteiler 100kΩ/100kΩ → halbiert Akkuspannung (GPIO 19 = USB D-, nicht verwenden!)
#define PIN_WAKEUP_BTN   2   // EXT0 Wakeup aus Deep Sleep (WLAN-Taster)
#define PIN_LED_R       18   // RGB LED Rot   + 100Ω Vorwiderstand
#define PIN_LED_G       38   // RGB LED Grün  + 100Ω Vorwiderstand
#define PIN_LED_B       47   // RGB LED Blau  + 100Ω Vorwiderstand (GPIO 48 = onboard WS2812, nicht verwenden!)

// ─── Akku-Messung ───────────────────────────────────────────────────────────
#define BATTERY_DIVIDER_RATIO   2.0f    // 100kΩ/100kΩ → Messwert × 2 = echte Spannung
#define BATTERY_MAX_VOLTAGE     4.2f
#define BATTERY_MIN_VOLTAGE     2.5f
#define ADC_REF_VOLTAGE         3.3f
#define ADC_RESOLUTION          4095.0f // 12-Bit ADC

// ─── WLAN Access Point ──────────────────────────────────────────────────────
// ESP32 hostet eigenes Netz — erreichbar unter http://192.168.4.1
#define WIFI_AP_SSID      "Bilderrahmen"
#define WIFI_AP_PASSWORD  "12345678"        // min. 8 Zeichen für WPA2
#define WIFI_AP_CHANNEL   1
#define WIFI_AP_MAX_CONN  2
#define WIFI_TIMEOUT_MS   (5UL * 60 * 1000) // 5 Min, dann AP aus → Deep Sleep

// ─── Bildwechsel ────────────────────────────────────────────────────────────
#define IMAGE_CHANGE_HOUR      2  // Uhrzeit des ersten Wechsels (0–23)
#define IMAGE_CHANGES_PER_DAY  1  // Erlaubte Werte: 1, 2, 3, 4, 6, 8, 12, 24
                                  // 1 → alle 24h | 2 → alle 12h | 4 → alle 6h

// ─── Motorlauf & Orientierungswechsel ───────────────────────────────────────
#define BALL_SWITCH_SETTLE_MS  500   // Wartezeit nach Wakeup bis Kugel zur Ruhe kommt
#define MPU_ANGLE_THRESHOLD    85.0f // Grad — Motor stoppt wenn MPU diesen Winkel meldet
#define MOTOR_POWER_OFF_DELAY  200   // ms nach Motorstop bevor Pins auf LOW (Strom sparen)

// ─── SD-Karte: Dateipfade ───────────────────────────────────────────────────
#define SD_IMAGES_DIR   "/images"
#define SD_CONFIG_FILE  "/config.json"
#define SD_INDEX_HTML   "/index.html"

// ─── Display-Auflösung ──────────────────────────────────────────────────────
#define DISPLAY_WIDTH   800
#define DISPLAY_HEIGHT  480
