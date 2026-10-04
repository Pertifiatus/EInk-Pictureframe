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

// Motor-Drehrichtung: true = invertiert (Portrait→Landscape dreht rückwärts)
#define MOTOR_DIR_INVERT  true

// Schrittmotor-Konfiguration (AccelStepper)
// Getriebe: Schneckenrad z=40, 1-gängige Schnecke → Übersetzung 40:1
// 28BYJ-48 HALF4WIRE: 4096 Schritte/Motorumdrehung × 40 = 163.840 Schritte/Ausgangsumdrehung
// 90° = 163.840 / 4 = 40.960 Schritte
#define MOTOR_MAX_SPEED       500.0f   // Schritte/Sek → 90° dauert ~82s
#define MOTOR_ACCELERATION    200.0f   // Schritte/Sek² — sanftes Anfahren/Abbremsen
#define MOTOR_STEPS_DEFAULT  40960     // Berechneter Wert für 90° (wird durch Kalibrierung bestätigt)
#define MOTOR_FINETUNE_SPEED  60.0f    // Schritte/Sek für IMU-Feinjustierung (sehr langsam)
#define MOTOR_FINETUNE_MAX   3000      // Max. Feinjustierungsschritte (~6.6°)

// ─── Orientierungssensor ────────────────────────────────────────────────────
// Kugelschalter SW-520D (45° Diagonale) ist verbaut, wird aber seit V11 nicht mehr genutzt:
// der MPU erkennt die Lage inkl. "auf dem Kopf" eindeutig. Pin bleibt frei (RTC-GPIO).
#define PIN_BALL_SWITCH  1

// ─── Sonstiges ──────────────────────────────────────────────────────────────
#define PIN_BATTERY_ADC  3   // Spannungsteiler 100kΩ/100kΩ → halbiert Akkuspannung (GPIO 19 = USB D-, nicht verwenden!)
#define PIN_WAKEUP_BTN   2   // EXT0 Wakeup aus Deep Sleep (WLAN-Taster)
#define PIN_LED_R       18   // RGB LED Rot   + 100Ω Vorwiderstand
#define PIN_LED_G       38   // RGB LED Grün  + 100Ω Vorwiderstand
#define PIN_LED_B       47   // RGB LED Blau  + 100Ω Vorwiderstand (GPIO 48 = onboard WS2812, nicht verwenden!)

// ─── Akku-Messung ───────────────────────────────────────────────────────────
#define BATTERY_DIVIDER_RATIO   2.0f    // 100kΩ/100kΩ → Messwert × 2 = echte Spannung
#define BATTERY_LOW_PCT         5.0f    // ≤ diesem Wert: roter Blitz unten rechts auf dem Bild

// ─── WLAN Access Point ──────────────────────────────────────────────────────
// ESP32 hostet eigenes Netz — erreichbar unter http://192.168.4.1
#define WIFI_AP_SSID      "Sibels-Erinnerungen"
#define WIFI_AP_PASSWORD  "PS250425"        // min. 8 Zeichen für WPA2
#define WIFI_AP_CHANNEL   1
#define WIFI_AP_MAX_CONN  2

// ─── Bildwechsel ────────────────────────────────────────────────────────────
#define IMAGE_CHANGE_HOUR      2  // Uhrzeit des ersten Wechsels (0–23)

// ─── Motorlauf & Orientierungswechsel ───────────────────────────────────────
#define MPU_OFFSET             0.20f // Grad — gemessene Schräglage des IMU-PCBs (Portrait-Rohwert ~0.19°)
// Achsen-Vorzeichen für den vorzeichenbehafteten Winkel (unterscheidet aufrecht / auf dem Kopf).
// Prüfen über die Logzeile "[MPU] ax=.. ay=..":
//   Rahmen AUFRECHT im Portrait     → ay muss positiv sein, sonst MPU_AY_SIGN = -1
//   Rahmen im RICHTIGEN Landscape   → ax muss positiv sein, sonst MPU_AX_SIGN = -1
#define MPU_AX_SIGN            1
#define MPU_AY_SIGN            1
#define MPU_ANGLE_THRESHOLD    88.0f // Grad — Landscape-Ziel nach Offset-Korrektur (stable ~88.37°, Schwelle 0.37° darunter für Jitter)
#define MPU_PORTRAIT_THRESHOLD  0.5f // Grad — Portrait-Ziel nach Offset-Korrektur (stable ~0.00°, Schwelle 0.5° darüber für Jitter)
#define MPU_SETTLE_MS          400   // ms warten nach Zielerreichung zur Stabilitätsprüfung
#define MOTOR_POWER_OFF_DELAY  200   // ms nach Motorstop bevor Pins auf LOW (Strom sparen)

// ─── Display-Auflösung ──────────────────────────────────────────────────────
#define DISPLAY_WIDTH   800
#define DISPLAY_HEIGHT  480
