// motor_module.h
// Phase 4: Schrittmotor-Steuerung und MPU-9250 Orientierungserkennung
//
// Getriebe: Schneckenrad z=40, 1-gängige Schnecke → Übersetzung 40:1
// 28BYJ-48 HALF4WIRE: 4096 Schritte/Motorumdrehung × 40 = 40.960 Schritte für 90°
//
// Ablauf Erststart (motor_steps_90 == 0 in config.json):
//   motorCalibrateFull() → IMU-geführt Portrait suchen → IMU-geführt Landscape suchen
//                        → Schrittzahl messen und zurückgeben
//
// Normalbetrieb (motor_steps_90 bekannt):
//   motorRotate(steps, dir) → Schnelldrehung mit gespeicherten Schritten
//                           → IMU-Feinjustierung der letzten Grad
//
// Benötigte Bibliothek: AccelStepper (Mike McCauley)

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <math.h>
#include "esp_task_wdt.h"
#include "config.h"

// ─── MPU-9250 Register ────────────────────────────────────────────────────────
#define MPU_ADDR         0x69   // AD0 auf HIGH → Adresse 0x69
#define MPU_REG_PWR      0x6B   // PWR_MGMT_1: 0x00 = wach
#define MPU_REG_ACCEL    0x3B   // ACCEL_XOUT_H (6 Bytes big-endian: Ax, Ay, Az)

// Portrait ≈ 0°, Landscape ≈ 90°
#define PORTRAIT_ANGLE_THRESH  MPU_PORTRAIT_THRESHOLD  // direkt gemessener Portrait-Schwellwert

// Probe-Schritte zur Richtungsbestimmung: ~2° Drehung des Ausgangs
// → ausreichend für messbare MPU-Winkeländerung
#define MOTOR_PROBE_STEPS   (MOTOR_STEPS_DEFAULT / 45)   // ≈ 910 Schritte ≈ 2°

// Abfrage-Intervall während des Kalibrierlaufs
#define MOTOR_CHECK_STEPS   (MOTOR_STEPS_DEFAULT / 90)   // ≈ 455 Schritte ≈ 1°

// Max. Schritte für Kalibrierlauf: 1,5 × 90° (Frame maximal 90° von Portrait entfernt)
#define MOTOR_SEARCH_MAX    (MOTOR_STEPS_DEFAULT + MOTOR_STEPS_DEFAULT / 2)

// ─── Schrittmotor-Instanz ─────────────────────────────────────────────────────
// 28BYJ-48 mit ULN2003: HALF4WIRE = Halbschrittmodus
// Reihenfolge IN1, IN3, IN2, IN4 → korrekte Sequenz für 28BYJ-48
static AccelStepper g_stepper(
  AccelStepper::HALF4WIRE,
  PIN_MOTOR_IN1, PIN_MOTOR_IN3, PIN_MOTOR_IN2, PIN_MOTOR_IN4
);

// ─── MPU-9250 aufwecken ───────────────────────────────────────────────────────
void mpuInit() {
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_PWR);
  Wire.write(0x00);
  if (Wire.endTransmission(true) == 0)
    Serial.println("[MPU] OK");
  else
    Serial.println("[MPU] WARNUNG: nicht gefunden");
  delay(50);
}

// ─── Neigungswinkel lesen ─────────────────────────────────────────────────────
// 0° ≈ Portrait, 90° ≈ Landscape.
// Berechnung: atan2(|Ax|, |Ay|)
// Annahme: X längs Frame-Breite, Y längs Frame-Höhe.
// Bei falscher Montage: Ax und Ay in der Formel tauschen.
// Gibt -1.0f bei I2C-Lesefehler zurück.
static float mpuReadAngle() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL);
  Wire.endTransmission(false);
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6, (bool)true) < 6) return -1.0f;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // Az nicht benötigt

  float angle = atan2f((float)abs(ax), (float)abs(ay)) * 180.0f / (float)M_PI;
  return fmaxf(0.0f, angle - MPU_OFFSET); // Montage-Schräglage korrigieren, Minwert 0°
}

// ─── Motor initialisieren ─────────────────────────────────────────────────────
void motorInit() {
  g_stepper.setMaxSpeed(MOTOR_MAX_SPEED);
  g_stepper.setAcceleration(MOTOR_ACCELERATION);
  g_stepper.disableOutputs();
  Serial.println("[Motor] initialisiert");
}

// ─── Motorpins stromlos schalten ──────────────────────────────────────────────
// Schneckengetriebe ist selbsthemmend → kein Haltestrom nötig
static void motorPowerOff() {
  delay(MOTOR_POWER_OFF_DELAY);
  g_stepper.disableOutputs();
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  digitalWrite(PIN_MOTOR_IN3, LOW);
  digitalWrite(PIN_MOTOR_IN4, LOW);
}

// ─── Hilfsfunktion: IMU-geführt auf Zielwinkel drehen ─────────────────────────
// Bestimmt Richtung per Probe, dreht bis Winkel die Schwelle erreicht.
// targetIsPortrait: true → Ziel < PORTRAIT_ANGLE_THRESH, false → Ziel >= MPU_ANGLE_THRESHOLD
// Gibt gezählte Schritte zurück, -1 bei Fehler (Timeout).
static int32_t motorFindAngle(bool targetIsPortrait) {
  constexpr int8_t DIR = MOTOR_DIR_INVERT ? -1 : 1;
  const float TARGET_THRESH = targetIsPortrait ? PORTRAIT_ANGLE_THRESH : MPU_ANGLE_THRESHOLD;
  const char* label = targetIsPortrait ? "Portrait" : "Landscape";

  // Bereits am Ziel?
  float angle = mpuReadAngle();
  Serial.printf("[Motor] Suche %s, Start-Winkel: %.1f°\n", label, angle);
  if (angle >= 0.0f) {
    bool atTarget = targetIsPortrait ? (angle < TARGET_THRESH) : (angle >= TARGET_THRESH);
    if (atTarget) {
      Serial.printf("[Motor] Bereits bei %s (%.1f°)\n", label, angle);
      return 0;
    }
  }

  // ── Richtung bestimmen: Probe-Schritt in erwarteter Zielrichtung ─────────
  // targetDir: Landscape erhöht Winkel (+1), Portrait verringert Winkel (-1)
  // Kombiniert mit DIR (MOTOR_DIR_INVERT) ergibt sich die korrekte Probe-Richtung.
  const int8_t targetDir = targetIsPortrait ? -1 : 1;
  g_stepper.setCurrentPosition(0);
  g_stepper.setMaxSpeed(MOTOR_MAX_SPEED * 0.6f);
  g_stepper.setAcceleration(MOTOR_ACCELERATION);
  g_stepper.move(DIR * targetDir * MOTOR_PROBE_STEPS);
  while (g_stepper.distanceToGo() != 0) { g_stepper.run(); esp_task_wdt_reset(); }

  float angle2 = mpuReadAngle();
  if (angle2 < 0.0f) angle2 = angle;

  const float PROBE_DEADBAND = 1.5f;
  int8_t dir;
  float angleDelta = angle2 - angle;
  if (fabsf(angleDelta) <= PROBE_DEADBAND) {
    dir = 1;
    Serial.printf("[Motor] Probe: kein messbarer Winkel (%.1f° → %.1f°) → vorwaerts\n", angle, angle2);
  } else {
    bool angleMovingRight = targetIsPortrait ? (angleDelta < 0.0f) : (angleDelta > 0.0f);
    dir = angleMovingRight ? 1 : -1;
    Serial.printf("[Motor] Richtung %s (%.1f° → %.1f°)\n",
                  dir > 0 ? "vorwaerts" : "rueckwaerts", angle, angle2);
  }

  // ── In Richtung Ziel drehen ───────────────────────────────────────────────
  g_stepper.setCurrentPosition(0);
  g_stepper.moveTo(DIR * targetDir * dir * MOTOR_SEARCH_MAX);

  int32_t lastCheck  = 0;
  int32_t totalSteps = (dir > 0) ? MOTOR_PROBE_STEPS : -MOTOR_PROBE_STEPS;
  bool    found      = false;

  while (g_stepper.distanceToGo() != 0) {
    g_stepper.run();
    esp_task_wdt_reset();

    int32_t pos = abs(g_stepper.currentPosition());
    if (pos - lastCheck >= MOTOR_CHECK_STEPS) {
      lastCheck = pos;
      float a = mpuReadAngle();
      if (a < 0.0f) continue;

      bool atTarget = targetIsPortrait ? (a < TARGET_THRESH) : (a >= TARGET_THRESH);
      Serial.printf("[Motor] Pos=%ld  Winkel=%.1f°\n", (long)(totalSteps + pos), a);

      if (atTarget) {
        g_stepper.stop();
        while (g_stepper.distanceToGo() != 0) { g_stepper.run(); esp_task_wdt_reset(); }
        delay(MPU_SETTLE_MS);
        esp_task_wdt_reset();
        float aSettle = mpuReadAngle();
        bool settled = targetIsPortrait ? (aSettle < TARGET_THRESH) : (aSettle >= TARGET_THRESH);
        Serial.printf("[Motor] Settle: %.1f° → %s\n", aSettle, settled ? "OK" : "zurueckgewippen");
        if (settled) {
          found = true;
          totalSteps += pos;
          Serial.printf("[Motor] %s stabil! Gesamt=%ld Schritte, Winkel=%.1f°\n",
                        label, (long)totalSteps, aSettle);
          break;
        }
        g_stepper.moveTo(DIR * dir * MOTOR_SEARCH_MAX);
        lastCheck = pos;
      }
    }
  }

  g_stepper.stop();
  while (g_stepper.distanceToGo() != 0) { g_stepper.run(); esp_task_wdt_reset(); }

  if (!found) {
    totalSteps += abs(g_stepper.currentPosition());
    Serial.printf("[Motor] WARNUNG: %s nicht gefunden (Timeout nach %ld Schritten)\n",
                  label, (long)totalSteps);
  }

  return found ? totalSteps : -1;
}

// ─── Vollkalibrierung ─────────────────────────────────────────────────────────
// Fährt IMU-geführt auf Portrait, dann auf Landscape und misst die Schritte.
// Nach der Kalibrierung ist der Rahmen im Landscape-Modus.
// Gibt gemessene Schritte zurück, 0 bei Fehler.
int32_t motorCalibrateFull() {
  Serial.println("[Motor] === Vollkalibrierung ===");
  mpuInit();
  g_stepper.enableOutputs();

  // Schritt 1: Portrait finden
  int32_t stepsToPortrait = motorFindAngle(true);
  if (stepsToPortrait < 0) {
    Serial.println("[Motor] Kalibrierung fehlgeschlagen: Portrait nicht gefunden");
    motorPowerOff();
    return 0;
  }

  delay(200);

  // Schritt 2: Von Portrait nach Landscape, Schritte zählen
  int32_t stepsToLandscape = motorFindAngle(false);
  if (stepsToLandscape < 0) {
    Serial.println("[Motor] Kalibrierung fehlgeschlagen: Landscape nicht gefunden");
    motorPowerOff();
    return 0;
  }

  Serial.printf("[Motor] Kalibrierung OK: %ld Schritte fuer 90°\n", (long)stepsToLandscape);
  motorPowerOff();
  return stepsToLandscape;
}

// ─── Normalbetrieb: Schnelldrehung + IMU-Feinjustierung ──────────────────────
// steps:    gespeicherte Schritte aus config.json (für 90°)
// toLandscape: true = Portrait→Landscape (+steps), false = Landscape→Portrait (−steps)
//
// Ablauf:
//   1. Schnelldrehung mit gespeicherten Schritten (volle Geschwindigkeit)
//   2. IMU-Feinjustierung: langsam weiterdrehen bis Winkel-Schwelle erreicht
//      (korrigiert Schleichfehler durch Last, Temperatur etc.)
// Gibt true zurück wenn Ziel sauber erreicht.
bool motorRotate(int32_t steps, bool toLandscape) {
  Serial.printf("[Motor] %s → %s (%ld Schritte + Feinjustierung)\n",
                toLandscape ? "Portrait" : "Landscape",
                toLandscape ? "Landscape" : "Portrait",
                (long)steps);

  mpuInit();
  g_stepper.enableOutputs();
  g_stepper.setMaxSpeed(MOTOR_MAX_SPEED);
  g_stepper.setAcceleration(MOTOR_ACCELERATION);

  // ── Schnelldrehung ────────────────────────────────────────────────────────
  constexpr int8_t DIR = MOTOR_DIR_INVERT ? -1 : 1;
  int32_t target = DIR * (toLandscape ? steps : -steps);
  g_stepper.setCurrentPosition(0);
  g_stepper.moveTo(target);

  while (g_stepper.distanceToGo() != 0) {
    g_stepper.run();
    esp_task_wdt_reset();
  }

  delay(100); // Motor zur Ruhe kommen lassen

  // ── IMU-Feinjustierung ────────────────────────────────────────────────────
  float angle = mpuReadAngle();
  Serial.printf("[Motor] Nach Schnelldrehung: %.1f° (Soll %s %.1f°)\n",
                angle,
                toLandscape ? ">=" : "<",
                toLandscape ? MPU_ANGLE_THRESHOLD : PORTRAIT_ANGLE_THRESH);

  bool atTarget = toLandscape ? (angle >= MPU_ANGLE_THRESHOLD)
                              : (angle < PORTRAIT_ANGLE_THRESH);

  // ── Feinjustierung + Settle-Bestätigung ──────────────────────────────────
  // Läuft immer — auch wenn Schnelldrehung bereits am Ziel war (Settle nötig).
  Serial.println("[Motor] Feinjustierung...");
  g_stepper.setMaxSpeed(MOTOR_FINETUNE_SPEED);
  g_stepper.setAcceleration(MOTOR_ACCELERATION);
  int8_t  fineDir  = DIR * (toLandscape ? 1 : -1);
  int32_t fineDone = 0;

  while (fineDone < MOTOR_FINETUNE_MAX) {
    if (atTarget) {
      // Settle-Bestätigung: warten, dann nochmal messen
      delay(MPU_SETTLE_MS);
      esp_task_wdt_reset();
      angle = mpuReadAngle();
      atTarget = toLandscape ? (angle >= MPU_ANGLE_THRESHOLD)
                             : (angle < PORTRAIT_ANGLE_THRESH);
      Serial.printf("[Motor] Settle: %.1f° → %s\n", angle, atTarget ? "OK" : "zurueckgewippen");
      if (atTarget) break;
    }
    // Ziel noch nicht stabil: einen Schritt Feinjustierung
    g_stepper.move(fineDir * MOTOR_CHECK_STEPS);
    while (g_stepper.distanceToGo() != 0) {
      g_stepper.run();
      esp_task_wdt_reset();
    }
    fineDone += MOTOR_CHECK_STEPS;
    angle = mpuReadAngle();
    atTarget = toLandscape ? (angle >= MPU_ANGLE_THRESHOLD)
                           : (angle < PORTRAIT_ANGLE_THRESH);
    Serial.printf("[Motor] Fein: %ld Schr, %.1f°\n", (long)fineDone, angle);
  }

  motorPowerOff();

  if (!atTarget)
    Serial.printf("[Motor] WARNUNG: Ziel nicht erreicht (Winkel=%.1f°)\n", angle);
  return atTarget;
}
