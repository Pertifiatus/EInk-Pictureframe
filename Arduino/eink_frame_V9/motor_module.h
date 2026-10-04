// motor_module.h
// Phase 4: Schrittmotor-Steuerung und MPU-9250 Orientierungserkennung
//
// Getriebe: Schneckenrad z=40, 1-gängige Schnecke → Übersetzung 40:1
// 28BYJ-48 HALF4WIRE: 4096 Schritte/Motorumdrehung × 40 = 40.960 Schritte für 90°
//
// Ablauf Kalibrierung (motorCalibrateFull):
//   1. Orientierung erkennen (MPU + Kippschalter) → grüner LED-Flash
//   2. 90° in Gegenrichtung drehen, Kippschalter als Richtungsbestätigung nutzen
//   3. Schritte speichern, dann zurückdrehen zur Startorientierung
//
// Normalbetrieb (motor_steps_90 bekannt):
//   motorRotate(steps, dir) → Schnelldrehung mit gespeicherten Schritten
//                           → IMU-Feinjustierung der letzten Grad
//
// Benötigte Bibliothek: AccelStepper (Mike McCauley)

// Forward-Deklarationen für LED-Funktionen (definiert in eink_frame_V9.ino)
extern void ledFlash(uint8_t r, uint8_t g, uint8_t b, uint16_t ms);
extern void ledSet(uint8_t r, uint8_t g, uint8_t b);

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

// Probe-Schritte zur Richtungsbestimmung: ~5° Drehung des Ausgangs
// Größerer Probe als vorher (2°→5°) für zuverlässigere MPU-Winkeländerung,
// da 2° zu nahe am MPU-Messrauschen liegt.
#define MOTOR_PROBE_STEPS   (MOTOR_STEPS_DEFAULT / 18)   // ≈ 2275 Schritte ≈ 5°

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
// targetIsPortrait: true → Ziel < PORTRAIT_ANGLE_THRESH, false → Ziel >= MPU_ANGLE_THRESHOLD
// Richtung deterministisch: dir=+1 → Portrait→negative Schritte, Landscape→positive Schritte
// (gilt für MOTOR_DIR_INVERT=true). Verifikations-Probe (5°) korrigiert auf dir=-1 falls nötig.
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

  // ── Richtung: deterministisch aus bekannter Startposition ────────────────
  // Regel (MOTOR_DIR_INVERT=true, DIR=-1):
  //   Portrait→Landscape: moveTo(-MAX) = negative Schritte  → dir=+1 ✓
  //   Landscape→Portrait: moveTo(+MAX) = positive Schritte  → dir=+1 ✓
  // dir=+1 liefert mit DIR*targetDir*dir*MAX stets die richtige physikalische Richtung.
  const int8_t targetDir = targetIsPortrait ? -1 : 1;
  int8_t dir = 1; // Basisrichtung: immer +1

  // Verifikations-Probe (5°): bestätigt Richtung anhand Winkeländerung.
  // Falls IMU eine Annäherung ans Ziel zeigt → dir bleibt +1.
  // Falls IMU Entfernung zeigt (z.B. falsches MOTOR_DIR_INVERT) → dir = -1.
  g_stepper.setCurrentPosition(0);
  g_stepper.setMaxSpeed(MOTOR_MAX_SPEED * 0.6f);
  g_stepper.setAcceleration(MOTOR_ACCELERATION);
  g_stepper.move(DIR * targetDir * dir * MOTOR_PROBE_STEPS);
  while (g_stepper.distanceToGo() != 0) { g_stepper.run(); esp_task_wdt_reset(); }

  float angle2 = mpuReadAngle();
  if (angle2 < 0.0f) angle2 = angle;

  // Probe hat Ziel bereits erreicht (Startwinkel war < PROBE_STEPS vom Ziel entfernt)?
  // Tritt auf wenn Frame z.B. bei 2° startet und 5°-Probe über 0° hinausschießt.
  {
    bool probeAlreadyAtTarget = targetIsPortrait ? (angle2 < TARGET_THRESH) : (angle2 >= TARGET_THRESH);
    if (probeAlreadyAtTarget) {
      Serial.printf("[Motor] Ziel bereits nach Probe erreicht (%.1f°)\n", angle2);
      g_stepper.stop();
      while (g_stepper.distanceToGo() != 0) { g_stepper.run(); esp_task_wdt_reset(); }
      motorPowerOff();
      return MOTOR_PROBE_STEPS;
    }
  }

  float angleDelta = angle2 - angle;
  bool movingTowardTarget = targetIsPortrait ? (angleDelta < 0.0f) : (angleDelta > 0.0f);
  Serial.printf("[Motor] Probe: %.1f°→%.1f° (Delta %+.2f°) → %s\n",
                angle, angle2, angleDelta, movingTowardTarget ? "Richtung OK" : "Richtung falsch");

  if (fabsf(angleDelta) > 0.8f && !movingTowardTarget) {
    dir = -1;
    Serial.println("[Motor] Probe korrigiert dir auf -1");
  }

  // ── In Richtung Ziel drehen (Zwei-Geschwindigkeiten) ────────────────────
  // Phase A: schnell bis 5° vor dem Ziel
  // Phase B: langsam (FINETUNE_SPEED) für finale Annäherung → minimaler Bremsweg
  // Bei FINETUNE_SPEED=60 st/s: Bremsweg = 60²/(2×200) ≈ 9 Schritte ≈ 0,02° → kein Überdreher
  g_stepper.setCurrentPosition(0);
  g_stepper.setMaxSpeed(MOTOR_MAX_SPEED * 0.6f); // Start: schnelle Suche
  g_stepper.moveTo(DIR * targetDir * dir * MOTOR_SEARCH_MAX);

  // Slow-down-Schwelle: 5° vor dem Ziel-Threshold
  const float SLOWDOWN_DEG   = 5.0f;
  const float SLOWDOWN_THRESH = targetIsPortrait
      ? (TARGET_THRESH + SLOWDOWN_DEG)   // Portrait: langsam wenn Winkel < 5,5°
      : (TARGET_THRESH - SLOWDOWN_DEG);  // Landscape: langsam wenn Winkel > 83°
  bool slowedDown = false;

  int32_t lastCheck  = 0;
  int32_t totalSteps = MOTOR_PROBE_STEPS;
  bool    found      = false;

  while (g_stepper.distanceToGo() != 0) {
    g_stepper.run();
    esp_task_wdt_reset();

    int32_t pos = abs(g_stepper.currentPosition());
    if (pos - lastCheck >= MOTOR_CHECK_STEPS) {
      lastCheck = pos;

      float a = mpuReadAngle();
      if (a < 0.0f) continue;

      // Geschwindigkeit reduzieren sobald wir nahe am Ziel sind
      if (!slowedDown) {
        bool nearTarget = targetIsPortrait ? (a < SLOWDOWN_THRESH) : (a >= SLOWDOWN_THRESH);
        if (nearTarget) {
          g_stepper.setMaxSpeed(MOTOR_FINETUNE_SPEED);
          slowedDown = true;
          Serial.printf("[Motor] Naehert sich %s (%.1f°) → langsam\n", label, a);
        }
      }

      bool atTarget = targetIsPortrait ? (a < TARGET_THRESH) : (a >= TARGET_THRESH);
      Serial.printf("[Motor] Pos=%ld  Winkel=%.1f°%s\n",
                    (long)(totalSteps + pos), a, slowedDown ? " [langsam]" : "");

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
        // Settle fehlgeschlagen: weiterfahren (Geschwindigkeit bleibt langsam)
        g_stepper.moveTo(DIR * targetDir * dir * MOTOR_SEARCH_MAX);
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

// ─── Kalibrierphase setzen ────────────────────────────────────────────────────
static void setCalibPhase(volatile char* out, const char* phase) {
  if (out) strncpy((char*)out, phase, 31);
  Serial.printf("[Calib] Phase: %s\n", phase);
}

// ─── Vollkalibrierung ─────────────────────────────────────────────────────────
// Erkennt Startorientierung per IMU + Kippschalter → grüner LED-Flash.
// Dreht 90° in Gegenrichtung, prüft Kippschalter als Bestätigung.
// Dreht zurück zur Startorientierung.
// phaseOut: optionaler char[32]-Puffer für Phasenstatus (für /api/status)
// endedInPortrait: optionaler Ausgabeparameter für Endorientierung
// Gibt gemessene Schritte zurück, 0 bei Fehler.
int32_t motorCalibrateFull(volatile char* phaseOut = nullptr, bool* endedInPortrait = nullptr) {
  Serial.println("[Motor] === Vollkalibrierung ===");
  mpuInit();
  g_stepper.enableOutputs();

  // ── Phase 1: Startorientierung erkennen ──────────────────────────────────
  setCalibPhase(phaseOut, "orient_detect");
  pinMode(PIN_BALL_SWITCH, INPUT); // Externer 10kΩ Pull-Up zu 3V3 vorhanden
  delay(100);
  esp_task_wdt_reset();

  float startAngle = mpuReadAngle();
  bool  switchState0 = digitalRead(PIN_BALL_SWITCH);

  // Grober Schwellwert: unter halber Landscape-Schwelle → Portrait
  bool startInPortrait = (startAngle >= 0.0f && startAngle < MPU_ANGLE_THRESHOLD / 2.0f);

  Serial.printf("[Calib] Start: %.1f° → %s, Kippschalter: %d\n",
                startAngle, startInPortrait ? "Portrait" : "Landscape", switchState0);

  // Kurzer grüner Blitz: Orientierung erkannt
  ledFlash(0, 255, 0, 200);
  delay(200);
  esp_task_wdt_reset();

  setCalibPhase(phaseOut, startInPortrait ? "portrait_found" : "landscape_found");
  delay(500);
  esp_task_wdt_reset();

  // ── Phase 1.2: Auf exakt 0° oder 90° ausrichten ──────────────────────────
  // Nötig wenn Rahmen schief steht (z. B. 45°); kein Kippschalter-Check hier.
  setCalibPhase(phaseOut, "align_start");
  ledSet(0, 60, 140); // Dunkelblau: Ausrichten
  {
    int32_t alignSteps = motorFindAngle(startInPortrait);
    Serial.printf("[Calib] Ausrichtung: %ld Schritte\n", (long)alignSteps);
    if (alignSteps < 0) {
      Serial.println("[Motor] Kalibrierung fehlgeschlagen: Ausrichtung nicht moeglich");
      ledSet(0,0,0);
      motorPowerOff();
      return 0;
    }
  }
  ledFlash(0, 255, 0, 150); // Kurzer grüner Flash: exakte Ausgangsposition erreicht
  delay(200);
  esp_task_wdt_reset();

  // ── Phase 2: 90° in Gegenrichtung drehen ─────────────────────────────────
  setCalibPhase(phaseOut, "rotating_90");
  ledSet(0, 120, 255); // Blau: Motor dreht

  // Kippschalter-Zustand vor der Rotation lesen (Referenz: LOW = korrekte Lage)
  uint8_t swBefore90 = digitalRead(PIN_BALL_SWITCH);
  Serial.printf("[Calib] Kippschalter vor 90°-Drehung: %d\n", swBefore90);

  int32_t steps90 = motorFindAngle(!startInPortrait);
  if (steps90 < 0) {
    Serial.printf("[Motor] Kalibrierung fehlgeschlagen: %s nicht gefunden\n",
                  startInPortrait ? "Landscape" : "Portrait");
    ledSet(0,0,0);
    motorPowerOff();
    return 0;
  }

  // Kippschalter-Bestätigung: nach 90° sollte er wieder LOW sein (richtige Lage).
  // Wenn HIGH, wurde die Zieldrehung nicht vollständig erreicht oder Richtung war falsch.
  uint8_t swAfter90 = digitalRead(PIN_BALL_SWITCH);
  Serial.printf("[Calib] Kippschalter nach 90°-Drehung: %d (%s)\n",
                swAfter90, swAfter90 == swBefore90 ? "OK - gleicher Zustand" : "WARNUNG - unveraendert != erwartet");

  delay(200);
  esp_task_wdt_reset();

  // ── Phase 3: Zurück zur Startorientierung ────────────────────────────────
  setCalibPhase(phaseOut, "rotating_back");
  ledSet(0, 80, 180); // Blau (dunkler): Rückdrehung

  int32_t stepsBack = motorFindAngle(startInPortrait);
  Serial.printf("[Calib] Rueckdrehung: %ld Schritte\n", (long)stepsBack);
  if (endedInPortrait) *endedInPortrait = startInPortrait;

  Serial.printf("[Motor] Kalibrierung OK: %ld Schritte fuer 90°\n", (long)steps90);
  setCalibPhase(phaseOut, "done");
  ledSet(0,0,0);
  motorPowerOff();
  return steps90;
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

  // ── Feinjustierung + Settle-Bestätigung (bidirektional) ─────────────────
  // Bidirektional: nach jedem Schritt prüfen ob Winkel besser oder schlechter wurde.
  // Verhindert Endlosfahrt wenn Motor überschossen hat und auf der falschen Seite steht
  // (MPU liefert abs()-Wert → +0.3° und −0.3° sehen gleich aus).
  Serial.println("[Motor] Feinjustierung...");
  g_stepper.setMaxSpeed(MOTOR_FINETUNE_SPEED);
  g_stepper.setAcceleration(MOTOR_ACCELERATION);

  // Startrichtung: nominell auf Ziel zu
  int8_t  fineDir     = DIR * (toLandscape ? 1 : -1);
  int32_t fineDone    = 0;
  float   prevAngle   = angle;
  bool    dirFlipped  = false; // Richtungsumkehr nur einmal erlaubt

  while (fineDone < MOTOR_FINETUNE_MAX) {
    if (atTarget) {
      delay(MPU_SETTLE_MS);
      esp_task_wdt_reset();
      angle = mpuReadAngle();
      atTarget = toLandscape ? (angle >= MPU_ANGLE_THRESHOLD)
                             : (angle < PORTRAIT_ANGLE_THRESH);
      Serial.printf("[Motor] Settle: %.1f° → %s\n", angle, atTarget ? "OK" : "zurueckgewippen");
      if (atTarget) break;
    }

    g_stepper.move(fineDir * MOTOR_CHECK_STEPS);
    while (g_stepper.distanceToGo() != 0) {
      g_stepper.run();
      esp_task_wdt_reset();
    }
    fineDone += MOTOR_CHECK_STEPS;
    angle = mpuReadAngle();
    atTarget = toLandscape ? (angle >= MPU_ANGLE_THRESHOLD)
                           : (angle < PORTRAIT_ANGLE_THRESH);

    // Richtungscheck: bewegen wir uns zum Ziel hin oder weg?
    // Für Portrait: Winkel soll kleiner werden; für Landscape: größer.
    bool gettingCloser = toLandscape ? (angle > prevAngle) : (angle < prevAngle);
    if (!atTarget && !dirFlipped && fabsf(angle - prevAngle) > 0.4f && !gettingCloser) {
      fineDir   = -fineDir;
      dirFlipped = true; // Nur einmal umkehren um Pendeln zu verhindern
      Serial.printf("[Motor] Fein: Richtungsumkehr bei %.1f° (Schritt entfernte sich)\n", angle);
    }

    prevAngle = angle;
    Serial.printf("[Motor] Fein: %ld Schr, %.1f°\n", (long)fineDone, angle);
  }

  motorPowerOff();

  if (!atTarget)
    Serial.printf("[Motor] WARNUNG: Ziel nicht erreicht (Winkel=%.1f°)\n", angle);
  return atTarget;
}

// ─── Manuelle Motorsteuerung ──────────────────────────────────────────────────
// steps: positive = vorwärts, negative = rückwärts (kein DIR-Invert angewandt)
// Für die 3D-Drucker-artige UI-Steuerung — keine IMU-Feinjustierung.
void motorManualMove(int32_t steps) {
  Serial.printf("[Motor] Manuell: %ld Schritte\n", (long)steps);
  g_stepper.enableOutputs();
  g_stepper.setMaxSpeed(MOTOR_MAX_SPEED);
  g_stepper.setAcceleration(MOTOR_ACCELERATION);
  g_stepper.setCurrentPosition(0);
  g_stepper.moveTo(steps);
  while (g_stepper.distanceToGo() != 0) {
    g_stepper.run();
    esp_task_wdt_reset();
  }
  motorPowerOff();
}
