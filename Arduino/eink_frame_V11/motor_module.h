// motor_module.h
// Phase 4: Schrittmotor-Steuerung und MPU-9250 Orientierungserkennung
//
// Getriebe: Schneckenrad z=40, 1-gängige Schnecke → Übersetzung 40:1
// 28BYJ-48 HALF4WIRE: 4096 Schritte/Motorumdrehung × 40 = 40.960 Schritte für 90°
//
// Ablauf Kalibrierung (motorCalibrateFull):
//   1. Orientierung erkennen (MPU, vorzeichenbehaftet) → grüner LED-Flash
//   2. Exakt auf 0°/90° ausrichten, dann 90° in Gegenrichtung drehen (Schritte zählen)
//   3. Schritte speichern, dann zurückdrehen zur Startorientierung
//
// Normalbetrieb (motor_steps_90 bekannt):
//   motorRotate(steps, dir) → Schnelldrehung mit gespeicherten Schritten
//                           → IMU-Feinjustierung der letzten Grad
//
// Benötigte Bibliothek: AccelStepper (Mike McCauley)

// Forward-Deklarationen für LED-Funktionen (definiert in eink_frame_V11.ino)
extern void ledFlash(uint8_t r, uint8_t g, uint8_t b, uint16_t ms);
extern void ledSet(uint8_t r, uint8_t g, uint8_t b);

#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <math.h>
#include "esp_task_wdt.h"
#include <SD.h>
#include "config.h"

// ─── Motor-Log (auch ohne USB lesbar) ─────────────────────────────────────────
// Alle Motor-Meldungen gehen zusätzlich nach /motor_log.txt auf der SD,
// im Browser lesbar unter http://192.168.4.1/api/log.
// Gepuffert im RAM und nur geschrieben wenn der Motor steht (mlogFlush in
// motorPowerOff / setCalibPhase) → SD-Zugriffe bremsen den Motor nicht.
static String g_mlogBuf;

static void mlog(const char* fmt, ...) {
  char line[192];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  Serial.print(line);
  g_mlogBuf += line;
}

void mlogFlush() {
  if (g_mlogBuf.length() == 0) return;
  File f = SD.open("/motor_log.txt", FILE_APPEND);
  if (f) { f.print(g_mlogBuf); f.close(); }
  g_mlogBuf = "";
}
// ─── MPU-9250 Register ────────────────────────────────────────────────────────
#define MPU_ADDR         0x69   // AD0 auf HIGH → Adresse 0x69
#define MPU_REG_PWR      0x6B   // PWR_MGMT_1: 0x00 = wach
#define MPU_REG_ACCEL    0x3B   // ACCEL_XOUT_H (6 Bytes big-endian: Ax, Ay, Az)

// Portrait ≈ 0°, Landscape ≈ 90°
#define PORTRAIT_ANGLE_THRESH  MPU_PORTRAIT_THRESHOLD  // direkt gemessener Portrait-Schwellwert

// Fahrstrecke bis zur Richtungsprüfung im laufenden Suchlauf: ~5° (2° liegt im MPU-Rauschen)
#define MOTOR_PROBE_STEPS   (MOTOR_STEPS_DEFAULT / 18)   // ≈ 2275 Schritte ≈ 5°

// Abfrage-Intervall während des Kalibrierlaufs
#define MOTOR_CHECK_STEPS   (MOTOR_STEPS_DEFAULT / 90)   // ≈ 455 Schritte ≈ 1°

// Max. Schritte für Kalibrierlauf: 2,25 × 90° (auf dem Kopf → aufrecht = 180° + Reserve)
#define MOTOR_SEARCH_MAX    (MOTOR_STEPS_DEFAULT * 2 + MOTOR_STEPS_DEFAULT / 4)

// ─── Schrittmotor-Instanz ─────────────────────────────────────────────────────
// 28BYJ-48 mit ULN2003: HALF4WIRE = Halbschrittmodus
// Reihenfolge IN1, IN3, IN2, IN4 → korrekte Sequenz für 28BYJ-48
static AccelStepper g_stepper(
  AccelStepper::HALF4WIRE,
  PIN_MOTOR_IN1, PIN_MOTOR_IN3, PIN_MOTOR_IN2, PIN_MOTOR_IN4
);

// ─── MPU-9250 aufwecken ───────────────────────────────────────────────────────
void mpuInit() {
  // 100 kHz statt 400 kHz: robuster gegen Störungen vom Motor (Log zeigte kaputte Reads)
  Wire.setClock(100000);
  for (int i = 0; i < 3; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_PWR);
    Wire.write(0x00);
    if (Wire.endTransmission(true) == 0) { mlog("[MPU] OK\n"); delay(50); return; }
    delay(10);
  }
  mlog("[MPU] WARNUNG: nicht gefunden\n");
  delay(50);
}
// ─── MPU-9250 schlafen legen ──────────────────────────────────────────────────
// Wach zieht der MPU ~3,5 mA – auch im Deep Sleep des ESP32 (Reset-Wert = wach).
// Sleep-Bit (0x40) → ~8 µA. mpuInit() weckt ihn beim nächsten Motorlauf wieder.
void mpuSleep() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_PWR);
  Wire.write(0x40);
  Wire.endTransmission(true);
}

// ─── Neigungswinkel lesen ─────────────────────────────────────────────────────
// Vorzeichenbehaftet, -180°..+180°:
//   0° = Portrait aufrecht, +90° = richtiges Landscape, ±180° = auf dem Kopf.
// (V10 nutzte |Ax|,|Ay| → "auf dem Kopf" las sich ebenfalls als 0° und wurde
//  als Portrait akzeptiert.) Vorzeichen über MPU_AX_SIGN / MPU_AY_SIGN in config.h.

// Ein Rohwert-Paar lesen. false bei I2C-Fehler oder unplausiblem Wert:
// der Rahmen steht senkrecht → |(ax, ay)| ≈ 1 g (16384 bei ±2 g). Gestörte Reads
// lieferten z.B. ax=511 ay=-1 bzw. ay=0 → wäre sonst exakt "89,8°" = falsches Ziel.
static bool mpuReadRaw(int16_t& ax, int16_t& ay) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_ACCEL);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6, (bool)true) < 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();  // feste Reihenfolge (nicht 2× read() in einem Ausdruck)
  ax = (int16_t)((b[0] << 8) | b[1]);
  ay = (int16_t)((b[2] << 8) | b[3]);
  float g = sqrtf((float)ax * ax + (float)ay * ay);
  return g > 0.75f * 16384.0f && g < 1.25f * 16384.0f;
}

// Median aus 3 gültigen Messungen (bis zu 8 Versuche) → einzelne Ausreißer fallen raus.
// Gibt NAN zurück, wenn keine einzige gültige Messung zustande kommt.
static float mpuReadAngle(bool logRaw = false) {
  float a[3];
  int n = 0;
  int16_t ax = 0, ay = 0;
  for (int tries = 0; tries < 8 && n < 3; tries++) {
    if (!mpuReadRaw(ax, ay)) { delayMicroseconds(300); continue; }
    a[n++] = atan2f((float)(MPU_AX_SIGN * ax), (float)(MPU_AY_SIGN * ay)) * 180.0f / (float)M_PI
           - MPU_OFFSET;  // Montage-Schräglage korrigieren
  }
  if (n == 0) { mlog("[MPU] keine gueltige Messung\n"); return NAN; }

  float angle = a[0];
  if (n == 3) {  // Median = der Wert zwischen den anderen beiden
    angle = fmaxf(fminf(a[0], a[1]), fminf(fmaxf(a[0], a[1]), a[2]));
  }
  if (logRaw) mlog("[MPU] ax=%d ay=%d → Winkel=%.1f° (%d/3 gueltig)\n", ax, ay, angle, n);
  return angle;
}
// Winkeldifferenz auf -180..+180 normieren
static float wrapDeg(float d) {
  while (d > 180.0f)  d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

// Liegt der Winkel im Zielfenster? Portrait: |a| < 0,5°. Landscape: 88°..135° (nur richtige Seite).
static bool mpuAtTarget(float a, bool portrait) {
  if (isnan(a)) return false;
  return portrait ? (fabsf(a) < PORTRAIT_ANGLE_THRESH)
                  : (a >= MPU_ANGLE_THRESHOLD && a < 135.0f);
}

// ─── Motor initialisieren ─────────────────────────────────────────────────────
void motorInit() {
  g_stepper.setMaxSpeed(MOTOR_MAX_SPEED);
  g_stepper.setAcceleration(MOTOR_ACCELERATION);
  g_stepper.disableOutputs();
  mlog("[Motor] initialisiert\n");
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
  mlogFlush();  // Motor steht → Log auf SD
}

// ─── Hilfsfunktion: IMU-geführt auf Zielwinkel drehen ─────────────────────────
// targetIsPortrait: true → Ziel 0° (aufrecht), false → Ziel +90° (richtiges Landscape)
// Richtung aus dem vorzeichenbehafteten Winkelfehler (kürzester Weg).
// Schritt-Konvention: DIR*(+1) = Winkel steigt (Portrait→Landscape, siehe MOTOR_DIR_INVERT).
// Nach ~5° wird die Richtung per IMU geprüft und ggf. umgekehrt — ohne Extra-Stopp wie in V10.
// Gibt gezählte Netto-Schritte zurück, -1 bei Fehler (Timeout / MPU).
static int32_t motorFindAngle(bool targetIsPortrait) {
  constexpr int8_t DIR = MOTOR_DIR_INVERT ? -1 : 1;
  const float TARGET_DEG   = targetIsPortrait ? 0.0f : 90.0f;
  const float SLOWDOWN_DEG = 5.0f;
  const char* label = targetIsPortrait ? "Portrait" : "Landscape";

  float a0 = mpuReadAngle(true);
  mlog("[Motor] Suche %s, Start-Winkel: %.1f°\n", label, a0);
  if (isnan(a0)) { mlog("[Motor] MPU-Lesefehler\n"); return -1; }
  if (mpuAtTarget(a0, targetIsPortrait)) {
    mlog("[Motor] Bereits bei %s (%.1f°)\n", label, a0);
    return 0;
  }

  int8_t dir = (wrapDeg(TARGET_DEG - a0) > 0.0f) ? 1 : -1;

  g_stepper.setCurrentPosition(0);
  g_stepper.setMaxSpeed(MOTOR_MAX_SPEED * 0.6f);
  g_stepper.setAcceleration(MOTOR_ACCELERATION);
  g_stepper.moveTo(DIR * dir * MOTOR_SEARCH_MAX);

  // Abbremsen (sanft, kein abrupter Stopp mehr) und in Gegenrichtung weiterfahren
  auto reverse = [&](const char* why, float a) {
    mlog("[Motor] %s bei %.1f° → Richtungsumkehr\n", why, a);
    dir = -dir;
    g_stepper.stop();
    while (g_stepper.distanceToGo() != 0) { g_stepper.run(); esp_task_wdt_reset(); }
    g_stepper.moveTo(DIR * dir * MOTOR_SEARCH_MAX);
  };

  bool     verified   = false;
  bool     slowedDown = false;
  bool     found      = false;
  bool     reversed   = false;
  long     lastCheck  = 0;
  uint32_t lastRamp   = 0;

  while (g_stepper.distanceToGo() != 0) {
    g_stepper.run();
    esp_task_wdt_reset();

    // V10 setzte setMaxSpeed(FINETUNE) direkt → AccelStepper springt sofort von 300 auf 60 st/s
    // (harter Ruck). Jetzt alle 20 ms um ACCELERATION×0,02 absenken = echte Bremsrampe.
    if (slowedDown && g_stepper.maxSpeed() > MOTOR_FINETUNE_SPEED && millis() - lastRamp >= 20) {
      lastRamp = millis();
      g_stepper.setMaxSpeed(fmaxf(MOTOR_FINETUNE_SPEED, g_stepper.maxSpeed() - MOTOR_ACCELERATION * 0.02f));
    }

    long cur = g_stepper.currentPosition();
    // Im Langsam-Modus 4× öfter prüfen, damit das schmale Portrait-Fenster (±0,5°) nicht übersprungen wird
    long interval = slowedDown ? MOTOR_CHECK_STEPS / 4 : MOTOR_CHECK_STEPS;
    if (labs(cur - lastCheck) < interval) continue;
    lastCheck = cur;

    float a = mpuReadAngle();
    if (isnan(a)) continue;
    float err = wrapDeg(TARGET_DEG - a);
    bool  atTarget = mpuAtTarget(a, targetIsPortrait);

    mlog("[Motor] Pos=%ld  Winkel=%.1f°%s\n", labs(cur), a, slowedDown ? " [langsam]" : "");

    // Richtungsprüfung nach ~5° Fahrt
    if (!verified && labs(cur) >= MOTOR_PROBE_STEPS) {
      verified = true;
      float moved = wrapDeg(a - a0);
      if (fabsf(moved) > 0.8f && (moved > 0.0f) != (dir > 0)) {
        reverse("Richtung falsch (MOTOR_DIR_INVERT / MPU_AX_SIGN pruefen)", a);
        reversed = true;
        continue;
      }
    }

    if (!slowedDown && fabsf(err) < SLOWDOWN_DEG) {
      slowedDown = true;
      mlog("[Motor] Naehert sich %s (%.1f°) → langsam\n", label, a);
    }

    if (atTarget) {
      g_stepper.stop();
      while (g_stepper.distanceToGo() != 0) { g_stepper.run(); esp_task_wdt_reset(); }
      delay(MPU_SETTLE_MS);
      esp_task_wdt_reset();
      float aSettle = mpuReadAngle();
      bool settled = mpuAtTarget(aSettle, targetIsPortrait);
      mlog("[Motor] Settle: %.1f° → %s\n", aSettle, settled ? "OK" : "zurueckgewippen");
      if (settled) { found = true; break; }
      // Settle fehlgeschlagen: weiterfahren (langsam)
      g_stepper.moveTo(DIR * dir * MOTOR_SEARCH_MAX);
      lastCheck = g_stepper.currentPosition();
    } else if (slowedDown && (err > 0.0f) != (dir > 0)) {
      reverse("Ziel ueberfahren", a);  // nahe am Ziel vorbeigerutscht
    }
  }

  g_stepper.stop();
  while (g_stepper.distanceToGo() != 0) { g_stepper.run(); esp_task_wdt_reset(); }

  int32_t totalSteps = labs(g_stepper.currentPosition());  // Netto-Schritte
  if (found) {
    mlog("[Motor] %s stabil! Gesamt=%ld Schritte%s\n",
                  label, (long)totalSteps, reversed ? " (nach Richtungsumkehr)" : "");
    return totalSteps;
  }
  mlog("[Motor] WARNUNG: %s nicht gefunden (Timeout nach %ld Schritten)\n", label, (long)totalSteps);
  return -1;
}

// ─── Kalibrierphase setzen ────────────────────────────────────────────────────
static void setCalibPhase(volatile char* out, const char* phase) {
  if (out) strncpy((char*)out, phase, 31);
  mlog("[Calib] Phase: %s\n", phase);
  mlogFlush();  // zwischen den Phasen steht der Motor
}

// ─── Vollkalibrierung ─────────────────────────────────────────────────────────
// Erkennt Startorientierung per IMU → grüner LED-Flash, richtet exakt aus.
// Dreht 90° in Gegenrichtung und zählt die Schritte.
// Dreht zurück zur Startorientierung.
// phaseOut: optionaler char[32]-Puffer für Phasenstatus (für /api/status)
// endedInPortrait: optionaler Ausgabeparameter für Endorientierung
// Gibt gemessene Schritte zurück, 0 bei Fehler.
int32_t motorCalibrateFull(volatile char* phaseOut = nullptr, bool* endedInPortrait = nullptr) {
  mlog("[Motor] === Vollkalibrierung ===\n");
  mpuInit();
  g_stepper.enableOutputs();

  // ── Phase 1: Startorientierung erkennen ──────────────────────────────────
  setCalibPhase(phaseOut, "orient_detect");
  delay(100);
  esp_task_wdt_reset();

  float startAngle = mpuReadAngle(true);
  if (isnan(startAngle)) {
    mlog("[Motor] Kalibrierung fehlgeschlagen: MPU nicht lesbar\n");
    motorPowerOff();
    return 0;
  }

  // Nur 45°..135° gilt als Landscape. Alles andere (auch "auf dem Kopf" / falsches
  // Landscape) wird zu Portrait aufrecht ausgerichtet.
  bool startInPortrait = !(startAngle >= 45.0f && startAngle <= 135.0f);

  mlog("[Calib] Start: %.1f° → %s\n", startAngle, startInPortrait ? "Portrait" : "Landscape");

  // Kurzer grüner Blitz: Orientierung erkannt
  ledFlash(0, 255, 0, 200);
  delay(200);
  esp_task_wdt_reset();

  setCalibPhase(phaseOut, startInPortrait ? "portrait_found" : "landscape_found");
  delay(500);
  esp_task_wdt_reset();

  // ── Phase 1.2: Auf exakt 0° oder 90° ausrichten ──────────────────────────
  // Nötig wenn Rahmen schief oder auf dem Kopf steht.
  setCalibPhase(phaseOut, "align_start");
  ledSet(0, 60, 140); // Dunkelblau: Ausrichten
  {
    int32_t alignSteps = motorFindAngle(startInPortrait);
    mlog("[Calib] Ausrichtung: %ld Schritte\n", (long)alignSteps);
    if (alignSteps < 0) {
      mlog("[Motor] Kalibrierung fehlgeschlagen: Ausrichtung nicht moeglich\n");
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

  int32_t steps90 = motorFindAngle(!startInPortrait);
  if (steps90 < 0) {
    mlog("[Motor] Kalibrierung fehlgeschlagen: %s nicht gefunden\n",
                  startInPortrait ? "Landscape" : "Portrait");
    ledSet(0,0,0);
    motorPowerOff();
    return 0;
  }

  delay(200);
  esp_task_wdt_reset();

  // ── Phase 3: Zurück zur Startorientierung ────────────────────────────────
  setCalibPhase(phaseOut, "rotating_back");
  ledSet(0, 80, 180); // Blau (dunkler): Rückdrehung

  int32_t stepsBack = motorFindAngle(startInPortrait);
  mlog("[Calib] Rueckdrehung: %ld Schritte\n", (long)stepsBack);
  if (endedInPortrait) *endedInPortrait = startInPortrait;

  // 90° müssen grob MOTOR_STEPS_DEFAULT Schritte sein. Deutlich weniger/mehr heißt:
  // Fehlmessung oder der Rahmen ist mechanisch gerutscht → Wert nicht speichern.
  bool plausible = steps90 > MOTOR_STEPS_DEFAULT * 3 / 4 && steps90 < MOTOR_STEPS_DEFAULT * 5 / 4;
  bool ok = plausible && stepsBack >= 0;
  if (!plausible)
    mlog("[Motor] FEHLER: %ld Schritte fuer 90° unplausibel (erwartet ~%d) – Fehlmessung oder Rahmen gerutscht\n",
         (long)steps90, MOTOR_STEPS_DEFAULT);
  if (stepsBack < 0)
    mlog("[Motor] FEHLER: Rueckdrehung zur Startlage fehlgeschlagen\n");
  if (ok) mlog("[Motor] Kalibrierung OK: %ld Schritte fuer 90°\n", (long)steps90);

  setCalibPhase(phaseOut, ok ? "done" : "failed");
  ledSet(0,0,0);
  motorPowerOff();
  return ok ? steps90 : 0;
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
  mlog("[Motor] %s → %s (%ld Schritte + Feinjustierung)\n",
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
  const float TARGET_DEG = toLandscape ? 90.0f : 0.0f;
  float angle = mpuReadAngle(true);
  mlog("[Motor] Nach Schnelldrehung: %.1f° (Soll %s)\n",
                angle, toLandscape ? "88..135°" : "|a| < 0,5°");

  bool atTarget = mpuAtTarget(angle, !toLandscape);

  // ── IMU-Feinjustierung (kontinuierlich) ──────────────────────────────────
  // Motor läuft durch ohne Chunk-Stop — IMU wird inline gelesen wie in motorFindAngle.
  // Settle-Vorprüfung: Hauptdrehung hat Ziel bereits erreicht → bestätigen und fertig.
  if (atTarget) {
    delay(MPU_SETTLE_MS);
    esp_task_wdt_reset();
    angle = mpuReadAngle();
    atTarget = mpuAtTarget(angle, !toLandscape);
    mlog("[Motor] Settle nach Hauptdrehung: %.1f° → %s\n", angle, atTarget ? "OK" : "Feinjustierung noetig");
    if (atTarget) { motorPowerOff(); return true; }
  }

  mlog("[Motor] Feinjustierung...\n");
  g_stepper.setMaxSpeed(MOTOR_FINETUNE_SPEED);
  g_stepper.setAcceleration(MOTOR_ACCELERATION);

  // Richtung aus vorzeichenbehaftetem Fehler (Winkel steigt bei DIR*(+1)); ohne MPU: Standardrichtung
  float   err        = isnan(angle) ? (toLandscape ? 1.0f : -1.0f) : wrapDeg(TARGET_DEG - angle);
  int8_t  fineDir    = DIR * (err > 0.0f ? 1 : -1);
  float   prevErr    = err;
  bool    dirFlipped = false;

  g_stepper.setCurrentPosition(0);
  g_stepper.moveTo((long)fineDir * MOTOR_FINETUNE_MAX);
  int32_t lastCheck = 0;

  while (g_stepper.distanceToGo() != 0) {
    g_stepper.run();
    esp_task_wdt_reset();

    // 1/4 Intervall: schmales Portrait-Fenster (±0,5°) nicht überspringen
    int32_t pos = abs(g_stepper.currentPosition());
    if (pos - lastCheck < MOTOR_CHECK_STEPS / 4) continue;
    lastCheck = pos;

    angle = mpuReadAngle();
    if (isnan(angle)) continue;

    atTarget = mpuAtTarget(angle, !toLandscape);
    err = wrapDeg(TARGET_DEG - angle);
    mlog("[Motor] Fein: %ld Schr, %.1f°\n", (long)pos, angle);

    bool gettingCloser = fabsf(err) < fabsf(prevErr);
    if (!atTarget && !dirFlipped && fabsf(err - prevErr) > 0.4f && !gettingCloser) {
      fineDir   = -fineDir;
      dirFlipped = true;
      g_stepper.stop();
      while (g_stepper.distanceToGo() != 0) { g_stepper.run(); esp_task_wdt_reset(); }
      g_stepper.setCurrentPosition(0);
      lastCheck = 0;
      g_stepper.moveTo((long)fineDir * MOTOR_FINETUNE_MAX);
      mlog("[Motor] Fein: Richtungsumkehr bei %.1f°\n", angle);
    }
    prevErr = err;

    if (atTarget) {
      g_stepper.stop();
      while (g_stepper.distanceToGo() != 0) { g_stepper.run(); esp_task_wdt_reset(); }
      delay(MPU_SETTLE_MS);
      esp_task_wdt_reset();
      angle = mpuReadAngle();
      atTarget = mpuAtTarget(angle, !toLandscape);
      mlog("[Motor] Settle: %.1f° → %s\n", angle, atTarget ? "OK" : "zurueckgewippen");
      if (atTarget) break;
      g_stepper.setCurrentPosition(0);
      lastCheck = 0;
      g_stepper.moveTo((long)fineDir * MOTOR_FINETUNE_MAX);
    }
  }

  motorPowerOff();

  if (!atTarget)
    mlog("[Motor] WARNUNG: Ziel nicht erreicht (Winkel=%.1f°)\n", angle);
  return atTarget;
}

// ─── Manuelle Motorsteuerung ──────────────────────────────────────────────────
// steps: positive = vorwärts, negative = rückwärts (kein DIR-Invert angewandt)
// Für die 3D-Drucker-artige UI-Steuerung — keine IMU-Feinjustierung.
void motorManualMove(int32_t steps) {
  mlog("[Motor] Manuell: %ld Schritte\n", (long)steps);
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
