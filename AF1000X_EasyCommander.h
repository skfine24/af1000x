#ifndef AF1000X_EASY_COMMANDER_H
#define AF1000X_EASY_COMMANDER_H

#include <Arduino.h>

/* ============================================================================
 * KO: EasyCommander (AF1000X ÏΩîÏñ¥ ò∏ôò)
 * EN: EasyCommander (AF1000X core compatible)
 * KO: currentMode Í∏∞Î∞ò, Î™®Îìú ÉÅàòäî ÏΩîÏñ¥ èôùºï¥ïº ï®
 * EN: Uses currentMode; mode constants must match core
 * ============================================================================
 */

// ============================================================================
// KO: ÏΩîÏñ¥ extern (AF1000X_CORE.h †úÍ≥)
// EN: Core externs (from AF1000X_CORE.h)
// ============================================================================
extern volatile uint8_t currentMode;    // KO: ÏΩîÏñ¥ ÉÅÉú / EN: core state
extern bool flightLock;                 // KO: ÑºÑú ã§å® ÎπÑÌñâ Í∏àÏ / EN: flight lock

extern float targetAltitude, currentAltitude;
extern float targetPosX, targetPosY, targetYaw;
extern float currentPosX, currentPosY, currentYaw;

extern float moveSpeedSteps[3];
extern float climbSpeedSteps[3];
extern float yawSpeedSteps[3];
extern int   speedLevel;

extern void updateSystem();
extern void updateFlight();

extern void autoTakeoff();
extern void autoLanding();
extern void emergencyStop();

// ============================================================================
// KO: MODE_* Í∞íÏ ÏΩîÏñ¥ èôùºï¥ïº ï®
// EN: MODE_* values must match core
// ============================================================================
#ifndef READY
  #define READY      0
  #define TAKEOFF    1
  #define HOVERING   2
  #define LANDING    3
  #define EMERGENCY  4
#endif

// ============================================================================
// KO: Ç¥Î∂ ú†ã∏ (†úñ¥ Î£®ÌîÑÎ• Î©àÏ∂îÏß ïääî Í∏)
// EN: Internal util (wait without stopping control loop)
// ============================================================================
static inline void _airgo_yield(unsigned long ms = 0) {
  if (ms == 0) {
    updateSystem();
    updateFlight();
    delay(20);
    return;
  }

  unsigned long start = millis();
  while (millis() - start < ms) {
    updateSystem();
    updateFlight();
    delay(20);
    if (currentMode == EMERGENCY) return;
  }
}

// ============================================================================
// KO: ïà†Ñ Í∞ìú
// EN: Safety guard
// ============================================================================
static inline bool _airgo_readyToFly() {
  if (flightLock) return false;
  if (currentMode == EMERGENCY) return false;
  return true;
}

// ============================================================================
// KO: 1) Í∏∞Î≥∏ ÎπÑÌñâ
// EN: 1) Core flight
// ============================================================================
static inline void takeoff() {
  if (!_airgo_readyToFly()) return;
  if (currentMode != READY) return;

  LOG_CMD_PRINTLN("[CMD] Takeoff");
  autoTakeoff();                // KO: hover ïôäµ takeoff / EN: hover-learn takeoff

  // KO: TAKEOFF Ï¢ÖÎ£åÍπåÏ Í∏
  // EN: Wait until TAKEOFF completes
  while (currentMode == TAKEOFF) _airgo_yield(0);

  // KO: HOVERING ÏßÑÏûÖ ã§å® ãú Ï¢ÖÎ£å
  // EN: Exit if not HOVERING
}

static inline void land() {
  if (currentMode == READY) return;

  LOG_CMD_PRINTLN("[CMD] Land");
  autoLanding();

  while (currentMode == LANDING) _airgo_yield(0);
}

static inline void stay(float sec) {
  if (sec <= 0) return;
  LOG_CMD_PRINTF("[CMD] Stay %.2f sec\n", sec);
  _airgo_yield((unsigned long)(sec * 1000.0f));
}

static inline void killMotors() {
  LOG_CMD_PRINTLN("[CMD] KILL MOTORS!");
  emergencyStop();
}

// ============================================================================
// KO: 2) CM ù¥èô Î™ÖÎ†π (†ïôïèÑäî ÑºÑú/úµï© Ñ±ä•óê ùòÏ°)
// EN: 2) CM moves (accuracy depends on sensors/fusion)
// ============================================================================
static inline void forwardCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  LOG_CMD_PRINTF("[CMD] Forward %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  // KO: ó§î©(yaw) Í∏∞Ï †ÑÏß
  // EN: Forward in heading frame
  float rad = targetYaw * DEG_TO_RAD;
  float moved = 0.0f;

  while (moved < dM) {
    float step = (moveSpeedSteps[speedLevel] * p) * 0.02f;  // KO: 50Hz Í∏∞Ï / EN: 50Hz base
    if (step < 0.001f) step = 0.001f;

    targetPosX += step * cosf(rad);
    targetPosY += step * sinf(rad);

    moved += step;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

static inline void backwardCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  LOG_CMD_PRINTF("[CMD] Backward %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float rad = targetYaw * DEG_TO_RAD;
  float moved = 0.0f;

  while (moved < dM) {
    float step = (moveSpeedSteps[speedLevel] * p) * 0.02f;
    if (step < 0.001f) step = 0.001f;

    targetPosX -= step * cosf(rad);
    targetPosY -= step * sinf(rad);

    moved += step;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

static inline void rightCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  LOG_CMD_PRINTF("[CMD] Right %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float rad = (targetYaw + 90.0f) * DEG_TO_RAD; // KO: ò§Î•∏Ï™Ω / EN: right
  float moved = 0.0f;

  while (moved < dM) {
    float step = (moveSpeedSteps[speedLevel] * p) * 0.02f;
    if (step < 0.001f) step = 0.001f;

    targetPosX += step * cosf(rad);
    targetPosY += step * sinf(rad);

    moved += step;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

static inline void leftCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  LOG_CMD_PRINTF("[CMD] Left %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float rad = (targetYaw - 90.0f) * DEG_TO_RAD; // KO: ôºÏ™ / EN: left
  float moved = 0.0f;

  while (moved < dM) {
    float step = (moveSpeedSteps[speedLevel] * p) * 0.02f;
    if (step < 0.001f) step = 0.001f;

    targetPosX += step * cosf(rad);
    targetPosY += step * sinf(rad);

    moved += step;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

static inline void upCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  LOG_CMD_PRINTF("[CMD] Up %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float startAlt = targetAltitude;
  while (targetAltitude < startAlt + dM) {
    targetAltitude += (climbSpeedSteps[speedLevel] * p) * 0.02f;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

static inline void downCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  LOG_CMD_PRINTF("[CMD] Down %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float startAlt = targetAltitude;
  while (targetAltitude > startAlt - dM) {
    targetAltitude -= (climbSpeedSteps[speedLevel] * p) * 0.02f;
    if (targetAltitude < 0.1f) targetAltitude = 0.1f;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

// ============================================================================
// KO: 3) öå†Ñ
// EN: 3) Rotation
// ============================================================================
static inline void turnAngle(float deg, float pwr) {
  if (currentMode != HOVERING) return;
  if (deg == 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;

  LOG_CMD_PRINTF("[CMD] TurnAngle %.1f deg (pwr=%.0f%%)\n", deg, pwr);

  float start = targetYaw;
  float goal  = start + deg;

  // KO: Í∞ÅÎèÑ ûòïë
  // EN: wrap
  while (goal > 180.0f) goal -= 360.0f;
  while (goal < -180.0f) goal += 360.0f;

  float dir = (deg > 0) ? 1.0f : -1.0f;

  while (fabsf(targetYaw - goal) > 1.0f) {
    targetYaw += dir * (yawSpeedSteps[speedLevel] * p) * 0.02f;

    // KO: Í∞ÅÎèÑ ûòïë
    // EN: wrap
    while (targetYaw > 180.0f) targetYaw -= 360.0f;
    while (targetYaw < -180.0f) targetYaw += 360.0f;

    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }

  targetYaw = goal;
  _airgo_yield(200);
}

static inline void turnTime(float sec, float pwr) {
  if (currentMode != HOVERING) return;
  if (sec <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;

  LOG_CMD_PRINTF("[CMD] TurnTime %.2f sec (pwr=%.0f%%)\n", sec, pwr);

  unsigned long ms = (unsigned long)(sec * 1000.0f);
  unsigned long start = millis();

  // KO: Ïß†ï ãúÍ∞ èôïà ïú Î∞©Ìñ• öå†Ñ òàãú
  // EN: Example: rotate in one direction for duration
  while (millis() - start < ms) {
    targetYaw += (yawSpeedSteps[speedLevel] * p) * 0.02f;
    while (targetYaw > 180.0f) targetYaw -= 360.0f;
    while (targetYaw < -180.0f) targetYaw += 360.0f;

    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

#endif // KO: AF1000X_EASY_COMMANDER_H / EN: AF1000X_EASY_COMMANDER_H