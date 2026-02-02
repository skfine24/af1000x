#ifndef AF1000X_EASY_COMMANDER_H
#define AF1000X_EASY_COMMANDER_H

#include <Arduino.h>

/* ============================================================================
 * AIRGO_EasyCommander.h  (AF1000X 코어 호환 버전)
 * ----------------------------------------------------------------------------
 * - 기존 "FlightMode mode" 대신, 코어의 "currentMode" (uint8_t)를 사용
 * - READY/TAKEOFF/HOVERING/LANDING/EMERGENCY 상수도 코어 값에 맞춰 정의
 * - 학생은 메인에서 takeoff(); forwardCm(10,90); 처럼 쓰면 됨
 * ========================================================================== */

// ---------------------------------------------------------------------------
// 코어(AF1000X.h)에서 제공되는 전역 변수/함수들
// ---------------------------------------------------------------------------
extern volatile uint8_t currentMode;    // 코어의 상태 변수
extern bool flightLock;                 // 센서 실패 비행 금지

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

// ---------------------------------------------------------------------------
// 코어의 MODE_* 값과 맞추기 (AF1000X.h와 동일해야 함)
// (AF1000X.h에서 READY=0, TAKEOFF=1, HOVERING=2, LANDING=3, EMERGENCY=4 로 사용중)
// ---------------------------------------------------------------------------
#ifndef READY
  #define READY      0
  #define TAKEOFF    1
  #define HOVERING   2
  #define LANDING    3
  #define EMERGENCY  4
#endif

// ---------------------------------------------------------------------------
// 내부 유틸: 제어 루프를 멈추지 않고 기다리는 함수
// - ms=0 이면 “한 틱만” 돌리고 리턴(루프 안에서 계속 호출하는 방식)
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// 안전 가드
// ---------------------------------------------------------------------------
static inline bool _airgo_readyToFly() {
  if (flightLock) return false;
  if (currentMode == EMERGENCY) return false;
  return true;
}

// ===========================================================================
// 1) Core Flight
// ===========================================================================
static inline void takeoff() {
  if (!_airgo_readyToFly()) return;
  if (currentMode != READY) return;

  Serial.println("[CMD] Takeoff");
  autoTakeoff();                // 코어의 hover-learn takeoff를 시작

  // TAKEOFF 상태가 끝날 때까지(=HOVERING 들어갈 때까지) 계속 제어
  while (currentMode == TAKEOFF) _airgo_yield(0);

  // TAKEOFF가 끝났는데도 HOVERING이 아니면(실패/락 등) 그냥 종료
}

static inline void land() {
  if (currentMode == READY) return;

  Serial.println("[CMD] Land");
  autoLanding();

  while (currentMode == LANDING) _airgo_yield(0);
}

static inline void stay(float sec) {
  if (sec <= 0) return;
  Serial.printf("[CMD] Stay %.2f sec\n", sec);
  _airgo_yield((unsigned long)(sec * 1000.0f));
}

static inline void killMotors() {
  Serial.println("[CMD] KILL MOTORS!");
  emergencyStop();
}

// ===========================================================================
// 2) CM 이동 명령 (정확도는 OpticalFlow/융합 성능에 의해 결정됨)
// - 목표좌표를 “조금씩” 이동시키며 updateFlight가 따라가게 한다.
// - pwr(%)는 속도 계수
// ===========================================================================
static inline void forwardCm(float cm, float pwr) {
  if (currentMode != HOVERING) return;
  if (cm <= 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;
  float dM = cm * 0.01f;

  Serial.printf("[CMD] Forward %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  // 헤딩(yaw) 기준으로 전진
  float rad = targetYaw * DEG_TO_RAD;
  float moved = 0.0f;

  while (moved < dM) {
    float step = (moveSpeedSteps[speedLevel] * p) * 0.02f;  // 50Hz 기준
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

  Serial.printf("[CMD] Backward %.1f cm (pwr=%.0f%%)\n", cm, pwr);

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

  Serial.printf("[CMD] Right %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float rad = (targetYaw + 90.0f) * DEG_TO_RAD; // 오른쪽
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

  Serial.printf("[CMD] Left %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float rad = (targetYaw - 90.0f) * DEG_TO_RAD; // 왼쪽
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

  Serial.printf("[CMD] Up %.1f cm (pwr=%.0f%%)\n", cm, pwr);

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

  Serial.printf("[CMD] Down %.1f cm (pwr=%.0f%%)\n", cm, pwr);

  float startAlt = targetAltitude;
  while (targetAltitude > startAlt - dM) {
    targetAltitude -= (climbSpeedSteps[speedLevel] * p) * 0.02f;
    if (targetAltitude < 0.1f) targetAltitude = 0.1f;
    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

// ===========================================================================
// 3) 회전
// ===========================================================================
static inline void turnAngle(float deg, float pwr) {
  if (currentMode != HOVERING) return;
  if (deg == 0) return;

  float p = constrain(pwr, 1.0f, 100.0f) / 100.0f;

  Serial.printf("[CMD] TurnAngle %.1f deg (pwr=%.0f%%)\n", deg, pwr);

  float start = targetYaw;
  float goal  = start + deg;

  // wrap
  while (goal > 180.0f) goal -= 360.0f;
  while (goal < -180.0f) goal += 360.0f;

  float dir = (deg > 0) ? 1.0f : -1.0f;

  while (fabsf(targetYaw - goal) > 1.0f) {
    targetYaw += dir * (yawSpeedSteps[speedLevel] * p) * 0.02f;

    // wrap
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

  Serial.printf("[CMD] TurnTime %.2f sec (pwr=%.0f%%)\n", sec, pwr);

  unsigned long ms = (unsigned long)(sec * 1000.0f);
  unsigned long start = millis();

  // 시간 동안 계속 같은 방향 회전(+)로 예시
  while (millis() - start < ms) {
    targetYaw += (yawSpeedSteps[speedLevel] * p) * 0.02f;
    while (targetYaw > 180.0f) targetYaw -= 360.0f;
    while (targetYaw < -180.0f) targetYaw += 360.0f;

    _airgo_yield(0);
    if (currentMode == EMERGENCY) break;
  }
}

#endif // AF1000X_EASY_COMMANDER_H
