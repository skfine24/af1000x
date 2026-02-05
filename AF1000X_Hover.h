#ifndef AF1000X_HOVER_H
#define AF1000X_HOVER_H

#include <Arduino.h>
#include <Preferences.h>
#include "AF1000X_PID.h"

/* ============================================================================
 * KO: Hover PWM 자동 학습 (A 방식)
 * EN: Hover PWM auto learning (A method)
 * KO: hoverThrottle 저장/로드 + 비행 중 점진 보정
 * EN: Store/load hoverThrottle + refine during flight
 * ============================================================================
 */

// ============================================================================
// KO: 코어 extern (AF1000X_CORE.h에 존재해야 함)
// EN: Core externs (must exist in AF1000X_CORE.h)
// ============================================================================
extern volatile uint8_t currentMode; // KO: uint8_t extern 허용 / EN: uint8_t extern ok
extern bool flightLock;
extern bool failsafe;

extern float currentAltitude; // KO: 융합 고도(m) / EN: fused altitude (m)
extern float targetAltitude;  // KO: 목표 고도(m) / EN: target altitude (m)
extern int   hoverThrottle;   // KO: 0..255 베이스 PWM / EN: 0..255 base PWM

extern Preferences prefs;

// KO: 저장 키
// EN: NVS key
static constexpr const char* KEY_HOVER_PWM = "hoverPWM";

// ============================================================================
// KO: Hover 상태 머신
// EN: Hover state machine
// ============================================================================
enum HoverLearnState : uint8_t {
  HL_IDLE = 0,
  HL_RAMP,
  HL_LIFTOFF_CONFIRM,
  HL_ASCEND_TO_1M,
  HL_STABILIZE,
  HL_READY,
  HL_ABORT
};

static HoverLearnState hl_state = HL_IDLE;

static uint32_t hl_t0_ms = 0;
static uint32_t hl_lastRamp_ms = 0;
static uint32_t hl_stableStart_ms = 0;

static float hl_alt0 = 0.0f;
static float hl_altPrev = 0.0f;
static float hl_vel = 0.0f;

static int hl_rampPwm = 0;
static int hl_liftoffPwm = 0;

static bool hl_active = false;
static bool hl_ready = false;

// ============================================================================
// KO: 내부 유틸
// EN: Internal utils
// ============================================================================
static inline int _clampi(int v, int lo, int hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline float _absf(float v){ return (v < 0) ? -v : v; }

static inline float _wrap01(float v){
  if(v < 0.0f) return 0.0f;
  if(v > 1.0f) return 1.0f;
  return v;
}

// ============================================================================
// KO: API
// EN: API
// ============================================================================

// KO: 호출 시점 = initAF1000X()에서 prefs.begin() 이후
// EN: Call after prefs.begin() in initAF1000X()
static inline void hover_begin() {
  int saved = prefs.getInt(KEY_HOVER_PWM, -1);

  if (saved >= 0 && saved <= 255) {
    hoverThrottle = saved;
  } else {
    // KO: 초기 안전 기본값 / EN: safe default
    hoverThrottle = HOVER_PWM_DEFAULT;
  }

  hl_state = HL_IDLE;
  hl_active = false;
  hl_ready = false;
}

// KO: takeoff()/autoTakeoff()에서 호출 가능
// EN: Can be called from takeoff() or autoTakeoff()
static inline void hover_startLearn() {
  if (flightLock) {
    hl_state = HL_ABORT;
    hl_active = false;
    hl_ready = false;
    return;
  }

  hl_active = true;
  hl_ready = false;

  hl_state = HL_RAMP;
  hl_t0_ms = millis();
  hl_lastRamp_ms = millis();
  hl_stableStart_ms = 0;

  hl_alt0 = currentAltitude;
  hl_altPrev = currentAltitude;
  hl_vel = 0.0f;

  // KO: 낮은 PWM에서 시작해 리프트오프 PWM 탐색
  // EN: Start low PWM and ramp to find liftoff PWM
  hl_rampPwm = _clampi(hoverThrottle - 25, RAMP_MIN_PWM, RAMP_MAX_PWM);
  hl_liftoffPwm = hl_rampPwm;

  // KO: TAKEOFF 모드로 전환 (코어 enum과 동일해야 함)
  // EN: Switch to TAKEOFF mode (enum must match core)
  if (currentMode == 0) { // KO: READY / EN: READY
    currentMode = 1;      // KO: TAKEOFF / EN: TAKEOFF
  }

  // KO: 목표 고도 1m
  // EN: Target altitude 1m
  targetAltitude = HOVER_TARGET_M;
}

// KO: hover 학습/상태 업데이트 (loop 50Hz 권장)
// EN: hover learn/state update (50Hz loop recommended)
static inline void hover_update() {
  // KO: 속도 추정 / EN: velocity estimate
  float alt = currentAltitude;
  hl_vel = (alt - hl_altPrev) / 0.02f; // KO: 50Hz 가정 / EN: assuming 50Hz
  hl_altPrev = alt;

  // KO: 실패 조건 (센서 락/통신 끊김/비상)
  // EN: Abort on lock/failsafe/emergency
  if (flightLock || failsafe || currentMode == 4 /*EMERGENCY*/) {
    hl_state = HL_ABORT;
  }

  switch (hl_state) {
    case HL_IDLE:
      hl_active = false;
      hl_ready  = false;
      break;

    case HL_RAMP: {
      // KO: 램프업으로 리프트오프 PWM 탐색
      // EN: Ramp up to find liftoff PWM
      uint32_t now = millis();
      if (now - hl_lastRamp_ms >= RAMP_STEP_MS) {
        hl_lastRamp_ms = now;
        hl_rampPwm += RAMP_STEP_PWM;
        hl_rampPwm = _clampi(hl_rampPwm, RAMP_MIN_PWM, RAMP_MAX_PWM);
        hoverThrottle = hl_rampPwm;
      }

      // KO: 리프트오프 감지 (상승 + 상승속도)
      // EN: Liftoff detect (rise + velocity)
      float rise = alt - hl_alt0;
      bool liftoff = (rise > LIFTOFF_RISE_M) && (hl_vel > LIFTOFF_VEL_MPS);

      if (liftoff) {
        hl_liftoffPwm = hoverThrottle;
        hl_state = HL_LIFTOFF_CONFIRM;
        hl_t0_ms = millis();
      }

      // KO: 타임아웃 -> 중단
      // EN: Timeout -> abort
      if (millis() - hl_t0_ms > 6000) {
        hl_state = HL_ABORT;
      }
    } break;

    case HL_LIFTOFF_CONFIRM: {
      // KO: 오탐 방지(약 400ms 확인)
      // EN: Avoid false lift (confirm ~400ms)
      bool stillUp = (alt - hl_alt0 > LIFTOFF_RISE_M) || (hl_vel > 0.02f);

      if (!stillUp) {
        // KO: 오탐 -> 다시 램프
        // EN: False lift -> back to ramp
        hl_alt0 = alt;
        hl_state = HL_RAMP;
        hl_lastRamp_ms = millis();
        break;
      }

      if (millis() - hl_t0_ms > 400) {
        // KO: 리프트오프 PWM 확정
        // EN: Confirm liftoff PWM
        hoverThrottle = hl_liftoffPwm;
        hl_state = HL_ASCEND_TO_1M;
        hl_t0_ms = millis();
        targetAltitude = HOVER_TARGET_M;
      }
    } break;

    case HL_ASCEND_TO_1M: {
      // KO: 코어 고도 제어가 1m로 상승하도록 둠
      // EN: Let core altitude control rise to 1m
      float err = targetAltitude - alt;

      // KO: 충분히 가까우면 안정화 단계
      // EN: Close enough -> stabilize
      if (_absf(err) < 0.10f) {
        hl_state = HL_STABILIZE;
        hl_stableStart_ms = 0;
      }

      // KO: 타임아웃 -> abort
      // EN: Timeout -> abort
      if (millis() - hl_t0_ms > 8000) {
        hl_state = HL_ABORT;
      }
    } break;

    case HL_STABILIZE: {
      float err = targetAltitude - alt;
      bool okBand = _absf(err) < HOVER_OK_BAND_M;
      bool okVel  = _absf(hl_vel) < HOVER_OK_VEL_MPS;

      if (okBand && okVel) {
        if (hl_stableStart_ms == 0) hl_stableStart_ms = millis();
        if (millis() - hl_stableStart_ms >= HOVER_STABLE_MS) {
          // KO: 안정 호버 성공
          // EN: Stable hover success
          hl_state = HL_READY;
          hl_ready = true;
          hl_active = false;

          // KO: HOVERING 모드 진입
          // EN: Enter HOVERING mode
          currentMode = 2;

          // KO: 저장 / EN: save
          prefs.putInt(KEY_HOVER_PWM, hoverThrottle);
        }
      } else {
        hl_stableStart_ms = 0;
      }

      // KO: 안정화 타임아웃 -> abort
      // EN: Stabilize timeout -> abort
      if (millis() - hl_t0_ms > 12000) {
        hl_state = HL_ABORT;
      }
    } break;

    case HL_READY:
      hl_ready = true;
      hl_active = false;
      break;

    case HL_ABORT:
    default:
      hl_active = false;
      hl_ready = false;
      // KO: 학습 실패 시 LANDING 유도
      // EN: On failure, steer to LANDING
      if (currentMode == 1 || currentMode == 2) {
        currentMode = 3; // KO: LANDING / EN: LANDING
      }
      break;
  }
}

// KO: updateFlight()의 zCmd로 hoverThrottle 보정 (매 tick)
// EN: Adjust hoverThrottle with zCmd from updateFlight() (every tick)
static inline void hover_onZcmd(float zCmd) {
  // KO: 학습/호버 중에만 천천히 보정
  // EN: Slow adjust only during learn/hover
  float gain = 0.0f;

  if (hl_state == HL_ASCEND_TO_1M || hl_state == HL_STABILIZE) {
    gain = LEARN_GAIN_ASCEND;
  } else if (currentMode == 2 /*HOVERING*/ && hl_state != HL_ABORT) {
    gain = LEARN_GAIN_HOVER;
  } else {
    return;
  }

  // KO: zCmd 방향으로 hoverThrottle을 소폭 이동
  // EN: Nudge hoverThrottle toward zCmd direction
  float delta = zCmd * 100.0f * gain;
  // KO: 급변 제한
  // EN: Clamp step
  if (delta >  1.0f) delta =  1.0f;
  if (delta < -1.0f) delta = -1.0f;

  hoverThrottle = _clampi((int)lroundf((float)hoverThrottle + delta), 60, 230);
}

// KO: 외부 상태 확인
// EN: External state access
static inline bool hover_isLearning() { return hl_active; }
static inline bool hover_isReady()    { return hl_ready; }
static inline uint8_t hover_state()   { return (uint8_t)hl_state; }

// KO: 필요 시 학습 강제 종료
// EN: Force abort if needed
static inline void hover_abort() {
  hl_state = HL_ABORT;
  hl_active = false;
  hl_ready = false;
}

#endif // KO: AF1000X_HOVER_H / EN: AF1000X_HOVER_H
