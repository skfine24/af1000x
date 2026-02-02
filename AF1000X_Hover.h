#ifndef AF1000X_HOVER_H
#define AF1000X_HOVER_H

#include <Arduino.h>
#include <Preferences.h>

/* ============================================================================
 * AF1000X_Hover.h
 * ----------------------------------------------------------------------------
 * 목적:
 *  - 자동 Hover PWM 학습 (A 방식)
 *  - 1m 도달 후 안정 호버링
 *  - hoverPWM 값을 Preferences(NVS)에 저장/로드
 *  - 비행 중에도 천천히 보정(학습)해서 점점 좋아지게
 *
 * 전제(코어에서 제공되어야 하는 것):
 *  - currentMode (READY/TAKEOFF/HOVERING/LANDING/EMERGENCY)
 *  - flightLock (센서 실패 시 true)
 *  - currentAltitude (m) : ToF+Baro 융합된 현재 고도
 *  - targetAltitude  (m) : 목표 고도
 *  - hoverThrottle   (int 0~255): 고도 제어의 베이스 PWM
 *  - prefs (Preferences) : 이미 begin("af1000x",false) 되어 있어야 함
 *  - failsafe (옵션) : RF failsafe 시 true면 학습 중단
 *
 * 코어와 연결(필수 2줄):
 *  1) initAF1000X() 끝쪽에서: hover_begin();
 *  2) updateFlight()에서 zCmd 계산 후: hover_onZcmd(zCmd);
 *     그리고 loop에서 매 tick: hover_update();
 *
 * ※ zCmd는 코어에서 쓰는 고도 PD 출력(예: m 단위)이라고 가정.
 *    기존 코드가 thr = hoverThrottle + (zCmd * 100) 이면 잘 맞음.
 * ========================================================================== */

// ---------------------------------------------------------------------------
// 코어 extern (이 이름들은 코어(AF1000X.h)에 이미 존재해야 함)
// ---------------------------------------------------------------------------
extern volatile uint8_t currentMode; // enum FlightMode를 uint8_t로 extern 해도 OK
extern bool flightLock;
extern bool failsafe;

extern float currentAltitude; // fused altitude (m)
extern float targetAltitude;  // target (m)
extern int   hoverThrottle;   // 0..255 base PWM

extern Preferences prefs;

// ---------------------------------------------------------------------------
// Hover Learn Parameters (안전 위주)
// ---------------------------------------------------------------------------
static constexpr float HOVER_TARGET_M      = 1.00f;
static constexpr float HOVER_OK_BAND_M     = 0.05f;   // ±5cm
static constexpr float HOVER_OK_VEL_MPS    = 0.12f;   // 안정 판정용 속도
static constexpr uint32_t HOVER_STABLE_MS  = 2000;

static constexpr float LIFTOFF_RISE_M      = 0.03f;   // 3cm 이상 상승 감지
static constexpr float LIFTOFF_VEL_MPS     = 0.05f;   // 상승 속도 감지
static constexpr uint32_t RAMP_STEP_MS     = 120;     // PWM 증가 간격
static constexpr int RAMP_STEP_PWM         = 1;       // PWM step
static constexpr int RAMP_MIN_PWM          = 80;      // 720 모터 최소
static constexpr int RAMP_MAX_PWM          = 200;     // 720 모터 안전 상한

static constexpr int PWM_MIN              = 0;
static constexpr int PWM_MAX              = 255;

// 학습/적응 속도 (너무 빠르면 위험, 너무 느리면 의미 없음)
static constexpr float LEARN_GAIN_ASCEND  = 0.18f;  // 상승 중(학습 좀 빠르게)
static constexpr float LEARN_GAIN_HOVER   = 0.03f;  // 호버 중(매우 천천히)

// 저장할 키
static constexpr const char* KEY_HOVER_PWM = "hoverPWM";

// ---------------------------------------------------------------------------
// Hover State Machine
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// 내부 유틸
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// API
// ---------------------------------------------------------------------------

// 호출 시점: initAF1000X()에서 prefs.begin() 이후
static inline void hover_begin() {
  int saved = prefs.getInt(KEY_HOVER_PWM, -1);

  if (saved >= 0 && saved <= 255) {
    hoverThrottle = saved;
  } else {
    // “처음 기체” 안전 기본값
    hoverThrottle = 120; // 720 모터용
  }

  hl_state = HL_IDLE;
  hl_active = false;
  hl_ready = false;
}

// takeoff() 내부에서 호출하거나, autoTakeoff()에서 호출해도 됨
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

  // 저장된 hoverThrottle 기준으로 “아주 낮게” 시작해서 안전하게 찾기
  hl_rampPwm = _clampi(hoverThrottle - 25, RAMP_MIN_PWM, RAMP_MAX_PWM);
  hl_liftoffPwm = hl_rampPwm;

  // TAKEOFF 모드로 전환(모터 출력 허용)
  // (코어에서 TAKEOFF가 1이라고 가정하지 않고, 외부에서 set 하는 방식이면 여기서만 set)
  // currentMode enum 값은 코어와 동일해야 함.
  // 일반적으로: READY=0, TAKEOFF=1, HOVERING=2, LANDING=3, EMERGENCY=4
  if (currentMode == 0) { // READY
    currentMode = 1;      // TAKEOFF
  }

  // 목표 고도는 최종 1m로(상승 제어 루프가 필요)
  targetAltitude = HOVER_TARGET_M;
}

// hover 학습/상태 업데이트: loop에서 50Hz로 계속 호출 (updateSystem() 뒤 추천)
static inline void hover_update() {
  // 속도 추정
  float alt = currentAltitude;
  hl_vel = (alt - hl_altPrev) / 0.02f; // 50Hz 가정
  hl_altPrev = alt;

  // 실패 조건 (센서 락/통신 끊김/비상)
  if (flightLock || failsafe || currentMode == 4 /*EMERGENCY*/) {
    hl_state = HL_ABORT;
  }

  switch (hl_state) {
    case HL_IDLE:
      hl_active = false;
      hl_ready  = false;
      break;

    case HL_RAMP: {
      // 이 단계에서는 “고도 제어”가 아니라 “리프트오프 PWM 찾기”가 목적
      // -> 코어 updateFlight()가 thr를 계산할 때 hoverThrottle을 그대로 쓰므로
      //    여기서 hoverThrottle을 ramp pwm으로 임시 갱신한다.
      uint32_t now = millis();
      if (now - hl_lastRamp_ms >= RAMP_STEP_MS) {
        hl_lastRamp_ms = now;
        hl_rampPwm += RAMP_STEP_PWM;
        hl_rampPwm = _clampi(hl_rampPwm, RAMP_MIN_PWM, RAMP_MAX_PWM);
        hoverThrottle = hl_rampPwm;
      }

      // 리프트오프 감지(3cm 이상 + 상승속도)
      float rise = alt - hl_alt0;
      bool liftoff = (rise > LIFTOFF_RISE_M) && (hl_vel > LIFTOFF_VEL_MPS);

      if (liftoff) {
        hl_liftoffPwm = hoverThrottle;
        hl_state = HL_LIFTOFF_CONFIRM;
        hl_t0_ms = millis();
      }

      // 너무 오래 못 뜨면 중단
      if (millis() - hl_t0_ms > 6000) {
        hl_state = HL_ABORT;
      }
    } break;

    case HL_LIFTOFF_CONFIRM: {
      // “잠깐 떠오른 오탐” 방지: 400ms 정도 추세를 확인
      bool stillUp = (alt - hl_alt0 > LIFTOFF_RISE_M) || (hl_vel > 0.02f);

      if (!stillUp) {
        // 오탐 -> 다시 램프
        hl_alt0 = alt;
        hl_state = HL_RAMP;
        hl_lastRamp_ms = millis();
        break;
      }

      if (millis() - hl_t0_ms > 400) {
        // 리프트오프 PWM 확정 → hoverThrottle 기반으로 고도 제어 시작
        hoverThrottle = hl_liftoffPwm;
        hl_state = HL_ASCEND_TO_1M;
        hl_t0_ms = millis();
        targetAltitude = HOVER_TARGET_M;
      }
    } break;

    case HL_ASCEND_TO_1M: {
      // 코어의 고도 제어(PD)가 targetAltitude=1m로 올려주도록 둔다.
      // 여기서는 “학습”만 한다(hoverThrottle이 너무 낮/높으면 천천히 보정).
      float err = targetAltitude - alt;

      // 충분히 가까워지면 안정화 단계로
      if (_absf(err) < 0.10f) {
        hl_state = HL_STABILIZE;
        hl_stableStart_ms = 0;
      }

      // 너무 오래 걸리면 abort (센서/추력 문제)
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
          // 1m 안정 호버 성공
          hl_state = HL_READY;
          hl_ready = true;
          hl_active = false;

          // 코어가 HOVERING 모드를 쓰면 여기서 진입시키는 게 좋다
          // READY=0, TAKEOFF=1, HOVERING=2
          currentMode = 2;

          // 저장
          prefs.putInt(KEY_HOVER_PWM, hoverThrottle);
        }
      } else {
        hl_stableStart_ms = 0;
      }

      // 안정화가 너무 오래 걸리면 abort
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
      // 안전: 학습 실패 시 착륙/정지 쪽으로
      // TAKEOFF/HOVERING 중이면 LANDING으로 유도하는 게 안전하다.
      // READY=0, TAKEOFF=1, HOVERING=2, LANDING=3
      if (currentMode == 1 || currentMode == 2) {
        currentMode = 3; // LANDING
      }
      break;
  }
}

// 코어 updateFlight()에서 zCmd 계산 후 호출 (매 tick)
// zCmd가 +면 “추력이 부족”, -면 “추력이 과함” 이라고 가정
static inline void hover_onZcmd(float zCmd) {
  // 학습 중/호버 중에만 천천히 보정
  // - ASCEND 단계에서는 조금 더 빨리
  // - HOVERING에서는 아주 천천히
  float gain = 0.0f;

  if (hl_state == HL_ASCEND_TO_1M || hl_state == HL_STABILIZE) {
    gain = LEARN_GAIN_ASCEND;
  } else if (currentMode == 2 /*HOVERING*/ && hl_state != HL_ABORT) {
    gain = LEARN_GAIN_HOVER;
  } else {
    return;
  }

  // 기존 코어가 thr = hoverThrottle + (zCmd*100) 이라면,
  // hoverThrottle을 zCmd 방향으로 조금 움직여주면 “필요 zCmd가 감소”한다.
  float delta = zCmd * 100.0f * gain;
  // 너무 급하지 않게 제한
  if (delta >  1.0f) delta =  1.0f;
  if (delta < -1.0f) delta = -1.0f;

  hoverThrottle = _clampi((int)lroundf((float)hoverThrottle + delta), 60, 230);
}

// 외부에서 상태 확인
static inline bool hover_isLearning() { return hl_active; }
static inline bool hover_isReady()    { return hl_ready; }
static inline uint8_t hover_state()   { return (uint8_t)hl_state; }

// 안전: 필요 시 학습 강제 종료
static inline void hover_abort() {
  hl_state = HL_ABORT;
  hl_active = false;
  hl_ready = false;
}

#endif // AF1000X_HOVER_H
