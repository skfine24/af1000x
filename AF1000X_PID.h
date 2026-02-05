#pragma once

// ============================================================================
// KO: AF1000X 비행 튜닝 / PID
// EN: AF1000X flight tuning / PID
// ============================================================================

// ============================================================================
// KO: 버전
// EN: Version
// ============================================================================
static const char FC_VERSION[] = "00a";

// ============================================================================
// KO: 타이밍 / 페일세이프
// EN: Timing / Failsafe
// ============================================================================
static const float    DEFAULT_LOOP_DT = 0.02f; // KO: 50Hz 기본 / EN: 50Hz nominal
static const uint32_t FAILSAFE_MS     = 3000; // KO: RX 타임아웃 / EN: RX timeout

// ============================================================================
// KO: 전원 (1S LiPo)
// EN: Power (1S LiPo)
// ============================================================================
static const float BATTERY_MIN_VOLTAGE     = 3.3f; // KO: 컧/착륙 기준 / EN: cut/land threshold
static const float BATTERY_WARNING_VOLTAGE = 3.6f; // KO: 경고 시작 / EN: warning threshold
static const float BATTERY_ARM_MIN_VOLTAGE = 3.4f; // KO: 이 값 아래 시동 금지 / EN: block arming below
static const float BATTERY_RTOP_KOHM = 200.0f; // KO: 분압 상단 / EN: divider top
static const float BATTERY_RBOT_KOHM = 100.0f; // KO: 분압 하단 / EN: divider bottom
// KO: 측정 보정 (필요 시 입력)
// EN: Optional calibration (set when measured)
static const float BATTERY_CAL_ACTUAL_V   = 0.0f; // KO: 멀티미터 값 / EN: multimeter
static const float BATTERY_CAL_MEASURED_V = 0.0f; // KO: 시리얼 측정값 / EN: serial read

// ============================================================================
// KO: 고도 융합 / 필터
// EN: Altitude fusion / filters
// ============================================================================
static const float ALT_SWITCH_M      = 1.0f;
static const float TOF_EMA_A         = 0.25f;
static const float BARO_EMA_A        = 0.05f;
static const float ALT_EMA_A         = 0.15f;
static const float TOF_JUMP_REJECT_M = 0.25f;

// ============================================================================
// KO: 자세 필터 (Roll/Pitch)
// EN: Attitude filter (Roll/Pitch)
// ============================================================================
static const float ATT_TAU   = 0.25f; // KO: 작을수록 빠름 / EN: smaller = faster
static const float ACC_Z_MIN = 0.20f; // KO: Z 가속도 최소값 / EN: accel Z threshold

// ============================================================================
// KO: 옵티컬 플로우
// EN: Optical flow
// ============================================================================
static const uint8_t FLOW_SQUAL_MIN         = 20;
static const float   FLOW_MAX_STEP_M        = 0.08f;
static const float   FLOW_TILT_COMP_ALT_M   = 1.0f;
static const float   FLOW_TILT_COMP_WINDOW_M = 0.25f;

// ============================================================================
// KO: PID / 게인 (기본값)
// EN: PID / gains (defaults)
// ============================================================================
static const float POS_KP         = 0.6f;
static const float YAW_KP_DEFAULT = 1.5f;
static const float ALT_KP_DEFAULT = 0.18f;
static const float ALT_KD_DEFAULT = 0.06f;

// ============================================================================
// KO: PID 한계 (오토튀 제한)
// EN: PID bounds (auto-tune clamps)
// ============================================================================
static const float YAW_KP_MIN = 0.6f;
static const float YAW_KP_MAX = 4.0f;
static const float ALT_KP_MIN = 0.08f;
static const float ALT_KP_MAX = 0.50f;
static const float ALT_KD_MIN = 0.02f;
static const float ALT_KD_MAX = 0.20f;

// ============================================================================
// KO: 비행 동작
// EN: Flight behavior
// ============================================================================
static const float TAKEOFF_TARGET_M = 1.0f;
static const float LAND_DESCEND_MPS = 0.25f;
static const float READY_ALT_M      = 0.08f;

// ============================================================================
// KO: 안전
// EN: Safety
// ============================================================================
static const float    TILT_KILL_DEG      = 55.0f;
static const uint32_t TILT_KILL_HOLD_MS  = 300;
static const float    ARM_LEVEL_DEG      = 6.0f; // KO: 시동 허용 최대 |roll|/|pitch| / EN: max |roll|/|pitch| to allow arming

// ============================================================================
// KO: 플립(360도)
// EN: Flip (360-degree)
// ============================================================================
static const float    FLIP_MIN_ALT_M      = 1.50f;
static const float    FLIP_MIN_VOLTAGE    = 3.6f;
static const float    FLIP_RATE_CMD       = 80.0f;
static const int      FLIP_THR_BOOST      = 20;
static const float    FLIP_TARGET_DEG     = 360.0f;
static const uint32_t FLIP_MAX_TIME_MS    = 900;
static const uint32_t FLIP_COOLDOWN_MS    = 800;

// ============================================================================
// KO: 뒤집힘 복구(셀프 라이트)
// EN: Self-righting (turtle mode)
// ============================================================================
static const float    TURTLE_INVERTED_DEG = 120.0f;
static const uint32_t TURTLE_WINDOW_MS    = 2000;
static const uint8_t  TURTLE_DOWN_COUNT   = 3;
static const uint32_t TURTLE_MAX_TIME_MS  = 1200;
static const int      TURTLE_THR          = 70;
static const float    TURTLE_RATE_CMD     = 70.0f;

// ============================================================================
// KO: 모터 시동 / 아이들 스핀
// EN: Motor arm / idle spin
// ============================================================================
static const uint32_t ARM_WINDOW_MS       = 1500; // KO: 2회 입력 창 / EN: 2x up window
static const uint32_t ARM_PULSE_MS        = 250; // KO: 약/강/약 펄스 / EN: low/high/low pulse
static const uint8_t  ARM_THROTTLE_HIGH   = 240;
static const uint8_t  ARM_THROTTLE_LOW    = 20;
static const uint8_t  ARM_TAKEOFF_THRESH  = 200;
static const int      ARM_IDLE_PWM_LOW    = 30;
static const int      ARM_IDLE_PWM_HIGH   = 60;
static const uint32_t ARM_DISARM_HOLD_MS  = 2000;

// ============================================================================
// KO: 자동 이륜
// EN: Auto takeoff
// ============================================================================
static const uint32_t AUTO_TAKEOFF_DELAY_MS = 1000;
static const float    AUTO_TAKEOFF_ALT_M    = 1.50f;

// ============================================================================
// KO: 플로우 보정
// EN: Flow calibration
// ============================================================================
static const float FLOW_K_DEFAULT = 0.0022f;

// ============================================================================
// KO: 호버 학습 / 자동 호버 튀닝
// EN: Hover learning / auto hover tuning
// ============================================================================
static const int      HOVER_PWM_DEFAULT  = 120; // KO: 720 모터 기준 / EN: 720 motor baseline
static const float    HOVER_TARGET_M     = 1.00f;
static const float    HOVER_OK_BAND_M    = 0.05f; // KO: ±5cm / EN: ±5cm band
static const float    HOVER_OK_VEL_MPS   = 0.12f; // KO: 안정 속도 / EN: stable velocity
static const uint32_t HOVER_STABLE_MS    = 2000;

static const float    LIFTOFF_RISE_M     = 0.03f;
static const float    LIFTOFF_VEL_MPS    = 0.05f;
static const uint32_t RAMP_STEP_MS       = 120;
static const int      RAMP_STEP_PWM      = 1;
static const int      RAMP_MIN_PWM       = 80;
static const int      RAMP_MAX_PWM       = 200;

static const int      PWM_MIN            = 0;
static const int      PWM_MAX            = 255;

static const float    LEARN_GAIN_ASCEND  = 0.18f;
static const float    LEARN_GAIN_HOVER   = 0.03f;

// ============================================================================
// KO: 오토튀 (ALT PD + YAW P)
// EN: Auto tune (ALT PD + YAW P)
// ============================================================================
static const float    ATUNE_MAX_ALT_M    = 1.20f;
static const float    ATUNE_ALT_STEP_M   = 0.12f;
static const float    ATUNE_YAW_STEP_DEG = 20.0f;
static const uint32_t ATUNE_TOTAL_MS     = 10000;
static const uint32_t ATUNE_ALT_PHASE_MS = 6000;
static const uint32_t ATUNE_YAW_PHASE_MS = 4000;
static const uint32_t ATUNE_ALT_UP_MS    = 3000;
static const uint32_t ATUNE_ALT_DOWN_MS  = 3000;

// ============================================================================
// KO: 속도 단계 (1~3)
// EN: Speed steps (1~3)
// ============================================================================
static const float MOVE_SPEED_STEPS[3]  = {0.30f, 0.60f, 1.00f};
static const float CLIMB_SPEED_STEPS[3] = {0.20f, 0.40f, 0.70f};
static const float YAW_SPEED_STEPS[3]   = {30.0f, 60.0f, 120.0f};

// ============================================================================
// KO: PWM
// EN: PWM
// ============================================================================
static const uint32_t PWM_FREQ = 24000;
static const uint8_t  PWM_RES  = 8;
