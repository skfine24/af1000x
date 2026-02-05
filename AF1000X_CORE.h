#ifndef AF1000X_H
#define AF1000X_H


#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Preferences.h>
#include <VL53L1X.h>
#include "ICM45686.h"
#ifdef __has_include
#  if __has_include(<Adafruit_NeoPixel.h>)
#    include <Adafruit_NeoPixel.h>
#    define HAS_USER_NEOPIXEL 1
#  else
#    define HAS_USER_NEOPIXEL 0
#  endif
#else
#  define HAS_USER_NEOPIXEL 0
#endif

// KO: GPIO / 핀맵
// EN: GPIO / pin map
#include "AF1000X_GPIO.h"

// KO: 비행 튜닝 / PID 상수
// EN: Flight tuning / PID constants
#include "AF1000X_PID.h"

// Serial log control (mute after POST if requested)
static bool g_serialMuteAfterPost = false;
#define LOG_PRINTF(...) do { if(!g_serialMuteAfterPost) Serial.printf(__VA_ARGS__); } while(0)
#define LOG_PRINTLN(...) do { if(!g_serialMuteAfterPost) Serial.println(__VA_ARGS__); } while(0)
#define LOG_PRINT(...) do { if(!g_serialMuteAfterPost) Serial.print(__VA_ARGS__); } while(0)
#define LOG_IMU_PRINTF(...) do { Serial.printf(__VA_ARGS__); } while(0)
#define LOG_CFG_PRINTF(...) do { Serial.printf(__VA_ARGS__); } while(0)
#define LOG_CFG_PRINTLN(...) do { Serial.println(__VA_ARGS__); } while(0)
#define LOG_CMD_PRINTF(...) do { Serial.printf(__VA_ARGS__); } while(0)
#define LOG_CMD_PRINTLN(...) do { Serial.println(__VA_ARGS__); } while(0)
#define LOG_CMD_PRINT(...) do { Serial.print(__VA_ARGS__); } while(0)




// KO: 떎젣 猷⑦봽 dt(珥), 硫붿씤 猷⑦봽뿉꽌 媛깆떊
// EN: Actual loop dt (seconds), updated in main loop
extern float g_loopDt;


// KO: Hover 븰뒿 뿤뜑
// EN: Hover learning header
#include "AF1000X_Hover.h"
#include "AF1000X_AutoTune.h"

#ifndef HOP_MAX
#define HOP_MAX 12
#endif

#ifndef HOP_CH_MIN
#define HOP_CH_MIN 5
#endif
#ifndef HOP_CH_MAX
#define HOP_CH_MAX 80
#endif

#include "AF1000X_BINDING.h"

/* ============================================================================
 * KO: AF1000X 肄붿뼱 (ESP32-S3)
 * EN: AF1000X core (ESP32-S3)
 * KO: 꽱꽌 POST, 怨좊룄 쑖빀(ToF+SPL06), 샃떚而 뵆濡쒖슦, 諛붿씤뵫/샇踰/삤넗뒥 룷븿
 * EN: Sensor POST, altitude fusion (ToF+SPL06), optical flow, binding/hover/auto-tune
 * ========================================================================== */

// ============================================================================
// KO: 紐⑤뱶 긽닔 (u8)
// EN: Mode constants (u8)
// ============================================================================
static constexpr uint8_t MODE_READY     = 0;
static constexpr uint8_t MODE_TAKEOFF   = 1;
static constexpr uint8_t MODE_HOVERING  = 2;
static constexpr uint8_t MODE_LANDING   = 3;
static constexpr uint8_t MODE_EMERGENCY = 4;

// ============================================================================
// KO: RF 구조체
// EN: RF structs
// ============================================================================
struct Signal {
  uint8_t throttle, roll, pitch, yaw, aux1, aux2;
  uint8_t speed; // KO: 1~3 / EN: 1~3
  uint8_t hop;   // KO: FHSS 솄 씤뜳뒪 / EN: FHSS hop index
};

// KO: AUX2 bit 정의
// EN: AUX2 bit definitions
static const uint8_t AUX2_HEADLESS = 0x01;
static const uint8_t AUX2_FLOW     = 0x02;
static const uint8_t AUX2_FLIP_READY = 0x04;
static const uint8_t AUX1_TAKEOFF = 1;
static const uint8_t AUX1_GYRO_RESET = 2;
static const uint8_t AUX1_SERVO_TOGGLE = 3;
static const uint8_t AUX1_LED_STEP = 4;
static const uint8_t AUX1_FLIP_ROLL_POS  = 5;
static const uint8_t AUX1_FLIP_ROLL_NEG  = 6;
static const uint8_t AUX1_FLIP_PITCH_POS = 7;
static const uint8_t AUX1_FLIP_PITCH_NEG = 8;
#include "AF1000X_WIFI.h"

struct Telemetry {
  float vbat, alt, posX, posY;
  int rssi;
};

// ============================================================================
// KO:  젙쓽 (AF1000X_GPIO.h濡 씠룞)
// EN: Pins (moved to AF1000X_GPIO.h)
// ============================================================================

// KO: FHSS 湲곕낯 뀒씠釉 (럹뼱留곸쑝濡 援먯껜)
// EN: FHSS default table (replaced by pairing)
static const uint8_t HOP_DEFAULT[HOP_MAX] = {63,64,65,66,67,68,69,70,71,72,73,74};
static const uint32_t HOP_SLOT_MS = 20;
static const uint32_t HOP_SCAN_MS = 20;

// ============================================================================
// KO: 쟾뿭 (Hover/EasyCommander뿉꽌 李몄“)
// EN: Globals (referenced by Hover/EasyCommander)
// ============================================================================
#ifdef AF1000X_IMPLEMENTATION

// KO: RF
// EN: RF
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Preferences prefs;
Signal receiverData{};
Signal receiverDataRc{};
Telemetry telemetryData{};
uint32_t lastRxMs = 0;
bool failsafe = false;
bool linkReady = false;

uint8_t hopTable[HOP_MAX] = {63,64,65,66,67,68,69,70,71,72,73,74};
uint8_t hopLen = HOP_MAX;

uint8_t hopSeed = 1;
uint32_t hopStartMs = 0;
bool hopSynced = false;
uint8_t hopIdxLast = 0xFF;
uint8_t hopScanIdx = 0;
uint32_t hopScanMs = 0;

float currentRoll  = 0.0f;
float currentPitch = 0.0f;


// KO: 紐⑤뱶 (Hover 뿤뜑뒗 uint8_t extern 湲곕)
// EN: Mode (Hover header expects uint8_t extern)
volatile uint8_t currentMode = MODE_READY;

// KO: 源 / 긽깭 (meters)
// EN: Targets / states (meters)
float targetAltitude = TAKEOFF_TARGET_M;
float currentAltitude = 0.0f;

float targetPosX = 0.0f, targetPosY = 0.0f;
float currentPosX = 0.0f, currentPosY = 0.0f;

float targetYaw = 0.0f;   // KO: deg / EN: deg
float currentYaw = 0.0f;  // KO: deg / EN: deg

int speedLevel = 1;
float moveSpeedSteps[3]  = {MOVE_SPEED_STEPS[0], MOVE_SPEED_STEPS[1], MOVE_SPEED_STEPS[2]};
float climbSpeedSteps[3] = {CLIMB_SPEED_STEPS[0], CLIMB_SPEED_STEPS[1], CLIMB_SPEED_STEPS[2]};
float yawSpeedSteps[3]   = {YAW_SPEED_STEPS[0], YAW_SPEED_STEPS[1], YAW_SPEED_STEPS[2]};

float trimRoll = 0.0f, trimPitch = 0.0f;

// KO: 뒠떇 寃뚯씤 (NVS 濡쒕뱶, 湲곕낯媛믪 AF1000X_PID.h)
// EN: Tunable gains (loaded from NVS, defaults from AF1000X_PID.h)
float altKp = ALT_KP_DEFAULT;
float altKd = ALT_KD_DEFAULT;
float yawKp = YAW_KP_DEFAULT;

// KO: 븘꽣 뙆씪誘명꽣 (NVS 濡쒕뱶, 湲곕낯媛믪 AF1000X_PID.h)
// EN: Tunable filter parameters (loaded from NVS, defaults from AF1000X_PID.h)
float attTau = ATT_TAU;
float accZMin = ACC_Z_MIN;
float tofEmaA = TOF_EMA_A;
float baroEmaA = BARO_EMA_A;
float altEmaA = ALT_EMA_A;
float tofJumpRejectM = TOF_JUMP_REJECT_M;

// KO: 怨좊룄 젣뼱 湲곕컲 PWM (Hover 븰뒿씠 옄룞 궛異)
// EN: Altitude control PWM (auto-learned by hover)
int hoverThrottle = HOVER_PWM_DEFAULT;

// KO: 諛고꽣由 긽깭 (1S LiPo)
// EN: Battery state (1S LiPo)
float batteryVoltage = 4.2f;      // KO: 1S 셿異 쟾븬 / EN: 1S full voltage
float batteryVoltageFilt = 4.2f;  // KO: EMA 븘꽣 / EN: EMA filtered
bool batteryLowWarning = false;
bool batteryCritical = false;

// KO: 꽱꽌 OK/씫
// EN: Sensor OK/lock
bool ok_imu=false, ok_baro=false, ok_tof=false, ok_flow=false;
bool flightLock = false;
bool calibrated = false;
bool flowUserEnabled = true;

// KO: 紐⑦꽣 떆룞 / 븘씠뱾 뒪 긽깭
// EN: Motor arm / idle spin state
bool motorArmedIdle = false;
static bool armAwaitSecond = false;
static bool armHighPrev = false;
static bool armPulsing = false;
static uint32_t armFirstUpMs = 0;
static uint32_t armPulseStartMs = 0;
static uint32_t armDisarmStartMs = 0;
static bool autoTakeoffPending = false;
static uint32_t autoTakeoffAtMs = 0;

static bool flipActive = false;
static uint8_t flipDir = 0;
static float flipAngleDeg = 0.0f;
static uint32_t flipStartMs = 0;
static uint32_t flipCooldownUntil = 0;

// EN: Self-righting (turtle mode) state
static bool turtleActive = false;
static uint8_t turtleAxis = 0;   // EN: 0=roll, 1=pitch
static int8_t turtleDir = 0;     // EN: +1 or -1
static uint32_t turtleStartMs = 0;
static uint8_t turtleDownCount = 0;
static uint32_t turtleFirstDownMs = 0;
static bool turtleLowPrev = false;

// EN: Self-righting (turtle mode) helper declarations
static inline void turtleResetTrigger();

// KO: 怨좊룄 븘꽣
// EN: Altitude filters
bool tofFiltInit=false;
  uint8_t tofFailCount = 0;
  static const uint8_t TOF_FAIL_LIMIT = 30;
float tof_m_filt=0.0f;
float tof_m_last_raw=0.0f;

bool baroInit=false;
float baro_alt_m_filt=0.0f;

bool altInit=false;
float alt_m_filt=0.0f;
float alt_m_prev=0.0f;

// KO: flowK (m/count/m)
// EN: flowK (m/count/m)
float flowK = FLOW_K_DEFAULT;

// KO: ToF
// EN: ToF
VL53L1X tof;

// KO: IMU (ICM45686)
// EN: IMU (ICM45686)
static ICM456xx IMU0(Wire, 0);
static ICM456xx IMU1(Wire, 1);
static ICM456xx* IMU = nullptr;
float gyroX_bias = 0.0f;
float gyroY_bias = 0.0f;
float gyroZ_bias = 0.0f;
float gyroX_dps = 0.0f;
float gyroY_dps = 0.0f;
float yaw_internal = 0.0f;
uint32_t lastYawUs = 0;

// KO: 모터 LEDC 채널
// EN: Motors LEDC channels
int ch1=-1,ch2=-1,ch3=-1,ch4=-1;

// EN: User servo PWM settings
static const uint32_t SERVO_FREQ = 50;
static const uint8_t SERVO_RES = 16;
static const int SERVO_MIN_US = 900;
static const int SERVO_MAX_US = 2000;
static const int SERVO_ANGLE_MIN = 0;
static const int SERVO_ANGLE_MAX = 180;
int chServo = -1;
int servoAngle = 0;

// EN: User LED color index (0..4)
uint8_t userLedIndex = 0;

#if HAS_USER_NEOPIXEL
Adafruit_NeoPixel userPixel(1, PIN_USER_LED, NEO_GRB + NEO_KHZ800);
#endif

// KO: 湲곗븬怨 湲곗媛
// EN: Baro baseline
float basePressurePa = 101325.0f;

// KO: 뵆濡쒖슦 罹섎━釉뚮젅씠뀡 긽깭
// EN: Flow calibration state
enum FlowCalState : uint8_t { FC_IDLE, FC_TAKE_HOVER, FC_FORWARD, FC_PAUSE1, FC_BACK, FC_PAUSE2, FC_DONE, FC_ABORT };
FlowCalState flowCal = FC_IDLE;
uint32_t flowCal_t0 = 0;
float flowCal_dist_m = 0.50f;
float flowCal_hoverAlt_m = 0.80f;
float flowCal_pwr = 35.0f;

int32_t flowCal_sum_dx = 0;
uint32_t flowCal_samples = 0;
float flowCal_height_acc = 0.0f;
uint32_t flowCal_height_n = 0;
float flowCal_suggestK = 0.0f;
bool flowCal_reported = false;

#else
extern RF24 radio;
extern Preferences prefs;
extern Signal receiverData;
extern Telemetry telemetryData;
extern uint32_t lastRxMs;
extern bool failsafe;

extern volatile uint8_t currentMode;

extern float targetAltitude, currentAltitude;
extern float targetPosX, targetPosY, currentPosX, currentPosY;
extern float targetYaw, currentYaw;

extern int speedLevel;
extern float moveSpeedSteps[3], climbSpeedSteps[3], yawSpeedSteps[3];

extern float trimRoll, trimPitch;
extern int hoverThrottle;
extern float altKp, altKd, yawKp;
extern float attTau, accZMin, tofEmaA, baroEmaA, altEmaA, tofJumpRejectM;

extern float batteryVoltage;
extern bool batteryLowWarning, batteryCritical;

extern bool ok_imu, ok_baro, ok_tof, ok_flow;
extern bool flightLock;
extern bool calibrated;

extern bool tofFiltInit;
  extern uint8_t tofFailCount;
extern float tof_m_filt, tof_m_last_raw;
extern bool baroInit;
extern float baro_alt_m_filt;
extern bool altInit;
extern float alt_m_filt, alt_m_prev;

extern float flowK;
extern VL53L1X tof;

extern float gyroX_bias, gyroY_bias, gyroZ_bias, yaw_internal;
extern float gyroX_dps, gyroY_dps;
extern bool flipActive;
extern uint8_t flipDir;
extern float flipAngleDeg;
extern uint32_t flipStartMs;
extern uint32_t flipCooldownUntil;
extern uint32_t lastYawUs;

extern int ch1,ch2,ch3,ch4;
extern int chServo;
extern int servoAngle;
extern uint8_t userLedIndex;
#if HAS_USER_NEOPIXEL
extern Adafruit_NeoPixel userPixel;
#endif

extern float basePressurePa;

extern enum FlowCalState : uint8_t { FC_IDLE, FC_TAKE_HOVER, FC_FORWARD, FC_PAUSE1, FC_BACK, FC_PAUSE2, FC_DONE, FC_ABORT } flowCal;
extern uint32_t flowCal_t0;
extern float flowCal_dist_m, flowCal_hoverAlt_m, flowCal_pwr;
extern int32_t flowCal_sum_dx;
extern uint32_t flowCal_samples;
extern float flowCal_height_acc;
extern uint32_t flowCal_height_n;
extern float flowCal_suggestK;
extern bool flowCal_reported;
#endif

// ============================================================================
// KO: 뿬띁
// EN: Helpers
// ============================================================================
static inline float clampf(float v,float lo,float hi){ return (v<lo)?lo:((v>hi)?hi:v); }
static inline void ledsInit(){
  pinMode(PIN_LED_IMU, OUTPUT);
  pinMode(PIN_LED_BARO, OUTPUT);
  pinMode(PIN_LED_TOF, OUTPUT);
  pinMode(PIN_LED_FLOW, OUTPUT);
  digitalWrite(PIN_LED_IMU,  HIGH);
  digitalWrite(PIN_LED_BARO, HIGH);
  digitalWrite(PIN_LED_TOF, LOW);
  digitalWrite(PIN_LED_FLOW, LOW);
}
static inline void ledSet(bool imu_ok2,bool baro_ok2,bool tof_ok2,bool flow_ok2){
  digitalWrite(PIN_LED_IMU,  imu_ok2  ? HIGH : LOW);
  digitalWrite(PIN_LED_BARO, baro_ok2 ? HIGH : LOW);
  digitalWrite(PIN_LED_TOF,  tof_ok2  ? HIGH : LOW);
  digitalWrite(PIN_LED_FLOW, flow_ok2 ? HIGH : LOW);
}
static inline void ledAll(bool on){
  digitalWrite(PIN_LED_IMU,  on?HIGH:LOW);
  digitalWrite(PIN_LED_BARO, on?HIGH:LOW);
  digitalWrite(PIN_LED_TOF,  on?HIGH:LOW);
  digitalWrite(PIN_LED_FLOW, on?HIGH:LOW);
}
static inline void ledFailPattern(){
  for(int i=0;i<6;i++){
    ledAll(true);  delay(80);
    ledAll(false); delay(80);
  }
}

// KO: POST 寃곌낵 몴떆(쟾泥 ON + 떎뙣 꽱꽌留 0.5珥 떒쐞 젏硫)
// EN: POST status display (all ON + failed sensors blink at 0.5s steps)
static inline void ledPostStatusDelay(bool imu_ok, bool baro_ok, bool tof_ok, bool flow_ok, uint32_t duration_ms){
  uint32_t start = millis();
  while((uint32_t)(millis() - start) < duration_ms){
    bool blink = ((millis() / 500) % 2) == 0; // KO: 0.5s 넗湲 / EN: toggle every 0.5s
    digitalWrite(PIN_LED_IMU,  imu_ok  ? HIGH : (blink ? HIGH : LOW));
    digitalWrite(PIN_LED_BARO, baro_ok ? HIGH : (blink ? HIGH : LOW));
    digitalWrite(PIN_LED_TOF,  tof_ok  ? HIGH : (blink ? HIGH : LOW));
    digitalWrite(PIN_LED_FLOW, flow_ok ? HIGH : (blink ? HIGH : LOW));
    delay(10);
  }
}

// EN: User LED (single-wire RGB/NeoPixel) helpers
static inline void userLedWrite(uint8_t idx){
  userLedIndex = idx % 5;
  uint8_t r = 0, g = 0, b = 0;
  switch(userLedIndex){
    case 0: r = 255; g = 0;   b = 0;   break; // red
    case 1: r = 255; g = 160; b = 0;   break; // yellow
    case 2: r = 0;   g = 255; b = 0;   break; // green
    case 3: r = 0;   g = 0;   b = 255; break; // blue
    case 4: r = 160; g = 0;   b = 255; break; // purple
  }
#if defined(ARDUINO_ARCH_ESP32)
  #if HAS_USER_NEOPIXEL
  userPixel.setPixelColor(0, userPixel.Color(r, g, b));
  userPixel.show();
  #else
  digitalWrite(PIN_USER_LED, (r || g || b) ? HIGH : LOW);
  #endif
#else
  digitalWrite(PIN_USER_LED, (r || g || b) ? HIGH : LOW);
#endif
}

static inline void userLedInit(){
  pinMode(PIN_USER_LED, OUTPUT);
#if HAS_USER_NEOPIXEL
  userPixel.begin();
  userPixel.show();
#endif
  userLedWrite(0);
}

static inline void userLedStep(){
  userLedWrite((uint8_t)(userLedIndex + 1));
}

// EN: User servo helpers (PWM 50Hz, 900-2000us)
static inline uint32_t servoUsToDuty(uint32_t us){
  const uint32_t maxDuty = (1u << SERVO_RES) - 1u;
  uint32_t duty = (uint32_t)((uint64_t)us * maxDuty * SERVO_FREQ / 1000000ull);
  if(duty > maxDuty) duty = maxDuty;
  return duty;
}

static inline void userServoWriteAngle(int angle){
  if(chServo < 0) return;
  angle = constrain(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX);
  servoAngle = angle;
  uint32_t us = (uint32_t)map(angle, SERVO_ANGLE_MIN, SERVO_ANGLE_MAX, SERVO_MIN_US, SERVO_MAX_US);
  ledcWrite(chServo, servoUsToDuty(us));
}

static inline void userServoInit(){
  chServo = ledcAttach(PIN_SERVO, SERVO_FREQ, SERVO_RES);
  userServoWriteAngle(SERVO_ANGLE_MIN);
}

static inline void userServoToggle(){
  int next = (servoAngle <= (SERVO_ANGLE_MIN + SERVO_ANGLE_MAX) / 2) ? SERVO_ANGLE_MAX : SERVO_ANGLE_MIN;
  userServoWriteAngle(next);
}

// ============================================================================
// KO: LED 규칙 (부팅/바인딩/저전압/자이로 리셋/플립 준비/헤드리스/자동 튜닝/정상)
// EN: LED state machine (boot/bind/low batt/gyro/flip ready/headless/auto-tune)
// KO: 우선순위: 부팅/바인딩 > 저전압 > 자이로 리셋 > 플립 준비 > 헤드리스 > 자동 튜닝 > 정상
// EN: Boot/bind has top priority; then low battery, gyro, flip ready, headless, auto-tune
// ============================================================================

static bool g_ledInvertedAtBoot = false;

static bool g_ledLowBattery = false;
static bool g_ledHeadless   = false;
static bool g_ledFlipReady  = false;
static bool g_ledBound      = false;
static bool g_ledAutoTune   = false;

// KO: 옄씠濡 由ъ뀑 븷땲硫붿씠뀡 (끉釉붾줈궧)
// EN: Gyro reset animation (non-blocking)
static bool     s_ledGyroAnimActive = false;
static uint8_t  s_ledGyroAnimCount  = 0;   // KO: ON 떒怨 셿猷 슏닔 / EN: ON phases completed
static bool     s_ledGyroAnimOn     = false;
static uint32_t s_ledGyroNextMs     = 0;

static inline void ledStartGyroResetAnim(){
  s_ledGyroAnimActive = true;
  s_ledGyroAnimCount  = 0;
  s_ledGyroAnimOn     = true;
  s_ledGyroNextMs     = millis();
}

// KO: 遺똿/諛붿씤뵫 떆뒪
// EN: Boot/binding sequence
enum LedBootState : uint8_t {
  LED_BOOT_BLINK_ALL = 0,
  LED_BOOT_CHASE_INVERTED = 1,
  LED_BOOT_BOUND_SOLID = 2,
  LED_BOOT_DONE = 3
};
static LedBootState s_ledBootState = LED_BOOT_BLINK_ALL;
static uint32_t s_ledBootStartMs = 0;
static bool s_ledPrevBound = false;

static inline bool _blink(uint32_t now, uint32_t onMs, uint32_t offMs){
  uint32_t period = onMs + offMs;
  uint32_t t = now % period;
  return (t < onMs);
}

static inline void _ledOneHot(uint8_t idx){
  digitalWrite(PIN_LED_IMU,  idx==0 ? HIGH : LOW);
  digitalWrite(PIN_LED_BARO, idx==1 ? HIGH : LOW);
  digitalWrite(PIN_LED_TOF,  idx==2 ? HIGH : LOW);
  digitalWrite(PIN_LED_FLOW, idx==3 ? HIGH : LOW);
}

// KO: 珥덇린솕 떆 1쉶 샇異 (IMU 泥댄겕 썑 inverted 뵆옒洹 쑀슚)
// EN: Call once during init (after IMU check so inverted flag is valid)
static inline void ledBootBegin(bool invertedAtBoot){
  g_ledInvertedAtBoot = invertedAtBoot;
  s_ledBootStartMs = millis();
  s_ledPrevBound = g_ledBound;
  s_ledBootState = invertedAtBoot ? LED_BOOT_CHASE_INVERTED : LED_BOOT_BLINK_ALL;
}

// KO: 궡遺 遺똿/諛붿씤뵫 뾽뜲씠듃 (씠 tick뿉꽌 LED 泥섎━ 떆 true)
// EN: Internal boot/bind updater (returns true if it handled LEDs this tick)
static inline bool ledBootTick(){
  uint32_t now = millis();

  // KO: 諛붿씤뵫 긽듅 뿉吏 媛먯
  // EN: Detect rising edge: not bound -> bound
  if (!s_ledPrevBound && g_ledBound) {
    s_ledBootState = LED_BOOT_BOUND_SOLID;
    s_ledBootStartMs = now;
  }
  s_ledPrevBound = g_ledBound;

  if (s_ledBootState == LED_BOOT_BOUND_SOLID) {
    ledAll(true);
    if (now - s_ledBootStartMs >= 1000) {
      s_ledBootState = LED_BOOT_DONE;
    }
    return true;
  }

  if (s_ledBootState == LED_BOOT_DONE) return false;

  if (s_ledBootState == LED_BOOT_BLINK_ALL) {
    bool on = _blink(now, 1000, 1000);
    ledAll(on);
    return true;
  }

  // KO: 뮘吏묓옒 泥댁씠뒪 (200ms 뒪뀦)
  // EN: inverted chase (200ms step)
  if (s_ledBootState == LED_BOOT_CHASE_INVERTED) {
    uint8_t step = (uint8_t)((now / 200) % 4);
    _ledOneHot(step);
    return true;
  }
  return false;
}

// KO: 硫붿씤 LED 뾽뜲씠듃 (留 猷⑦봽 샇異)
// EN: Main LED update (call every loop)
static inline void ledTick(){
  uint32_t now = millis();

  // KO: 0) 遺똿/諛붿씤뵫 슦꽑
  // EN: 0) Boot/bind sequence has top priority
  if (ledBootTick()) return;

  // KO: A) 쟾븬
  // EN: A) Low battery
  if (g_ledLowBattery) {
    bool on = _blink(now, 1000, 1000); // KO: 1s on / 1s off / EN: 1s on / 1s off
    ledAll(on);
    return;
  }

  // KO: B) 옄씠濡 由ъ뀑 븷땲硫붿씠뀡
  // EN: B) Gyro reset animation
  if (s_ledGyroAnimActive) {
    if (now >= s_ledGyroNextMs) {
      if (s_ledGyroAnimOn) {
        s_ledGyroAnimOn = false;
        s_ledGyroNextMs = now + 500;
      } else {
        s_ledGyroAnimOn = true;
        s_ledGyroNextMs = now + 1000;
        s_ledGyroAnimCount++;
        if (s_ledGyroAnimCount >= 3) {
          s_ledGyroAnimActive = false;
        }
      }
    }
    ledAll(s_ledGyroAnimOn);
    return;
  }

  // KO: C) 플립 준비: 모든 LED가 0.5초 ON/0.5초 OFF 점멸.
  // EN: C) Flip ready (all LEDs 0.5s on / 0.5s off)
  if (g_ledFlipReady) {
    bool on = _blink(now, 500, 500);
    ledAll(on);
    return;
  }

  // KO: D) 헤드리스 모드: LED1 LED2는 상시 ON, LED3 LED4는 2초 ON/1초 OFF 점멸.
  // EN: D) Headless (LED1/2 ON, LED3/4 2s on / 1s off)
  if (g_ledHeadless) {
    bool on34 = (now % 3000) < 2000;
    digitalWrite(PIN_LED_IMU,  HIGH);
    digitalWrite(PIN_LED_BARO, HIGH);
    digitalWrite(PIN_LED_TOF,  on34 ? HIGH : LOW);
    digitalWrite(PIN_LED_FLOW, on34 ? HIGH : LOW);
    return;
  }

  // KO: E) 자동 튜닝: LED1 LED2 ON, LED3 LED4 OFF로 시작해 1.5초마다 교대.
  // EN: E) Auto tune (LED1/2 on, LED3/4 off, swap every 1.5s)
  if (g_ledAutoTune) {
    bool phase = ((now / 1500) % 2) == 0;
    digitalWrite(PIN_LED_IMU,  phase ? HIGH : LOW);
    digitalWrite(PIN_LED_BARO, phase ? HIGH : LOW);
    digitalWrite(PIN_LED_TOF,  phase ? LOW  : HIGH);
    digitalWrite(PIN_LED_FLOW, phase ? LOW  : HIGH);
    return;
  }

  // KO: F) 정상 상태: LED1~LED3는 센서 OK 표시. LED4는 Flow가 OK이고 AUX2로 Flow가 활성화된 경우에만 ON.
  // EN: F) Normal (keep existing sensor LED logic)
}

static inline float readBatteryVoltage(){
  int raw = analogRead(PIN_BAT_ADC);
  float vadc = (raw / 4095.0f) * 3.3f;
  float divider = (BATTERY_RTOP_KOHM + BATTERY_RBOT_KOHM) / BATTERY_RBOT_KOHM;
  float vbat = vadc * divider;
  if(BATTERY_CAL_ACTUAL_V > 0.0f && BATTERY_CAL_MEASURED_V > 0.0f){
    vbat *= (BATTERY_CAL_ACTUAL_V / BATTERY_CAL_MEASURED_V);
  }
  return vbat;
}

// KO: 諛고꽣由 泥댄겕
// EN: Battery check
static inline void checkBattery(){
  batteryVoltage = readBatteryVoltage();

  // KO: EMA 濡쒖슦뙣뒪 (吏꽣 諛⑹)
  // EN: EMA low-pass to prevent jitter
  const float alpha = 0.20f; // KO: 0..1 (겢닔濡 鍮좊쫫) / EN: 0..1 (higher=faster)
  batteryVoltageFilt = batteryVoltageFilt + alpha * (batteryVoltage - batteryVoltageFilt);

  if(batteryVoltageFilt < BATTERY_MIN_VOLTAGE){
    if(!batteryCritical){
      batteryCritical = true;
      LOG_PRINTF("슑截 CRITICAL: Battery %.2fV - Emergency landing!\n", batteryVoltageFilt);
    }
    // KO: 鍮꾪뻾 以묒씠硫 媛뺤젣 李⑸쪠
    // EN: Force landing if in flight
    if(currentMode == MODE_HOVERING || currentMode == MODE_TAKEOFF){
      currentMode = MODE_LANDING;
    }
  } else if(batteryVoltageFilt < BATTERY_WARNING_VOLTAGE){
    if(!batteryLowWarning){
      batteryLowWarning = true;
      LOG_PRINTF("슑截 WARNING: Battery low %.2fV\n", batteryVoltageFilt);
    }
  } else {
    batteryLowWarning = false;
    batteryCritical = false;
  }
}



// ============================================================================
// KO: 諛붿씤뵫 LED 썒 (AF1000X_BINDING.h 궗슜)
// EN: Binding LED hooks (used by AF1000X_BINDING.h)
// KO: 떎젣 뙣꽩 ledTick()/ledBootBegin()/g_ledBound濡 젣뼱
// EN: Actual patterns controlled by ledTick()/ledBootBegin()/g_ledBound
// ============================================================================
inline void binding_ledAll(bool on) { ledAll(on); }

// KO: 럹뼱留/諛붿씤뵫 湲 以 諛섎났 샇異
// EN: Called repeatedly while waiting for pairing/binding
inline void binding_ledPairingTick() {
  ledTick();
}

// KO: 諛붿씤뵫 셿猷 떆 1쉶 샇異
// EN: Called once when binding completes
inline void binding_ledBoundOnce() {
  ledAll(true);
}

// KO: 諛붿씤뵫 뿉윭 떆 샇異
// EN: Called on binding error
inline void binding_ledErrorOnce() {
  // KO: 吏㏃ 뿉윭 뵆옒떆
  // EN: quick error flash
  for(int i=0;i<3;i++){ ledAll(true); delay(120); ledAll(false); delay(120); }
}

static inline void motorsInit(){
  pinMode(PIN_M1, OUTPUT);
  pinMode(PIN_M2, OUTPUT);
  pinMode(PIN_M3, OUTPUT);
  pinMode(PIN_M4, OUTPUT);

  ch1 = ledcAttach(PIN_M1, PWM_FREQ, PWM_RES);
  ch2 = ledcAttach(PIN_M2, PWM_FREQ, PWM_RES);
  ch3 = ledcAttach(PIN_M3, PWM_FREQ, PWM_RES);
  ch4 = ledcAttach(PIN_M4, PWM_FREQ, PWM_RES);

  ledcWrite(ch1, 0);
  ledcWrite(ch2, 0);
  ledcWrite(ch3, 0);
  ledcWrite(ch4, 0);
}
static inline void motorsOff(){
  ledcWrite(ch1, 0);
  ledcWrite(ch2, 0);
  ledcWrite(ch3, 0);
  ledcWrite(ch4, 0);
}
static inline void motorsIdle(int pwm){
  int v = constrain(pwm, 0, 255);
  ledcWrite(ch1, v);
  ledcWrite(ch2, v);
  ledcWrite(ch3, v);
  ledcWrite(ch4, v);
}
static inline void motorControl(int thr, float r, float p, float ycmd){
  // EN: In READY/EMERGENCY, keep motors off unless turtle mode is active
  if((currentMode==MODE_READY || currentMode==MODE_EMERGENCY) && !turtleActive){
    motorsOff();
    return;
  }

  float fr = r + trimRoll;
  float fp = p + trimPitch;

  int m1 = constrain((int)(thr - fr + fp + ycmd), 0, 255);
  int m2 = constrain((int)(thr - fr - fp - ycmd), 0, 255);
  int m3 = constrain((int)(thr + fr - fp + ycmd), 0, 255);
  int m4 = constrain((int)(thr + fr + fp - ycmd), 0, 255);

  ledcWrite(ch1, m1);
  ledcWrite(ch2, m2);
  ledcWrite(ch3, m3);
  ledcWrite(ch4, m4);
}

// ============================================================================
// KO: RF
// EN: RF
// ============================================================================
static inline void radioInit(){
  radio.begin();
  radio.setChannel(PAIR_CHANNEL);
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.openReadingPipe(1, RF_ADDR);
  radio.startListening();
  lastRxMs = millis();
}

static inline uint8_t hopIndexForMs(uint32_t now){
  if(HOP_SLOT_MS == 0 || hopLen == 0) return 0;
  uint32_t slots = (now - hopStartMs) / HOP_SLOT_MS;
  return (uint8_t)((slots + hopSeed) % hopLen);
}

static inline void fhssSetChannelIdx(uint8_t idx){
  if(hopLen == 0) return;
  if(idx >= hopLen) idx = 0;
  if(idx == hopIdxLast) return;
  hopIdxLast = idx;
  radio.stopListening();
  radio.setChannel(hopTable[idx]);
  radio.startListening();
}

static inline void fhssScanTick(uint32_t now){
  if(hopLen == 0) return;
  if(now - hopScanMs >= HOP_SCAN_MS){
    hopScanMs = now;
    hopScanIdx = (uint8_t)((hopScanIdx + 1) % hopLen);
    fhssSetChannelIdx(hopScanIdx);
  }
}

static inline void fhssResync(uint8_t rxHop, uint32_t now){
  if(hopLen == 0) return;
  if(rxHop >= hopLen) return;
  uint8_t seedMod = (uint8_t)(hopSeed % hopLen);
  uint8_t offset = (uint8_t)((rxHop + hopLen - seedMod) % hopLen);
  hopStartMs = now - (uint32_t)offset * HOP_SLOT_MS;
  hopSynced = true;
}

static inline void fhssUpdate(uint32_t now){
  if(hopLen == 0){
    fhssSetChannelIdx(0);
    return;
  }
  if(hopSynced){
    uint8_t idx = hopIndexForMs(now);
    fhssSetChannelIdx(idx);
  } else {
    fhssScanTick(now);
  }
}

static inline void fhssInit(){
  if(hopLen == 0 || hopLen > HOP_MAX){
    hopLen = HOP_MAX;
    for(uint8_t i = 0; i < HOP_MAX; i++) hopTable[i] = HOP_DEFAULT[i];
  }
  hopSeed = binding_getHopSeed();
  if(hopSeed == 0) hopSeed = 1;
  hopStartMs = millis();
  hopSynced = false;
  hopIdxLast = 0xFF;
  hopScanIdx = 0;
  hopScanMs = hopStartMs;
}
static inline void updateRadio(){
  uint32_t now = millis();
  fhssUpdate(now);

  if(radio.available()){
    radio.read(&receiverDataRc, sizeof(Signal));
    lastRxMs = now;
    failsafe = false;
    fhssResync(receiverDataRc.hop, now);

    if(receiverDataRc.speed >= 1 && receiverDataRc.speed <= 3){
      speedLevel = receiverDataRc.speed - 1;
    }

    telemetryData.vbat = batteryVoltage;
    telemetryData.alt  = currentAltitude;
    telemetryData.posX = currentPosX;
    telemetryData.posY = currentPosY;
    telemetryData.rssi = 1;

    radio.writeAckPayload(1, &telemetryData, sizeof(Telemetry));
  }

  if(now - lastRxMs > FAILSAFE_MS){
  if(!wifiActive){
    failsafe = true;
    hopSynced = false;
    if(currentMode != MODE_READY && currentMode != MODE_EMERGENCY){
      // EN: On failsafe, hold current pos/yaw then land
      targetPosX = currentPosX;
      targetPosY = currentPosY;
      targetYaw  = currentYaw;

      currentMode = MODE_LANDING;
    }
  } else {
    failsafe = false;
  }
}

}

// ============================================================================
// KO: IMU (ICM45686) 슂 쟻遺
// EN: IMU (ICM45686) yaw integration
// ============================================================================
static inline float safeDt(float dt){ return (dt<=0.0f || dt>0.1f) ? 0.0f : dt; }

static inline bool imu_init_auto(){
  int ret = IMU0.begin();
  if(ret == 0) IMU = &IMU0;
  else {
    ret = IMU1.begin();
    if(ret == 0) IMU = &IMU1;
  }
  if(IMU == nullptr) return false;

  IMU->startAccel(200, 16);
  IMU->startGyro (200, 2000);

  gyroX_bias = 0.0f;
  gyroY_bias = 0.0f;
  gyroZ_bias = 0.0f;
  yaw_internal = 0.0f;
  currentRoll = 0.0f;
  currentPitch = 0.0f;
  lastYawUs = micros();
  return true;
}
static inline bool imu_calibrate_gyroZ(uint16_t samples=800, uint16_t delay_ms=2){
  if(!ok_imu || IMU == nullptr) return false;

  float sumX=0.0f, sumY=0.0f, sumZ=0.0f; uint16_t got=0;
  inv_imu_sensor_data_t d{};
  for(uint16_t i=0;i<samples;i++){
    if(IMU->getDataFromRegisters(d) == 0){
      sumX += (float)d.gyro_data[0];
      sumY += (float)d.gyro_data[1];
      sumZ += (float)d.gyro_data[2];
      got++;
    }
    delay(delay_ms);
  }
  if(got < (uint16_t)(samples*0.6f)) return false;

  gyroX_bias = sumX / (float)got;
  gyroY_bias = sumY / (float)got;
  gyroZ_bias = sumZ / (float)got;
  yaw_internal = 0.0f;
  currentRoll = 0.0f;
  currentPitch = 0.0f;
  lastYawUs = micros();
  return true;
}
static inline void imu_updateYaw(){
  if(!ok_imu || IMU == nullptr) return;
  inv_imu_sensor_data_t d{};
  if(IMU->getDataFromRegisters(d) != 0) return;

  float ax = (float)d.accel_data[0];
  float ay = (float)d.accel_data[1];
  float az = (float)d.accel_data[2];

  uint32_t now = micros();
  float dt = (now - lastYawUs) * 1e-6f;
  lastYawUs = now;
  dt = safeDt(dt);
  if(dt <= 0.0f) return;

  float gx = (float)d.gyro_data[0] - gyroX_bias;
  float gy = (float)d.gyro_data[1] - gyroY_bias;
  float gz = (float)d.gyro_data[2] - gyroZ_bias;
  gyroX_dps = gx;
  gyroY_dps = gy;

  // KO: 濡/뵾移 긽蹂 븘꽣 (옄씠濡 + 媛냽룄)
  // EN: Complementary filter for roll/pitch (gyro + accel)
  float accRoll = currentRoll;
  float accPitch = currentPitch;
  if (fabsf(az) >= accZMin) {
    // KO: 떒쐞/뒪耳씪씠 嫄곗튌뼱룄 뮘吏묓옒 媛먯뿉뒗 異⑸텇
    // EN: Scale need not be perfect for inverted detection
    accRoll  = atan2f(ay, az) * 57.2957795f;
    accPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2957795f;
  }
  float alpha = attTau / (attTau + dt);
  currentRoll  = alpha * (currentRoll  + gx * dt) + (1.0f - alpha) * accRoll;
  currentPitch = alpha * (currentPitch + gy * dt) + (1.0f - alpha) * accPitch;

  yaw_internal += gz * dt;

  while(yaw_internal > 180.0f) yaw_internal -= 360.0f;
  while(yaw_internal < -180.0f) yaw_internal += 360.0f;

  currentYaw = yaw_internal;
}
static inline void yawReset(){
  yaw_internal = 0.0f;
  lastYawUs = micros();
  currentYaw = 0.0f;
  targetYaw = 0.0f;
}

// ============================================================================
// KO: SPL06 理쒖냼 뱶씪씠踰 (I2C -> 븬젰 -> 怨좊룄)
// EN: SPL06 minimal driver (I2C -> pressure -> altitude)
// ============================================================================
static uint8_t  spl_addr = 0;
static bool     spl_ok = false;
static int16_t  spl_c0, spl_c1;
static int32_t  spl_c00, spl_c10;
static int16_t  spl_c01, spl_c11, spl_c20, spl_c21;
static int16_t  spl_c30 = 0;
static uint32_t kT = 524288;
static uint32_t kP = 253952;

static inline bool i2cProbe(uint8_t addr){
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}
static inline bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, size_t n){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if(Wire.endTransmission(false) != 0) return false;
  if(Wire.requestFrom((int)addr, (int)n) != (int)n) return false;
  for(size_t i=0;i<n;i++) buf[i]=Wire.read();
  return true;
}
static inline bool i2cWrite1(uint8_t addr, uint8_t reg, uint8_t val){
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}
static inline int32_t getTwosComplement(uint32_t raw, uint8_t len){
  if(raw & ((uint32_t)1 << (len-1))){
    return (int32_t)(raw - ((uint32_t)1 << len));
  }
  return (int32_t)raw;
}
static inline bool spl_init(){
  const uint8_t addrs[]={0x76,0x77};
  for(auto a : addrs){
    if(i2cProbe(a)){
      spl_addr = a;
      break;
    }
  }
  if(!spl_addr) return false;

  uint8_t id=0;
  if(!i2cRead(spl_addr, 0x0D, &id,1)) return false;
  if((id&0xF0)!=0x10) return false;

  uint8_t coef[18];
  if(!i2cRead(spl_addr, 0x10, coef, 18)) return false;

  spl_c0  = (int16_t)(((uint16_t)coef[0] << 4) | (((uint16_t)coef[1] >> 4) & 0x0F));
  spl_c0  = (int16_t)getTwosComplement((uint32_t)spl_c0, 12);

  spl_c1  = (int16_t)((((uint16_t)coef[1] & 0x0F) << 8) | (uint16_t)coef[2]);
  spl_c1  = (int16_t)getTwosComplement((uint32_t)spl_c1, 12);

  spl_c00 = (int32_t)(((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (((uint32_t)coef[5] >> 4) & 0x0F));
  spl_c00 = getTwosComplement((uint32_t)spl_c00, 20);

  spl_c10 = (int32_t)((((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | (uint32_t)coef[7]);
  spl_c10 = getTwosComplement((uint32_t)spl_c10, 20);

  spl_c01 = (int16_t)(((uint16_t)coef[8] << 8) | (uint16_t)coef[9]);
  spl_c11 = (int16_t)(((uint16_t)coef[10] << 8) | (uint16_t)coef[11]);
  spl_c20 = (int16_t)(((uint16_t)coef[12] << 8) | (uint16_t)coef[13]);
  spl_c21 = (int16_t)(((uint16_t)coef[14] << 8) | (uint16_t)coef[15]);
  spl_c30 = (int16_t)(((uint16_t)coef[16] << 8) | (uint16_t)coef[17]);

  uint8_t prs_cfg = (0x01 << 4) | 0x03;
  uint8_t tmp_cfg = (0x80) | (0x01 << 4) | 0x03;
  if(!i2cWrite1(spl_addr, 0x06, prs_cfg)) return false;
  if(!i2cWrite1(spl_addr, 0x07, tmp_cfg)) return false;

  uint8_t cfg = 0x00 | 0x04;
  if(!i2cWrite1(spl_addr, 0x08, cfg)) return false;

  uint8_t meas = 0x07;
  if(!i2cWrite1(spl_addr, 0x09, meas)) return false;

  spl_ok = true;
  return true;
}
static inline bool spl_readPressurePa(float &out){
  if(!spl_ok) return false;
  uint8_t buf[6];
  if(!i2cRead(spl_addr, 0x00, buf,6)) return false;

  int32_t praw = (int32_t)((((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2]));
  praw = getTwosComplement((uint32_t)praw, 24);

  int32_t traw = (int32_t)((((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5]));
  traw = getTwosComplement((uint32_t)traw, 24);

  float praw_sc = (float)praw / (float)kP;
  float traw_sc = (float)traw / (float)kT;

  float pComp = (float)spl_c00
               + praw_sc * ((float)spl_c10 + praw_sc * ((float)spl_c20 + praw_sc * (float)spl_c30))
               + traw_sc * ((float)spl_c01)
               + traw_sc * praw_sc * ((float)spl_c11 + praw_sc * (float)spl_c21);

  out = pComp;
  return true;
}

// ============================================================================
// KO: ToF (VL53L1X)
// EN: ToF (VL53L1X)
// ============================================================================
static inline bool tof_init(){
  pinMode(PIN_VL53_XSHUT, OUTPUT);
  digitalWrite(PIN_VL53_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_VL53_XSHUT, HIGH);
  delay(10);

  tof.setTimeout(500);
  if(!tof.init()) return false;

  tof.setDistanceMode(VL53L1X::Short);
  tof.setMeasurementTimingBudget(20000);
  tof.startContinuous(20);
  return true;
}
static inline bool tof_read_m(float &out){
  uint16_t mm = tof.read(false);
  if(tof.timeoutOccurred()) return false;
  out = (float)mm * 0.001f;
  return true;
}

// ============================================================================
// KO: PMW3901 理쒖냼 뱶씪씠踰 (SPI 샃떚而 뵆濡쒖슦)
// EN: PMW3901 minimal (SPI) optical flow
// ============================================================================
static uint8_t pmw_read(uint8_t reg){
  digitalWrite(PIN_FLOW_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(reg & 0x7F);
  delayMicroseconds(160);
  uint8_t val = SPI.transfer(0);
  digitalWrite(PIN_FLOW_CS, HIGH);
  delayMicroseconds(1);
  return val;
}
static void pmw_write(uint8_t reg, uint8_t val){
  digitalWrite(PIN_FLOW_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(reg | 0x80);
  SPI.transfer(val);
  digitalWrite(PIN_FLOW_CS, HIGH);
  delayMicroseconds(1);
}
static inline bool pmw_init(){
  pinMode(PIN_FLOW_CS, OUTPUT);
  digitalWrite(PIN_FLOW_CS, HIGH);
  pinMode(PIN_FLOW_MOTION, INPUT);

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE3));

  pmw_write(0x3A, 0x5A);
  delay(5);

  uint8_t pid = pmw_read(0x00);
  uint8_t inv_pid = pmw_read(0x5F);
  if(pid != 0x49 || inv_pid != 0xB6) return false;

  pmw_write(0x7F, 0x00);
  pmw_write(0x55, 0x01);
  pmw_write(0x50, 0x07);
  pmw_write(0x7F, 0x0E);
  pmw_write(0x43, 0x10);

  pmw_write(0x7F, 0x00);
  pmw_write(0x51, 0x7B);
  pmw_write(0x50, 0x00);
  pmw_write(0x55, 0x00);
  pmw_write(0x7F, 0x0E);
  pmw_write(0x43, 0x00);
  pmw_write(0x7F, 0x00);

  delay(10);
  return true;
}
struct PMWData {
  int16_t dx, dy;
  uint8_t squal;
  bool valid;
};
static inline PMWData pmw_burst(){
  PMWData r{0,0,0,false};

  pmw_write(0x7F, 0x00);
  digitalWrite(PIN_FLOW_CS, LOW);
  delayMicroseconds(2);
  SPI.transfer(0x16);
  delayMicroseconds(35);

  uint8_t buf[12];
  for(int i=0;i<12;i++) buf[i] = SPI.transfer(0);
  digitalWrite(PIN_FLOW_CS, HIGH);

  r.dx = (int16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
  r.dy = (int16_t)((uint16_t)buf[4] | ((uint16_t)buf[5] << 8));
  r.squal = buf[6];
  r.valid = (buf[0] & 0x80) ? true : false;

  return r;
}

// ============================================================================
// KO: 怨좊룄 쑖빀 (ToF + Baro)
// EN: Altitude fusion (ToF + Baro)
// ============================================================================
static inline void updateAltitudeFusion(){
  float tofNew;
  if(ok_tof){
    if(tof_read_m(tofNew)){
      tofFailCount = 0;
      if(!tofFiltInit){
        tof_m_filt = tofNew;
        tof_m_last_raw = tofNew;
        tofFiltInit = true;
      } else {
        if(fabsf(tofNew - tof_m_last_raw) > tofJumpRejectM){
          // KO: 급격한 점프 거부
          // EN: reject sudden jump
        } else {
          tof_m_filt += tofEmaA * (tofNew - tof_m_filt);
          tof_m_last_raw = tofNew;
        }
      }
    } else {
      if(tofFailCount < 255) tofFailCount++;
      if(tofFailCount >= TOF_FAIL_LIMIT){
        ok_tof = false;
        tofFiltInit = false;
      }
    }
  }

  float pPa=0.0f;
  if(ok_baro && spl_readPressurePa(pPa)){
    float h_m = (1.0f - powf(pPa/basePressurePa, 0.190295f)) * 44307.7f;
    if(!baroInit){
      baro_alt_m_filt = h_m;
      baroInit = true;
    } else {
      baro_alt_m_filt += baroEmaA * (h_m - baro_alt_m_filt);
    }
  }

  float fused = 0.0f;
  if(tofFiltInit && baroInit){
    if(tof_m_filt < ALT_SWITCH_M){
      fused = tof_m_filt;
    } else {
      fused = baro_alt_m_filt;
    }
  } else if(tofFiltInit){
    fused = tof_m_filt;
  } else if(baroInit){
    fused = baro_alt_m_filt;
  } else {
    fused = 0.0f;
  }

  if(!altInit){
    alt_m_filt = fused;
    alt_m_prev = fused;
    altInit = true;
  } else {
    alt_m_prev = alt_m_filt;
    alt_m_filt += altEmaA * (fused - alt_m_filt);
  }

  currentAltitude = alt_m_filt;
}

// ============================================================================
// KO: 뵆濡쒖슦 + 슂 쉶쟾쑝濡 쐞移 異붿젙
// EN: Position from flow + yaw rotation
// ============================================================================
static inline void updatePositionFromFlow(){
  if(!ok_flow || !flowUserEnabled) return;

  PMWData f = pmw_burst();
  if(!f.valid || f.squal < FLOW_SQUAL_MIN) return;

  // KO: 뵆濡쒖슦 罹섎━釉뚮젅씠뀡 늻쟻
  // EN: Flow calibration accumulation
  if(flowCal == FC_FORWARD || flowCal == FC_BACK){
    if(currentAltitude > 0.20f){
      flowCal_sum_dx += f.dx;
      flowCal_samples++;
      flowCal_height_acc += currentAltitude;
      flowCal_height_n++;
    }
  }

  float h = currentAltitude;
  if(h < 0.10f) h = 0.10f;

  float dX_body = (float)f.dx * flowK * h;
  float dY_body = (float)f.dy * flowK * h;

  // KO: 濡/뵾移 湲곕컲 떥듃 蹂댁젙 (1m 洹쇱쿂뿉꽌留)
  // EN: Tilt compensation using roll/pitch estimate (only near 1m)
  if(fabsf(currentAltitude - FLOW_TILT_COMP_ALT_M) <= FLOW_TILT_COMP_WINDOW_M){
    float rollRad = currentRoll * DEG_TO_RAD;
    float pitchRad = currentPitch * DEG_TO_RAD;
    float cosR = cosf(rollRad);
    float cosP = cosf(pitchRad);
    if(fabsf(cosR) < 0.35f) cosR = (cosR >= 0.0f) ? 0.35f : -0.35f;
    if(fabsf(cosP) < 0.35f) cosP = (cosP >= 0.0f) ? 0.35f : -0.35f;
    dX_body /= cosR;
    dY_body /= cosP;
  }

  if(fabsf(dX_body) > FLOW_MAX_STEP_M) dX_body = (dX_body>0) ? FLOW_MAX_STEP_M : -FLOW_MAX_STEP_M;
  if(fabsf(dY_body) > FLOW_MAX_STEP_M) dY_body = (dY_body>0) ? FLOW_MAX_STEP_M : -FLOW_MAX_STEP_M;

  float rad = currentYaw * DEG_TO_RAD;
  float c = cosf(rad);
  float s = sinf(rad);
  float dX_world = dX_body * c - dY_body * s;
  float dY_world = dX_body * s + dY_body * c;

  currentPosX += dX_world;
  currentPosY += dY_world;
}

// ============================================================================
// KO: 뵆濡쒖슦 옄룞 罹섎━釉뚮젅씠뀡
// EN: Flow auto calibration
// ============================================================================
static inline void startFlowAutoCal(float dist_m, float hoverAlt_m, float pwr){
  if(currentMode != MODE_READY){
    LOG_CMD_PRINTLN("FLOW CAL: must be in READY mode.");
    return;
  }

  flowCal_dist_m = dist_m;
  flowCal_hoverAlt_m = hoverAlt_m;
  flowCal_pwr = pwr;
  flowCal_sum_dx = 0;
  flowCal_samples = 0;
  flowCal_height_acc = 0.0f;
  flowCal_height_n = 0;
  flowCal_suggestK = 0.0f;
  flowCal_reported = false;

  flowCal = FC_TAKE_HOVER;
  flowCal_t0 = millis();

  hover_startLearn();

  LOG_CMD_PRINTF("FLOW CAL START: dist=%.2fm hover=%.2fm pwr=%.0f%%\n", dist_m, hoverAlt_m, pwr);
}

static inline void flowCalTick(){
  if(flowCal == FC_IDLE) return;

  switch(flowCal){
    case FC_TAKE_HOVER: {
      if(hover_isReady()){
        targetAltitude = flowCal_hoverAlt_m;
        if(fabsf(currentAltitude - flowCal_hoverAlt_m) < 0.10f){
          flowCal = FC_FORWARD;
          flowCal_t0 = millis();
          LOG_CMD_PRINTLN("FLOW CAL: hovering stable -> forward");
        }
      }
      if((millis() - flowCal_t0) > 15000){
        flowCal = FC_ABORT;
        LOG_CMD_PRINTLN("FLOW CAL: ABORT (timeout hover)");
      }
    } break;

    case FC_FORWARD: {
      float step_m = (moveSpeedSteps[speedLevel] * (flowCal_pwr/100.0f)) * g_loopDt;
      if(step_m < 0.002f) step_m = 0.002f;
      targetPosX += step_m;

      if(fabsf(currentPosX - targetPosX) > flowCal_dist_m){
        flowCal_t0 = millis();
        flowCal = FC_PAUSE1;
      }
    } break;

    case FC_PAUSE1: {
      if(millis() - flowCal_t0 > 500){
        flowCal_t0 = millis();
        flowCal = FC_BACK;
      }
    } break;

    case FC_BACK: {
      float step_m = (moveSpeedSteps[speedLevel] * (flowCal_pwr/100.0f)) * g_loopDt;
      if(step_m < 0.002f) step_m = 0.002f;
      targetPosX -= step_m;

      if(fabsf(currentPosX - targetPosX) < 0.10f || (millis() - flowCal_t0) > 6000){
        flowCal_t0 = millis();
        flowCal = FC_PAUSE2;
      }
    } break;

    case FC_PAUSE2: {
      if(millis() - flowCal_t0 > 500){
        if(flowCal_samples < 50 || flowCal_height_n < 50){
          flowCal = FC_ABORT;
          LOG_CMD_PRINTLN("FLOW CAL: ABORT (not enough samples)");
          break;
        }
        float avgH = flowCal_height_acc / (float)flowCal_height_n;
        float counts = (float)abs(flowCal_sum_dx);
        if(avgH < 0.10f || counts < 5.0f){
          flowCal = FC_ABORT;
          LOG_CMD_PRINTLN("FLOW CAL: ABORT (bad data)");
          break;
        }

        flowCal_suggestK = flowCal_dist_m / (avgH * counts);
        flowK = flowCal_suggestK;
        prefs.putFloat("flowK", flowK);

        flowCal = FC_DONE;
        LOG_CMD_PRINTLN("FLOW CAL: DONE");
      }
    } break;

    default: break;
  }

  if(flowCal == FC_DONE && !flowCal_reported){
    flowCal_reported = true;
    float avgH = (flowCal_height_n ? flowCal_height_acc/(float)flowCal_height_n : 0.0f);
    LOG_CMD_PRINTF("FLOW CAL RESULT: avgH=%.2fm dxCounts=%ld flowK=%.6f (saved)\n",
                  avgH, (long)flowCal_sum_dx, flowK);
    LOG_CMD_PRINTLN("If measured actual distance, send: D60.0  (cm)");
  }
}

// ============================================================================
// KO: 뒠떇 뜡봽 뿬띁 (Serial)
// EN: Tuning dump helpers (Serial)
// ============================================================================
static inline void printTuning(bool asPidDefaults){
  if(asPidDefaults){
    LOG_CFG_PRINTLN("---- PID.h defaults (copy) ----");
    LOG_CFG_PRINTF("static const float ALT_KP_DEFAULT = %.4ff;\n", altKp);
    LOG_CFG_PRINTF("static const float ALT_KD_DEFAULT = %.4ff;\n", altKd);
    LOG_CFG_PRINTF("static const float YAW_KP_DEFAULT = %.4ff;\n", yawKp);
    LOG_CFG_PRINTF("static const float FLOW_K_DEFAULT = %.6ff;\n", flowK);
    LOG_CFG_PRINTF("static const int   HOVER_PWM_DEFAULT = %d;\n", hoverThrottle);
    LOG_CFG_PRINTF("static const float ATT_TAU = %.3ff;\n", attTau);
    LOG_CFG_PRINTF("static const float ACC_Z_MIN = %.3ff;\n", accZMin);
    LOG_CFG_PRINTF("static const float TOF_EMA_A = %.3ff;\n", tofEmaA);
    LOG_CFG_PRINTF("static const float BARO_EMA_A = %.3ff;\n", baroEmaA);
    LOG_CFG_PRINTF("static const float ALT_EMA_A = %.3ff;\n", altEmaA);
    LOG_CFG_PRINTF("static const float TOF_JUMP_REJECT_M = %.3ff;\n", tofJumpRejectM);
    LOG_CFG_PRINTLN("---- end ----");
    return;
  }

  LOG_CFG_PRINTLN("---- Tuning dump ----");
  LOG_CFG_PRINTF("ALT_KP=%.4f ALT_KD=%.4f YAW_KP=%.4f\n", altKp, altKd, yawKp);
  LOG_CFG_PRINTF("FLOW_K=%.6f HOVER_PWM=%d\n", flowK, hoverThrottle);
  LOG_CFG_PRINTF("ATT_TAU=%.3f ACC_Z_MIN=%.3f\n", attTau, accZMin);
  LOG_CFG_PRINTF("TOF_EMA_A=%.3f BARO_EMA_A=%.3f ALT_EMA_A=%.3f TOF_JUMP_REJECT_M=%.3f\n",
                tofEmaA, baroEmaA, altEmaA, tofJumpRejectM);
  LOG_CFG_PRINTF("GYRO_BIAS X=%.4f Y=%.4f Z=%.4f\n", gyroX_bias, gyroY_bias, gyroZ_bias);
  LOG_CFG_PRINTLN("---- end ----");
}

// ============================================================================
// KO: 떆由ъ뼹 紐낅졊 (뀒뒪듃/罹섎━釉뚮젅씠뀡/샇踰꾪븰뒿)
// EN: Serial commands (test/calibration/hover learn)
// ============================================================================
static bool imuStreamEnabled = false;
static uint32_t imuStreamNextMs = 0;
static const uint32_t IMU_STREAM_PERIOD_MS = 50; // 20 Hz

static inline void serialCommands(){
  // KO: 끉釉붾줈궧 씪씤 由щ뜑 (젣뼱 猷⑦봽 뒪넧 諛⑹)
  // EN: Non-blocking line reader (prevents control-loop stalls)
  static char cmdBuf[96];
  static uint8_t cmdLen = 0;

  while(Serial.available()){
    char c = (char)Serial.read();
    if(c == '\r') continue;

    if(c == '\n'){
      cmdBuf[cmdLen] = 0;
      cmdLen = 0;

      String s(cmdBuf);
      s.trim();
      if(s.length()==0) continue;

      if(s == "C"){ startFlowAutoCal(0.50f, 0.80f, 35.0f); continue; }
      if(s == "Y0"){ yawReset(); LOG_CMD_PRINTLN("Yaw reset."); continue; }
      if(s == "1510"){ // start IMU stream
        imuStreamEnabled = true;
        imuStreamNextMs = 0;
        LOG_CMD_PRINTLN("IMU STREAM ON");
        continue;
      }
      if(s == "1511"){ // stop IMU stream
        imuStreamEnabled = false;
        LOG_CMD_PRINTLN("IMU STREAM OFF");
        continue;
      }

      if(s == "T"){ // KO: 씠瑜(hover 븰뒿) / EN: takeoff (hover learn)
        if(currentMode == MODE_READY) {
          hover_startLearn();
          LOG_CMD_PRINTLN("Takeoff: hover learn started.");
        }
        continue;
      }
      if(s == "L"){
        if(currentMode != MODE_READY) currentMode = MODE_LANDING;
        LOG_CMD_PRINTLN("Landing.");
        continue;
      }
      if(s == "K"){
        currentMode = MODE_EMERGENCY;
        motorsOff();
        LOG_CMD_PRINTLN("KILL MOTORS!");
        continue;
      }

      if(s.startsWith("D")){
        float actual_cm = s.substring(1).toFloat();
        if(actual_cm > 5.0f && actual_cm < 300.0f){
          float avgH = (flowCal_height_n ? flowCal_height_acc/(float)flowCal_height_n : currentAltitude);
          float counts = (float)abs(flowCal_sum_dx);
          if(avgH > 0.10f && counts > 5.0f){
            float actual_m = actual_cm * 0.01f;
            flowK = actual_m / (avgH * counts);
            prefs.putFloat("flowK", flowK);
            LOG_CMD_PRINTF("FLOW CAL REFINE: actual=%.1fcm => flowK=%.6f (saved)\n", actual_cm, flowK);
          }
        }
        continue;
      }

      // KO: 諛고꽣由 泥댄겕 紐낅졊 (raw + filtered)
      // EN: Battery check command (raw + filtered)
      if(s == "B"){
        LOG_CMD_PRINTF("Battery: %.2fV (filt %.2fV)\n", batteryVoltage, batteryVoltageFilt);
        continue;
      }

      // KO: 뒠떇 뜡봽
      // EN: Tuning dump
      if(s == "PID?" || s == "TUNE?"){
        printTuning(false);
        continue;
      }
      // KO: AF1000X_PID.h 遺숈뿬꽔湲 뒪땲렖 異쒕젰
      // EN: Print snippet to paste into AF1000X_PID.h
      if(s == "PIDDEF"){
        printTuning(true);
        continue;
      }

      // KO: 븣 닔 뾾뒗 紐낅졊 臾댁떆
      // EN: Unknown command -> ignore
      continue;
    }

    // KO: 臾몄옄 늻쟻
    // EN: Accumulate characters
    if(cmdLen < (sizeof(cmdBuf)-1)){
      cmdBuf[cmdLen++] = c;
    } else {
      // KO: 삤踰꾪뵆濡쒖슦 -> 踰꾪띁 由ъ뀑
      // EN: overflow -> reset buffer
      cmdLen = 0;
    }
  }
}

static inline void imuStreamTick(uint32_t now){
  if(!imuStreamEnabled) return;
  if((int32_t)(now - imuStreamNextMs) < 0) return;
  imuStreamNextMs = now + IMU_STREAM_PERIOD_MS;
  LOG_IMU_PRINTF("IMU %.3f %.3f %.3f %.2f\n", currentRoll, currentPitch, currentYaw, batteryVoltageFilt);
}


// ============================================================================
// KO: POST + 罹섎━釉뚮젅씠뀡
// EN: POST + calibration
// ============================================================================
static inline bool postSensors(){
  ok_imu  = imu_init_auto();
  ok_baro = spl_init();
  ok_tof  = tof_init();
  ok_flow = pmw_init();

  ledSet(ok_imu, ok_baro, ok_tof, ok_flow);

  // Required = IMU + BARO (ToF/Flow optional)
  flightLock = !(ok_imu && ok_baro);

  const bool anyFail = !(ok_imu && ok_baro && ok_tof && ok_flow);
  if(anyFail){
    ledPostStatusDelay(ok_imu, ok_baro, ok_tof, ok_flow, 3000);
  }

  if(flightLock){
    LOG_PRINTF("POST FAIL -> FLIGHT LOCK (IMU=%d BARO=%d TOF=%d FLOW=%d)\n",
                  ok_imu, ok_baro, ok_tof, ok_flow);
    return false;
  }
  LOG_PRINTF("POST OK (IMU=%d BARO=%d TOF=%d FLOW=%d)\n",
                ok_imu, ok_baro, ok_tof, ok_flow);
  return true;
}

static inline bool calibrateSensors(){
  // KO: 湲곗븬 湲곗媛
  // EN: Baro baseline
  float sumP=0; int got=0;
  for(int i=0;i<200;i++){
    float p;
    if(spl_readPressurePa(p)){ sumP += p; got++; }
    delay(10);
  }
  if(got < 120) return false;
  basePressurePa = sumP / got;

  // KO: 븘꽣 由ъ뀑
  // EN: reset filters
  altInit=false;
  baroInit=false;
  tofFiltInit=false;
  tofFailCount = 0;

  // KO: IMU 옄씠濡 諛붿씠뼱뒪 (샃뀡)
  // EN: IMU gyro bias (optional)
  if(ok_imu){
    if(!imu_calibrate_gyroZ()){
      ok_imu = false;
    } else {
      tune_saveGyroBias();
    }
  }
  yawReset();

  calibrated = true;
  return true;
}

// ============================================================================
// KO: 怨듦컻 API
// EN: Public APIs
// ============================================================================
static inline void calibrate(){
  if(currentMode == MODE_READY){
    LOG_PRINTLN("AF: calibrate (ground)");
    if(!calibrateSensors()){
      LOG_PRINTLN("AF: calibrate FAIL -> lock");
      flightLock = true;
      ledFailPattern();
    } else {
      LOG_PRINTLN("AF: calibrate OK");
    }
  }
}

static inline void autoTakeoff(){
  if(flightLock){
    LOG_PRINTLN("AF: FLIGHT LOCK - takeoff blocked");
    currentMode = MODE_READY;
    motorsOff();
    return;
  }
  if(currentMode == MODE_READY){
    hover_startLearn(); // KO: A 諛⑹떇 떆옉 / EN: start A method
  }
}

static inline void autoLanding(){
  if(currentMode != MODE_READY){
    currentMode = MODE_LANDING;
  }
}

static inline void emergencyStop(){
  currentMode = MODE_EMERGENCY;
  motorsOff();
  motorArmedIdle = false;
  armAwaitSecond = false;
  armHighPrev = false;
  armPulsing = false;
  autoTakeoffPending = false;
  flipActive = false;
  flipDir = 0;
  turtleActive = false;
  turtleResetTrigger();
}

// ============================================================================
// KO: 플립(360도)
// EN: Flip (360-degree)
// ============================================================================
static inline bool flipCanStart(){
  if(flipActive) return false;
  if(millis() < flipCooldownUntil) return false;
  if(flightLock || failsafe) return false;
  if(!ok_imu) return false;
  if(g_ledHeadless) return false;
  if(currentMode != MODE_HOVERING) return false;
  if(batteryVoltageFilt < FLIP_MIN_VOLTAGE) return false;
  if(currentAltitude < FLIP_MIN_ALT_M) return false;
  return true;
}

static inline void flipStart(uint8_t dir){
  if(!flipCanStart()) return;
  flipActive = true;
  flipDir = dir;
  flipAngleDeg = 0.0f;
  flipStartMs = millis();
}

static inline void flipFinish(){
  flipActive = false;
  flipDir = 0;
  flipCooldownUntil = millis() + FLIP_COOLDOWN_MS;
  targetAltitude = currentAltitude;
  targetYaw = currentYaw;
  targetPosX = currentPosX;
  targetPosY = currentPosY;
}

static inline bool flipUpdate(){
  if(!flipActive) return false;
  if(failsafe || flightLock || currentMode == MODE_EMERGENCY){
    emergencyStop();
    flipActive = false;
    flipDir = 0;
    return false;
  }
  if(batteryVoltageFilt < FLIP_MIN_VOLTAGE){
    flipFinish();
    return false;
  }
  uint32_t now = millis();
  float rate = 0.0f;
  if(flipDir == AUX1_FLIP_ROLL_POS || flipDir == AUX1_FLIP_ROLL_NEG){
    rate = fabsf(gyroX_dps);
  } else if(flipDir == AUX1_FLIP_PITCH_POS || flipDir == AUX1_FLIP_PITCH_NEG){
    rate = fabsf(gyroY_dps);
  } else {
    flipFinish();
    return false;
  }
  flipAngleDeg += rate * g_loopDt;
  if(flipAngleDeg >= FLIP_TARGET_DEG || (now - flipStartMs) >= FLIP_MAX_TIME_MS){
    flipFinish();
    return false;
  }
  int thr = hoverThrottle + FLIP_THR_BOOST;
  thr = constrain(thr, 0, 255);
  float r = 0.0f;
  float p = 0.0f;
  if(flipDir == AUX1_FLIP_ROLL_POS) r = FLIP_RATE_CMD;
  else if(flipDir == AUX1_FLIP_ROLL_NEG) r = -FLIP_RATE_CMD;
  else if(flipDir == AUX1_FLIP_PITCH_POS) p = FLIP_RATE_CMD;
  else if(flipDir == AUX1_FLIP_PITCH_NEG) p = -FLIP_RATE_CMD;
  motorControl(thr, r, p, 0.0f);
  return true;
}

// ============================================================================
// EN: Self-righting (turtle mode)
// ============================================================================
static inline bool turtleIsInverted(){
  if(!ok_imu) return false;
  float absRoll = fabsf(currentRoll);
  float absPitch = fabsf(currentPitch);
  return (absRoll >= TURTLE_INVERTED_DEG || absPitch >= TURTLE_INVERTED_DEG);
}

static inline void turtleResetTrigger(){
  turtleDownCount = 0;
  turtleFirstDownMs = 0;
  turtleLowPrev = false;
}

static inline void turtleStop(){
  turtleActive = false;
  turtleDir = 0;
}

static inline void turtleStart(){
  if(currentMode != MODE_EMERGENCY) return;
  if(failsafe || flightLock || !ok_imu) return;
  if(!turtleIsInverted()) return;

  float absRoll = fabsf(currentRoll);
  float absPitch = fabsf(currentPitch);
  bool rollInv = absRoll >= TURTLE_INVERTED_DEG;
  bool pitchInv = absPitch >= TURTLE_INVERTED_DEG;
  if(!rollInv && !pitchInv) return;

  if(rollInv && (!pitchInv || absRoll <= absPitch)){
    turtleAxis = 0; // roll
    turtleDir = (currentRoll > 0.0f) ? -1 : 1;
  } else {
    turtleAxis = 1; // pitch
    turtleDir = (currentPitch > 0.0f) ? -1 : 1;
  }

  turtleActive = true;
  turtleStartMs = millis();
}

static inline bool turtleUpdate(){
  if(!turtleActive) return false;
  if(currentMode != MODE_EMERGENCY || failsafe || flightLock){
    turtleStop();
    return false;
  }
  if(!turtleIsInverted()){
    turtleStop();
    return false;
  }
  if(millis() - turtleStartMs >= TURTLE_MAX_TIME_MS){
    turtleStop();
    return false;
  }

  float r = 0.0f;
  float p = 0.0f;
  if(turtleAxis == 0) r = TURTLE_RATE_CMD * turtleDir;
  else p = TURTLE_RATE_CMD * turtleDir;

  motorControl(TURTLE_THR, r, p, 0.0f);
  return true;
}

static inline void turtleTriggerTick(){
  if(turtleActive) return;
  if(currentMode != MODE_EMERGENCY || failsafe || flightLock || !ok_imu){
    turtleResetTrigger();
    return;
  }
  if(!turtleIsInverted()){
    turtleResetTrigger();
    return;
  }

  uint32_t now = millis();
  bool low = (receiverData.throttle <= ARM_THROTTLE_LOW);

  if(low && !turtleLowPrev){
    if(turtleDownCount == 0) turtleFirstDownMs = now;
    if(now - turtleFirstDownMs <= TURTLE_WINDOW_MS){
      turtleDownCount++;
    } else {
      turtleDownCount = 1;
      turtleFirstDownMs = now;
    }
    if(turtleDownCount >= TURTLE_DOWN_COUNT){
      turtleResetTrigger();
      turtleStart();
    }
  }
  if(turtleDownCount > 0 && (now - turtleFirstDownMs > TURTLE_WINDOW_MS)){
    turtleResetTrigger();
  }
  turtleLowPrev = low;
}

// ============================================================================
// EN: updateSystem / updateFlight
// ============================================================================
static inline void updateSystem(){
  // KO: 諛붿씤뵫 긽깭 슦꽑 뾽뜲씠듃
  // EN: Always update binding state first
  binding_update();
  g_ledBound = binding_isBound();
  wifiInputTick();

  static bool lastBound = false;
  if(g_ledBound && !lastBound){
    fhssInit();
  }
  lastBound = g_ledBound;

  static bool lastLink = false;
  if(linkReady && !lastLink){
    fhssInit();
  }
  lastLink = linkReady;

  // KO: 諛고꽣由 泥댄겕 (10Hz, 諛붿씤뵫 쟾뿉룄)
  // EN: Battery check (10Hz) even before binding completes
  static uint32_t lastBatMs = 0;
  uint32_t nowBatMs = millis();
  if(nowBatMs - lastBatMs >= 100){
    lastBatMs = nowBatMs;
    checkBattery();
  }
  g_ledLowBattery = (batteryLowWarning || batteryCritical);
  bool batteryOkForArm = (batteryVoltageFilt >= BATTERY_ARM_MIN_VOLTAGE);

  // KO: 헤드리스 상태
  // EN: Headless flag (updated after we have fresh RX data)
  g_ledHeadless = false;
  g_ledFlipReady = false;
  g_ledAutoTune = tune_isActive();

  // KO: 遺똿/諛붿씤뵫 以묒뿉룄 LED 援щ룞
  // EN: Drive LEDs during boot/binding too
  ledTick();

  if (!g_ledBound || (!linkReady && !wifiActive)) {
    // KO: 誘몃컮슫뱶/留곹겕 誘몄鍮 -> 젣뼱/鍮꾪뻾 湲덉
    // EN: Not bound or link not ready -> no control / no flight
    currentMode = MODE_READY;
    motorsOff();
    motorArmedIdle = false;
    armAwaitSecond = false;
    armHighPrev = false;
    armPulsing = false;
    autoTakeoffPending = false;
    flipActive = false;
    flipDir = 0;
    turtleActive = false;
    turtleResetTrigger();
    g_ledFlipReady = false;
    return;
  }
  updateRadio();
  wifiSelectInput(receiverDataRc, receiverData);
  if(receiverData.speed >= 1 && receiverData.speed <= 3){
    speedLevel = receiverData.speed - 1;
  }

  // KO: AUX1=2 옄씠濡 珥덇린솕 슂泥 (READY + 뒪濡쒗 궙쓬)
  // EN: Gyro init request via AUX1=2 (READY + low throttle)
  {
    static uint32_t gyroReqStart = 0;
    static bool gyroReqLatched = false;
    if(receiverData.aux1 == AUX1_GYRO_RESET && currentMode == MODE_READY && receiverData.throttle < 20){
      if(gyroReqStart == 0) gyroReqStart = millis();
      if(!gyroReqLatched && (millis() - gyroReqStart >= 200)){
        if(imu_calibrate_gyroZ()){
          tune_saveGyroBias();
        }
        yawReset();
        ledStartGyroResetAnim();
        gyroReqLatched = true;
      }
    } else {
      gyroReqStart = 0;
      gyroReqLatched = false;
    }
  }

// KO: 헤드리스/Flow/플립 준비 (TX aux2 bits)
// EN: Headless/flow/flip ready (TX aux2 bits)
  g_ledHeadless = (receiverData.aux2 & AUX2_HEADLESS) != 0;
  flowUserEnabled = (receiverData.aux2 & AUX2_FLOW) != 0;
  g_ledFlipReady = (receiverData.aux2 & AUX2_FLIP_READY) != 0;
  if(g_ledHeadless) g_ledFlipReady = false;

  // EN: User outputs (servo/LED) via AUX1 pulses
  {
    static uint8_t lastAux1 = 0;
    uint8_t aux1 = receiverData.aux1;
    if(aux1 == AUX1_SERVO_TOGGLE && lastAux1 != AUX1_SERVO_TOGGLE){
      userServoToggle();
    }
    if(aux1 == AUX1_LED_STEP && lastAux1 != AUX1_LED_STEP){
      userLedStep();
    }
    if((aux1 == AUX1_FLIP_ROLL_POS || aux1 == AUX1_FLIP_ROLL_NEG ||
        aux1 == AUX1_FLIP_PITCH_POS || aux1 == AUX1_FLIP_PITCH_NEG) && lastAux1 != aux1){
      flipStart(aux1);
    }
    lastAux1 = aux1;
  }

  serialCommands();  

  if(ok_imu) imu_updateYaw();
  else currentYaw = 0.0f;
  imuStreamTick(millis());

  // EN: Turtle trigger (three throttle downs)
  turtleTriggerTick();

  bool levelOkForArm = ok_imu &&
                       (fabsf(currentRoll) <= ARM_LEVEL_DEG) &&
                       (fabsf(currentPitch) <= ARM_LEVEL_DEG);
  bool inverted = turtleIsInverted();
  bool armOk = (!flightLock && !failsafe && batteryOkForArm && levelOkForArm && !inverted);


  // EN: Motor arm (idle spin) - left stick double-up within window
  {
    if(currentMode != MODE_READY || !armOk){
      motorArmedIdle = false;
      armAwaitSecond = false;
      armHighPrev = false;
      armPulsing = false;
      armDisarmStartMs = 0;
      autoTakeoffPending = false;
    } else {
      bool high = receiverData.throttle >= ARM_THROTTLE_HIGH;

      if(armAwaitSecond && (millis() - armFirstUpMs > ARM_WINDOW_MS)){
        armAwaitSecond = false;
      }

      if(high && !armHighPrev){
        if(armAwaitSecond){
          motorArmedIdle = true;
          armAwaitSecond = false;
          armPulsing = true;
          armPulseStartMs = millis();
        } else {
          armAwaitSecond = true;
          armFirstUpMs = millis();
        }
      }

      armHighPrev = high;

      // EN: Disarm if throttle fully low for hold time
      if(motorArmedIdle && receiverData.throttle <= ARM_THROTTLE_LOW){
        if(armDisarmStartMs == 0) armDisarmStartMs = millis();
        if(millis() - armDisarmStartMs >= ARM_DISARM_HOLD_MS){
          motorArmedIdle = false;
          armPulsing = false;
          autoTakeoffPending = false;
        }
      } else {
        armDisarmStartMs = 0;
      }
    }
  }

  // EN: Auto takeoff / landing request (AUX1 pulse)
  {
    static bool aux1Prev = false;
    bool aux1Now = (receiverData.aux1 == AUX1_TAKEOFF);
    if(aux1Now && !aux1Prev){
      if(currentMode == MODE_READY && armOk){
        if(motorArmedIdle){
          autoTakeoffPending = true;
          autoTakeoffAtMs = millis();
        } else {
          motorArmedIdle = true;
          armPulsing = true;
          armPulseStartMs = millis();
          autoTakeoffPending = true;
          autoTakeoffAtMs = millis() + AUTO_TAKEOFF_DELAY_MS;
        }
        targetAltitude = currentAltitude;
        targetYaw = currentYaw;
        targetPosX = currentPosX;
        targetPosY = currentPosY;
      } else if(currentMode == MODE_TAKEOFF || currentMode == MODE_HOVERING){
        autoTakeoffPending = false;
        motorArmedIdle = false;
        armPulsing = false;
        autoLanding();
      }
    }
    aux1Prev = aux1Now;
  }

  if(autoTakeoffPending && currentMode == MODE_READY && armOk){
    if(millis() >= autoTakeoffAtMs){
      autoTakeoffPending = false;
      motorArmedIdle = false;
      armPulsing = false;
      targetAltitude = AUTO_TAKEOFF_ALT_M;
      targetYaw = currentYaw;
      targetPosX = currentPosX;
      targetPosY = currentPosY;
      currentMode = MODE_TAKEOFF;
    }
  }

  // EN: Takeoff start: if armed idle and throttle pushed up
  if(motorArmedIdle && currentMode == MODE_READY && armOk){
    if(receiverData.throttle >= ARM_TAKEOFF_THRESH){
      motorArmedIdle = false;
      armPulsing = false;
      autoTakeoffPending = false;
      targetAltitude = currentAltitude;
      targetYaw = currentYaw;
      currentMode = MODE_TAKEOFF;
    }
  }

  // EN: Throttle down -> TAKEOFF back to READY
  if(currentMode == MODE_TAKEOFF && receiverData.throttle <= ARM_THROTTLE_LOW){
    currentMode = MODE_READY;
    motorsOff();
    motorArmedIdle = false;
    armPulsing = false;
    autoTakeoffPending = false;
  }

  // EN: Tilt kill: emergency stop if roll/pitch exceeds threshold
  {
    static uint32_t tiltStartMs = 0;
    if(ok_imu && currentMode != MODE_READY && currentMode != MODE_EMERGENCY && !flipActive){
      float absRoll = fabsf(currentRoll);
      float absPitch = fabsf(currentPitch);
      if(absRoll > TILT_KILL_DEG || absPitch > TILT_KILL_DEG){
        if(tiltStartMs == 0) tiltStartMs = millis();
        if(millis() - tiltStartMs >= TILT_KILL_HOLD_MS){
          emergencyStop();
          tiltStartMs = 0;
        }
      } else {
        tiltStartMs = 0;
      }
    } else {
      tiltStartMs = 0;
    }
  }

  updateAltitudeFusion();
  updatePositionFromFlow();
  flowCalTick();

  // KO: Hover 긽깭癒몄떊 tick (A 諛⑹떇)
  // EN: Hover state machine tick (A method)
  hover_update();

  // KO: Hover 以鍮 썑 삤넗뒥 (ALT PD + YAW P)
  // EN: Auto tune after hover ready (ALT PD + YAW P)
  tune_update();
  g_ledAutoTune = tune_isActive();

  // ==========================================================================
  // KO: 뒪떛 엯젰 泥섎━ (以묐났 젣嫄)
  // EN: Stick input handling (duplicate removed)
  // ==========================================================================
  if ((currentMode == MODE_HOVERING || currentMode == MODE_TAKEOFF) && !failsafe && !tune_isActive() && !flipActive) {
    // KO: 1) 뜲뱶議 꽕젙 (以묒븰 127 湲곗 짹10 臾댁떆)
    // EN: 1) Deadzone (ignore 짹10 around 127)
    int rawRoll = receiverData.roll;
    int rawPitch = receiverData.pitch;
    int rawThr = receiverData.throttle;
    int rawYaw = receiverData.yaw;

    float vX_cmd = 0.0f; // KO: 醫뚯슦 씠룞 냽룄 / EN: lateral velocity cmd
    float vY_cmd = 0.0f; // KO: 쟾썑 씠룞 냽룄 / EN: forward velocity cmd

    // KO: Roll (醫뚯슦)
    // EN: Roll (lateral)
    if (abs(rawRoll - 127) > 10) {
      float input = (float)(rawRoll - 127) / 127.0f;
      vX_cmd = input * moveSpeedSteps[speedLevel]; 
    }

    // KO: Pitch (쟾썑)
    // EN: Pitch (forward/back)
    if (abs(rawPitch - 127) > 10) {
      float input = (float)(rawPitch - 127) / 127.0f;
      vY_cmd = input * moveSpeedSteps[speedLevel];
    }

    // KO: Yaw (쉶쟾)
    // EN: Yaw (rotation)
    if (abs(rawYaw - 127) > 10) {
      float input = (float)(rawYaw - 127) / 127.0f;
      targetYaw += input * yawSpeedSteps[speedLevel] * g_loopDt;
      // KO: 媛곷룄 옒븨
      // EN: Wrap angle
      while(targetYaw > 180.0f) targetYaw -= 360.0f;
      while(targetYaw < -180.0f) targetYaw += 360.0f;
    }

    // KO: Throttle (怨좊룄 誘몄꽭 議곗젙)
    // EN: Throttle (altitude trim)
    if (abs(rawThr - 127) > 20) {
      float input = (float)(rawThr - 127) / 127.0f;
      targetAltitude += input * climbSpeedSteps[speedLevel] * g_loopDt;
      // KO: 怨좊룄 븞쟾 젣븳 (0.1m ~ 2.5m)
      // EN: Altitude limits (0.1m ~ 2.5m)
      if (targetAltitude < 0.1f) targetAltitude = 0.1f;
      if (targetAltitude > 2.5f) targetAltitude = 2.5f;
    }

    // KO: 2) 醫뚰몴怨 蹂솚 (뱶濡 뿤뵫 湲곗)
    // EN: 2) Frame transform (drone heading)
    if (vX_cmd != 0.0f || vY_cmd != 0.0f) {
      float rad = currentYaw * DEG_TO_RAD;
      float c = cosf(rad);
      float s = sinf(rad);

      float dX_world = (vY_cmd * c - vX_cmd * s) * g_loopDt; 
      float dY_world = (vY_cmd * s + vX_cmd * c) * g_loopDt;

      // KO: 븞쟾 젣븳
      // EN: Safety clamp
      const float MAX_TARGET_STEP = 0.12f * g_loopDt;
      dX_world = clampf(dX_world, -MAX_TARGET_STEP, MAX_TARGET_STEP);
      dY_world = clampf(dY_world, -MAX_TARGET_STEP, MAX_TARGET_STEP);

      targetPosX += dX_world;
      targetPosY += dY_world;
    }
  }
  // ============================================================


  // KO: LED 뾽뜲씠듃
  // EN: LED update
  ledSet(ok_imu, ok_baro, ok_tof, ok_flow && flowUserEnabled);
  // KO: 理쒖쥌 LED tick (긽깭 뾽뜲씠듃 썑)
  // EN: Final LED tick (after state updates)
  ledTick();

}

static inline void updateFlight(){
  if(currentMode == MODE_READY || currentMode == MODE_EMERGENCY){
    if(currentMode == MODE_EMERGENCY){
      if(turtleUpdate()) return;
    }
    if(currentMode == MODE_READY && motorArmedIdle){
      uint32_t now = millis();
      int pwm = ARM_IDLE_PWM_LOW;
      if(armPulsing){
        uint32_t dt = now - armPulseStartMs;
        if(dt < ARM_PULSE_MS){
          pwm = ARM_IDLE_PWM_LOW;
        } else if(dt < (ARM_PULSE_MS * 2)){
          pwm = ARM_IDLE_PWM_HIGH;
        } else if(dt < (ARM_PULSE_MS * 3)){
          pwm = ARM_IDLE_PWM_LOW;
        } else {
          armPulsing = false;
          pwm = ARM_IDLE_PWM_LOW;
        }
      }
      motorsIdle(pwm);
      return;
    }
    motorsOff();
    return;
  }


  if(flipUpdate()) return;
  // KO: TAKEOFF -> HOVERING 쟾솚 hover header媛 븞젙 떆 닔뻾
  // EN: TAKEOFF -> HOVERING handled by hover header when stable
  // KO: LANDING
  // EN: LANDING
  if(currentMode == MODE_LANDING){
    targetAltitude -= LAND_DESCEND_MPS * g_loopDt;
    targetAltitude = clampf(targetAltitude, 0.0f, 3.0f);
    if(currentAltitude < READY_ALT_M){
      currentMode = MODE_READY;
      motorsOff();
      return;
    }
  }

  // KO: Position P
  // EN: Position P
  float ex = (targetPosX - currentPosX);
  float ey = (targetPosY - currentPosY);
  float r_cmd = ex * POS_KP;
  float p_cmd = ey * POS_KP;

  // KO: Yaw P
  // EN: Yaw P
  float eyaw = (targetYaw - currentYaw);
  while(eyaw > 180.0f) eyaw -= 360.0f;
  while(eyaw < -180.0f) eyaw += 360.0f;
  float y_cmd = eyaw * yawKp;

  // KO: Altitude PD
  // EN: Altitude PD
  float zErr = (targetAltitude - currentAltitude);
  float zVel = (alt_m_filt - alt_m_prev) / g_loopDt;
  float zCmd = (zErr * altKp) - (zVel * altKd);

  // KO: Hover 븰뒿씠 hoverThrottle쓣 옄룞 蹂댁젙
  // EN: Hover learning auto-adjusts hoverThrottle
  hover_onZcmd(zCmd);

  int thr = hoverThrottle + (int)(zCmd * 100.0f);
  thr = constrain(thr, 0, 255);

  motorControl(thr, r_cmd, p_cmd, y_cmd);
}

// ============================================================================
// KO: 珥덇린솕
// EN: init
// ============================================================================
static inline void initAF1000X(){
  // init log suppressed (banner+POST only)

  ledsInit();
  motorsInit();
  userLedInit();
  userServoInit();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  radioInit();

  prefs.begin("af1000x", false);
  flowK = prefs.getFloat("flowK", flowK);
  // flowK load log suppressed (banner+POST only)
  tune_loadFromNVS();
  tune_saveFilterParamsIfMissing();

  // KO: POST + 罹섎━釉뚮젅씠뀡
  // EN: POST + calibration
  postSensors();
  g_serialMuteAfterPost = true;
  if(flightLock){
    LOG_PRINTLN("AF1000X: POST FAIL (IMU/BARO) - HARD LOCK");
    motorsOff();
    while(true){
      ledPostStatusDelay(ok_imu, ok_baro, ok_tof, ok_flow, 3000);
    }
  }
  if(!flightLock){
    if(!calibrateSensors()){
      flightLock = true;
      ledFailPattern();
    }
  }

  
  // ============================================================================
  // KO: LED 遺똿 떆뒪 珥덇린솕 (쟾泥 젏硫 삉뒗 inverted chase)
  // EN: LED boot sequence init (blink-all or inverted chase)
  // ============================================================================
  g_ledBound = false;
  g_ledHeadless = false;
  g_ledFlipReady = false;
  g_ledLowBattery = false;

  bool invertedAtBoot = false;
  if(ok_imu && IMU != nullptr){
    inv_imu_sensor_data_t d{};
    if(IMU->getDataFromRegisters(d) == 0){
      float az = (float)d.accel_data[2];
      invertedAtBoot = (az < 0.0f);
    }
  }
  ledBootBegin(invertedAtBoot);

binding_begin();
  fhssInit();

  hover_begin(); // KO: hoverPWM 濡쒕뱶 / EN: load hoverPWM

  currentMode = MODE_READY;
  LOG_PRINTF("AF1000X: READY (lock=%d)\n", (int)flightLock);
  LOG_PRINTLN("쐟 Fixed: Duplicate stick input removed");
  LOG_PRINTLN("쐟 Added: Battery protection");
}

#endif // KO: AF1000X_H / EN: AF1000X_H
