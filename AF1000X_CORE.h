#ifndef AF1000X_H
#define AF1000X_H


#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <RF24.h>
#include <Preferences.h>
#include <VL53L1X.h>
#include "ICM45686.h"

// PATCH: actual loop dt (seconds), updated in main loop
extern float g_loopDt;


// Hover 학습 헤더(너가 만든 파일)
#include "AF1000X_Hover.h"
#include "AF1000X_BINDING.h"

/* ============================================================================
 * AF1000X Core (ESP32-S3) - FIXED VERSION
 * - Sensor POST + LED
 * - Altitude fusion (ToF + SPL06)
 * - Optical flow integration (PMW3901) + yaw rotation
 * - Flight lock if sensors fail
 * - Hover PWM auto-learning (via AF1000X_Hover.h)
 * - {0xE8, 0xE8, 0xF0, 0xF0, 0xE1} 페어링 주소
 * 
 * FIXES:
 * - 중복된 스틱 입력 처리 코드 제거 (1027-1106 라인 문제 해결)
 * - 배터리 저전압 보호 추가
 * - 안전성 개선
 * ========================================================================== */

// ----------------------- Mode constants (u8) -----------------------
static constexpr uint8_t MODE_READY     = 0;
static constexpr uint8_t MODE_TAKEOFF   = 1;
static constexpr uint8_t MODE_HOVERING  = 2;
static constexpr uint8_t MODE_LANDING   = 3;
static constexpr uint8_t MODE_EMERGENCY = 4;

// ----------------------- RF structs ------------------------
struct Signal {
  uint8_t throttle, roll, pitch, yaw, aux1, aux2;
  uint8_t speed; // 1~3
};

struct Telemetry {
  float vbat, alt, posX, posY;
  int rssi;
};

// ============================================================================
// Pins (필요하면 너 PCB에 맞춰 수정)
// ============================================================================
static const int PIN_RF_CE   = 2;
static const int PIN_RF_CSN  = 5;
static const uint8_t RF_ADDR[5] = {0xB1,0xB2,0xB3,0xB4,0x01};

// Motors (너가 확정한 값)
static const int PIN_M1 = 10;
static const int PIN_M2 = 11;
static const int PIN_M3 = 12;
static const int PIN_M4 = 13;

// I2C
static const int PIN_I2C_SDA = 3;
static const int PIN_I2C_SCL = 4;

// SPI
static const int PIN_SPI_SCK  = 33;
static const int PIN_SPI_MISO = 34;
static const int PIN_SPI_MOSI = 14;

// ToF
static const int PIN_VL53_XSHUT = 35;

// Optical Flow
static const int PIN_FLOW_CS     = 38;
static const int PIN_FLOW_MOTION = 37;

// Battery ADC
static const int PIN_BAT_ADC = 1;

// LEDs (센서 상태)
static const int PIN_LED_IMU  = 6;
static const int PIN_LED_BARO = 7;
static const int PIN_LED_TOF  = 8;
static const int PIN_LED_FLOW = 9;

// ============================================================================
// Tuning constants
// ============================================================================
static const float DEFAULT_LOOP_DT = 0.02f;       // 50Hz nominal (PATCH)
static const uint32_t FAILSAFE_MS = 300;

// ✅ 배터리 보호 추가 (1S LiPo - 720 모터용)
static const float BATTERY_MIN_VOLTAGE = 3.3f;   // 1S LiPo 최저 전압
static const float BATTERY_WARNING_VOLTAGE = 3.5f; // 경고 전압

static const float ALT_SWITCH_M = 1.0f;
static const float TOF_EMA_A  = 0.25f;
static const float BARO_EMA_A = 0.05f;
static const float ALT_EMA_A  = 0.15f;

static const float TOF_JUMP_REJECT_M = 0.25f;

static const uint8_t FLOW_SQUAL_MIN = 20;
static const float   FLOW_MAX_STEP_M = 0.08f;

static const float POS_KP = 0.6f;
static const float YAW_KP = 1.5f;
static const float ALT_KP = 0.18f;
static const float ALT_KD = 0.06f;

static const float TAKEOFF_TARGET_M = 1.0f;
static const float LAND_DESCEND_MPS = 0.25f;
static const float READY_ALT_M      = 0.08f;

// PWM
static const uint32_t PWM_FREQ = 24000;
static const uint8_t  PWM_RES  = 8;

// ============================================================================
// Globals (Hover/EasyCommander에서 참조 가능한 이름들)
// ============================================================================
#ifdef AF1000X_IMPLEMENTATION

// RF
RF24 radio(PIN_RF_CE, PIN_RF_CSN);
Preferences prefs;
Signal receiverData{};
Telemetry telemetryData{};
uint32_t lastRxMs = 0;
bool failsafe = false;

float currentRoll  = 0.0f;
float currentPitch = 0.0f;


// mode (Hover 헤더가 uint8_t extern 기대)
volatile uint8_t currentMode = MODE_READY;

// targets / states (meters)
float targetAltitude = TAKEOFF_TARGET_M;
float currentAltitude = 0.0f;

float targetPosX = 0.0f, targetPosY = 0.0f;
float currentPosX = 0.0f, currentPosY = 0.0f;

float targetYaw = 0.0f;   // deg
float currentYaw = 0.0f;  // deg

int speedLevel = 1;
float moveSpeedSteps[3]  = {0.30f, 0.60f, 1.00f};
float climbSpeedSteps[3] = {0.20f, 0.40f, 0.70f};
float yawSpeedSteps[3]   = {30.0f, 60.0f, 120.0f};

float trimRoll = 0.0f, trimPitch = 0.0f;

// 고도 제어 기반 PWM (Hover 학습이 이 값을 자동으로 찾음)
int hoverThrottle = 120;  // 720 모터 기본값

// ✅ 배터리 상태 변수 추가 (1S LiPo)
float batteryVoltage = 4.2f;  // 1S 완충 전압
float batteryVoltageFilt = 4.2f; // PATCH: filtered (EMA)
bool batteryLowWarning = false;
bool batteryCritical = false;

// 센서 OK/락
bool ok_imu=false, ok_baro=false, ok_tof=false, ok_flow=false;
bool flightLock = false;
bool calibrated = false;

// altitude filters
bool tofFiltInit=false;
float tof_m_filt=0.0f;
float tof_m_last_raw=0.0f;

bool baroInit=false;
float baro_alt_m_filt=0.0f;

bool altInit=false;
float alt_m_filt=0.0f;
float alt_m_prev=0.0f;

// flowK (m/count/m)
float flowK = 0.0022f;

// ToF
VL53L1X tof;

// IMU (ICM45686)
static ICM456xx IMU0(Wire, 0);
static ICM456xx IMU1(Wire, 1);
static ICM456xx* IMU = nullptr;
float gyroZ_bias = 0.0f;
float yaw_internal = 0.0f;
uint32_t lastYawUs = 0;

// motors ledc channels
int ch1=-1,ch2=-1,ch3=-1,ch4=-1;

// Baro baseline
float basePressurePa = 101325.0f;

// Flow cal state
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

extern float batteryVoltage;
extern bool batteryLowWarning, batteryCritical;

extern bool ok_imu, ok_baro, ok_tof, ok_flow;
extern bool flightLock;
extern bool calibrated;

extern bool tofFiltInit;
extern float tof_m_filt, tof_m_last_raw;
extern bool baroInit;
extern float baro_alt_m_filt;
extern bool altInit;
extern float alt_m_filt, alt_m_prev;

extern float flowK;
extern VL53L1X tof;

extern float gyroZ_bias, yaw_internal;
extern uint32_t lastYawUs;

extern int ch1,ch2,ch3,ch4;

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
// Helpers
// ============================================================================
static inline float clampf(float v,float lo,float hi){ return (v<lo)?lo:((v>hi)?hi:v); }
static inline void ledsInit(){
  pinMode(PIN_LED_IMU, OUTPUT);
  pinMode(PIN_LED_BARO, OUTPUT);
  pinMode(PIN_LED_TOF, OUTPUT);
  pinMode(PIN_LED_FLOW, OUTPUT);
  digitalWrite(PIN_LED_IMU, LOW);
  digitalWrite(PIN_LED_BARO, LOW);
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

// ============================================================================
// LED State Machine (Integrated) - Boot/Binding + LowBatt + GyroReset + Headless
// Requirements implemented:
// 1) Power on: all LEDs blink (1s on / 1s off)
// 2) If inverted at boot: LED1->2->3->4 chase
// 3) When pairing/binding completes: all LEDs ON
// 4) After 1s: return to existing LED sequence (sensor LEDs etc.)
// Extra (from earlier):
// - Low battery: all LEDs 1s on / 1s off (highest priority AFTER boot/bind)
// - Gyro init: all LEDs 1s on / 0.5s off x3
// - Headless: LED3/LED4 2s on / 1s off
// ============================================================================

static bool g_ledInvertedAtBoot = false;

static bool g_ledLowBattery = false;
static bool g_ledHeadless   = false;
static bool g_ledBound      = false;

// Gyro reset animation (non-blocking)
static bool     s_ledGyroAnimActive = false;
static uint8_t  s_ledGyroAnimCount  = 0;   // number of ON phases completed
static bool     s_ledGyroAnimOn     = false;
static uint32_t s_ledGyroNextMs     = 0;

static inline void ledStartGyroResetAnim(){
  s_ledGyroAnimActive = true;
  s_ledGyroAnimCount  = 0;
  s_ledGyroAnimOn     = true;
  s_ledGyroNextMs     = millis();
}

// Boot/binding sequence
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

// Call once during init (after IMU check so inverted flag is valid)
static inline void ledBootBegin(bool invertedAtBoot){
  g_ledInvertedAtBoot = invertedAtBoot;
  s_ledBootStartMs = millis();
  s_ledPrevBound = g_ledBound;
  s_ledBootState = invertedAtBoot ? LED_BOOT_CHASE_INVERTED : LED_BOOT_BLINK_ALL;
}

// Internal: boot/bind updater (returns true if it handled LEDs this tick)
static inline bool ledBootTick(){
  uint32_t now = millis();

  // Detect rising edge: not bound -> bound
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

  // inverted chase: 200ms step
  if (s_ledBootState == LED_BOOT_CHASE_INVERTED) {
    uint8_t step = (uint8_t)((now / 200) % 4);
    _ledOneHot(step);
    return true;
  }
  return false;
}

// Main LED update: call every loop
static inline void ledTick(){
  uint32_t now = millis();

  // 0) Boot/bind sequence has top priority until done
  if (ledBootTick()) return;

  // A) Low battery (highest after boot)
  if (g_ledLowBattery) {
    bool on = _blink(now, 1000, 1000); // 1s on / 1s off
    ledAll(on);
    return;
  }

  // B) Gyro reset animation
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

  // C) Headless: LED3/LED4 2s on / 1s off
  if (g_ledHeadless) {
    bool on34 = (now % 3000) < 2000;
    digitalWrite(PIN_LED_IMU,  LOW);
    digitalWrite(PIN_LED_BARO, LOW);
    digitalWrite(PIN_LED_TOF,  on34 ? HIGH : LOW);
    digitalWrite(PIN_LED_FLOW, on34 ? HIGH : LOW);
    return;
  }

  // D) Normal: do not override (existing sensor LED logic remains)
}

static inline float readBatteryVoltage(){
  int raw = analogRead(PIN_BAT_ADC);
  float vadc = (raw / 4095.0f) * 3.3f;
  // 1S 배터리: 분압비 없이 직접 연결 또는 낮은 분압비
  // PCB 회로에 맞춰 조정 필요 (현재는 직접 연결로 가정)
  return vadc * 1.27f; // 4.2V 측정을 위한 보정 계수 (PCB에 맞춰 조정)
}

// ✅ 배터리 체크 함수 추가
static inline void checkBattery(){
  batteryVoltage = readBatteryVoltage();

  // PATCH: EMA low-pass to prevent jitter
  const float alpha = 0.20f; // 0..1 (higher=faster)
  batteryVoltageFilt = batteryVoltageFilt + alpha * (batteryVoltage - batteryVoltageFilt);

  if(batteryVoltageFilt < BATTERY_MIN_VOLTAGE){
    if(!batteryCritical){
      batteryCritical = true;
      Serial.printf("⚠️ CRITICAL: Battery %.2fV - Emergency landing!\n", batteryVoltageFilt);
    }
    // 비행 중이면 강제 착륙
    if(currentMode == MODE_HOVERING || currentMode == MODE_TAKEOFF){
      currentMode = MODE_LANDING;
    }
  } else if(batteryVoltageFilt < BATTERY_WARNING_VOLTAGE){
    if(!batteryLowWarning){
      batteryLowWarning = true;
      Serial.printf("⚠️ WARNING: Battery low %.2fV\n", batteryVoltageFilt);
    }
  } else {
    batteryLowWarning = false;
    batteryCritical = false;
  }
}



// --------------------------------------------------------------------------
// Binding LED hooks (used by AF1000X_BINDING.h)
// - We keep these as thin wrappers so the binding logic doesn't need changes.
// - Actual patterns are controlled by ledTick() + ledBootBegin() + g_ledBound flag.
// --------------------------------------------------------------------------
inline void binding_ledAll(bool on) { ledAll(on); }

// Called repeatedly while waiting for pairing/binding.
// We let ledTick() drive the required boot/inverted patterns.
inline void binding_ledPairingTick() {
  ledTick();
}

// Called once when binding completes.
// We do NOT use delay() here; the "all LEDs ON for 1s" is handled in ledBootTick()
// when g_ledBound rises false->true.
inline void binding_ledBoundOnce() {
  ledAll(true);
}

// Called on binding error
inline void binding_ledErrorOnce() {
  // quick error flash (non-blocking would be nicer, but keep it short)
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
static inline void motorControl(int thr, float r, float p, float ycmd){
  if(currentMode==MODE_READY || currentMode==MODE_EMERGENCY){ motorsOff(); return; }

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
// RF
// ============================================================================
static inline void radioInit(){
  radio.begin();
  radio.setDataRate(RF24_250KBPS);
  radio.setAutoAck(true);
  radio.enableAckPayload();
  radio.openReadingPipe(1, RF_ADDR);
  radio.startListening();
  lastRxMs = millis();
}
static inline void updateRadio(){
  if(radio.available()){
    radio.read(&receiverData, sizeof(Signal));
    lastRxMs = millis();
    failsafe = false;

    if(receiverData.speed >= 1 && receiverData.speed <= 3){
      speedLevel = receiverData.speed - 1;
    }

    telemetryData.vbat = batteryVoltage;
    telemetryData.alt  = currentAltitude;
    telemetryData.posX = currentPosX;
    telemetryData.posY = currentPosY;
    telemetryData.rssi = 1;

    radio.writeAckPayload(1, &telemetryData, sizeof(Telemetry));
  }

  if(millis() - lastRxMs > FAILSAFE_MS){
  failsafe = true;
  if(currentMode != MODE_READY && currentMode != MODE_EMERGENCY){

    // ✅ 추가: 착륙 시 목표를 현재로 리셋 (옆으로 도망 방지)
    targetPosX = currentPosX;
    targetPosY = currentPosY;
    targetYaw  = currentYaw;

    currentMode = MODE_LANDING;
   }
  }
}

// ============================================================================
// IMU (ICM45686) yaw integration
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

  gyroZ_bias = 0.0f;
  yaw_internal = 0.0f;
  lastYawUs = micros();
  return true;
}
static inline bool imu_calibrate_gyroZ(uint16_t samples=800, uint16_t delay_ms=2){
  if(!ok_imu || IMU == nullptr) return false;

  float sum=0.0f; uint16_t got=0;
  inv_imu_sensor_data_t d{};
  for(uint16_t i=0;i<samples;i++){
    if(IMU->getDataFromRegisters(d) == 0){
      sum += (float)d.gyro_data[2];
      got++;
    }
    delay(delay_ms);
  }
  if(got < (uint16_t)(samples*0.6f)) return false;

  gyroZ_bias = sum / (float)got;
  yaw_internal = 0.0f;
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

  if (fabsf(az) < 0.2f) return;

  // 단위/스케일 정확하지 않아도 "뒤집힘 감지"엔 충분
  currentRoll  = atan2f(ay, az) * 57.2957795f;
  currentPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2957795f;

  uint32_t now = micros();
  float dt = (now - lastYawUs) * 1e-6f;
  lastYawUs = now;
  dt = safeDt(dt);
  if(dt <= 0.0f) return;

  float gz = (float)d.gyro_data[2] - gyroZ_bias;
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
// SPL06 minimal driver (I2C) -> pressure -> altitude
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
// ToF (VL53L1X)
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
// PMW3901 minimal (SPI) optical flow
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
// Altitude fusion (ToF + Baro)
// ============================================================================
static inline void updateAltitudeFusion(){
  float tofNew;
  if(ok_tof && tof_read_m(tofNew)){
    if(!tofFiltInit){
      tof_m_filt = tofNew;
      tof_m_last_raw = tofNew;
      tofFiltInit = true;
    } else {
      if(fabsf(tofNew - tof_m_last_raw) > TOF_JUMP_REJECT_M){
        // reject
      } else {
        tof_m_filt += TOF_EMA_A * (tofNew - tof_m_filt);
        tof_m_last_raw = tofNew;
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
      baro_alt_m_filt += BARO_EMA_A * (h_m - baro_alt_m_filt);
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
    alt_m_filt += ALT_EMA_A * (fused - alt_m_filt);
  }

  currentAltitude = alt_m_filt;
}

// ============================================================================
// Position from Flow + Yaw rotation
// ============================================================================
static inline void updatePositionFromFlow(){
  if(!ok_flow) return;

  PMWData f = pmw_burst();
  if(!f.valid || f.squal < FLOW_SQUAL_MIN) return;

  // Flow calibration
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
// Flow auto calibration
// ============================================================================
static inline void startFlowAutoCal(float dist_m, float hoverAlt_m, float pwr){
  if(currentMode != MODE_READY){
    Serial.println("FLOW CAL: must be in READY mode.");
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

  Serial.printf("FLOW CAL START: dist=%.2fm hover=%.2fm pwr=%.0f%%\n", dist_m, hoverAlt_m, pwr);
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
          Serial.println("FLOW CAL: hovering stable -> forward");
        }
      }
      if((millis() - flowCal_t0) > 15000){
        flowCal = FC_ABORT;
        Serial.println("FLOW CAL: ABORT (timeout hover)");
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
          Serial.println("FLOW CAL: ABORT (not enough samples)");
          break;
        }
        float avgH = flowCal_height_acc / (float)flowCal_height_n;
        float counts = (float)abs(flowCal_sum_dx);
        if(avgH < 0.10f || counts < 5.0f){
          flowCal = FC_ABORT;
          Serial.println("FLOW CAL: ABORT (bad data)");
          break;
        }

        flowCal_suggestK = flowCal_dist_m / (avgH * counts);
        flowK = flowCal_suggestK;
        prefs.putFloat("flowK", flowK);

        flowCal = FC_DONE;
        Serial.println("FLOW CAL: DONE");
      }
    } break;

    default: break;
  }

  if(flowCal == FC_DONE && !flowCal_reported){
    flowCal_reported = true;
    float avgH = (flowCal_height_n ? flowCal_height_acc/(float)flowCal_height_n : 0.0f);
    Serial.printf("FLOW CAL RESULT: avgH=%.2fm dxCounts=%ld flowK=%.6f (saved)\n",
                  avgH, (long)flowCal_sum_dx, flowK);
    Serial.println("If measured actual distance, send: D60.0  (cm)");
  }
}

// ============================================================================
// Serial commands (테스트/캘리브레이션/호버학습)
// ============================================================================
static inline void serialCommands(){
  // PATCH: Non-blocking line reader (prevents control-loop stalls)
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
      if(s == "Y0"){ yawReset(); Serial.println("Yaw reset."); continue; }

      if(s == "T"){ // takeoff (hover learn)
        if(currentMode == MODE_READY) {
          hover_startLearn();
          Serial.println("Takeoff: hover learn started.");
        }
        continue;
      }
      if(s == "L"){
        if(currentMode != MODE_READY) currentMode = MODE_LANDING;
        Serial.println("Landing.");
        continue;
      }
      if(s == "K"){
        currentMode = MODE_EMERGENCY;
        motorsOff();
        Serial.println("KILL MOTORS!");
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
            Serial.printf("FLOW CAL REFINE: actual=%.1fcm => flowK=%.6f (saved)\n", actual_cm, flowK);
          }
        }
        continue;
      }

      // ✅ 배터리 체크 명령 (raw + filtered)
      if(s == "B"){
        Serial.printf("Battery: %.2fV (filt %.2fV)\n", batteryVoltage, batteryVoltageFilt);
        continue;
      }

      // Unknown command -> ignore
      continue;
    }

    // Accumulate characters
    if(cmdLen < (sizeof(cmdBuf)-1)){
      cmdBuf[cmdLen++] = c;
    } else {
      // overflow -> reset buffer
      cmdLen = 0;
    }
  }
}


// ============================================================================
// POST + Calibration
// ============================================================================
static inline bool postSensors(){
  ok_imu  = imu_init_auto();
  ok_baro = spl_init();
  ok_tof  = tof_init();
  ok_flow = pmw_init();

  ledSet(ok_imu, ok_baro, ok_tof, ok_flow);

  // 필수: Baro + ToF + Flow (IMU는 실패해도 비행은 가능하게 설계했지만 권장)
  flightLock = !(ok_baro && ok_tof && ok_flow);

  if(flightLock){
    Serial.printf("POST FAIL -> FLIGHT LOCK (IMU=%d BARO=%d TOF=%d FLOW=%d)\n",
                  ok_imu, ok_baro, ok_tof, ok_flow);
    ledFailPattern();
    return false;
  }
  Serial.printf("POST OK (IMU=%d BARO=%d TOF=%d FLOW=%d)\n",
                ok_imu, ok_baro, ok_tof, ok_flow);
  return true;
}

static inline bool calibrateSensors(){
  // Baro baseline
  float sumP=0; int got=0;
  for(int i=0;i<200;i++){
    float p;
    if(spl_readPressurePa(p)){ sumP += p; got++; }
    delay(10);
  }
  if(got < 120) return false;
  basePressurePa = sumP / got;

  // reset filters
  altInit=false;
  baroInit=false;
  tofFiltInit=false;

  // IMU gyro bias (optional)
  if(ok_imu){
    if(!imu_calibrate_gyroZ()){
      ok_imu = false;
    }
  }
  yawReset();

  calibrated = true;
  return true;
}

// ============================================================================
// Public APIs
// ============================================================================
static inline void calibrate(){
  if(currentMode == MODE_READY){
    Serial.println("AF: calibrate (ground)");
    if(!calibrateSensors()){
      Serial.println("AF: calibrate FAIL -> lock");
      flightLock = true;
      ledFailPattern();
    } else {
      Serial.println("AF: calibrate OK");
    }
  }
}

static inline void autoTakeoff(){
  if(flightLock){
    Serial.println("AF: FLIGHT LOCK - takeoff blocked");
    currentMode = MODE_READY;
    motorsOff();
    return;
  }
  if(currentMode == MODE_READY){
    hover_startLearn(); // ✅ A 방식 시작
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
}

// ============================================================================
// updateSystem / updateFlight
// ============================================================================
static inline void updateSystem(){
  // Always update binding state first
  binding_update();
  g_ledBound = binding_isBound();

  // Battery check (10Hz) even before binding completes
  static uint32_t lastBatMs = 0;
  uint32_t nowBatMs = millis();
  if(nowBatMs - lastBatMs >= 100){
    lastBatMs = nowBatMs;
    checkBattery();
  }
  g_ledLowBattery = (batteryLowWarning || batteryCritical);

  // Headless flag (updated after we have fresh RX data)
  g_ledHeadless = false;

  // Drive LEDs during boot/binding too
  ledTick();

  if (!g_ledBound) {
    // Not bound yet -> no control / no flight
    currentMode = MODE_READY;
    motorsOff();
    return;
  }
updateRadio();
  // Headless from TX (aux2)
  g_ledHeadless = (receiverData.aux2 != 0);

  serialCommands();  

  if(ok_imu) imu_updateYaw();
  else currentYaw = 0.0f;

  updateAltitudeFusion();
  updatePositionFromFlow();
  flowCalTick();

  // ✅ Hover 상태머신 tick (A 방식)
  hover_update();

  // ============================================================
  // ✅ [FIXED] 조종기 스틱 입력 처리 (중복 제거)
  // ============================================================
  if (currentMode == MODE_HOVERING && !failsafe) {
    // 1. 데드존 설정 (중앙 127 기준 ±10 정도는 무시)
    int rawRoll = receiverData.roll;
    int rawPitch = receiverData.pitch;
    int rawThr = receiverData.throttle;
    int rawYaw = receiverData.yaw;

    float vX_cmd = 0.0f; // 좌우 이동 속도 명령
    float vY_cmd = 0.0f; // 전후 이동 속도 명령

    // Roll (좌우)
    if (abs(rawRoll - 127) > 10) {
      float input = (float)(rawRoll - 127) / 127.0f;
      vX_cmd = input * moveSpeedSteps[speedLevel]; 
    }

    // Pitch (전후)
    if (abs(rawPitch - 127) > 10) {
      float input = (float)(rawPitch - 127) / 127.0f;
      vY_cmd = input * moveSpeedSteps[speedLevel];
    }

    // Yaw (회전)
    if (abs(rawYaw - 127) > 10) {
      float input = (float)(rawYaw - 127) / 127.0f;
      targetYaw += input * yawSpeedSteps[speedLevel] * g_loopDt;
      // Wrap angle
      while(targetYaw > 180.0f) targetYaw -= 360.0f;
      while(targetYaw < -180.0f) targetYaw += 360.0f;
    }

    // Throttle (고도 미세 조정)
    if (abs(rawThr - 127) > 20) {
      float input = (float)(rawThr - 127) / 127.0f;
      targetAltitude += input * climbSpeedSteps[speedLevel] * g_loopDt;
      // 고도 안전 제한 (0.1m ~ 2.5m)
      if (targetAltitude < 0.1f) targetAltitude = 0.1f;
      if (targetAltitude > 2.5f) targetAltitude = 2.5f;
    }

    // 2. 좌표계 변환 (드론 헤딩 기준)
    if (vX_cmd != 0.0f || vY_cmd != 0.0f) {
      float rad = currentYaw * DEG_TO_RAD;
      float c = cosf(rad);
      float s = sinf(rad);

      float dX_world = (vY_cmd * c - vX_cmd * s) * g_loopDt; 
      float dY_world = (vY_cmd * s + vX_cmd * c) * g_loopDt;

      // 안전 제한
      const float MAX_TARGET_STEP = 0.08f * g_loopDt;
      dX_world = clampf(dX_world, -MAX_TARGET_STEP, MAX_TARGET_STEP);
      dY_world = clampf(dY_world, -MAX_TARGET_STEP, MAX_TARGET_STEP);

      targetPosX += dX_world;
      targetPosY += dY_world;
    }
  }
  // ============================================================


  // LED update
  ledSet(ok_imu, ok_baro, ok_tof, ok_flow);
  // Final LED tick (after state updates)
  ledTick();

}

static inline void updateFlight(){
  if(currentMode == MODE_READY || currentMode == MODE_EMERGENCY){
    motorsOff();
    return;
  }

  // TAKEOFF -> HOVERING은 hover header가 안정되면 currentMode=HOVERING으로 바꿈
  // LANDING
  if(currentMode == MODE_LANDING){
    targetAltitude -= LAND_DESCEND_MPS * g_loopDt;
    targetAltitude = clampf(targetAltitude, 0.0f, 3.0f);
    if(currentAltitude < READY_ALT_M){
      currentMode = MODE_READY;
      motorsOff();
      return;
    }
  }

  // Position P
  float ex = (targetPosX - currentPosX);
  float ey = (targetPosY - currentPosY);
  float r_cmd = ex * POS_KP;
  float p_cmd = ey * POS_KP;

  // Yaw P
  float eyaw = (targetYaw - currentYaw);
  while(eyaw > 180.0f) eyaw -= 360.0f;
  while(eyaw < -180.0f) eyaw += 360.0f;
  float y_cmd = eyaw * YAW_KP;

  // Altitude PD
  float zErr = (targetAltitude - currentAltitude);
  float zVel = (alt_m_filt - alt_m_prev) / g_loopDt;
  float zCmd = (zErr * ALT_KP) - (zVel * ALT_KD);

  // ✅ Hover 학습이 hoverThrottle을 자동으로 맞추도록 피드백
  hover_onZcmd(zCmd);

  int thr = hoverThrottle + (int)(zCmd * 100.0f);
  thr = constrain(thr, 0, 255);

  motorControl(thr, r_cmd, p_cmd, y_cmd);
}

// ============================================================================
// init
// ============================================================================
static inline void initAF1000X(){
  Serial.println("AF1000X: init (FIXED VERSION)");

  ledsInit();
  motorsInit();

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  radioInit();

  prefs.begin("af1000x", false);
  flowK = prefs.getFloat("flowK", flowK);
  Serial.printf("flowK=%.6f (loaded)\n", flowK);

  // POST + calibration
  postSensors();
  if(!flightLock){
    if(!calibrateSensors()){
      flightLock = true;
      ledFailPattern();
    }
  }

  
  // ------------------------------------------------------------
  // LED boot sequence init (blink-all OR inverted chase)
  // ------------------------------------------------------------
  g_ledBound = false;
  g_ledHeadless = false;
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

  hover_begin(); // hoverPWM 로드(hoverThrottle에 적용)

  currentMode = MODE_READY;
  Serial.printf("AF1000X: READY (lock=%d)\n", (int)flightLock);
  Serial.println("✅ Fixed: Duplicate stick input removed");
  Serial.println("✅ Added: Battery protection");
}

#endif // AF1000X_H
