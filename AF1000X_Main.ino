#define AF1000X_IMPLEMENTATION
#include "AF1000X_CORE.h"
#include "AF1000X_EasyCommander.h"
float g_loopDt = 0.02f;  // global loop dt (seconds)

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // keep any legacy Serial reads snappy
  delay(300);

  initAF1000X();

  // 자동 보정(지상)
  calibrate();

  Serial.println("================================");
  Serial.println("AF1000X FIXED VERSION - Ready");
  Serial.println("================================");
  Serial.println("Serial commands:");
  Serial.println("  T  -> auto takeoff (hover learn to 1m)");
  Serial.println("  L  -> land");
  Serial.println("  K  -> kill motors (EMERGENCY)");
  Serial.println("  C  -> flow auto calibration");
  Serial.println("  Y0 -> yaw reset");
  Serial.println("  D60.0 -> refine flowK with 60.0cm measured");
  Serial.println("  B  -> check battery voltage");
  Serial.println("================================");
  Serial.println("⚠️  SAFETY CHECKS:");
  Serial.println("  1. Binding status: Check LEDs");
  Serial.println("  2. Battery voltage: Type 'B'");
  Serial.println("  3. Sensor status: Check POST");
  Serial.println("================================");
}

void loop() {
  static uint32_t lastUs = micros();
  uint32_t nowUs = micros();
  uint32_t elapsedUs = nowUs - lastUs;
  lastUs = nowUs;

  float dt = elapsedUs * 1e-6f;
  if (dt < 0.005f) dt = 0.005f;
  if (dt > 0.050f) dt = 0.050f;
  g_loopDt = dt;

  updateSystem();
  updateFlight();

  delay(20);
}