#ifndef AF1000X_WIFI_H
#define AF1000X_WIFI_H

#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <WiFiUdp.h>
#endif

// Wi-Fi control input (APK compatible, minimal mapping).

extern bool wifiActive;
extern uint32_t wifiLastPktMs;
extern Signal wifiSignal;

static inline void emergencyStop();

void wifiInputBegin();
void wifiInputEnd();
void wifiInputTick();
bool wifiRcOverrideActive(uint32_t now, const Signal& rc);
void wifiSelectInput(const Signal& rc, Signal& out);

#ifdef AF1000X_IMPLEMENTATION

#if defined(ARDUINO_ARCH_ESP32)
static WiFiUDP wifiUdp;
static const uint16_t WIFI_UDP_PORT = 8800;
static const uint32_t WIFI_ACTIVE_TIMEOUT_MS = 1000;
static const uint8_t WIFI_RC_OVERRIDE_DEADBAND = 12;
static const uint32_t WIFI_RC_OVERRIDE_HOLD_MS = 500;
static uint32_t wifiRcOverrideUntil = 0;
static uint32_t wifiEmergencyLastMs = 0;
#endif

bool wifiActive = false;
uint32_t wifiLastPktMs = 0;
Signal wifiSignal{};

static inline void _wifiSetDefaults() {
  wifiSignal.throttle = 0;
  wifiSignal.roll = 128;
  wifiSignal.pitch = 128;
  wifiSignal.yaw = 128;
  wifiSignal.aux1 = 0;
  wifiSignal.aux2 = 0;
  wifiSignal.speed = 1;
  wifiSignal.hop = 0;
}

void wifiInputBegin() {
#if defined(ARDUINO_ARCH_ESP32)
  wifiUdp.begin(WIFI_UDP_PORT);
#endif
  wifiLastPktMs = 0;
  wifiActive = false;
  _wifiSetDefaults();
}

void wifiInputEnd() {
#if defined(ARDUINO_ARCH_ESP32)
  wifiUdp.stop();
#endif
  wifiActive = false;
  wifiLastPktMs = 0;
}

void wifiInputTick() {
#if defined(ARDUINO_ARCH_ESP32)
  const uint32_t now = millis();
  int packetSize = wifiUdp.parsePacket();
  while (packetSize > 0) {
    uint8_t buf[200];
    int len = wifiUdp.read(buf, sizeof(buf));
    if (len >= 24 && buf[0] == 0xEF) {
      wifiLastPktMs = now;
      wifiSignal.throttle = buf[20];
      wifiSignal.roll = buf[21];
      wifiSignal.pitch = buf[22];
      wifiSignal.yaw = buf[23];
      wifiSignal.aux1 = 0;
      wifiSignal.aux2 = 0;
      wifiSignal.speed = 1;

      const bool emergency = (len > 29 && buf[27] == 0x11 && buf[29] == 0x3c);
      const bool gyroReset = (wifiSignal.throttle <= 32 &&
                              wifiSignal.roll <= 32 &&
                              wifiSignal.pitch <= 32 &&
                              wifiSignal.yaw <= 32);
      const bool takeoff = (len > 29 && buf[29] == 0x2d);

      if (emergency) {
        if (now - wifiEmergencyLastMs > 500) {
          wifiEmergencyLastMs = now;
          emergencyStop();
        }
      } else if (gyroReset) {
        wifiSignal.aux1 = AUX1_GYRO_RESET;
      } else if (takeoff) {
        wifiSignal.aux1 = AUX1_TAKEOFF;
      }
    }
    packetSize = wifiUdp.parsePacket();
  }

  const bool apConnected = (WiFi.softAPgetStationNum() > 0);
  const bool recent = (wifiLastPktMs != 0) && ((uint32_t)(now - wifiLastPktMs) <= WIFI_ACTIVE_TIMEOUT_MS);
  wifiActive = apConnected && recent;
#endif
}

bool wifiRcOverrideActive(uint32_t now, const Signal& rc) {
#if defined(ARDUINO_ARCH_ESP32)
  if (!wifiActive) {
    wifiRcOverrideUntil = 0;
    return false;
  }
  int dr = abs((int)rc.roll - 128);
  int dp = abs((int)rc.pitch - 128);
  int dy = abs((int)rc.yaw - 128);
  if (dr > WIFI_RC_OVERRIDE_DEADBAND || dp > WIFI_RC_OVERRIDE_DEADBAND || dy > WIFI_RC_OVERRIDE_DEADBAND) {
    wifiRcOverrideUntil = now + WIFI_RC_OVERRIDE_HOLD_MS;
  }
  if (wifiRcOverrideUntil != 0 && (int32_t)(now - wifiRcOverrideUntil) < 0) {
    return true;
  }
#endif
  return false;
}

void wifiSelectInput(const Signal& rc, Signal& out) {
#if defined(ARDUINO_ARCH_ESP32)
  const uint32_t now = millis();
  if (!wifiActive) {
    out = rc;
    return;
  }
  if (wifiRcOverrideActive(now, rc)) {
    out = rc;
  } else {
    out = wifiSignal;
  }
#else
  out = rc;
#endif
}

#endif // AF1000X_IMPLEMENTATION

#endif // AF1000X_WIFI_H
