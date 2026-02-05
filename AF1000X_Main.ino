#define AF1000X_IMPLEMENTATION
#include "AF1000X_CORE.h"
#include "AF1000X_EasyCommander.h"
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include "esp32-hal-bt.h"
#include "esp_system.h"
#include <stdio.h>
#endif
float g_loopDt = 0.02f;  // KO: Í∏Î°úÎ≤å Î£®ÌîÑ dt(Ï¥) / EN: global loop dt (seconds)

#if defined(ARDUINO_ARCH_ESP32)
static char WIFI_AP_SSID[20] = "SYUBEA_000000";
static const uint32_t WIFI_AP_TIMEOUT_MS = 15000;
static bool wifiApActive = false;
static bool wifiAutoOffDone = false;
static uint32_t wifiApStartMs = 0;

static void buildWifiApSsid() {
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  snprintf(WIFI_AP_SSID, sizeof(WIFI_AP_SSID), "SYUBEA_%02X%02X%02X", mac[3], mac[4], mac[5]);
}

static void startWifiApAfterPost() {
  buildWifiApSsid();
  WiFi.mode(WIFI_AP);
  IPAddress ip(192, 168, 169, 1);
  IPAddress gw(192, 168, 169, 1);
  IPAddress mask(255, 255, 255, 0);
  WiFi.softAPConfig(ip, gw, mask);
  wifiApActive = WiFi.softAP(WIFI_AP_SSID);
  wifiInputBegin();
  wifiApStartMs = millis();
  wifiAutoOffDone = false;
  if (wifiApActive) {
    LOG_PRINT("WiFi AP ON: ");
    LOG_PRINTLN(WIFI_AP_SSID);
  } else {
    LOG_PRINTLN("WiFi AP start failed");
  }
}

static void wifiAutoOffTick() {
  if (!wifiApActive || wifiAutoOffDone) return;
  if (WiFi.softAPgetStationNum() > 0) {
    wifiAutoOffDone = true;
    return;
  }
  if ((uint32_t)(millis() - wifiApStartMs) >= WIFI_AP_TIMEOUT_MS) {
    WiFi.softAPdisconnect(true);
    wifiInputEnd();
    WiFi.mode(WIFI_OFF);
    wifiApActive = false;
    wifiAutoOffDone = true;
    LOG_PRINTLN("WiFi AP OFF (no client)");
  }
}

static void disableRadiosDefault() {
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);
  WiFi.setSleep(true);
  btStop();
}
#endif

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // KO: †àÍ±∞Ïãú Serial ùëãµ Îπ†Î•¥Í≤ / EN: keep legacy Serial reads snappy
  delay(1000);
#if defined(ARDUINO_ARCH_ESP32)
  disableRadiosDefault();
#endif


  LOG_PRINTLN();
  LOG_PRINTLN("SYUBEA Co., LTD");
  LOG_PRINTLN("www.1510.co.kr");
  LOG_PRINT("Model : AF1000X FC - Ver.");
  LOG_PRINTLN(FC_VERSION);
  LOG_PRINTLN();
  initAF1000X();
#if defined(ARDUINO_ARCH_ESP32)
  startWifiApAfterPost();
#endif
  LOG_PRINTLN("Use `PID?` / `PIDDEF` outputs to back up key tuning values");  
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
#if defined(ARDUINO_ARCH_ESP32)
  wifiAutoOffTick();
#endif

  delay(20);
}