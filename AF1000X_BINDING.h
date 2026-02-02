#pragma once
#include <Arduino.h>
#include <RF24.h>
#include <Preferences.h>

// ============================================================================
// AF1000X Binding (Drone-side)
// ----------------------------------------------------------------------------
// - Pairing trigger: within 5s after boot, drone inverted -> pairing listen
// - Pairing addr: 0xE8E8F0F0E1
// - TX sends its normal address (5 bytes) repeatedly for ~2s
// - Drone stores that address to NVS and then listens ONLY to it
// - If not bound => flightLock = true (READY stay, no flight)
// ============================================================================

// ---- externs provided by AF1000X.h ----
extern RF24 radio;
extern Preferences prefs;

extern volatile uint8_t currentMode;
extern bool flightLock;

extern float currentRoll;   // deg
extern float currentPitch;  // deg

// ---- timing ----
static constexpr uint32_t BOOT_WINDOW_MS  = 5000;
static constexpr uint32_t PAIR_LISTEN_MS  = 4000;
static constexpr uint32_t LED_BLINK_MS    = 120;

// ---- pairing address (must match AR1000X pairing pipe) ----
static const uint8_t PAIR_ADDR[5] = {0xE8, 0xE8, 0xF0, 0xF0, 0xE1};

// ---- NVS keys ----
static constexpr const char* NVS_KEYB = "bound";
static constexpr const char* NVS_KEY0 = "tx0";
static constexpr const char* NVS_KEY1 = "tx1";
static constexpr const char* NVS_KEY2 = "tx2";
static constexpr const char* NVS_KEY3 = "tx3";
static constexpr const char* NVS_KEY4 = "tx4";

// ---- local state ----
static uint8_t g_boundAddr[5] = {0xB1,0xB2,0xB3,0xB4,0x01};
static bool    g_isBound = false;

enum BindState : uint8_t {
  BS_NORMAL = 0,
  BS_WAIT_WINDOW,
  BS_PAIRING_LISTEN,
  BS_BOUND_OK,
  BS_ERROR
};

static BindState g_bs = BS_WAIT_WINDOW;
static uint32_t  g_bootMs  = 0;
static uint32_t  g_stateMs = 0;

// ---- LED hooks (AF1000X.h에서 구현해주면 됨) ----
void binding_ledAll(bool on);
void binding_ledPairingTick();
void binding_ledBoundOnce();
void binding_ledErrorOnce();

// ---- inverted detect ----
static inline bool binding_isInverted() {
  return (fabsf(currentRoll) > 120.0f) || (fabsf(currentPitch) > 120.0f);
}

// ---- NVS load/save ----
static inline void binding_loadFromNVS() {
  g_isBound = prefs.getBool(NVS_KEYB, false);
  if (!g_isBound) return;
  g_boundAddr[0] = (uint8_t)prefs.getUChar(NVS_KEY0, g_boundAddr[0]);
  g_boundAddr[1] = (uint8_t)prefs.getUChar(NVS_KEY1, g_boundAddr[1]);
  g_boundAddr[2] = (uint8_t)prefs.getUChar(NVS_KEY2, g_boundAddr[2]);
  g_boundAddr[3] = (uint8_t)prefs.getUChar(NVS_KEY3, g_boundAddr[3]);
  g_boundAddr[4] = (uint8_t)prefs.getUChar(NVS_KEY4, g_boundAddr[4]);
}

static inline void binding_saveToNVS(const uint8_t addr[5]) {
  prefs.putUChar(NVS_KEY0, addr[0]);
  prefs.putUChar(NVS_KEY1, addr[1]);
  prefs.putUChar(NVS_KEY2, addr[2]);
  prefs.putUChar(NVS_KEY3, addr[3]);
  prefs.putUChar(NVS_KEY4, addr[4]);
  prefs.putBool(NVS_KEYB, true);
}

// ---- apply listening pipe ----
static inline void binding_listenOn(const uint8_t addr[5]) {
  radio.stopListening();
  radio.openReadingPipe(1, addr);
  radio.startListening();
}

// ---- public helpers ----
static inline bool binding_isBound() { return g_isBound; }

// ---- begin (call after radioInit + prefs.begin) ----
static inline void binding_begin() {
  g_bootMs  = millis();
  g_stateMs = g_bootMs;

  binding_loadFromNVS();

  if (g_isBound) {
    binding_listenOn(g_boundAddr);
    // 바운드면 바인딩 락은 해제 (센서 락은 AF1000X.h에서 따로)
    flightLock = false;
    g_bs = BS_NORMAL;
    binding_ledBoundOnce();
  } else {
    // 미바운드면 비행 락
    flightLock = true;
    currentMode = 0;
    g_bs = BS_WAIT_WINDOW;
  }
}

// ---- tick (call inside updateSystem) ----
static inline void binding_update() {
  const uint32_t now = millis();

  switch (g_bs) {
    case BS_NORMAL:
      return;

    case BS_WAIT_WINDOW: {
      // 부팅 5초 안에 뒤집히면 페어링 모드
      if ((now - g_bootMs) <= BOOT_WINDOW_MS) {
        if (binding_isInverted()) {
          g_bs = BS_PAIRING_LISTEN;
          g_stateMs = now;

          binding_listenOn(PAIR_ADDR);

          flightLock = true;
          currentMode = 0;

          binding_ledAll(false);
          return;
        }
      } else {
        // 창 종료: 미바운드면 에러 상태로 고정 (전원 껐다 켜서 다시 시도)
        binding_listenOn(PAIR_ADDR);
        flightLock = true;
        currentMode = 0;
        g_bs = BS_ERROR;
        binding_ledErrorOnce();
      }
    } break;

    case BS_PAIRING_LISTEN: {
      binding_ledPairingTick();

      if (now - g_stateMs > PAIR_LISTEN_MS) {
        g_bs = BS_ERROR;
        binding_ledErrorOnce();
        flightLock = true;
        currentMode = 0;
        return;
      }

      if (radio.available()) {
        uint8_t addr[5];
        radio.read(&addr, 5);

        // sanity: not all 00 / not all FF
        uint8_t orv = addr[0] | addr[1] | addr[2] | addr[3] | addr[4];
        if (orv == 0x00 || orv == 0xFF) return;

        memcpy(g_boundAddr, addr, 5);
        binding_saveToNVS(g_boundAddr);
        g_isBound = true;

        binding_listenOn(g_boundAddr);

        flightLock = false;
        g_bs = BS_BOUND_OK;
        binding_ledBoundOnce();
        return;
      }
    } break;

    case BS_BOUND_OK:
      g_bs = BS_NORMAL;
      return;

    case BS_ERROR:
      flightLock = true;
      currentMode = 0;
      return;
  }
}
