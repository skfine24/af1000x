#pragma once

// ============================================================================
// KO: AF1000X GPIO / 핀 맵
// EN: AF1000X GPIO / pin map
// ============================================================================

// KO: RF
// EN: RF
static const int PIN_RF_CE   = 2;
static const int PIN_RF_CSN  = 5;
static const uint8_t RF_ADDR[5] = {0xB1,0xB2,0xB3,0xB4,0x01};

// KO: 모터
// EN: Motors
static const int PIN_M1 = 10;
static const int PIN_M2 = 11;
static const int PIN_M3 = 12;
static const int PIN_M4 = 13;

// KO: I2C
// EN: I2C
static const int PIN_I2C_SDA = 3;
static const int PIN_I2C_SCL = 4;

// KO: SPI
// EN: SPI
static const int PIN_SPI_SCK  = 33;
static const int PIN_SPI_MISO = 34;
static const int PIN_SPI_MOSI = 14;

// KO: ToF
// EN: ToF
static const int PIN_VL53_XSHUT = 35;

// KO: 옵티컬 플로우
// EN: Optical flow
static const int PIN_FLOW_CS     = 38;
static const int PIN_FLOW_MOTION = 37;

// KO: 배터리 ADC
// EN: Battery ADC
static const int PIN_BAT_ADC = 1;

// KO: LED (센서 상태)
// EN: LEDs (sensor status)
static const int PIN_LED_IMU  = 6;
static const int PIN_LED_BARO = 7;
static const int PIN_LED_TOF  = 8;
static const int PIN_LED_FLOW = 9;

// EN: User LED (single-wire RGB/NeoPixel data)
static const int PIN_USER_LED = 17;

// EN: User servo PWM
static const int PIN_SERVO = 36;
