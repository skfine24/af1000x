# AF1000X FC for Smart Education Drone

## Purpose
This document describes the AF1000X Flight Controller (FC) as a smart education drone reference platform. It specifies supported features, startup behavior, indicators, and hardware guidelines in a product manual format.

## Key Features (Summary)
- Auto takeoff and auto landing
- Altitude hold hover and position hold hover
- External micro servo support
- External LED support and AUX power output (for FPV camera)
- Headless mode
- Safety cut off (emergency stop and failsafe)
- Control mode 1 and 2 supported (default: mode 2)
- DIY auto setup: hover learning + auto tuning

## Automatic Setup (2)
1. Hover learning: automatically adjusts `hoverThrottle` based on stable hover and saves it.
2. Auto tuning: runs altitude PD and yaw P tuning after hover ready and stores the results.

## Accessory I/O (Servo + User LED)
- Micro servo PWM output at 50 Hz (GPIO36), 900 to 2000 us range.
- User LED output on GPIO17. Default targets WS2812C (single wire NeoPixel class).
- WS2801C power supply pads are supported; control requires a dedicated SPI build.
- AUX power output is reserved for FPV camera power.

## Remote Accessory Control
- AR1000X can trigger accessory actions via AUX1 commands.
- LED step: cycles user LED color on each command.
- Servo toggle: switches between min and max angle positions.

## Power On Sequence
1. Power on and wait 1 second.
2. Serial banner prints once.
3. Sensor POST runs (IMU, BARO, ToF, Flow).
4. LED POST display: all LEDs ON, failed sensors blink at 0.5 second steps.
5. If any sensor failed, the status is held for 3 seconds.
6. If IMU or BARO fails, the system enters HARD LOCK (flight disabled) and keeps error LEDs on.
7. If ToF fails, altitude hold continues using barometer only.
8. If Flow fails, position correction is disabled.

POST message examples:
```
POST OK (IMU=1 BARO=1 TOF=1 FLOW=1)
POST FAIL -> FLIGHT LOCK (IMU=0 BARO=1 TOF=1 FLOW=1)
```

## LED Rules (Power On)
1. Sensor LEDs map to IMU, BARO, ToF, and Flow (LED1 to LED4).
2. Boot sequence has top priority over all other LED states.
3. If the board is detected inverted at boot, LEDs run an inverted chase (200 ms per step).
4. Otherwise, all LEDs blink together (1 s ON / 1 s OFF) during boot.
5. When binding completes, all LEDs stay ON for 1 s, then switch to normal logic.
6. During POST, all LEDs turn ON and only failed sensors blink at 0.5 s steps. If any failure occurs, the status is held for 3 s.

## LED Rules (Runtime)
1. Priority order: Boot Bind > Low Battery > Gyro Reset > Headless > Auto Tune > Normal.
2. Low battery: all LEDs blink together (1 s ON / 1 s OFF).
3. Gyro reset animation: all LEDs repeat 0.5 s ON / 1 s OFF for 3 cycles.
4. Headless mode: LED3 LED4 blink 2 s ON / 1 s OFF, LED1 LED2 stay OFF.
5. Auto tune: LED1 LED2 ON while LED3 LED4 OFF, then swap every 1.5 s.
6. Normal state: LED1 to LED3 show sensor OK. LED4 is ON only if Flow is OK and Flow is enabled by AUX2.

## Sensor Policy (POST)
- Required sensors: IMU + BARO
- Optional sensors: ToF + Flow
- IMU or BARO failure triggers HARD LOCK (flight disabled)
- ToF failure falls back to barometer only
- Flow failure disables position correction
- ToF runtime failover: 30 consecutive read failures disable ToF and fall back to barometer

## RF Link (nRF24L01 Standard)
- Recommended RF module: nRF24L01 2.4 GHz (standard)
- The controller scans nearby spectrum and selects low energy channels to build a hop table (FHSS).
- Binding creates a unique ID (TX address) and hop table, then stores them in NVS
- Binding mode: power on AF1000X upside down, then pair with AR1000X

### Identification Code
- Binding ID is a 5 byte TX address (nRF24L01 address format)
- Example format: `0xE7 0xE7 0xE7 0xE7 0xE7`
- Generated per pairing session and stored on both sides
- Used to lock the drone to the controller during normal operation

### Frequency Channel Table (Default)
Default hop table used before pairing overwrites it:
`63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74`

## Processor and Connectivity
- MCU: ESP32-S3FN8
- Wireless: Wi-Fi, Bluetooth
- RF link: nRF24L01 (2.4 GHz)

## Onboard Sensors
- IMU: ICM45686
- Barometer: SPL06
- ToF: VL53L1X
- Optical flow: PMW3901

## DIY Support Specs (Recommended)
- Motor: 720 to 8050 class
- Battery: 1S LiPo, 25C or higher
- All up weight: about 70 g

## Reference Hardware (Current Build)
These values describe the current reference build and may vary with your frame or parts.
- Weight (including battery): about 65 g
- Motor: 720 class, about 52,000 rpm at 1S
- Propeller: 58 mm
- Battery: 1S LiPo (4.15 V to 3.3 V), 600 mAh, 25C
- Motor to motor distance: 120.208 mm

## Flight Controller (AF1000X FC)
- Board version: 01.10
- Size: 35 mm x 35 mm
- Mounting hole: 2 mm diameter, 1.5 mm from board edge

See `IMG/README.md` for board images and mechanical details.

## Recommended Controller
- AR1000X Remote Controller

## Quick Start (Binding)
1. Power on AF1000X and flip it upside down. LEDs 1 to 4 will light sequentially.
2. Power on AR1000X while holding GPIO15 (bind button). GPIO25 LED will blink.
3. When pairing completes, LEDs remain on.

## Repository Layout
- `AF1000X_Main.ino` - main entry
- `AF1000X_AutoTune.h` - auto tuning logic
- `AF1000X_BINDING.h` - binding and FHSS helpers
- `AF1000X_CORE.h` - core flight control logic
- `AF1000X_EasyCommander.h` - high level command helpers
- `AF1000X_GPIO.h` - pin definitions
- `AF1000X_Hover.h` - hover and altitude control
- `AF1000X_PID.h` - PID defaults and tuning constants

## Safety / Warnings
- Always remove propellers before firmware updates or bench testing.
- Perform first flights in a safe, open area with clear line of sight.
- Do not operate the drone when battery voltage is below the recommended range.
- If POST indicates IMU/BARO failure, do not attempt to fly. Power down and inspect sensors.
- Keep hands and loose objects away from rotating parts.
