# AF1000X (Smart Drone) / AR1000X (Drone Controller) Operation Manual (EN)

## 1. Document Overview
- Scope: AF1000X smart drone, AR1000X drone controller
- Purpose: Describe power-up, pairing, link, FHSS, input source switching, safety behavior, and PC hub control in a product-manual style.
- Sources: `AR1000X_Main.ino`, `AR1000X_Controller_Operation.md`, `AR1000X_Serial_Command_Reference.md`, user-provided operation sequence

## 2. Terminology
- AF1000X: Smart drone (receiver / airframe)
- AR1000X: Drone controller (transmitter / PC hub)

## 3. Safety Notes
- Perform PC control only after `ARM 1`.
- Use `EMERGENCY` or the stick emergency combo for immediate stop.
- On link loss, AF1000X enters failsafe and performs auto-landing or motor cut per spec.
- Minimize stick interference during auto takeoff/autotune.
- Maintain stable power on both AR1000X and AF1000X during pairing.

## 4. System Architecture
```
PC (USB Serial 115200)
 -> AR1000X Drone Controller (RP2040, nRF24L01+)
 -> RF (FHSS)
 -> AF1000X Smart Drone
```
- AR1000X acts as a safety gate and allows PC control only after `ARM 1`.
- If the `JOY` stream stops for more than 200 ms, control falls back to sticks.

## 5. AR1000X Drone Controller Operation

## 5.1 Power-On and Boot
1. Initialize USB CDC at 115200 and set the USB product name to `SYUBEA AF1000X Controller`.
2. Boot-time button holds determine modes and initial behavior.
3. Initialize RF with `PAIR_CHANNEL=76`, `RF24_250KBPS`, `PA_LOW`, and open the TX pipe.
4. Load the FHSS hop table from EEPROM.
5. If EEPROM has no table, build `hopTable` from a scan.
6. If the pairing button is held during boot, start pairing immediately.

## 5.2 Pairing, Binding, and Link
- Pairing repeatedly transmits `LinkPayload` on `PAIR_CHANNEL=76` until ACK is received.
- `LinkPayload` contains controller name (`AF1000X`), TX address, hop table, flags, and CRC.
- On pairing success, `linkReady=true`.
- `boundOK` becomes true only after successful control packet transmission; pairing alone does not guarantee binding.
- When `linkReady=false`, the controller re-sends link payload every 500 ms in the background.
- If a new hop table is ready, it is applied after a successful link and saved to EEPROM.

## 5.3 FHSS Operation
- Scan range: channels 5 to 80 (2405 to 2480 MHz)
- Sampling: 4 RPD samples per channel
- Selection: 12 lowest-interference channels (`HOP_MAX=12`)
- Hop slot: 20 ms (`HOP_SLOT_MS=20`)
- Hop index is included in every packet.

## 5.4 Input Source Switching
- Input paths are split into `stickSig` (physical sticks) and `pcSig` (PC).
- When sticks are active, PC override is immediately cleared.
- PC input is valid only after `ARM 1`; otherwise the controller returns `IGN`.
- If the `JOY` stream pauses for more than 200 ms, control returns to sticks.

## 5.5 Button and Mode Summary
- Button functions and GPIO mapping follow Appendix B.
- Speed, headless, optical flow, and trim are runtime toggles.
- Auto takeoff/land and gyro reset are requested via AUX1 pulses.

## 5.6 Safety and Emergency
- `EMERGENCY` cuts throttle immediately and attempts to transmit a packet right away.
- `STOP` is handled the same as `EMERGENCY`.
- The stick emergency combo triggers when both sticks are held to the lower diagonals for 2 seconds.
- On emergency, PC override is cleared and a neutral packet is transmitted.

## 5.7 PC Hub Control
- Link: USB CDC 115200, line ending `\r` or `\n`, case-insensitive
- Core commands: `ARM 1`, `ARM 0`, `BIND`, `PAIR`, `JOY ...`, `TAKEOFF`, `LAND`, `SPEED 1/2/3`, `HEADLESS`, `FLOW ON/OFF`, `EMERGENCY`, `STOP`, `BAT?`, `ID?`, `DBUG`
- dd3 move commands: `FORWARD|BACK|LEFT|RIGHT|UP|DOWN|CW|CCW <power> <time_ms>`
- Response codes: `OK`, `IGN`, `ERR`

## 6. AF1000X Smart Drone Operation

## 6.1 Power-On and Pairing Sequence
1. Power on the AF1000X and flip it upside down.
2. LEDs 1 to 4 light sequentially.
3. Power on AR1000X while holding `GPIO15`.
4. The AR1000X LED1 (LED1_POWER) blinks.
5. When pairing completes, LEDs remain solid on both devices.

## 6.2 LED Indication Spec (Fixed)
- LEDs 1-4 are used for binding status indication.
- Sequential order: LED1 → LED2 → LED3 → LED4.
- Sequential interval: 250 ms.
- On binding complete, LEDs 1-4 stay solid ON.

## 6.3 Auto Takeoff and Autotune Spec (Fixed)
- Purpose: PID tuning
- Trigger: auto takeoff request (AUX1 pulse)
- Total duration: about 30 seconds
- Target altitude: about 1 m
- Calibration: PWM values are adjusted during ascent
- Completion: switches to normal flight mode after autotune

## 6.4 Link-Loss Failsafe Spec (Design Fixed)
Detection: 500 ms without a valid control packet.
Sequence:
1. Ramp throttle down to zero over 1 second
2. Maintain auto-landing mode during the 1 to 3 second window (gentle descent)
3. Cut motors after 3 seconds if not landed
Indication: link-loss LED pattern follows Appendix D.

## 6.5 Control Signal Reception
- AF1000X receives stick or PC control signals transmitted by AR1000X.
- `AUX1` is used for takeoff/land, gyro reset, LED step, and servo toggle requests.
- `AUX2` is a bitfield for headless and optical flow state.

## 6.6 Sensors and Outputs
- RF: nRF24L01+
- Range sensor: VL53L1X
- IMU: ICM45686
- LED: WS2812C (GPIO17)
- Servo: GPIO36 (0 to 180 degrees, 900 to 2000 us)

## 6.7 Operational Notes
- Minimize stick input during auto takeoff/autotune.
- Maintain stable power on both AR1000X and AF1000X during pairing.

## 7. Operating Scenarios

## 7.1 First-Time Binding
1. Boot AR1000X with `GPIO15` held to run pairing.
2. Wait until LED1 stays solid on.
3. When AF1000X responds, `boundOK` becomes true and LED1 stays solid.

## 7.2 Stick Operation
1. Stick activity clears PC override.
2. Headless, flow, and trim update immediately via button toggles.

## 7.3 PC Hub Operation
1. Send `ARM 1` to enable PC control.
2. Use `JOY` streaming or dd3 commands.
3. If the stream pauses for more than 200 ms, control returns to sticks.

## 7.4 Emergency Stop
1. Send `EMERGENCY` or `STOP`.
2. The stick emergency combo (2-second hold) also cuts throttle.

## 8. Appendices

## Appendix A. Diagrams and Images

![Figure A-1. System schematic](SCH_Schematic1_1-P1_2026-02-03.png)
Figure A-1. Full schematic. Use for power, MCU, and RF connectivity reference.

![Figure A-2. RP2040 pin summary](_crop_rp_pins.png)
Figure A-2. RP2040 pin summary for GPIO placement.

![Figure A-3. Joystick wiring](_crop_joystick.png)
Figure A-3. Joystick wiring summary for axes and center reference.

![Figure A-4. MUX wiring](_crop_mux.png)
Figure A-4. MUX base wiring for ADC multiplexing path.

![Figure A-5. MUX wiring detail](_crop_mux2.png)
Figure A-5. MUX wiring detail for S0 and ADC connections.

## Appendix B. AR1000X Pin Map (Summary)
| Category | Signal | GPIO | Notes |
| --- | --- | --- | --- |
| RF | RF_CE | 6 | nRF24L01+ CE |
| RF | RF_CSN | 5 | nRF24L01+ CSN |
| MUX | MUX_S0 | 22 | Channel select |
| MUX | MUX_S1 | - | Tied to GND (unused) |
| MUX | MUX_S2 | - | Tied to GND (unused) |
| MUX | MUX_ADC | 29 | ADC input |
| ADC | ADC_THROTTLE | 26 | Throttle |
| ADC | ADC_YAW | 27 | Yaw |
| ADC | ADC_PITCH | 28 | Pitch |
| BTN | BTN_PAIR_SPEED | 15 | Pair/speed |
| BTN | BTN_AUTO | 11 | Auto takeoff/gyro |
| BTN | BTN_TRIM_MODE | 14 | Trim/mode |
| BTN | BTN_HEADLESS | 13 | Headless |
| BTN | BTN_FLOW_TOGGLE | 10 | Flow/ID regen |
| BTN | BTN_LED_SERVO | 12 | LED/servo |
| LED | LED1_POWER | 25 | Power/bind |
| LED | LED2_HEADLESS | 23 | Headless |
| LED | LED3_TRIM | 24 | Trim |

## Appendix C. AR1000X LED Timing
| LED | State | Pattern | Period |
| --- | --- | --- | --- |
| LED1 (POWER) | Low battery | Blink | 0.5 s |
| LED1 (POWER) | Not bound | Blink | 1.0 s |
| LED1 (POWER) | Bound | Solid ON | - |
| LED1 (POWER) | Pairing | Blink | 0.25 s |
| LED2 (HEADLESS) | ON | Blink | 1.0 s |
| LED2 (HEADLESS) | OFF | OFF | - |
| LED3 (TRIM) | ON | Blink | 0.5 s |
| LED3 (TRIM) | OFF | OFF | - |

## Appendix D. AF1000X LED Timing (Fixed Spec)
| State | LED1 | LED2 | LED3 | LED4 | Notes |
| --- | --- | --- | --- | --- | --- |
| Binding prepare | Sequential | Sequential | Sequential | Sequential | 250 ms interval |
| Binding timeout (30 s) | Blink together | Blink together | Blink together | Blink together | 0.5 s period, return to binding prepare after 5 s |
| Binding complete | Solid ON | Solid ON | Solid ON | Solid ON | - |
| Link loss (in flight) | Blink together | Blink together | Blink together | Blink together | 1.0 s period, during failsafe |

## 9. Improvement Log
| Area | Item | Proposal | Status |
| --- | --- | --- | --- |
| Safety/Failsafe | Link-loss behavior | Define rampdown, auto-landing, and motor-cut sequence on link loss | Spec fixed (to implement) |
| Safety/Failsafe | Emergency feedback | Add LED or buzzer pattern on emergency | Spec pending |
| LED/Autotune | LED error pattern | Define LED pattern for pairing timeout and link loss | Spec fixed (to implement) |
| LED/Autotune | Autotune failure handling | Define fallback (auto land or motor cut) on failure | Spec pending |
| Pairing/Link | Retry policy | Specify retry counts and timeouts for pairing | Spec pending |
| Pairing/Link | Quality metrics | Add link-quality logging and success-rate tracking | Proposal |
| Document | Manual tone | Normalize structure and tone | Done |
| Document | Image appendix | Add captions and explanations | Done |
| Document | English manual | Provide English version | Done |
