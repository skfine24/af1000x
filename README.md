# AF1000X FC for Smart Education Drone (EN)

## Purpose
This document describes the AF1000X Flight Controller (FC) as a smart education drone reference platform. It specifies supported features, startup behavior, indicators, and hardware guidelines in a product manual format.

## Key Features (Summary)
- Auto takeoff and auto landing
- Altitude hold hover and position hold hover
- External micro servo support
- External LED support and AUX power output (for FPV camera)
- Headless mode
- 360-degree flip (roll/pitch)
- Self-righting (turtle mode)
- Wi-Fi control input (APK-compatible, experimental)
- Wi-Fi app control (APK-compatible, experimental)
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

## Flip (360-degree)
- Trigger: short press on AR1000X GPIO13 enters flip-ready (LED2 blinks at 0.5 s).
- Direction: choose within 3 s using right stick roll/pitch. If not selected, it returns to normal.
- Conditions: altitude >= 1.5 m, battery >= 3.6 V, headless OFF.
- During flip: tilt kill, attitude hold, and altitude hold are suspended; rate-based flip is executed.
- After flip: returns to previous state and resumes normal control.

## Self-righting (Turtle)
- Trigger: in EMERGENCY (cutoff) mode, move left stick to throttle-low 3 times within 2 s.
- Conditions: inverted detected (|roll| or |pitch| >= 120°), IMU OK, failsafe/flightLock OFF.
- Action: rotates using the shorter axis (roll/pitch) to return upright; stops when upright or after 1.2 s.

## Arming / Auto Takeoff
- Arming requires level attitude: |roll| and |pitch| <= 6°, and not inverted.
- Auto takeoff: if already armed, ascent starts immediately; otherwise it arms then waits 1 s before ascending.

## Power On Sequence
1. Power on and wait 1 second.
2. Serial banner prints once.
3. Sensor POST runs (IMU, BARO, ToF, Flow).
4. LED POST display: all LEDs ON, failed sensors blink at 0.5 second steps.
5. If any sensor failed, the status is held for 3 seconds.
6. If IMU or BARO fails, the system enters HARD LOCK (flight disabled) and keeps error LEDs on.
7. If ToF fails, altitude hold continues using barometer only.
8. If Flow fails, position correction is disabled.
9. After POST, Wi-Fi AP starts for app control; if no client connects within 15 s, Wi-Fi turns off.

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
1. Priority order: Boot Bind > Low Battery > Gyro Reset > Flip Ready > Headless > Auto Tune > Normal.
2. Low battery: all LEDs blink together (1 s ON / 1 s OFF).
3. Gyro reset animation: all LEDs repeat 0.5 s ON / 1 s OFF for 3 cycles.
4. Flip ready: all LEDs blink together (0.5 s ON / 0.5 s OFF).
5. Headless mode: LED1 LED2 stay ON, LED3 LED4 blink 2 s ON / 1 s OFF.
6. Auto tune: LED1 LED2 ON while LED3 LED4 OFF, then swap every 1.5 s.
7. Normal state: LED1 to LED3 show sensor OK. LED4 is ON only if Flow is OK and Flow is enabled by AUX2.

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

## Wi-Fi App Control (Experimental)
- Connect to SSID `SYUBEA_XXXXXX` (MAC-based suffix)
- Target IP/Port: `192.168.169.1:8800` (UDP)
- Wi-Fi AP starts after POST and auto-off after 15 s if no client connects

## Wi-Fi Control (Experimental)
- AP starts after POST; if no client connects within 15 s, Wi-Fi turns off.
- Test SSID: `SYUBEA_XXXXXX` (MAC-based suffix)
- IP: `192.168.169.1`, UDP port: `8800`
- Wi-Fi input maps to RC channels (Throttle/Roll/Pitch/Yaw).
- Supported commands: headless, auto takeoff, gyro reset, emergency stop, flip.
- RC override is always available; stick movement takes priority over Wi-Fi input.

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
- `AF1000X_WIFI.h` - Wi-Fi input bridge (experimental)
- `WIFI.MD` - Wi-Fi command reference (experimental)
- `af1000x_wifi_controller_ui.py` - Wi-Fi app test UI (Mode 2)

## Safety / Warnings
- Always remove propellers before firmware updates or bench testing.
- Perform first flights in a safe, open area with clear line of sight.
- Do not operate the drone when battery voltage is below the recommended range.
- If POST indicates IMU/BARO failure, do not attempt to fly. Power down and inspect sensors.
- Keep hands and loose objects away from rotating parts.

---

# 스마트 교육용 드론 AF1000X FC (KR)

## 목적
이 문서는 스마트 교육용 드론 레퍼런스 플랫폼으로서 AF1000X 비행 컨트롤러(FC)를 설명합니다. 지원 기능, 시작 동작, 표시등 규칙, 하드웨어 가이드를 제품 매뉴얼 형식으로 규정합니다.

## 주요 기능(요약)
- 자동 이륙 및 자동 착륙
- 고도 유지 호버 및 위치 유지 호버
- 외부 마이크로 서보 지원
- 외부 LED 지원 및 AUX 전원 출력(FPV 카메라용)
- 헤드리스 모드
- 360도 공중회전(롤/피치)
- 셀프 라이트(뒤집힘 복구)
- 안전 차단(긴급 정지 및 페일세이프)
- 조종 모드 1과 2 지원(기본값: 모드 2)
- DIY 자동 설정: 호버 학습 + 자동 튜닝
- Wi-Fi 앱을 통한 드론 조종(실험)

## 자동 설정(2)
1. 호버 학습: 안정적인 호버를 기준으로 `hoverThrottle`을 자동으로 보정하고 저장합니다.
2. 자동 튜닝: 호버 준비가 완료되면 고도 PD와 요 P 튜닝을 수행하고 결과를 저장합니다.

## 액세서리 I/O(서보 + 사용자 LED)
- 마이크로 서보 PWM 출력 50 Hz(GPIO36), 900~2000 us 범위.
- 사용자 LED 출력 GPIO17. 기본 대상은 WS2812C(단선 NeoPixel 계열).
- WS2801C 전원 패드는 지원되며, 제어에는 전용 SPI 빌드가 필요합니다.
- AUX 전원 출력은 FPV 카메라 전원용으로 예약됩니다.

## 원격 액세서리 제어
- AR1000X는 AUX1 명령으로 액세서리 동작을 트리거할 수 있습니다.
- LED 단계: 명령마다 사용자 LED 색상을 순환합니다.
- 서보 토글: 최소/최대 각도 위치로 전환합니다.

## 플립(360도)
- 트리거: AR1000X GPIO13 짧게 누르면 플립 준비(LED2 0.5초 점멸).
- 방향 선택: 우측 레버 롤/피치 방향을 3초 이내에 지정. 미지정 시 정상 복귀.
- 조건: 고도 1.5 m 이상, 배터리 3.6 V 이상, 헤드리스 OFF.
- 플립 중: 틸트킬/자세 홀드/고도 홀드를 일시 해제하고 각속도 기반으로 회전.
- 완료 후: 이전 상태로 복귀하고 정상 제어를 재개합니다.

## 셀프 라이트(뒤집힘 복구)
- 트리거: EMERGENCY(컷 오프) 상태에서 왼쪽 레버 스로틀 하단을 2초 내 3회 입력.
- 조건: 뒤집힘 감지(|roll| 또는 |pitch| >= 120°), IMU 정상, failsafe/flightLock OFF.
- 동작: 롤/피치 중 복귀가 빠른 축으로 회전해 정자세 복귀, 정자세가 되거나 1.2초 후 종료.

## 시동 / 자동이륙
- 시동 조건: |roll|, |pitch| <= 6° 그리고 뒤집힘 아님.
- 자동이륙: 이미 시동 상태면 즉시 상승, 아니면 시동 후 1초 대기 뒤 상승.

## 전원 켜짐 시퀀스
1. 전원 인가 후 1초 대기.
2. 시리얼 배너가 한 번 출력됨.
3. 센서 POST 실행(IMU, BARO, ToF, Flow).
4. LED POST 표시: 모든 LED ON, 실패한 센서는 0.5초 간격으로 점멸.
5. 센서 실패가 있으면 상태를 3초 유지.
6. IMU 또는 BARO 실패 시 HARD LOCK 진입(비행 비활성화), 에러 LED 유지.
7. ToF 실패 시 바로미터만 사용하여 고도 유지.
8. Flow 실패 시 위치 보정 비활성화.

POST 메시지 예시:
```
POST OK (IMU=1 BARO=1 TOF=1 FLOW=1)
POST FAIL -> FLIGHT LOCK (IMU=0 BARO=1 TOF=1 FLOW=1)
```
9. POST 완료 후 Wi-Fi AP 시작, 15초 내 연결 없으면 Wi-Fi 자동 종료.

## LED 규칙(전원 켬)
1. 센서 LED는 IMU, BARO, ToF, Flow에 매핑됨(LED1~LED4).
2. 부팅 시퀀스가 모든 LED 상태보다 최우선.
3. 부팅 시 보드가 뒤집힌 상태로 감지되면 LED가 역방향 체이스(200 ms/스텝).
4. 그렇지 않으면 부팅 중 모든 LED가 동시에 점멸(1초 ON/1초 OFF).
5. 바인딩이 완료되면 모든 LED가 1초간 켜진 뒤 정상 로직으로 전환.
6. POST 동안 모든 LED ON, 실패 센서만 0.5초 간격 점멸. 실패가 있으면 상태를 3초 유지.

## LED 규칙(런타임)
1. 우선순위: 부팅/바인딩 > 저전압 > 자이로 리셋 > 플립 준비 > 헤드리스 > 자동 튜닝 > 정상.
2. 저전압: 모든 LED가 동시에 점멸(1초 ON/1초 OFF).
3. 자이로 리셋 애니메이션: 모든 LED가 0.5초 ON/1초 OFF를 3회 반복.
4. 플립 준비: 모든 LED가 0.5초 ON/0.5초 OFF 점멸.
5. 헤드리스 모드: LED1 LED2는 상시 ON, LED3 LED4는 2초 ON/1초 OFF 점멸.
6. 자동 튜닝: LED1 LED2 ON, LED3 LED4 OFF로 시작해 1.5초마다 교대.
7. 정상 상태: LED1~LED3는 센서 OK 표시. LED4는 Flow가 OK이고 AUX2로 Flow가 활성화된 경우에만 ON.

## 센서 정책(POST)
- 필수 센서: IMU + BARO
- 선택 센서: ToF + Flow
- IMU 또는 BARO 실패 시 HARD LOCK(비행 비활성화)
- ToF 실패 시 바로미터만 사용
- Flow 실패 시 위치 보정 비활성화
- ToF 런타임 페일오버: 연속 30회 읽기 실패 시 ToF 비활성화 후 바로미터로 전환

## RF 링크(nRF24L01 표준)
- 권장 RF 모듈: nRF24L01 2.4 GHz(표준)
- 컨트롤러가 주변 스펙트럼을 스캔하여 저에너지 채널을 선택, 홉 테이블(FHSS) 생성
- 바인딩 시 고유 ID(TX 주소)와 홉 테이블 생성 후 NVS에 저장
- 바인딩 모드: AF1000X 전원을 켠 뒤 뒤집어서 AR1000X와 페어링

### 식별 코드
- 바인딩 ID는 5바이트 TX 주소(nRF24L01 주소 형식)
- 예시 형식: `0xE7 0xE7 0xE7 0xE7 0xE7`
- 페어링 세션마다 생성되어 양쪽에 저장됨
- 정상 동작 시 드론을 컨트롤러에 고정(락)하는 데 사용됨

### 주파수 채널 테이블(기본)
페어링 전 기본 홉 테이블:
`63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74`


## Wi-Fi 앱 조종(실험)
- SSID: `SYUBEA_XXXXXX` (MAC 기반)
- 대상 IP/포트: `192.168.169.1:8800` (UDP)
- POST 후 AP 시작, 15초 내 미연결 시 자동 OFF

## 프로세서 및 연결성
- MCU: ESP32-S3FN8
- 무선: Wi-Fi, Bluetooth
- RF 링크: nRF24L01(2.4 GHz)

## 온보드 센서
- IMU: ICM45686
- 바로미터: SPL06
- ToF: VL53L1X
- 옵티컬 플로우: PMW3901

## DIY 지원 사양(권장)
- 모터: 720~8050 클래스
- 배터리: 1S LiPo, 25C 이상
- 비행 중량: 약 70 g

## 레퍼런스 하드웨어(현재 빌드)
이 값들은 현재 레퍼런스 빌드 기준이며, 프레임이나 부품에 따라 달라질 수 있습니다.
- 중량(배터리 포함): 약 65 g
- 모터: 720 클래스, 1S 기준 약 52,000 rpm
- 프로펠러: 58 mm
- 배터리: 1S LiPo(4.15 V ~ 3.3 V), 600 mAh, 25C
- 모터 간 거리: 120.208 mm

## 비행 컨트롤러(AF1000X FC)
- 보드 버전: 01.10
- 크기: 35 mm x 35 mm
- 장착 홀: 지름 2 mm, 보드 가장자리에서 1.5 mm

보드 이미지 및 기구 치수는 `IMG/README.md`를 참고하세요.

## 권장 컨트롤러
- AR1000X 리모컨

## 빠른 시작(바인딩)
1. AF1000X 전원을 켜고 뒤집습니다. LED1~4가 순차 점등됩니다.
2. AR1000X 전원을 켜면서 GPIO15(바인드 버튼)를 누릅니다. GPIO25 LED가 점멸합니다.
3. 페어링이 완료되면 LED가 계속 켜진 상태가 됩니다.

## 저장소 구성
- `AF1000X_Main.ino` - 메인 엔트리
- `AF1000X_AutoTune.h` - 자동 튜닝 로직
- `AF1000X_BINDING.h` - 바인딩 및 FHSS 헬퍼
- `AF1000X_CORE.h` - 코어 비행 제어 로직
- `AF1000X_EasyCommander.h` - 고수준 명령 헬퍼
- `AF1000X_GPIO.h` - 핀 정의
- `AF1000X_Hover.h` - 호버 및 고도 제어
- `AF1000X_PID.h` - PID 기본값 및 튜닝 상수
- `AF1000X_WIFI.h` - Wi-Fi 입력 브릿지(실험)
- `WIFI.MD` - Wi-Fi 명령 정리(실험)
- `af1000x_wifi_controller_ui.py` - Wi-Fi 앱 테스트 UI (Mode 2)

## 안전 / 경고
- 펌웨어 업데이트나 벤치 테스트 전에는 반드시 프로펠러를 분리하세요.
- 첫 비행은 안전하고 개방된 장소에서 시야 확보 상태로 진행하세요.
- 배터리 전압이 권장 범위 이하일 때는 운용하지 마세요.
- POST에서 IMU/BARO 실패가 표시되면 비행하지 말고 전원을 끈 후 센서를 점검하세요.
- 회전 부품 근처에 손이나 느슨한 물체를 가까이하지 마세요.

---

# スマート教育用ドローン AF1000X FC (JP)

## 目的
このドキュメントは、スマート教育用ドローンのリファレンスプラットフォームとしてのAF1000Xフライトコントローラ(FC)を説明します。対応機能、起動動作、インジケータ、ハードウェアガイドラインを製品マニュアル形式で規定します。

## 主な機能(概要)
- 自動離陸および自動着陸
- 高度保持ホバーおよび位置保持ホバー
- 外部マイクロサーボ対応
- 外部LED対応およびAUX電源出力(FPVカメラ用)
- ヘッドレスモード
- 360度フリップ(ロール/ピッチ)
- セルフライト(転倒復帰)
- セーフティカットオフ(緊急停止およびフェイルセーフ)
- 操縦モード1と2に対応(デフォルト: モード2)
- DIY自動セットアップ: ホバー学習 + 自動チューニング
- Wi-Fiアプリでドローン操作 (実験)

## 自動セットアップ(2)
1. ホバー学習: 安定したホバーを基準に`hoverThrottle`を自動調整して保存します。
2. 自動チューニング: ホバー準備完了後に高度PDとヨーPのチューニングを実行し、結果を保存します。

## アクセサリI/O(サーボ + ユーザーLED)
- マイクロサーボPWM出力 50 Hz(GPIO36)、900〜2000 usレンジ。
- ユーザーLED出力 GPIO17。デフォルト対象はWS2812C(単線のNeoPixel系)。
- WS2801Cの電源パッドをサポート。制御には専用SPIビルドが必要です。
- AUX電源出力はFPVカメラ電源用に予約されています。

## リモートアクセサリ制御
- AR1000XはAUX1コマンドでアクセサリ動作をトリガーできます。
- LEDステップ: コマンドごとにユーザーLEDの色を循環します。
- サーボトグル: 最小/最大角度位置を切り替えます。

## フリップ(360度)
- トリガー: AR1000X GPIO13を短押しするとフリップ準備(LED2が0.5秒点滅)。
- 方向選択: 右スティックのロール/ピッチ方向を3秒以内に指定。未指定なら通常に復帰。
- 条件: 高度1.5 m以上、バッテリー3.6 V以上、ヘッドレスOFF。
- フリップ中: ティルトキル/姿勢ホールド/高度ホールドを一時停止し、角速度ベースで回転。
- 完了後: 以前の状態に戻り、通常制御へ復帰。

## セルフライト(転倒復帰)
- トリガー: EMERGENCY(カットオフ)状態で左スロットルを2秒以内に3回下げる。
- 条件: 反転検出(|roll|または|pitch| >= 120°)、IMU正常、failsafe/flightLock OFF。
- 動作: ロール/ピッチの短い軸で回転して正立復帰、正立または1.2秒で停止。

## アーミング / 自動離陸
- アーミング条件: |roll|, |pitch| <= 6° かつ反転でないこと。
- 自動離陸: 既にアーミング済みなら即上昇、未アーミングならアーミング後1秒待機。

## 電源オンシーケンス
1. 電源オン後、1秒待機。
2. シリアルバナーが1回出力される。
3. センサーPOSTを実行(IMU, BARO, ToF, Flow)。
4. LED POST表示: すべてのLEDがON、失敗センサーは0.5秒間隔で点滅。
5. センサー失敗がある場合、状態を3秒保持。
6. IMUまたはBARO失敗時はHARD LOCK(飛行無効)に入り、エラーLEDを保持。
7. ToF失敗時はバロメータのみで高度保持を継続。
8. Flow失敗時は位置補正が無効。

POSTメッセージ例:
```
POST OK (IMU=1 BARO=1 TOF=1 FLOW=1)
POST FAIL -> FLIGHT LOCK (IMU=0 BARO=1 TOF=1 FLOW=1)
```
9. POST完了後にWi-Fi AP開始。15秒以内に接続がなければWi-Fi自動OFF。

## LEDルール(電源オン)
1. センサーLEDはIMU, BARO, ToF, Flowに対応(LED1〜LED4)。
2. ブートシーケンスは他のすべてのLED状態より優先。
3. 起動時に基板が反転して検出された場合、LEDは逆方向チェイス(200 ms/ステップ)。
4. それ以外は起動中、全LEDが同時に点滅(1秒ON/1秒OFF)。
5. バインド完了時、全LEDが1秒ONの後、通常ロジックへ切り替え。
6. POST中は全LED ON、失敗センサーのみ0.5秒間隔で点滅。失敗がある場合、状態を3秒保持。

## LEDルール(ランタイム)
1. 優先順位: ブート/バインド > 低電圧 > ジャイロリセット > フリップ準備 > ヘッドレス > 自動チューニング > 通常。
2. 低電圧: 全LEDが同時に点滅(1秒ON/1秒OFF)。
3. ジャイロリセットアニメーション: 全LEDが0.5秒ON/1秒OFFを3サイクル繰り返し。
4. フリップ準備: 全LEDが0.5秒ON/0.5秒OFFで点滅。
5. ヘッドレスモード: LED1 LED2は常時ON、LED3 LED4は2秒ON/1秒OFF点滅。
6. 自動チューニング: LED1 LED2 ON、LED3 LED4 OFFで開始し、1.5秒ごとに交代。
7. 通常状態: LED1〜LED3はセンサーOK表示。LED4はFlowがOKかつAUX2でFlowが有効な場合のみON。

## センサーポリシー(POST)
- 必須センサー: IMU + BARO
- 任意センサー: ToF + Flow
- IMUまたはBARO失敗はHARD LOCK(飛行無効)をトリガー
- ToF失敗はバロメータへフォールバック
- Flow失敗は位置補正を無効化
- ToFランタイムフェイルオーバー: 連続30回の読み取り失敗でToFを無効化し、バロメータにフォールバック

## RFリンク(nRF24L01 標準)
- 推奨RFモジュール: nRF24L01 2.4 GHz(標準)
- コントローラが周辺スペクトラムをスキャンし、低エネルギーチャンネルを選択してホップテーブル(FHSS)を作成。
- バインディングで一意のID(TXアドレス)とホップテーブルを生成し、NVSに保存
- バインディングモード: AF1000Xを上下逆さまにして電源オンし、AR1000Xとペアリング

### 識別コード
- バインディングIDは5バイトのTXアドレス(nRF24L01アドレス形式)
- 例: `0xE7 0xE7 0xE7 0xE7 0xE7`
- ペアリングセッションごとに生成され、両側に保存
- 通常動作時にドローンをコントローラにロックするために使用

### 周波数チャンネルテーブル(デフォルト)
ペアリング前のデフォルトホップテーブル:
`63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74`


## Wi-Fiアプリ操作 (実験)
- SSID: `SYUBEA_XXXXXX` (MACベース)
- 送信先IP/ポート: `192.168.169.1:8800` (UDP)
- POST後にAP開始。15秒以内に接続がなければ自動OFF

## プロセッサと接続性
- MCU: ESP32-S3FN8
- 無線: Wi-Fi, Bluetooth
- RFリンク: nRF24L01(2.4 GHz)

## オンボードセンサー
- IMU: ICM45686
- 気圧計: SPL06
- ToF: VL53L1X
- オプティカルフロー: PMW3901

## DIY対応仕様(推奨)
- モータ: 720〜8050クラス
- バッテリー: 1S LiPo, 25C以上
- 離陸重量: 約70 g

## リファレンスハードウェア(現行ビルド)
これらの値は現行のリファレンスビルドを示し、フレームや部品により変わる場合があります。
- 重量(バッテリー含む): 約65 g
- モータ: 720クラス、1Sで約52,000 rpm
- プロペラ: 58 mm
- バッテリー: 1S LiPo(4.15 V〜3.3 V), 600 mAh, 25C
- モータ間距離: 120.208 mm

## フライトコントローラ(AF1000X FC)
- ボードバージョン: 01.10
- サイズ: 35 mm x 35 mm
- 取付穴: 直径2 mm、ボード端から1.5 mm

ボード画像と機構寸法は`IMG/README.md`を参照してください。

## 推奨コントローラ
- AR1000X リモートコントローラ

## クイックスタート(バインディング)
1. AF1000Xの電源を入れて上下逆さまにします。LED1〜4が順番に点灯します。
2. GPIO15(バインドボタン)を押しながらAR1000Xの電源を入れます。GPIO25 LEDが点滅します。
3. ペアリング完了後、LEDは点灯したままになります。

## リポジトリ構成
- `AF1000X_Main.ino` - メインエントリ
- `AF1000X_AutoTune.h` - 自動チューニングロジック
- `AF1000X_BINDING.h` - バインディングとFHSSヘルパ
- `AF1000X_CORE.h` - コア飛行制御ロジック
- `AF1000X_EasyCommander.h` - 高レベルコマンドヘルパ
- `AF1000X_GPIO.h` - ピン定義
- `AF1000X_Hover.h` - ホバーと高度制御
- `AF1000X_PID.h` - PIDデフォルトとチューニング定数
- `AF1000X_WIFI.h` - Wi-Fi入力ブリッジ(実験)
- `WIFI.MD` - Wi-Fiコマンド整理(実験)
- `af1000x_wifi_controller_ui.py` - Wi-FiアプリテストUI (Mode 2)

## 安全 / 警告
- ファームウェア更新やベンチテストの前に、必ずプロペラを外してください。
- 初飛行は安全で開けた場所で、見通しを確保して実施してください。
- バッテリー電圧が推奨範囲を下回る場合は運用しないでください。
- POSTでIMU/BAROの失敗が示された場合は飛行せず、電源を切ってセンサーを点検してください。
- 回転部品の近くに手や緩い物を近づけないでください。
