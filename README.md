# AF1000X Smart Drone

AF1000X is a smart micro drone and reference platform built for education and rapid prototyping. It combines sensor fusion, automation, and safety focused behavior so builders can learn the full stack while still getting stable flight and repeatable results.

### Key Features (Summary)
- Auto takeoff and auto landing
- Altitude hold hover and position hold hover
- External micro servo support
- External LED support and AUX power output (for FPV camera)
- Headless mode
- Safety cut off (emergency stop and failsafe)
- Control mode 1 and 2 supported (default: mode 2)
- DIY auto setup: hover learning + auto tuning

### Automatic Setup (2)
1. Hover learning: automatically adjusts `hoverThrottle` based on stable hover and saves it.
2. Auto tuning: runs altitude PD and yaw P tuning after hover ready and stores the results.

### Accessory I/O (Servo + User LED)
- Micro servo PWM output at 50 Hz (GPIO36), 900 to 2000 us range.
- User LED output on GPIO17. Default targets WS2812C (single wire NeoPixel class).
- WS2801C power supply pads are supported; control requires a dedicated SPI build.
- AUX power output is reserved for FPV camera power.

### Remote Accessory Control
- AR1000X can trigger accessory actions via AUX1 commands.
- LED step: cycles user LED color on each command.
- Servo toggle: switches between min and max angle positions.

### Power On Sequence
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

### LED Rules (Power On)
1. Sensor LEDs map to IMU, BARO, ToF, and Flow (LED1 to LED4).
2. Boot sequence has top priority over all other LED states.
3. If the board is detected inverted at boot, LEDs run an inverted chase (200 ms per step).
4. Otherwise, all LEDs blink together (1 s ON / 1 s OFF) during boot.
5. When binding completes, all LEDs stay ON for 1 s, then switch to normal logic.
6. During POST, all LEDs turn ON and only failed sensors blink at 0.5 s steps. If any failure occurs, the status is held for 3 s.

### LED Rules (Runtime)
1. Priority order: Boot Bind > Low Battery > Gyro Reset > Headless > Auto Tune > Normal.
2. Low battery: all LEDs blink together (1 s ON / 1 s OFF).
3. Gyro reset animation: all LEDs repeat 0.5 s ON / 1 s OFF for 3 cycles.
4. Headless mode: LED3 LED4 blink 2 s ON / 1 s OFF, LED1 LED2 stay OFF.
5. Auto tune: LED1 LED2 ON while LED3 LED4 OFF, then swap every 1.5 s.
6. Normal state: LED1 to LED3 show sensor OK. LED4 is ON only if Flow is OK and Flow is enabled by AUX2.

### Sensor Policy (POST)
- Required sensors: IMU + BARO
- Optional sensors: ToF + Flow
- IMU or BARO failure triggers HARD LOCK (flight disabled)
- ToF failure falls back to barometer only
- Flow failure disables position correction
- ToF runtime failover: 30 consecutive read failures disable ToF and fall back to barometer

### RF Link (nRF24L01 Standard)
- Recommended RF module: nRF24L01 2.4 GHz (standard)
- The controller scans nearby spectrum and selects low energy channels to build a hop table (FHSS).
- Binding creates a unique ID (TX address) and hop table, then stores them in NVS
- Binding mode: power on AF1000X upside down, then pair with AR1000X

#### Identification Code
- Binding ID is a 5 byte TX address (nRF24L01 address format)
- Example format: `0xE7 0xE7 0xE7 0xE7 0xE7`
- Generated per pairing session and stored on both sides
- Used to lock the drone to the controller during normal operation

#### Frequency Channel Table (Default)
Default hop table used before pairing overwrites it:
`63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74`

### Processor and Connectivity
- MCU: ESP32-S3FN8
- Wireless: Wi-Fi, Bluetooth
- RF link: nRF24L01 (2.4 GHz)

### Onboard Sensors
- IMU: ICM45686
- Barometer: SPL06
- ToF: VL53L1X
- Optical flow: PMW3901

### DIY Support Specs (Recommended)
- Motor: 720 to 8050 class
- Battery: 1S LiPo, 25C or higher
- All up weight: about 70 g

### Reference Hardware (Current Build)
These values describe the current reference build and may vary with your frame or parts.
- Weight (including battery): about 65 g
- Motor: 720 class, about 52,000 rpm at 1S
- Propeller: 58 mm
- Battery: 1S LiPo (4.15 V to 3.3 V), 600 mAh, 25C
- Motor to motor distance: 120.208 mm

### Flight Controller (AF1000X FC)
- Board version: 01.10
- Size: 35 mm x 35 mm
- Mounting hole: 2 mm diameter, 1.5 mm from board edge

See `IMG/README.md` for board images and mechanical details.

### Recommended Controller
- AR1000X Remote Controller

### Quick Start (Binding)
1. Power on AF1000X and flip it upside down. LEDs 1 to 4 will light sequentially.
2. Power on AR1000X while holding GPIO15 (bind button). GPIO25 LED will blink.
3. When pairing completes, LEDs remain on.

### Repository Layout
- `AF1000X_Main.ino` - main entry
- `AF1000X_CORE.h` - core flight control logic
- `00a.md` - key features for v00a
- `IMG/` - board images and mechanical specs
- `TOOL/` - utilities (updater, IMU viewer)

---

# AF1000X
AF1000X는 교육 및 신속한 프로토타이핑을 위해 설계된 스마트 마이크로 드론이자 레퍼런스 플랫폼입니다. 센서 퓨전, 자동화, 안전 중심의 비행 로직을 결합하여, 사용자가 안정적인 비행 결과물을 얻으면서도 드론의 풀 스택 기술을 학습할 수 있도록 지원합니다.

### 주요 기능 (요약)
* **자동 이착륙:** 버튼 하나로 자동 이륙 및 착륙 지원
* **호버링 유지:** 고도 유지(Altitude Hold) 및 위치 유지(Position Hold) 지원
* **확장성:** 외부 마이크로 서보 및 LED 지원, FPV 카메라용 AUX 전원 출력
* **헤드리스 모드:** 기체 방향에 상관없이 조종자 기준으로 제어
* **안전 기능:** 비상 정지(Emergency Stop) 및 페일세이프(Failsafe) 차단
* **조종 모드:** 모드 1 및 모드 2 지원 (기본값: 모드 2)
* **DIY 자동 설정:** 호버링 학습 + 자동 튜닝 기능 제공

### 자동 설정 (Automatic Setup)
1. **호버 학습 (Hover learning):** 안정적인 호버링 상태를 분석하여 `hoverThrottle` 값을 자동으로 조정하고 저장합니다.
2. **자동 튜닝 (Auto tuning):** 호버링 준비가 완료되면 고도 PD 및 요(Yaw) P 제어 값을 자동으로 튜닝하여 저장합니다.

### 액세서리 I/O (서보 및 사용자 LED)
* **마이크로 서보:** PWM 출력 (GPIO36, 50Hz, 900~2000us 범위)
* **사용자 LED:** GPIO17 출력 (기본 설정: WS2812C NeoPixel)
* **WS2801C 지원:** 전원 패드가 제공되며, 제어를 위해서는 전용 SPI 빌드가 필요합니다.
* **AUX 전원:** FPV 카메라 전원 공급을 위한 전용 출력 포트입니다.

### 전원 가동 시퀀스 (Power On Sequence)
1. 전원 On 후 1초 대기.
2. 시리얼 배너(Serial Banner) 1회 출력.
3. **센서 POST 실행:** IMU, BARO(기압계), ToF(거리), Flow(광학 흐름) 점검.
4. **LED POST 표시:** 모든 LED가 켜진 후, 실패한 센서에 해당하는 LED가 0.5초 간격으로 점멸.
5. 오류 발생 시 해당 상태를 3초간 유지.
6. **IMU 또는 BARO 실패 시:** 시스템 **HARD LOCK** (비행 불가) 상태 진입 및 에러 LED 유지.
7. **ToF 실패 시:** 기압계(Barometer)만을 이용해 고도 유지 비행을 계속합니다.
8. **Flow 실패 시:** 위치 보정 기능이 비활성화됩니다.

### LED 작동 규칙 (런타임)
| 우선순위 | 상태 | LED 동작 패턴 |
| :--- | :--- | :--- |
| 1 (최고) | 부트/바인딩 | 초기화 및 연결 상태 표시 |
| 2 | 배터리 부족 | 모든 LED가 동시에 점멸 (1초 ON / 1초 OFF) |
| 3 | 자이로 리셋 | 모든 LED가 0.5초 ON / 1초 OFF 패턴으로 3회 반복 |
| 4 | 헤드리스 모드 | LED3, 4번 2초 ON / 1초 OFF (LED1, 2번은 OFF) |
| 5 | 자동 튜닝 | LED1, 2번과 LED3, 4번이 1.5초 간격으로 교차 점멸 |
| 6 (일반) | 정상 상태 | LED1~3: 센서 정상 표시 / LED4: Flow 활성화 시 ON |

### 하드웨어 사양
* **MCU:** ESP32-S3FN8 (Wi-Fi, Bluetooth 지원)
* **RF Link:** nRF24L01 (2.4 GHz, 주파수 호핑 FHSS 지원)
* **온보드 센서:**
  * IMU: ICM45686
  * Barometer: SPL06
  * ToF: VL53L1X
  * Optical Flow: PMW3901
* **권장 사양 (DIY):**
  * 모터: 720 ~ 8050 급
  * 배터리: 1S LiPo, 25C 이상
  * 기체 무게: 약 65g ~ 70g (배터리 포함)

### 빠른 시작 (바인딩)
1. AF1000X의 전원을 켜고 **기체를 뒤집습니다**. LED 1~4가 순차적으로 점등됩니다.
2. AR1000X 조종기의 GPIO15(바인드 버튼)를 누른 상태로 전원을 켭니다.
3. 페어링이 완료되면 LED가 모두 켜진 상태로 유지됩니다.

---

# AF1000X
AF1000Xは、教育および迅速なプロトタイピングのために設計されたスマートマイクロドローン兼リファレンスプラットフォームです。センサーフュージョン、自動化、安全重視の飛行ロジックを組み合わせることで、安定した飛行性能を維持しながら、ドローンのフルスタック技術を学ぶことができます。

### 主な機能
* **自動離着陸:** ワンボタンでの自動離陸および着陸。
* **ホバリング維持:** 高度維持（Altitude Hold）および位置維持（Position Hold）。
* **拡張性:** 外部マイクロサーボおよびLEDのサポート、FPVカメラ用AUX電源出力。
* **ヘッドレスモード:** 機体の向きに関係なく、操縦者視点での制御が可能。
* **安全機能:** 緊急停止（Emergency Stop）およびフェイルセーフ機能。
* **操作モード:** モード1およびモード2に対応（デフォルト：モード2）。
* **DIY自動セットアップ:** ホバリング学習 + オートチューニング機能。

### 自動セットアップ (Automatic Setup)
1. **ホバリング学習 (Hover learning):** 安定したホバリングに基づいて `hoverThrottle` 値を自動調整し、保存します。
2. **オートチューニング (Auto tuning):** ホバリング準備完了後、高度PDおよびヨー(Yaw) P制御のチューニングを行い、結果を保存します。

### アクセサリ I/O (サーボ & ユーザーLED)
* **マイクロサーボ:** PWM出力 (GPIO36, 50Hz, 900〜2000us)。
* **ユーザーLED:** GPIO17出力 (デフォルト: WS2812C NeoPixel)。
* **WS2801C対応:** 電源パッドを装備。制御には専用のSPIビルドが必要です。
* **AUX電源:** FPVカメラ電源用の専用出力ポート。

### 電源投入シーケンス (POST)
1. 電源投入後、1秒間待機。
2. シリアルバナーを1回表示。
3. **センサーPOST実行:** IMU、BARO（気圧計）、ToF（距離）、Flow（オプティカルフロー）を順次チェック。
4. **LED POST表示:** すべてのLEDが点灯し、エラーのあるセンサーに対応するLEDが0.5秒間隔で点滅。
5. エラー発生時はその状態を3秒間保持。
6. **IMU/BARO失敗時:** システムは **HARD LOCK**（飛行禁止）状態になり、エラーLEDが点灯し続けます。
7. **ToF失敗時:** 気圧計のみを使用して高度維持を継続します。
8. **Flow失敗時:** 位置補正機能が無効化されます。

### LED作動ルール (ランタイム)
| 優先順位 | 状態 | LEDパターン |
| :--- | :--- | :--- |
| 1 (最高) | 起動/バインド | 初期化および接続状態を表示 |
| 2 | 低バッテリー | すべてのLEDが同時に点滅 (1秒点灯 / 1秒消灯) |
| 3 | ジャイロリセット | すべてのLEDが0.5秒点灯 / 1秒消灯を3回繰り返す |
| 4 | ヘッドレスモード | LED3, 4が2秒点灯 / 1秒消灯 (LED1, 2は消灯) |
| 5 | オートチューニング | LED1, 2とLED3, 4が1.5秒ごとに交互に点滅 |
| 6 (通常) | 通常状態 | LED1〜3: センサー正常 / LED4: Flow有効時に点灯 |

### ハードウェア仕様
* **MCU:** ESP32-S3FN8 (Wi-Fi, Bluetooth対応)
* **RFリンク:** nRF24L01 (2.4 GHz, FHSS対応)
* **搭載センサー:**
  * IMU: ICM45686
  * 気圧計: SPL06
  * ToF: VL53L1X
  * オプティカルフロー: PMW3901
* **推奨スペック (DIY):**
  * モーター: 720 〜 8050 クラス
  * バッテリー: 1S LiPo, 25C 以上
  * 総重量: 約 65g 〜 70g (バッテリー含む)

### クイックスタート (バインド方法)
1. AF1000Xの電源を入れ、**機体を逆さまにします**。LED 1〜4が順番に点灯します。
2. AR1000X送信機のGPIO15（バインドボタン）を押しながら電源を入れます。
3. ペアリングが完了すると、LEDが常時点灯に変わります。
