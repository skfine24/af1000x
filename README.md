
# AF1000X – Educational Micro Drone Firmware  
AF1000X – 교육용 마이크로 드론 펌웨어

---

## 🇰🇷 한국어 소개

AF1000X는 **교육·연구 목적의 마이크로 드론**을 위해 설계된 오픈소스 펌웨어 프로젝트입니다.  
회로(PCB) 설계부터 펌웨어, 제어 로직까지 **드론 시스템 전체를 이해하고 학습할 수 있도록** 구성되었습니다.

본 프로젝트는 단순한 완구 드론이 아닌,  
👉 **“구조와 동작 원리를 이해할 수 있는 드론”**을 목표로 합니다.

현재 코드는 **실제 비행 가능한 단계**까지 구현되어 있으며,  
PCB 수령 후 실기 테스트 및 튜닝을 예정하고 있습니다.

---

### ✨ 주요 기능 (한국어)

#### 🧠 비행 제어
- IMU 기반 자세 안정화
- 고도 유지 / 호버링
- 자동 이륙 / 자동 착륙
- 헤드리스(Headless) 모드

#### 📡 무선 통신
- nRF24L01 기반 2.4GHz 통신
- 페어링 / 바인딩 구조 분리
- 재부팅 후에도 안정적인 재연결

#### 💡 LED 상태 표시 (교육 친화적)
- **부팅**
  - 정상 부팅: 모든 LED 깜박임
  - 뒤집힌 상태 부팅: LED1→2→3→4 체이서
- **바인딩 완료**: 모든 LED ON (1초)
- **저전압 경고**: 모든 LED 1초 ON / 1초 OFF
- **자이로 초기화**: 모든 LED 1초 ON / 0.5초 OFF ×3
- **헤드리스 모드**: LED3 / LED4 2초 ON / 1초 OFF

👉 LED만 보고도 드론의 상태를 직관적으로 확인할 수 있습니다.

#### 🎮 조종기 연동
- MODE 2 기본 / MODE 1 선택 가능 (전원 ON 시 버튼)
- 속도 단계 전환
- 자동 이륙 / 착륙
- 헤드리스 토글
- 수평 트림 (비영구, 전원 OFF 시 초기화)

---

### 📁 프로젝트 구조

```
AF1000X/
├── AF1000X_Main.ino
├── AF1000X_CORE.h
├── AF1000X_BINDING.h
├── AF1000X_Hover.h
├── AF1000X_EasyCommander.h
└── README.md
```

---

### 🎯 프로젝트 목표
- 교육용 / 학습용 드론 플랫폼
- 회로 + 펌웨어 + 제어 이론 통합 학습
- 대회 및 연구용 베이스 플랫폼 제공
- 확장과 커스터마이징이 쉬운 구조

---

### 🧪 개발 상태
- 컴파일 에러 없음
- LED 상태 머신 통합 완료
- 바인딩 및 재부팅 시퀀스 안정화
- PCB 실물 테스트 예정

⚠️ 하드웨어 리비전에 따라 전압 계수, IMU 방향 보정이 필요할 수 있습니다.

---

## 🇺🇸 English Description

AF1000X is an **open-source firmware project for an educational micro drone**.  
It is designed to help learners understand the **entire drone system**, from PCB hardware to firmware and flight control logic.

Rather than being a toy drone, AF1000X aims to be:  
👉 **“A drone you can truly understand.”**

The firmware has reached a **real flight-capable stage**,  
and hardware testing will begin once the custom PCB is available.

---

### ✨ Key Features (English)

#### 🧠 Flight Control
- IMU-based attitude stabilization
- Altitude hold / hovering
- Auto takeoff & auto landing
- Headless flight mode

#### 📡 Wireless Communication
- 2.4GHz nRF24L01 radio link
- Separated pairing / binding logic
- Stable reconnection after reboot

#### 💡 LED State Indication
- **Boot**
  - Normal boot: all LEDs blinking
  - Inverted boot: LED1→2→3→4 chasing pattern
- **Binding complete**: all LEDs ON for 1 second
- **Low battery warning**: all LEDs 1s ON / 1s OFF
- **Gyro initialization**: all LEDs 1s ON / 0.5s OFF ×3
- **Headless mode**: LED3 / LED4 2s ON / 1s OFF

LED patterns are designed to provide **clear, intuitive feedback** without any tools.

#### 🎮 Transmitter Support
- MODE 2 by default / MODE 1 selectable at power-on
- Speed level switching
- Auto takeoff / landing trigger
- Headless mode toggle
- Trim adjustment (RAM only, resets on power cycle)

---

### 🎯 Project Goals
- Educational and research-oriented drone platform
- Integrated learning of hardware, firmware, and control theory
- Suitable for students, makers, and competitions
- Highly customizable and extendable codebase

---

### 📜 License
This project is primarily intended for **educational and research use**.  
The license will be defined in a future update.

---

### 🙌 Contribution
Ideas, improvements, and test feedback are always welcome.

---

**AF1000X – Build, Learn, Fly.**
