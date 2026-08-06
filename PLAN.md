# HopeJR Desk 개발 계획 (FR3 Desk Inspired Research Interface)

## 1. 프로젝트 목표

HopeJR을 단순한 원격 제어 로봇이 아니라 **연구용 Robot Operating Interface (HopeJR Desk)** 로 개발한다.

전체적인 UI와 사용 흐름은 Franka Research 3의 Desk를 참고하되, HopeJR의 하드웨어 특성(Feetech Servo, Modular Robot, 저가형 연구 플랫폼)에 맞게 기능을 재설계한다.

핵심 목표는 다음과 같다.

- Robot 상태 모니터링
- Motion 및 Task 관리
- Teaching Interface
- Hand Control
- Robot Configuration
- Diagnostics
- Dataset Recording
- 연구자가 코드를 수정하지 않고 대부분의 실험을 수행할 수 있는 GUI 제공

---

# 2. 전체 시스템 구조

```
                     HopeJR Desk (Web Interface)

 ┌────────────────────────────────────────────────────┐
 │ Dashboard                                           │
 │ Robot │ Teaching │ Tasks │ Hand │ Config │ Logs     │
 └────────────────────────────────────────────────────┘
                         │
                 Robot Manager Server
                         │
      ┌───────────────┬───────────────┐
      │               │               │
 Motion Controller  Hand Controller  Diagnostics
      │               │               │
      └───────────────┴───────────────┘
                         │
                  Feetech Servo Bus
                         │
                     HopeJR Robot
```

---

# 3. Dashboard

Robot의 현재 상태를 실시간으로 확인한다.

표시 정보

- Robot Connected
- Servo ON/OFF
- Emergency Stop 상태
- 현재 Task
- Joint Position
- Joint Velocity
- Servo Current
- Battery (사용 시)
- Motor Temperature
- Communication Status

---

# 4. Robot Control

기본적인 Robot 제어 기능 제공

기능

- Servo Enable
- Servo Disable
- Home Position 이동
- Joint Reset
- Emergency Stop
- Recover
- Current Monitoring
- Servo Reboot

추가 기능

- Servo Scan
- Servo ID 변경
- Servo Firmware 확인

---

# 5. Motion Teaching

## 목표

Franka Desk의 Hand Guiding과 유사한 사용자 경험을 제공하되,
Feetech Servo 특성을 고려하여 Position Servo 기반 Teaching을 구현한다.

---

## Teaching Mode

Teaching 시작

↓

Servo Stiffness 감소

↓

Current 및 Position Error 기반 외력 추정

↓

Admittance Controller

↓

새로운 Position 생성

↓

Servo Position Command

↓

Trajectory Recording

↓

Save

---

## Force Estimation

FT Sensor 대신

다음 정보를 이용한다.

- Motor Current
- Position Error
- Joint Velocity

이를 이용하여 사람이 로봇을 움직이는 힘을 추정한다.

---

## Teaching 기능

- Start Teaching
- Pause
- Resume
- Stop
- Record
- Replay
- Save Motion
- Load Motion
- Motion Edit
- Motion Delete

---

# 6. Task Editor

Franka Desk와 유사한 Drag & Drop 기반 Task 구성

예시

Move

↓

Move

↓

Grasp

↓

Move

↓

Release

↓

Wait

↓

Home

지원 Task

- Move Joint
- Move Cartesian
- Waypoint
- Wait
- Delay
- Grasp
- Release
- Hand Pose
- Loop
- Condition
- User Confirmation

향후

- Camera Trigger
- AI Inference
- Vision Detection

등도 추가 가능하도록 설계한다.

---

# 7. Hand Control

HopeJR Hand는 손가락별 독립 제어를 제공한다.

Finger Mapping

1 = Thumb

2 = Index

3 = Middle

4 = Ring

5 = Pinky

사용 예

15

↓

Thumb + Pinky Grip

123

↓

Thumb + Index + Middle Grip

---

## Finger State Machine

각 손가락은 독립적으로 동작한다.

Idle

↓

Closing

↓

Contact Detection

↓

Holding

↓

Release

---

## Contact Detection

Grip Force Sensor 대신

Servo Current Threshold를 이용한다.

Closing

↓

Current 증가

↓

Threshold 초과

↓

Stop

↓

Holding

사용자가 Threshold를 조절할 수 있도록 한다.

---

# 8. Robot Configuration

HopeJR은 다양한 센서 및 장치를 장착할 수 있으므로
Robot Configuration 기능을 제공한다.

설정 항목

- Joint Limit
- Velocity Limit
- Acceleration Limit
- Current Limit
- Servo Direction
- Home Offset
- Tool Offset (TCP)
- Payload 설정

---

## Link Parameter Management

연구 과정에서 카메라, 센서, Gripper, 추가 장비 등을 장착하면
각 링크의 질량과 무게중심(CoM), 관성(Inertia)이 변경된다.

이를 위해 GUI에서 다음 항목을 수정할 수 있도록 한다.

- Link Mass
- Center of Mass
- Inertia Tensor
- Payload 위치
- End-effector 질량

변경된 값은

- Dynamics 계산
- Gravity Compensation
- Admittance Control
- Simulation
- Motion Planning

등에서 공통으로 사용한다.

향후 URDF 자동 생성 또는 수정 기능과도 연동 가능하도록 설계한다.

---

# 9. Diagnostics

Robot 상태를 실시간으로 확인한다.

표시 항목

- Joint Current
- Motor Temperature
- Voltage
- Communication Delay
- Packet Loss
- Servo Error
- Joint Error
- Current History
- Position Error

로그 저장 기능 제공

---

# 10. Dataset Recording

HopeJR Desk의 핵심 기능 중 하나로
모든 실험 데이터를 자동 기록한다.

기록 대상

- Joint Position
- Joint Velocity
- Joint Current
- Command Position
- Command Velocity
- Hand State
- Task 정보
- Camera Image (향후)
- IMU (향후)

저장 형식

- ROS Bag
- HDF5
- CSV
- JSON Metadata

향후 VLA 및 Imitation Learning Dataset으로 바로 활용할 수 있도록 한다.

---

# 11. Logs

실험 기록 관리

- Error Log
- Motion Log
- Servo Log
- Teaching Log
- Dataset Log

다운로드 기능 제공

---

# 12. Settings

설정 기능

- Network
- Robot Name
- Servo Configuration
- Camera Configuration
- Theme
- User Management

---

# 13. 향후 확장 기능

Vision Module

- Camera Calibration
- Object Detection
- Pose Estimation

Simulation

- Isaac Sim 연동
- MuJoCo 연동
- Digital Twin

AI Module

- VLM
- VLA
- LLM Interface

Cloud

- Dataset Sync
- Motion Sharing
- Remote Monitoring

---

# 14. 개발 방향

HopeJR Desk는 Franka Desk를 단순히 모방하는 것이 아니라,
저가형 연구 플랫폼에서도 동일한 수준의 사용성을 제공하면서 연구 확장성을 극대화하는 것을 목표로 한다.

특히 다음 네 가지를 핵심 가치로 삼는다.

1. **Ease of Use**
   - GUI만으로 대부분의 실험 수행 가능

2. **Research Friendly**
   - Motion Teaching, Dataset Recording, Parameter Editing 등 연구에 필요한 기능을 기본 제공

3. **Hardware Adaptability**
   - 센서, 카메라, 그리퍼, 링크 질량 등 하드웨어 변경 사항을 GUI에서 손쉽게 반영 가능

4. **Scalability**
   - 향후 Vision, Simulation, AI, Digital Twin과 자연스럽게 연동 가능한 구조로 설계