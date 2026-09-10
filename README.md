# 2027 Intelligent Robotic Manipulation (RT758)

DGIST · Prof. Minho Hwang (SurgLab) · 2026 Fall
수업 실습 코드 정리 리포지토리 (TA)

---

## 주차별 실습

| 폴더 | 주차 | 주제 | 내용 |
|---|---|---|---|
| [`0904/`](0904/) | W2 (9/3) | ROS 2 basics / 환경 구축 | **Omni(master) → OMY-3M(slave) 1:1 조인트 매핑 텔레옵 (RViz)** |

> 각 폴더의 `README.md`에 실행 방법 · 구조 · 구현 메모가 정리되어 있음.

작업 이어받을 때는 [`ai/README.md`](ai/README.md)를 먼저 볼 것 — 리포 구조, 토픽/프레임 규약,
겪은 함정, 검증 방법, 다음 주차 할 일이 정리된 인수인계 문서다.
`ai/verify.sh`는 기구학이 tf와 일치하는지 실측 검증한다.

---

## 강의 일정

| 주차 | 날짜 | 주제 | |
|---|---|---|---|
| W1 | 8/27 | Course overview & introduction to robotic manipulation | |
| W2 | 9/3 | Environment setup, basic ROS2 / Isaac Sim workflow | |
| W3 | 9/10 | Robotics coding (coordinate transform, FK, Jacobian, IK, trajectory planning) | |
| W4 | 9/17 | Teleoperation I – master-slave control, motion mapping | Teleoperation |
| W5 | 9/24 | No class – 추석 | |
| W6 | 10/1 | Teleoperation II – motion scaling, clutching, camera-aware mapping | Teleoperation |
| W7 | 10/8 | Teleoperation demonstration | Teleoperation |
| W8 | 10/15 | Mid-term exam (no class) | |
| W9 | 10/22 | Camera calibration, eye-to-hand / hand-eye calibration | Automation |
| W10 | 10/29 | Pose estimation: detection, segmentation, 6D pose | Automation |
| W11 | 11/5 | No regular class – alternative assignment | |
| W12 | 11/12 | Visual servoing: eye-to-hand / eye-in-hand target approach | Automation |
| W13 | 11/19 | Visual servoing: imitation learning | Automation |
| W14 | 11/26 | Manipulation pipeline integration | |
| W15 | 12/3 | Term project presentation & DEMO | |
| W16 | 12/10 | Final exam (no class) | |

중간고사까지의 목표는 **teleoperation**: 입력 장치(master)의 자세를 로봇 매니퓰레이터(slave)로 옮기는 것.

---

## 실습 환경

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12 (numpy)
- NVIDIA Isaac Sim
- RViz2, `robot_state_publisher`, `joint_state_publisher_gui`

```bash
sudo apt install ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-joint-state-publisher-gui \
                 ros-$ROS_DISTRO-rviz2
pip install numpy
```

수업 코드는 `colcon build` 없이 **Python에서 ROS 2 모듈을 직접 사용하는 경량 구조**를 쓴다.
패키지는 `omni_sim`(master) / `omy_sim`(slave) / `omy_teleop`(연결) 세 개로 나눈다.

Ubuntu 24.04가 아닌 머신(예: 20.04 워크스테이션)에서는 [`docker/`](docker/)로 같은 Jazzy 환경을 띄울 수 있다.
```bash
./docker/build.sh                                 # 최초 1회
./docker/run.sh python3 omy_teleop/omyTeleop.py   # 실행 (X11 + NVIDIA GPU 패스스루)
```

---

## 로봇

| | 장치 | 문서 |
|---|---|---|
| Master | 3D Systems Geomagic Touch (Phantom Omni), 6-DOF | [Touch](https://www.3dsystems.com/haptics) |
| Slave | ROBOTIS OMY-3M, 6-DOF | [docs.robotis.com/docs/systems/omy](https://docs.robotis.com/docs/systems/omy/introduction/) |
