# Week 2 (09/04) — Master–Slave Joint-to-Joint Teleoperation in RViz

Omni(master) → OMY-3M(slave) **1:1 조인트 매핑 텔레오퍼레이션**을 ROS 2 + RViz로 구현한 것.
RViz 창 두 개(마스터 / 슬레이브)를 각각 띄우고, 마스터 조인트 값을 그대로 슬레이브 조인트로 보낸다.

| | 장치 | 모델 | 조인트 |
|---|---|---|---|
| Master | 3D Systems Geomagic Touch (Phantom Omni) | `omni_sim` | `m1` ~ `m6` (6-DOF) |
| Slave | ROBOTIS OMY-3M | `omy_sim` | `joint1` ~ `joint6` (6-DOF) |

---

## 1. 폴더 구조

수업 자료 *Week 2: ROS basics — "Software Structure for This Class"* 슬라이드 구성을 그대로 따랐다.
`colcon build` 없이 Python에서 ROS 2 모듈을 바로 쓰는 경량 구조다.

```
0904/
├── omni_sim/                       # 마스터 (Omni) 시뮬레이션 / 시각화
│   ├── description/                # 로봇 모델 & 기구학 기술
│   │   ├── urdf/omni.urdf          #   링크 / 조인트 정의 (m1 ~ m6)
│   │   ├── meshes/*.stl            #   비주얼 메쉬
│   │   └── __init__.py             #   load_urdf(): package:// → file:// 치환
│   ├── launch/                     # ROS 2 구성요소 실행 / 설정
│   │   ├── omni_rviz.launch.py     #   robot_state_publisher + JSP GUI + RViz
│   │   └── omni.rviz               #   RViz 설정 (fixed frame: omni/base)
│   ├── kinematics/                 # 기구학 유틸
│   │   ├── omniVar.py              #   조인트 이름 / 리밋 / 링크 길이 / DH 파라미터
│   │   └── omniKinematics.py       #   FK (modified DH)
│   ├── omniRviz.py                 # RViz 시각화 실행 (omni_rviz.launch.py 호출)
│   └── omniRvizMotion.py           # ROS 2 모션 래퍼 (조인트 read/write)
│
├── omy_sim/                        # 슬레이브 (OMY-3M) — omni_sim과 동일 구조
│   ├── description/urdf/omy_3m.urdf
│   ├── description/meshes/*.stl
│   ├── launch/omy_rviz.launch.py, omy.rviz
│   ├── kinematics/omyVar.py, omyKinematics.py     # FK + Jacobian
│   ├── omyRviz.py
│   └── omyRvizMotion.py
│
└── omy_teleop/                     # 마스터 ↔ 슬레이브 연결
    ├── omyTeleop.py                # 실행 진입점 (과제 실행 파일)
    ├── mapping/jointMapping.py     # 매핑 유틸 (JointMapper)
    └── teleop/jointTeleop.py       # 텔레옵 모듈 (JointTeleop 노드)
```

---

## 2. 실행

### 준비
```bash
# Ubuntu 24.04 + ROS 2 Jazzy 기준
sudo apt install ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-joint-state-publisher-gui \
                 ros-$ROS_DISTRO-rviz2
pip install numpy

source /opt/ros/$ROS_DISTRO/setup.bash
cd <repo>/0904
```

Ubuntu 24.04가 아닌 머신이면 [`docker/`](../docker/) 컨테이너로 동일 환경을 쓸 수 있다.
```bash
./docker/build.sh                                 # 최초 1회 (irm:jazzy 이미지)
./docker/run.sh python3 omy_teleop/omyTeleop.py   # 과제 실행
./docker/run.sh                                   # 셸 진입 (자동으로 /workspace/0904)
```

### 전체 실행 (과제 데모)
```bash
python3 omy_teleop/omyTeleop.py
```
RViz 두 창(마스터 / 슬레이브)과 조인트 슬라이더 창이 뜬다.
**Joint State Publisher 창의 슬라이더를 움직이면 OMY가 같은 조인트 값으로 따라간다.**

옵션:
```bash
python3 omy_teleop/omyTeleop.py --rate 200        # 루프 주기 200 Hz (기본 100)
python3 omy_teleop/omyTeleop.py --no-launch       # RViz가 이미 떠 있을 때 텔레옵만
python3 omy_teleop/omyTeleop.py --log-period 0    # 콘솔 출력 끄기
```

### 개별 실행 (디버깅용)
```bash
python3 omni_sim/omniRviz.py            # 마스터만 (슬라이더 포함)
python3 omy_sim/omyRviz.py --gui        # 슬레이브만 (슬라이더 포함)
python3 omni_sim/omniRvizMotion.py      # 마스터 조인트 / EE 위치 출력
python3 omy_sim/omyRvizMotion.py        # 슬레이브 joint1 sine 구동 테스트
python3 omy_teleop/mapping/jointMapping.py   # 매핑 표 출력
```

---

## 3. ROS 2 그래프

```
 ┌─────────────────────────┐                       ┌─────────────────────────┐
 │ joint_state_publisher_  │                       │        RViz (master)    │
 │        gui (slider)     │                       │  /omni/robot_description│
 └───────────┬─────────────┘                       └───────────▲─────────────┘
             │ /omni/joint_states                              │ /tf, /tf_static
             ▼                                                 │
 ┌─────────────────────────┐   robot_state_publisher (frame_prefix "omni/")
 │   OmniRvizMotion (sub)  │◄──────────────────────────────────┘
 └───────────┬─────────────┘
             │ q_master (m1..m6)
             ▼
 ┌─────────────────────────┐
 │  JointMapper (1:1)      │   q_slave = sign * scale * (q_master - off_m) + off_s
 └───────────┬─────────────┘
             │ q_slave (joint1..joint6)
             ▼
 ┌─────────────────────────┐   robot_state_publisher (frame_prefix "omy/")
 │   OmyRvizMotion (pub)   │──────────────────────────────────┐
 └───────────┬─────────────┘                                  │ /tf, /tf_static
             │ /omy/joint_states                              ▼
             └────────────────────────────────────►┌─────────────────────────┐
                                                   │        RViz (slave)     │
                                                   │  /omy/robot_description │
                                                   └─────────────────────────┘
```

| 항목 | 마스터 | 슬레이브 |
|---|---|---|
| 조인트 토픽 | `/omni/joint_states` | `/omy/joint_states` |
| URDF 토픽 | `/omni/robot_description` | `/omy/robot_description` |
| tf 프리픽스 | `omni/` | `omy/` |
| RViz fixed frame | `omni/base` | `omy/world` |
| 조인트 이름 | `m1` … `m6` | `joint1` … `joint6` |

두 로봇이 **같은 `/tf` 트리**를 쓰기 때문에 프레임 이름이 겹치면 안 된다.
그래서 `robot_state_publisher`의 `frame_prefix` 파라미터로 각각 `omni/`, `omy/`를 붙였다.

---

## 4. 매핑 내용

`omy_teleop/mapping/jointMapping.py`

```
q_slave[i] = clip( sign[i] * scale[i] * (q_master[i] - master_offset[i]) + slave_offset[i] )
```

이번 주 기본값은 `sign = +1`, `scale = 1`, `offset = 0` → **`q_slave = q_master`**, 즉 조인트 값 그대로 전달.
`m1→joint1`, `m2→joint2`, …, `m6→joint6` 순서로 1:1 대응된다.
결과는 OMY의 조인트 리밋(±2π)으로 clip 된다.

`JointState` 메시지는 이름(`name`) 기준으로 재정렬해서 읽는다.
`joint_state_publisher_gui`가 조인트 순서를 보장하지 않기 때문에 인덱스로 읽으면 안 된다.

> **주의 — 이건 조인트 공간 매핑이지 작업 공간 매핑이 아니다.**
> 두 팔은 링크 길이도 조인트 축 배치도 다르므로 엔드이펙터 자세는 일치하지 않는다.
> 슬라이드의 목표식 `T(master_base→master_ee) = T(slave_base→slave_ee)` 를 만족시키는 것은
> W4(Teleoperation I – motion mapping)에서 FK/IK를 붙여서 다룬다.
> 그래서 `scale` / `sign` / `offset` 자리를 미리 만들어 두었다 (지금은 항등).

---

## 5. 구현 메모

- **`colcon build` 없음.** URDF는 `description/__init__.py`의 `load_urdf()`가 읽어서
  `package://omni_sim/...` → 절대경로 `file://...` 로 치환한 뒤
  `robot_description` 파라미터에 **문자열**로 넘긴다.
  ament index가 없어도 RViz가 메쉬를 찾을 수 있다.
- **런치 파일이 python 패키지를 import** 할 수 있도록, 각 launch 파일이
  `sys.path`에 `0904/` 를 넣는다. 어느 위치에서 실행해도 동작한다.
- **슬레이브 슬라이더는 기본 off.** `omy_rviz.launch.py`의 `gui` 기본값이 `false`다.
  텔레옵 명령과 슬라이더가 같은 토픽을 두고 싸우기 때문.
- **마스터가 아직 없으면** 텔레옵 노드가 슬레이브를 HOME(전부 0, 팔을 위로 세운 자세)으로
  잡아 둔다. `/omy/joint_states`가 하나도 안 오면 RViz에 모델이 안 그려지기 때문.
- 실기(실제 Geomagic Touch)를 연결할 때는 Touch 드라이버가 `/omni/joint_states`를
  퍼블리시하도록만 바꾸면 되고, 나머지 코드는 그대로 쓸 수 있다.
  (`omni_rviz.launch.py`를 `gui:=false`로 실행)

### 검증한 것 (ROS 2 Jazzy 컨테이너에서 실제 실행)
- 런치: `/omni/joint_states`, `/omni/robot_description`, `/omy/joint_states`,
  `/omy/robot_description` 토픽 생성 확인, `frame_prefix`가 `omni/` / `omy/`로 적용됨
- 텔레옵 파이프라인: 마스터 → `/omy/joint_states`로 조인트 값 그대로 전달, 100 Hz 유지
  (슬라이더 GUI는 10 Hz로 퍼블리시)
- **FK vs tf**: 랜덤 8개 자세에서 `OmniKinematics.fk_pose` ↔ `omni/base→omni/stylus`,
  `OmyKinematics.fk_pose` ↔ `omy/world→omy/end_effector_link` 최대 오차 **3.3e-16 m**
- 해석적 Jacobian vs FK 수치미분: 두 로봇 모두 최대 오차 1e-7 이하
- GUI 실행: RViz 2개 + 슬라이더 창 정상 렌더링, 메쉬 로딩 에러 없음 (NVIDIA GL)
- 매핑: 마스터가 조인트 순서를 섞어 보내도 `joint1..joint6`에 올바르게 전달됨

### Omni 기구학 규약 주의
`OmniKinematics.fk()`는 **URDF 프레임 기준**이라 RViz/tf와 일치한다.
연구실 ROS 1 코드에서 쓰던 modified-DH 테이블(`omniVar.dhparam`, `OmniKinematics.fk_dh`)은
Geomagic 장치 자체의 프레임 규약이라 URDF의 `base` / `stylus` 프레임과 일치하지 않는다.
(같은 자세에서 DH는 `[-0.156, 0.016, 0.063]`, URDF/tf는 `[-0.017, -0.165, 0.066]`)
tf와 맞춰야 하는 계산에는 `fk()`를, 기존 코드와 비교할 때만 `fk_dh()`를 쓸 것.

---

## 6. 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| RViz에 로봇이 안 보이고 `No transform from [...]` | `/omni/joint_states` 또는 `/omy/joint_states`가 안 오는 상태. `ros2 topic hz /omni/joint_states` 확인 |
| 로봇은 보이는데 메쉬가 하얀 박스 / 안 보임 | 리포지토리 경로에 공백·한글이 있으면 확인. `load_urdf()`가 percent-encoding 하지만 경로는 ASCII 권장 |
| 슬라이더를 움직여도 OMY가 안 움직임 | 텔레옵 노드가 떠 있는지 확인 (`ros2 node list`에 `/omy_joint_teleop`) |
| `ModuleNotFoundError: omni_sim` | `0904/` 디렉토리에서 실행하거나, `omyTeleop.py`처럼 진입점 스크립트로 실행 |
| `ModuleNotFoundError: rclpy` | ROS 2가 없는 환경. `source /opt/ros/$ROS_DISTRO/setup.bash` 하거나 [`docker/`](../docker/) 컨테이너 사용 |
| `ros2 topic pub`으로 테스트하면 로봇이 안 움직임 | **`ros2 topic pub`은 `header.stamp=0`을 보내고 robot_state_publisher가 그런 메시지를 버린다.** 테스트용 퍼블리셔에서도 `header.stamp`를 채울 것 (`node.get_clock().now().to_msg()`) |
| RViz 창이 겹쳐서 뜸 | `omni.rviz` / `omy.rviz`의 `Window Geometry`의 `X`, `Y` 값 조정 |

---

## 7. 출처 / 라이선스

| 파일 | 출처 | 라이선스 |
|---|---|---|
| `omni_sim/description/` | [fsuarez6/phantom_omni](https://github.com/fsuarez6/phantom_omni) `omni_description` | BSD |
| `omy_sim/description/` | [ROBOTIS-GIT/open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) (`jazzy`) `open_manipulator_description` | Apache-2.0 |

두 URDF 모두 메쉬 URI 접두사만 이 폴더 구조에 맞게 수정했고, 기구학 값은 원본 그대로다.
