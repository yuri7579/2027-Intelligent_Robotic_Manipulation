# AI 작업 인수인계 (Handoff Context)

> **이 문서의 용도**: 다른 AI 어시스턴트(ChatGPT/Claude 등)에게 이 리포지토리의 다음 작업을
> 시킬 때, 배경 설명을 처음부터 다시 하지 않기 위한 컨텍스트 문서.
> **이 파일 전체를 그대로 붙여넣고 "다음 작업"만 말하면 된다.**
>
> 최종 갱신: 2026-09-10 / 대상 커밋: `57d2c9c`

---

## 0. 한 줄 요약

DGIST **RT758 Intelligent Robotic Manipulation (2026 Fall, 황민호 교수)** 조교(TA)가
수업 실습 코드를 주차별로 정리하는 리포지토리. 현재 **W2(0904)** 까지 완료 —
Omni(master) → OMY-3M(slave) **1:1 조인트 매핑 텔레오퍼레이션**을 ROS 2 + RViz로 구현하고
Jazzy 환경에서 실측 검증까지 끝냈다.

- GitHub: `yuri7579/2027-Intelligent_Robotic_Manipulation`
- 로컬: `/home/surglab/jonghyun/기타/지능형로봇조작/2027-Intelligent_Robotic_Manipulation`

---

## 1. 수업 맥락

| 주차 | 날짜 | 주제 | 상태 |
|---|---|---|---|
| W1 | 8/27 | Course overview | - |
| W2 | 9/3 | 환경 구축, ROS 2 / Isaac Sim 기본 | **완료 → `0904/`** |
| W3 | 9/10 | Robotics coding (coordinate transform, FK, Jacobian, IK, trajectory planning) | **다음 작업** |
| W4 | 9/17 | Teleoperation I – master-slave control, **motion mapping** | 예정 |
| W5 | 9/24 | 추석 휴강 | - |
| W6 | 10/1 | Teleoperation II – motion scaling, clutching, camera-aware mapping | 예정 |
| W7 | 10/8 | Teleoperation demonstration | 예정 |
| W9~ | 10/22~ | 카메라 캘리브레이션, pose estimation, visual servoing, imitation learning | 예정 |

**중간고사까지의 목표**: teleoperation.
슬라이드의 핵심 목표식은 두 엔드이펙터 자세를 일치시키는 것:

```
 master_base T master_ee   =   slave_base T slave_ee
```

W2는 여기까지 못 갔고 **조인트 공간 1:1 매핑**만 했다. 작업 공간 매핑(FK/IK 사용)이 W4 과제다.

### 하드웨어

| | 장치 | 조인트 | 비고 |
|---|---|---|---|
| Master | 3D Systems Geomagic Touch (Phantom Omni), 6-DOF | `m1` ~ `m6` | 현재는 슬라이더 GUI로 대체, 실기 미연결 |
| Slave | ROBOTIS OMY-3M, 6-DOF | `joint1` ~ `joint6` | RViz 시뮬레이션만, 실기 미연결 |

---

## 2. 리포지토리 구조

```
2027-Intelligent_Robotic_Manipulation/
├── README.md            강의 일정 + 주차별 폴더 인덱스
├── .gitignore
├── ai/                  ← 이 문서 (AI 인수인계용)
│   ├── README.md
│   └── verify.sh        FK vs tf 실측 검증 스크립트 (컨테이너 안에서 실행)
├── docker/              Ubuntu 24.04가 아닌 머신용 Jazzy 컨테이너
│   ├── Dockerfile       osrf/ros:jazzy-desktop + joint_state_publisher_gui
│   ├── build.sh         → irm:jazzy 이미지 빌드
│   └── run.sh           X11 + NVIDIA GPU 패스스루로 실행
└── 0904/                W2 실습 (주차별 폴더는 MMDD 형식)
    ├── README.md        실행법 / ROS 2 그래프 / 매핑 설명 / 트러블슈팅
    ├── omni_sim/        마스터 패키지
    ├── omy_sim/         슬레이브 패키지 (omni_sim과 동일 구조)
    └── omy_teleop/      마스터-슬레이브 연결
```

### 패키지 구조 — 수업 슬라이드가 지정한 형식 (반드시 유지할 것)

교수님 슬라이드 *"Software Structure for This Class"* 가 아래 구조를 명시했다.
새 주차 폴더를 만들 때도 이 구조를 따른다.

```
omni_sim/                        # 마스터
├── description/                 # 로봇 모델 (URDF, meshes, joint/link 정보)
│   ├── urdf/omni.urdf
│   ├── meshes/*.stl
│   └── __init__.py              #   load_urdf(): package:// → file:// 치환
├── launch/                      # ROS 2 구성요소 실행/설정
│   ├── omni_rviz.launch.py      #   robot_state_publisher + JSP GUI + RViz
│   └── omni.rviz
├── kinematics/                  # FK/IK, joint↔pose 변환
│   ├── omniVar.py               #   조인트 이름/리밋/URDF 기하 상수/DH
│   └── omniKinematics.py        #   fk(), fk_pose(), jacobian(), fk_dh()
├── omniRviz.py                  # RViz 시각화 실행 (omni_rviz.launch.py 호출)
└── omniRvizMotion.py            # ROS 2 모션 래퍼 (조인트 read/write)

omy_sim/                         # 슬레이브 — 동일 구조
├── description/urdf/omy_3m.urdf, meshes/, __init__.py
├── launch/omy_rviz.launch.py, omy.rviz
├── kinematics/omyVar.py, omyKinematics.py
├── omyRviz.py
└── omyRvizMotion.py

omy_teleop/                      # 연결
├── omyTeleop.py                 # 실행 진입점
├── mapping/jointMapping.py      # JointMapper
└── teleop/jointTeleop.py        # JointTeleop 노드 + spin_teleop()
```

---

## 3. 현재 구현 상태 (W2 / `0904/`)

### 실행

```bash
cd 0904
python3 omy_teleop/omyTeleop.py          # RViz 2개 + 슬라이더 창, 100 Hz 텔레옵
python3 omy_teleop/omyTeleop.py --rate 200 --no-launch --log-period 0
python3 omni_sim/omniRviz.py             # 마스터만
python3 omy_sim/omyRviz.py --gui         # 슬레이브만 (슬라이더 포함)
```

`omyTeleop.py`가 하는 일: ① 마스터 RViz 실행 → ② 슬레이브 RViz 실행 →
③ 모션 래퍼 import → ④ 마스터 조인트 읽기 → ⑤ 매핑 → ⑥ 슬레이브로 전송 → ⑦ 실시간 시각화.

### 네이밍/토픽 규약 (다음 주차에서도 그대로 유지)

| 항목 | 마스터 | 슬레이브 |
|---|---|---|
| 네임스페이스 | `/omni` | `/omy` |
| 조인트 토픽 | `/omni/joint_states` | `/omy/joint_states` |
| URDF 토픽 | `/omni/robot_description` | `/omy/robot_description` |
| tf 프리픽스 | `omni/` | `omy/` |
| RViz fixed frame | `omni/base` | `omy/world` |
| EE 프레임 | `omni/stylus` | `omy/end_effector_link` |
| 조인트 이름 | `m1` … `m6` | `joint1` … `joint6` |

두 로봇이 **같은 전역 `/tf` 트리**를 공유한다.
프레임 충돌을 막으려고 `robot_state_publisher`의 `frame_prefix` 파라미터를 쓴다.
(네임스페이스로 tf를 분리하지 **않았다** — RViz 두 개가 각각 전역 /tf를 보면 되기 때문)

### 매핑 (W2 범위)

`omy_teleop/mapping/jointMapping.py`의 `JointMapper`:

```
q_slave[i] = clip( sign[i] * scale[i] * (q_master[i] - master_offset[i]) + slave_offset[i] )
```

W2 기본값은 `sign=+1, scale=1, offset=0` → **`q_slave = q_master`** (조인트 값 그대로).
`m1→joint1` … `m6→joint6`. 결과는 OMY 리밋(±2π)으로 clip.
**scale/sign/offset 자리는 W4(motion scaling)를 위해 미리 만들어 둔 것**이니 W4에서는
코드 구조를 바꾸지 말고 숫자만 채우거나 서브클래스를 추가하는 방향으로 갈 것.

### 기구학

두 패키지 모두 **URDF의 joint origin/rpy/axis에서 직접** FK를 계산한다 (하드코딩 DH 아님).
따라서 결과가 robot_state_publisher가 내는 tf와 항상 일치한다.

- `OmniKinematics.fk()` / `.fk_pose()` / `.jacobian()` — URDF 프레임 기준
- `OmyKinematics.fk()` / `.fk_pose()` / `.jacobian()` — URDF 프레임 기준
- `OmniKinematics.fk_dh()` + `omniVar.dhparam()` — 연구실 ROS 1 코드의 modified-DH.
  **Geomagic 장치 프레임 규약이라 URDF의 `base`/`stylus`와 일치하지 않음.**
  같은 자세에서 DH는 `[-0.156, 0.016, 0.063]`, URDF/tf는 `[-0.017, -0.165, 0.066]`.
  tf와 맞춰야 하면 `fk()`를 쓸 것. (이건 W2 작업 중 실제로 발견해서 고친 버그)

**IK는 아직 없다. W3에서 추가할 부분.**

---

## 4. 개발 환경

### 이 워크스테이션 (중요)

**Ubuntu 20.04 + ROS Noetic(ROS 1)이라 ROS 2가 네이티브로 없다.**
`python3 omy_teleop/omyTeleop.py`를 그냥 실행하면 `ModuleNotFoundError: No module named 'rclpy'`.

→ **반드시 Docker 컨테이너 안에서 실행할 것.**

```bash
./docker/build.sh                                 # 최초 1회 (irm:jazzy, ~4GB)
./docker/run.sh python3 omy_teleop/omyTeleop.py   # 과제 실행
./docker/run.sh                                   # 셸 진입 (자동으로 /workspace/0904)
```

- 리포지토리가 컨테이너의 `/workspace`에 마운트된다 (호스트 uid/gid 그대로 → 파일 권한 유지)
- X11(`DISPLAY=:1`) + NVIDIA GPU(RTX 4060 Ti) 패스스루 확인 완료, RViz 정상 렌더링
- 수업 공식 환경은 **Ubuntu 24.04 + ROS 2 Jazzy** 네이티브. 학생들에게 배포하는 코드는
  네이티브 기준으로 작성하고, docker/는 조교 편의용 보조 수단이다.

### 의존성

ROS 2 Jazzy + `robot_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `numpy`.
Python 코드는 `colcon build` 없이 **rclpy를 직접 import**하는 경량 구조 (교수님 지시).

---

## 5. 반드시 알아야 할 함정들 (실제로 겪은 것들)

1. **`ros2 topic pub`으로 조인트를 쏘면 로봇이 안 움직인다.**
   `ros2 topic pub`은 `header.stamp = 0`을 보내고 `robot_state_publisher`가 그런
   메시지를 **버린다**. 테스트용 퍼블리셔에서도 반드시
   `msg.header.stamp = node.get_clock().now().to_msg()`를 채울 것.
   (증상: `/tf`가 아예 안 나오거나 "Tf has two or more unconnected trees")

2. **`package://` 는 colcon build 없이는 RViz가 못 찾는다.**
   ament index가 없기 때문. 각 패키지의 `description/__init__.py`에 있는
   `load_urdf()`가 `package://omni_sim/` → 절대 `file://` 경로로 치환하고,
   `urllib.parse.quote()`로 percent-encoding까지 한다 (경로에 한글/공백이 있어서).
   URDF를 새로 추가하면 같은 처리를 해줄 것.

3. **`JointState`는 인덱스가 아니라 이름(`name`)으로 읽어야 한다.**
   `joint_state_publisher_gui`가 조인트 순서를 보장하지 않는다.
   모션 래퍼의 `_reorder()`가 이 처리를 한다.

4. **`joint_state_publisher_gui`는 `ros-jazzy-desktop`에 포함돼 있지 않다.**
   따로 `apt install ros-jazzy-joint-state-publisher-gui` 필요 (Dockerfile에 반영됨).

5. **RViz는 헤드리스(`QT_QPA_PLATFORM=offscreen`)로 검증할 수 없다.**
   Ogre가 GLX display를 못 열어서 SIGABRT로 죽는다. 실제 X 디스플레이가 필요하다.
   → RViz 설정 검증은 GUI를 실제로 띄워서 할 것.

6. **Ubuntu 24.04 베이스 이미지에는 uid 1000짜리 `ubuntu` 유저가 이미 있다.**
   호스트 uid를 맞추려면 먼저 `userdel`해야 한다 (Dockerfile에 반영됨).

7. **슬레이브 슬라이더 GUI는 기본 off** (`omy_rviz.launch.py`의 `gui` 기본값 `false`).
   텔레옵 명령과 슬라이더가 같은 토픽을 두고 싸우기 때문.

8. **런치 파일이 python 패키지를 import** 해야 해서, 각 launch 파일이 `sys.path`에
   `0904/`를 넣는다. 새 launch 파일도 같은 처리를 할 것.

---

## 6. 검증 방법 (재현 가능)

```bash
./docker/run.sh bash /workspace/ai/verify.sh
```

`ai/verify.sh`가 하는 일: 두 로봇의 state publisher 실행 → 텔레옵 실행 →
랜덤 자세를 마스터에 퍼블리시 → **tf에서 읽은 EE 위치와 `fk_pose()` 결과를 비교**.

### W2 완료 시점의 검증 결과

- **FK vs tf**: 랜덤 8자세, Omni(`omni/base→omni/stylus`) / OMY(`omy/world→omy/end_effector_link`)
  최대 오차 **3.3e-16 m**
- **해석적 Jacobian vs FK 수치미분**: 두 로봇 모두 최대 오차 **1e-7 이하**
- 텔레옵: `/omni/joint_states` 10 Hz(GUI) → `/omy/joint_states` 100 Hz, 값 그대로 전달
- `frame_prefix` 적용 확인 (`omni/`, `omy/`), 네임스페이스 토픽 4개 정상
- GUI: RViz 2개 + 슬라이더 창 정상 렌더링, 메쉬 로딩 에러 없음
- 매핑: 마스터가 조인트 순서를 섞어 보내도 `joint1..joint6`에 정확히 전달

**새 기구학 코드를 추가하면 반드시 tf와 대조해서 검증할 것.** (하드코딩 DH를 믿지 말 것)

---

## 7. 코드 컨벤션

- 파일명: 패키지 폴더는 `snake_case`, 모듈/클래스 진입점 파일은 `camelCase`
  (`omniRvizMotion.py`, `omyTeleop.py`, `jointMapping.py`) — 교수님 슬라이드 명명 그대로.
- 클래스는 `PascalCase` (`OmniRvizMotion`, `JointMapper`, `JointTeleop`).
- 주석/docstring은 **영어**, README는 **한국어** (학생 대상).
- 기구학 상수는 반드시 URDF에서 가져오고, 출처를 주석으로 남긴다.
- numpy 기반, 외부 의존성 최소화 (scipy도 안 씀).
- 커밋 메시지: 제목은 영문/한글 혼용 가능, 본문은 한국어로 무엇을 왜 바꿨는지.

---

## 8. 다음 작업 (제안)

### W3 (9/10) — `0910/` : Robotics coding

슬라이드 주제: coordinate transform, FK, Jacobian, IK, trajectory planning.
`0904/`에 FK와 Jacobian은 이미 있으므로, 추가로 필요한 것:

- [ ] **IK**: OMY-3M 6-DOF 역기구학. Jacobian 기반 수치해(DLS/pseudo-inverse)가 무난.
      해석해를 원하면 손목 3축이 한 점에서 만나는지(spherical wrist) 먼저 확인 필요 —
      URDF상 joint4(y)/joint5(z)/joint6(y) 원점이 서로 떨어져 있으므로 **spherical wrist가
      아니다.** 수치해로 가는 게 안전.
- [ ] **Trajectory planning**: joint space 5차 다항식 또는 trapezoidal profile.
- [ ] **Coordinate transform 유틸**: rpy/quaternion/rotation matrix 변환, `tf2`와 값 대조.
- [ ] 검증: IK(FK(q)) ≈ q, tf와 대조.

### W4 (9/17) — `0917/` : Teleoperation I, motion mapping

- [ ] 작업 공간 매핑으로 전환: `master_base T master_ee = slave_base T slave_ee`를
      만족시키도록 마스터 EE pose → 슬레이브 IK.
- [ ] 두 베이스 프레임 사이의 정렬(alignment) 변환 정의.
- [ ] `JointMapper`의 scale/offset 활용 또는 `PoseMapper` 추가.
- [ ] 클러치(clutching) 훅 자리 마련 (W6에서 본격적으로).

---

## 9. 출처 / 라이선스

| 파일 | 출처 | 라이선스 |
|---|---|---|
| `0904/omni_sim/description/` | [fsuarez6/phantom_omni](https://github.com/fsuarez6/phantom_omni) `omni_description` | BSD |
| `0904/omy_sim/description/` | [ROBOTIS-GIT/open_manipulator](https://github.com/ROBOTIS-GIT/open_manipulator) (`jazzy`) `open_manipulator_description` | Apache-2.0 |

두 URDF 모두 **메쉬 URI 접두사만** 이 폴더 구조에 맞게 수정했고 기구학 값은 원본 그대로다.
교수님 강의 PDF는 리포지토리에 올리지 않았다.
