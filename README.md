# Isaac Sim Dual-Robot Parcel Sorting & Palletizing

### M0609 · RMPFlow · VGC10 · ROS 2 · Vision

**`Parcel_sorting_IsaacSim`은 Doosan M0609 로봇 두 대의 흡착 이송·팔레타이징과 택배/QR 인식을 다루는 Isaac Sim 기반 물류 자동화 프로젝트입니다.** Robot A/B가 컨베이어의 박스를 집어 팔레트에 적재하는 제어 코드와, YOLO11·QR 디코딩·PyQt5 모니터링을 담당하는 ROS 2 `cobot3` 패키지를 포함합니다.

로봇·그리퍼·컨베이어·팔레트의 USD 장면, RMPFlow motion control, PhysX 물체 결합, 카메라 인식을 함께 다루는 **로봇 디지털 트윈 / Physical AI 개발 사례**입니다. 핵심 엔지니어링 작업은 중복된 A/B 제어 코드를 공통 모듈로 통합하고, 기능 동등성 검수에서 발견한 로봇별 상태 혼선과 흡착 결과 전달 문제를 수정한 것입니다.

> **현재 상태:** 듀얼 로봇 팔레타이징과 Vision 노드의 구현이 있으며, 정적 검사 및 unittest/mock 기반 회귀 검사를 수행했습니다. QR 결과에서 로봇 구역 명령으로 이어지는 연결은 미완성입니다. **전체 Isaac Sim / PhysX / ROS 2 런타임 검증은 별도 smoke test 단계로 남아 있습니다.**

[코드 검수 결과](docs/development/REGRESSION_REVIEW_REPORT.md) · [리팩터링 분석](docs/development/REFACTOR_ANALYSIS.md) · [런타임 검증 체크리스트](docs/ISAAC_SIM_SMOKE_TEST.md)

---

## 📌 주요 기능 (Key Features)

### 1. Isaac Sim 기반 Dual M0609 Palletizing

- **Robot A / Robot B:** 각 로봇의 box, pick-ready zone, pallet slot, tool/joint 경로를 별도 설정으로 관리합니다. standalone A/B와 shared World 기반 dual 실행을 지원합니다.
- **Motion control:** M0609 URDF·robot description·RMPFlow 설정을 사용해 Cartesian 목표를 제어하고, 관절 회전 및 home 복귀 단계와 조합합니다.
- **VGC10 suction tool:** tool pose를 따라가는 VGC10 visual과 박스 윗면 흡착 조건을 계산합니다. grasp 시 로봇 link와 박스를 `UsdPhysics.FixedJoint`로 연결하고, release 시 joint를 제거합니다.
- **Palletizing sequence:** 박스 정지/준비 구역 판정 → 접근·정렬 → 흡착 → 안전 높이로 lift → joint swing → slot 접근·yaw 정렬·하강 → release → home → 다음 박스 선택.
- **Scene interaction:** 박스 physics 활성화 순서, 구역별 게이트, 적재 후 가상 forklift 이송과 팔레트 하강을 profile에 따라 실행합니다. 가상 forklift는 pallet/cargo를 freeze하고 Xform으로 이동시키는 방식입니다.
- **ROS 2 Bridge:** dual runner가 Bridge를 활성화해 기존 USD 카메라 graph를 사용하는 구조입니다. 실제 카메라 publisher 생성과 토픽 전송은 런타임 검증 대상입니다.

### 2. 택배 / QR 인식 (Parcel & QR Detection)

- **탐지:** YOLO11 모델로 택배(`package`)와 QR 라벨(`qr_label`) bbox를 탐지하고, 박스 중심에서 가장 가까운 QR 라벨을 매칭합니다.
- **디코딩:** QR bbox를 crop하여 `pyzbar`로 우선 판독하고, 실패 시 OpenCV `QRCodeDetector`로 fallback합니다. 성공한 문자열을 `/qr_code`로 발행합니다.
- **라벨 미검출 알림:** QR 라벨이 검출되지 않은 프레임의 박스에 대해 `/parcel_no_label`로 `NO_LABEL`을 발행합니다. 예외 박스를 물리적으로 분리하는 기능은 별도 통합 과제입니다.
- **기존 학습 결과:** 기존 프로젝트 기록에는 Isaac Sim Replicator 합성 이미지 **300장**, YOLO 학습 **mAP50 = 0.995**가 남아 있습니다. `Vision/models/`의 가중치는 포함되어 있으나, 현재 저장소에는 해당 데이터셋 생성·학습·평가를 재현하는 코드가 확인되지 않습니다. 이 수치는 이번 검수에서 재측정하지 않았으며, Replicator 생성 framework 구현 완료를 의미하지 않습니다.

### 3. 상태 모니터링과 GUI

- `parcel_hub_node`가 영상 재배포, detector/QR 상태 수집, 워치독 재명령과 `/hub/state`, `/hub/alert` 발행을 담당합니다.
- PyQt5 GUI는 컨베이어 영상, YOLO 시각화, QR crop과 판독 결과, 노드 상태를 표시하는 구성을 제공합니다.
- GUI의 제어 버튼은 구현되어 있지만 detector/hub와 일부 토픽명이 다르고, 시뮬레이터의 정지·reset 명령 수신 코드도 연결되어 있지 않습니다. 통합 제어 완료나 안전 기능 검증으로 해석하지 않습니다.
- PatchCore 이상 탐지는 기본 실행 파이프라인에서 제외된 **별도 prototype**입니다.

---

## 🏗️ 시스템 설계 (System Architecture)

### 로봇 실행과 Vision의 연결 경계

```text
Isaac Sim / Conveyor_lift.usd
  ├─ shared World → Robot A worker / Robot B worker
  │                   └─ RMPFlow + VGC10 + Physics FixedJoint → palletizing
  ├─ box schedule / gate / virtual forklift
  └─ 기존 USD camera graph + ROS 2 Bridge
       │ /rgb (실제 발행 확인 필요)
       ▼
image_transport → /rgb/compressed → parcel_hub_node
                                      │ /hub/rgb/compressed
                         ┌────────────┴──────────────┐
                         ▼                           ▼
                parcel_detector_node         qr_decoder_node
                         └─ /parcel_with_qr ────────►│
                                                    ├─ /qr_crop_image
                                                    └─ /qr_code (판독 문자열)
영상 · detection · QR 결과 · hub 상태 ────────────────► PyQt5 GUI

/qr_code ── [변환 bridge 미구현] ──► /tmp/zone_command.txt의 A 또는 B
                                            └─ dual runner의 gate 제어
```

현재 로봇의 pick 대상과 적재 위치는 **USD 장면의 box/zone/slot 정보**로 결정합니다. Vision bbox를 로봇 좌표로 변환하거나 `/qr_code`로 로봇 목표를 직접 지정하는 통합은 구현되어 있지 않습니다. `ZONE_A`/`ZONE_B` 등의 QR 문자열을 gate 입력 `A`/`B`로 변환하는 writer도 저장소에 없습니다. 다중 PC에서는 시뮬레이터 측 파일 전달 방식까지 연결해야 합니다.

### 분산 실행 구성

기존 개발 환경은 Vision PC와 Isaac Sim PC를 유선 LAN으로 연결한 구성이었습니다. 아래 주소는 해당 환경의 예시이며, 현재 checkout의 전체 통합 동작을 보증하는 설정은 아닙니다.

| PC | Hostname | IP 예시 | 역할 |
|---|---|---|---|
| Vision PC | `vision` | `10.0.0.1` | YOLO11, QR, PyQt5 GUI, ROS 2 hub |
| Isaac Sim PC | `IsaacSim05` | `10.0.0.2` | M0609 A/B, USD/PhysX, 카메라 graph, gate |

ROS domain은 실행 스크립트 기준 `103`입니다. 실제 장비의 DDS/QoS와 네트워크 설정, 카메라 토픽을 맞춰야 합니다.

---

## 🛠️ Engineering: 공통 모듈과 회귀 검수

### 중복 스크립트에서 공통 core로

과거에는 A/B 각각의 대형 단일 Python 스크립트와, 소스를 문자열로 내장하여 `exec(compile(...))`로 실행하는 dual wrapper가 있었습니다. 현재는 `palletizing/` 공통 core와 네 개의 명시적 profile을 사용합니다. **Robot A/B entrypoint는 SimulationApp을 생성한 뒤 config와 공통 bootstrap을 호출하는 얇은 구조**입니다.

| 공통 모듈 | 역할 |
|---|---|
| `bootstrap.py` | standalone app/World/reset/close lifecycle, 확장 및 logging 설정 |
| `config.py` | A/B standalone·dual의 `RobotCellConfig`와 동작 정책 |
| `diagnostics.py` | pose, joint, bbox, yaw 진단 및 mass/inertia 보조 처리 |
| `forklift.py` | B standalone의 home 복귀와 병행하는 pallet lowering coordinator |
| `palletizing.py` | 박스/구역 판정, slot·goal 계산, task와 motion 목표 계산 |
| `physics.py` | rigid body·collision·FixedJoint attach/release |
| `scene.py` | USD prim/transform/bbox, VGC10 visual/follow, 흡착 판정 |
| `settings.py` | 공통 threshold, 좌표·관절값, timing 및 motion 설정 |
| `worker.py` | 로봇별 실행 상태, 공통 palletizing sequence, 가상 forklift coroutine |

세 entrypoint는 Isaac 의존 모듈보다 먼저 `SimulationApp`을 생성합니다. `palletizing.config`는 단독 import해도 app/World나 Stage를 생성하지 않습니다. Dual은 A/B task 등록 후 master가 shared `World.reset()`을 한 번 호출합니다. 일반 루프 외에 worker 초기 준비·release 관찰 구간에도 World step이 있으므로, 모든 물리 step을 wrapper만 수행하는 구조는 아닙니다.

### A/B 동작 차이 보존

| Profile | 대상 | 적재 및 scene 동작 |
|---|---|---|
| A standalone | `m0609_A`, `OriBoxA_*`, `APalt` | 4개 적재 설정, lower/settle 중 fused yaw 보정, 4개 release 후 가상 forklift |
| B standalone | `m0609_B`, `OriBoxB_*`, `BPalt` | 반복 slot marker, 두 번째 release 후 home 복귀와 병행하는 BPalt 하강, 4개 후 가상 forklift |
| A dual | A 경로, 독립 tool/joint 상태 | 2개 적재 후 APalt 가상 forklift |
| B dual | B 경로, 독립 tool/joint 상태 | 2개 적재, pallet lowering/forklift 비활성 |

`A_STANDALONE_CONFIG`, `B_STANDALONE_CONFIG`, `A_DUAL_CONFIG`, `B_DUAL_CONFIG`로 이러한 차이를 표현합니다. 리팩터링 검수는 기존 RMPFlow 설정과 motion 수치·이벤트 순서를 보존하는 데 초점을 맞췄습니다.

### 리팩터링 후 회귀 검수에서 확인하고 해결한 문제

단순한 파일 분리 이후 **실행 경계에서 기능 동등성이 유지되는지** 검수했습니다. 지연 초기화와 shared mutable state의 회귀를 mock 테스트로 재현하고 다음을 수정했습니다.

- **A/B scene setup 격리:** `World.reset()`의 지연 task 초기화와 중첩 callback에도 소유 worker context를 적용해, A가 B 설정으로 초기화되는 문제를 해결했습니다.
- **발견한 prim 경로 전달:** USD 로딩에서 실제로 찾은 robot/tool 경로를 해당 worker와 scene/physics helper에 전달하도록 보완했습니다.
- **흡착 상태 전달:** 같은 프레임의 최신 흡착 anchor를 FixedJoint 생성에 전달하고 실패 시 이전 결과를 지워, stale state 사용을 방지했습니다.
- **진단 상태 격리:** B joint 검색을 active robot 경로 기준으로 수정하고 흡착 진단 counter를 A/B별로 분리했습니다.
- **종료·정리 경로 보완:** standalone 초기화 오류와 dual main 종료 경로에서 소유 app의 `close()`가 실행되도록 `finally` 처리를 보완했습니다.

기존 global helper와 긴 worker 흐름은 일부 남아 있습니다. Context 격리는 단일 스레드의 순차 실행과 callback 재진입을 위한 것이며, 완전한 instance 기반 설계나 멀티스레드 안전성을 주장하지 않습니다. 상세 근거는 [Regression review report](docs/development/REGRESSION_REVIEW_REPORT.md), 초기 구조 분석은 [Refactor analysis](docs/development/REFACTOR_ANALYSIS.md)에 기록되어 있습니다.

### Validation 상태

| 검증 범위 | 상태 |
|---|---|
| 원본/리팩터링 코드·설정 비교, AST/구문 검사, import 구조 검사 | 회귀 검수에서 수행 |
| unittest 및 mock World 회귀 검사 | audit 기록 기준 **17개 통과** — config, A/B context, 흡착 상태, lowering, 종료 등 |
| 실제 Isaac Sim / PhysX / ROS 2 전체 smoke test | **미완료 — 별도 환경에서 실행 필요** |

**Static and mock-based regression checks were performed. Full Isaac Sim / PhysX / ROS 2 runtime validation remains a separate smoke-test step.**

실제 검증은 A standalone → B standalone → dual 순서로 진행하며, FixedJoint 접촉·운반 안정성, RMPFlow 궤적과 slot 정렬, release/home 반복, 가상 forklift·B 병렬 하강, 카메라/ROS 토픽·gate timing, 종료 시 비동기 작업 처리를 확인해야 합니다. [Smoke test checklist](docs/ISAAC_SIM_SMOKE_TEST.md)에 절차와 로그 기준이 있습니다.

---

## 📂 Repository Structure

주요 파일과 디렉터리만 표시했습니다. 저장소 이름은 `Parcel_sorting_IsaacSim`, Vision의 ROS 2 패키지 이름은 `cobot3`입니다.

```text
Parcel_sorting_IsaacSim/
├── README.md
├── isaacsim_dual_robot_palletizing/
│   ├── Conveyor_lift.usd
│   ├── M0609/
│   │   ├── robot_a_palletizing_forklift.py
│   │   ├── robot_b_palletizing_forklift.py
│   │   ├── run_ab_dual_robot_ros_gate.py
│   │   ├── palletizing/           # 위 공통 core 모듈
│   │   ├── rmpflow/              # controllers, YAML, URDF
│   │   ├── assets/               # VGC10 등
│   │   ├── Collected_m0609_camera/
│   │   ├── Collected_m0609_gripper/
│   │   ├── doosan-robot2/
│   │   └── onrobot_rg2/
│   ├── ori0/
│   ├── oriA/
│   ├── oriB/
│   ├── warehouse/
│   └── omniverse-content-production.s3-us-west-2.amazonaws.com/
├── Vision/                       # ROS 2 package: cobot3
│   ├── cobot3/
│   │   ├── parcel_detector_node.py
│   │   ├── qr_decoder_node.py
│   │   ├── parcel_hub_node.py
│   │   ├── parcel_control_gui.py
│   │   ├── patchcore_anomaly_node.py
│   │   ├── talker.py
│   │   └── listener.py
│   ├── launch/parcel_detector.launch.py
│   ├── models/                   # YOLO 및 PatchCore 가중치
│   ├── scripts/start_vision.sh
│   ├── resource/
│   ├── test/                     # ROS package lint tests
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   └── LICENSE
├── tests/
│   ├── test_palletizing_config.py
│   └── test_palletizing_runtime.py
└── docs/
    ├── ISAAC_SIM_SMOKE_TEST.md
    └── development/
        ├── REGRESSION_REVIEW_REPORT.md
        └── REFACTOR_ANALYSIS.md
```

---

## 🔍 Vision 주요 노드 설명

### 1. `parcel_hub_node` — 중앙 허브

영상 재배포 및 각 노드 상태 감시를 담당하는 중앙 허브 노드입니다.

- `/rgb/compressed` 영상 수신 → `/hub/rgb/compressed`로 재발행
- detector/QR decoder 워치독 (타임아웃: detector 5초 / QR decoder 8초, 최대 3회 재명령)
- `/state/simulation` 구독은 있으나 저장소 Python에 대응 publisher는 없음
- `/hub/state`, `/hub/alert`를 통해 GUI에 상태 정보 제공
- `/cmd/detection_enable`, `/cmd/conf_threshold`, `/cmd/qr_enable`, `/cmd/relay_enable` 명령 처리(중계)

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| Subscribe | `/rgb/compressed` | `sensor_msgs/CompressedImage` | 입력 영상 |
| Publish | `/hub/rgb/compressed` | `sensor_msgs/CompressedImage` | 허브 재배포 영상 |
| Subscribe | `/state/detector` | `std_msgs/String` | detector 상태 |
| Subscribe | `/state/qr_decoder` | `std_msgs/String` | QR decoder 상태 |
| Publish | `/hub/state` | `std_msgs/String` | 통합 상태 |
| Publish | `/hub/alert` | `std_msgs/String` | 경고 메시지 |

### 2. `parcel_detector_node` — YOLO 감지

YOLO11 모델로 택배 박스와 QR 라벨을 감지하는 노드입니다.

- 입력 영상에서 `package`, `qr_label` 감지 (`vision_msgs/Detection2DArray`로 발행)
- 박스 ↔ QR 라벨 매칭 → `/parcel_with_qr` 발행 / QR 라벨이 검출되지 않은 프레임의 박스 → `/parcel_no_label`
- 감지 결과 시각화 이미지(annotated) 발행
- `/cmd/detection_enable`, `/cmd/conf_threshold`로 감지 on/off 및 confidence threshold 제어

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| Subscribe | `/hub/rgb/compressed` | `sensor_msgs/CompressedImage` | 허브에서 받은 영상 |
| Publish | `/parcel_detections` | `vision_msgs/Detection2DArray` | 전체 감지 결과 |
| Publish | `/parcel_detections/annotated` | `sensor_msgs/Image` | 시각화 이미지 |
| Publish | `/parcel_with_qr` | `vision_msgs/Detection2DArray` | package + qr_label 매칭 결과 |
| Publish | `/parcel_no_label` | `std_msgs/String` | QR 라벨 미검출 알림 (`NO_LABEL`) |
| Publish | `/state/detector` | `std_msgs/String` | detector 상태 |
| Subscribe | `/cmd/detection_enable` | `std_msgs/Bool` | 감지 on/off |
| Subscribe | `/cmd/conf_threshold` | `std_msgs/Float32` | YOLO confidence threshold 변경 |

### 3. `qr_decoder_node` — QR 판독

YOLO가 감지한 QR 라벨 영역을 crop한 뒤 QR 값을 디코딩하는 노드입니다.

- `/parcel_with_qr`에서 package + qr_label bbox 수신, `/hub/rgb/compressed`에서 최신 프레임 캐시
- QR 영역 crop → `pyzbar` 우선 디코딩, 실패 시 OpenCV `QRCodeDetector`로 fallback
- 디코딩 성공 시 `/qr_code` 발행 (`NO_QR`은 발행하지 않음), crop 이미지는 `/qr_crop_image`로 발행

| 방향 | 토픽 | 타입 | 설명 |
|---|---|---|---|
| Subscribe | `/hub/rgb/compressed` | `sensor_msgs/CompressedImage` | QR crop용 영상 |
| Subscribe | `/parcel_with_qr` | `vision_msgs/Detection2DArray` | package + qr_label bbox |
| Publish | `/qr_code` | `std_msgs/String` | 판독 문자열 (예: `ZONE_A`, JSON 아님) |
| Publish | `/qr_crop_image` | `sensor_msgs/Image` | QR crop 확인용 이미지 |
| Publish | `/state/qr_decoder` | `std_msgs/String` | QR decoder 상태 |
| Subscribe | `/cmd/qr_enable` | `std_msgs/Bool` | enable 상태 갱신 (현재 decode 중단에는 미반영) |

### 4. `parcel_control_gui` — 중앙 제어 GUI

PyQt5 기반 GUI 노드입니다.

- 컨베이어 영상 / YOLO 감지 결과 / QR crop 이미지 / QR 판독 결과 표시
- 허브, detector, QR decoder 상태 확인
- emergency stop, reset, threshold 조절 등 제어 UI 제공

| 토픽 | 설명 |
|---|---|
| `/rgb/compressed` | 컨베이어 입력 영상 |
| `/parcel_detections/annotated` | YOLO 시각화 결과 |
| `/parcel_with_qr` | QR이 포함된 택배 감지 결과 |
| `/parcel_no_label` | GUI는 CompressedImage 구독 — detector의 String과 타입 불일치 |
| `/qr_crop_image` | QR crop 이미지 |
| `/qr_code` | QR 판독 문자열 |
| `/hub/state` | 허브 통합 상태 |
| `/hub/alert` | 시스템 경고 |

> GUI에는 optional import 처리가 있지만 실제 영상·ROS 통신에는 관련 의존성이 필요합니다. 캡처 경로는 `~/parcel_captures/` 아래에 생성됩니다. 제어 토픽 및 메시지 타입의 연결 한계는 아래 통합 과제를 참고하세요.

### 5. `patchcore_anomaly_node` — 이상 감지 prototype

PatchCore 기반 박스 이상 감지 노드입니다. RGB 이미지와 YOLO bbox로 박스 영역을 crop하고, PatchCore memory bank/threshold로 정상·훼손 여부를 판단해 `/parcel_anomaly`로 발행합니다.

**별도 prototype이며 기본 `start_vision.sh` 파이프라인에는 포함되어 있지 않습니다.** raw `/rgb`와 detector bbox, 저장된 memory bank/threshold를 사용합니다. 기본 분류 시스템에 통합되거나 성능 검증이 완료된 기능으로 보지 않습니다.

---

## 💻 개발 환경 (Environment)

기존 프로젝트에 기록된 개발 환경입니다. 현재 리팩터링 결과의 전체 런타임 검증 환경과는 구분합니다.

- **OS / Middleware:** Ubuntu 22.04 LTS / ROS 2 Humble
- **Simulator:** NVIDIA Isaac Sim 5.1.0 — 로봇 스크립트는 Isaac Sim 제공 Python 사용
- **Vision:** Python 3.10, OpenCV, PyQt5, Ultralytics YOLO, PyTorch, pyzbar
- **ROS 2 package:** `Vision/`의 `cobot3`, `ROS_DOMAIN_ID=103`
- **기존 장비 기록:** Isaac Sim PC의 RTX 5080, X11 디스플레이와 유선 LAN 구성

정확한 PyTorch/CUDA 버전 조합은 저장소에 고정되어 있지 않습니다. GPU 추론 및 Isaac Sim 실행 호환성은 사용할 장비에서 확인해야 합니다.

---

## 📦 의존성 (Installation)

<details>
<summary>두 PC 실행용 LAN / FastDDS 설정 예시</summary>

기존 개발 환경의 설정 예시입니다. IP와 `enp131s0`는 실제 장비에 맞춰 바꿉니다.

| 항목 | 내용 |
|---|---|
| 연결 방식 | 유선 기가비트 LAN, PC 간 직결 (스위치 미경유) |
| Vision PC (`vision`) | 10.0.0.1 |
| Isaac Sim PC (`IsaacSim05`) | 10.0.0.2 |
| 유선 인터페이스명 | `enp131s0` (양쪽 PC 동일) |
| IP 할당 방식 | 정적 IP (netplan) |
| DDS 미들웨어 | FastDDS |
| ROS_DOMAIN_ID | `103` |

**a) netplan 고정 IP 설정**

각 PC에서 `/etc/netplan/99-wired-static.yaml` 파일을 생성합니다.

Vision PC (`vision`):
```bash
sudo nano /etc/netplan/99-wired-static.yaml
```
```yaml
network:
  version: 2
  ethernets:
    enp131s0:
      addresses:
        - 10.0.0.1/24
```

Isaac Sim PC (`IsaacSim05`):
```bash
sudo nano /etc/netplan/99-wired-static.yaml
```
```yaml
network:
  version: 2
  ethernets:
    enp131s0:
      addresses:
        - 10.0.0.2/24
```

양쪽 PC에서 권한 설정 후 적용:
```bash
sudo chmod 600 /etc/netplan/99-wired-static.yaml
sudo netplan apply
```

**b) 연결 확인 (ping)**

Vision PC에서: `ping 10.0.0.2`
Isaac Sim PC에서: `ping 10.0.0.1`
양쪽 모두 응답이 오면 성공입니다.

**c) FastDDS 유선 인터페이스 전용 설정**

각 PC에서 자신의 IP만 화이트리스트에 넣은 FastDDS XML 프로필을 생성합니다. (서로 다른 IP를 사용하므로 PC별로 파일 내용이 다릅니다)

Isaac Sim PC (`IsaacSim05`, IP `10.0.0.2`):
```bash
mkdir -p ~/.ros
cat > ~/.ros/fastdds_wired.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_wired</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>10.0.0.2</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="default_profile" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_wired</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
```

Vision PC (`vision`, IP `10.0.0.1`):
```bash
mkdir -p ~/.ros
cat > ~/.ros/fastdds_wired.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_wired</transport_id>
      <type>UDPv4</type>
      <interfaceWhiteList>
        <address>10.0.0.1</address>
      </interfaceWhiteList>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="default_profile" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_wired</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
    </rtps>
  </participant>
</profiles>
EOF
```

> ⚠️ `interfaceWhiteList`의 `<address>`는 **자기 자신의 IP**를 넣습니다 (Vision PC → `10.0.0.1`, Isaac Sim PC → `10.0.0.2`).

**d) 환경변수 설정 (양쪽 PC 동일하게)**
```bash
export ROS_DOMAIN_ID=103
export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds_wired.xml
```
`start_vision.sh`는 `ROS_DOMAIN_ID=103`을 설정합니다. FastDDS 프로필 경로는 실행 터미널에서 별도로 설정해야 합니다.

**e) ROS2 토픽 통신 확인**
```bash
ros2 topic list
ros2 topic hz /rgb
```

양쪽 PC의 ROS domain, 프로필의 로컬 IP, 실제 publisher/subscriber QoS를 확인합니다.

</details>

### 1. ROS2 패키지

```bash
sudo apt update
sudo apt install ros-humble-desktop \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  ros-humble-image-transport \
  ros-humble-compressed-image-transport
sudo apt install python3-opencv python3-numpy python3-pyqt5 python3-pyzbar tmux python3-colcon-common-extensions -y
```
> **`ros-humble-vision-msgs`는 Vision 통신에 필요합니다.** detector, QR decoder, GUI가 `Detection2DArray`를 사용하며, GUI의 optional import 처리만으로 실제 ROS 기능을 사용할 수는 없습니다.
> `image-transport`, `compressed-image-transport`는 Isaac Sim에서 발행하는 원본 `/rgb`를 `/rgb/compressed`로 변환하는 데 필요합니다 (아래 "4. 영상 압축 변환" 참고).

### 2. Python (pip) 패키지

**a) PyTorch (CUDA)**

Vision 환경의 PyTorch 및 GPU 사용 가능 여부를 확인합니다. CUDA/PyTorch 설치 조합은 사용 장비에 맞춰 구성해야 하며, 아래 나머지 패키지 목록만으로 GPU 호환성이 보장되지는 않습니다.

```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

**b) 나머지 Python 패키지**
```bash
pip3 install ultralytics torch torchvision pillow opencv-python-headless numpy PyQt5 pyzbar
```
> 기존 환경에서는 PyQt5와 OpenCV의 Qt 플러그인 충돌을 고려해 headless OpenCV를 사용했습니다. 위 목록은 버전이 고정된 재현 환경이 아니므로 실제 의존성 조합을 확인해야 합니다.

**c) pyzbar 시스템 의존성 (libzbar0)**
```bash
sudo apt install libzbar0 -y
```
> `pyzbar`는 내부적으로 시스템 라이브러리 `libzbar0`를 필요로 합니다. 누락 시 import 시점(`ImportError: Unable to find zbar shared library`)에 오류가 발생합니다.

**d) 의존성 선언 범위**

현재 `package.xml`에는 ROS2 기본 의존성 위주로 작성되어 있고, 실행에 필요한 `ultralytics`, `pyzbar`, `PyQt5`, `torch`, `torchvision`, `Pillow` 등의 외부 Python 패키지는 명시되어 있지 않습니다. `colcon build`만으로는 이 패키지들이 설치되지 않으므로, 외부 Python 의존성을 **별도로** 준비해야 합니다. 저장소에는 이를 모두 고정한 requirements 파일이 없습니다.

### 3. YOLO11 가중치 파일

학습된 YOLO11 가중치(`.pt`)는 `Vision/models/` 폴더 내에 포함되어 있습니다. 기본 실행 스크립트(`start_vision.sh`)는 **`parcel_qr_det.pt`** 를 사용합니다.

| 파일명 | 용도 |
|---|---|
| `parcel_qr_det.pt` | 택배 박스 + QR 라벨 감지용 **기본 모델** (start_vision.sh 사용) |
| `parcel_box_baseline.pt` | 박스 감지 baseline 모델 |
| `parcel_box_conveyor_det.pt` | 컨베이어 환경 박스 감지 모델 |
| `parcel_box_isaac_det.pt` | Isaac Sim 합성데이터 학습 박스 감지 모델 |
| `patchcore_memory_bank.pt`, `patchcore_threshold.pt` | PatchCore prototype용 (기본 실행 제외) |

가중치 파일은 저장소에 포함되어 있습니다. 추론에는 위 의존성과 ROS 2 빌드 환경이 필요합니다.

### 4. 영상 압축 변환 (image_transport republish)

USD 카메라 graph가 발행하도록 구성된 원본 이미지 토픽(`/rgb`)을 허브가 받을 수 있는 압축 포맷(`/rgb/compressed`)으로 변환해야 합니다. **Vision PC(`vision`)** 에서 `start_vision.sh` 실행 시 자동으로 함께 실행되며, 단독 실행 시에는 다음 명령을 사용합니다.

```bash
source /opt/ros/humble/setup.bash
ros2 run image_transport republish raw \
  --ros-args \
  --remap in:=/rgb \
  --remap out/compressed:=/rgb/compressed
```

- `in:=/rgb`: Isaac Sim ROS2 Bridge가 발행하는 원본 이미지 토픽
- `out/compressed:=/rgb/compressed`: `parcel_hub_node`가 구독하는 입력 토픽 (`input_topic` 파라미터 기본값과 일치)

### 5. Isaac Sim ROS2 Bridge 관련

- Dual은 `SimulationApp` 생성 후 Bridge를 활성화하며, standalone profile의 기본 Bridge 설정은 비활성입니다.
- 기존 환경의 `isaac_python` alias는 저장소에 정의되어 있지 않습니다. 실행 시에는 설치된 Isaac Sim의 `python.sh`를 사용합니다.
- 기존 환경에서는 spdlog 심볼 충돌을 피하기 위해 Bridge 관련 `LD_LIBRARY_PATH`를 Isaac 실행 환경에 한정했습니다.

---

## 🔨 Vision 빌드 방법

저장소의 `Vision/`이 ROS 2 패키지 루트입니다. 아래 예시는 이 디렉터리를 `~/cobot3_ws/src/cobot3`로 복사하거나 심볼릭 링크한 배치를 전제로 합니다. 저장소 자체의 폴더명은 `Vision`으로 유지합니다.

```text
cobot3_ws/
└── src/
    └── cobot3/   # Parcel_sorting_IsaacSim/Vision을 가리키는 패키지 경로
```

```bash
source /opt/ros/humble/setup.bash
cd ~/cobot3_ws
colcon build --symlink-install --packages-select cobot3
source install/setup.bash
```

`start_vision.sh`는 패키지 위치의 두 단계 상위를 워크스페이스 루트로 계산합니다. 저장소를 임의 경로에 clone한 뒤 `Vision/scripts/start_vision.sh`를 바로 실행하면 `install/setup.bash` 탐색 위치가 맞지 않을 수 있습니다.

---

## 🚀 실행 순서 (How to Run)

다음은 실제 환경 검증을 위한 실행 경로입니다. A/B standalone과 dual은 각각 별도 프로세스로 실행하고, 한 실행을 종료한 뒤 다음 profile을 시작합니다.

### 1. Isaac Sim: standalone 또는 dual 선택

Isaac Sim 설치 경로와 checkout 경로를 실제 위치로 바꿉니다. dual의 ROS 2 환경과 domain을 먼저 준비합니다.

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=103
cd /path/to/Parcel_sorting_IsaacSim/isaacsim_dual_robot_palletizing/M0609

# 아래 중 하나를 선택하여 실행
/path/to/isaac-sim/python.sh robot_a_palletizing_forklift.py
/path/to/isaac-sim/python.sh robot_b_palletizing_forklift.py
/path/to/isaac-sim/python.sh run_ab_dual_robot_ros_gate.py
```

카메라 검증은 dual에서 기존 `/World/Graph/ROS_Camera`와 `ROS2CameraHelper`, `/rgb` publisher를 확인합니다. Bridge 활성화 로그만으로 영상 발행 성공을 판단하지 않습니다.

### 2. Vision: 자동 실행 스크립트

위 워크스페이스 배치와 빌드를 완료하고 `/rgb` 입력을 준비한 뒤 실행합니다.

```bash
cd ~/cobot3_ws/src/cobot3
bash scripts/start_vision.sh
```

스크립트는 모델과 `install/setup.bash` 경로를 검사하고 tmux의 `vision` 세션을 구성합니다. 같은 이름의 기존 tmux 세션은 종료합니다. Isaac Sim은 별도로 실행해야 합니다.

| 순서 | 컴포넌트 | 실행 방식 | 지연 |
|---|---|---|---|
| 1 | `image_transport republish` | `ros2 run` | 즉시 |
| 2 | `parcel_hub_node` | `ros2 run` | 2초 |
| 3 | `parcel_detector_node` (`parcel_qr_det.pt`) | `ros2 launch` | 4초 |
| 4 | `qr_decoder_node` | `ros2 run` | 6초 |
| 5 | `parcel_control_gui` | `ros2 run` | 10초 |

> tmux 패널 전환: `Ctrl+B → 화살표키`, 세션에서 나가기: `Ctrl+B → D`.
> 시작 스크립트의 `publish_only_on_change` 인자는 현재 QR decoder에 선언·사용되지 않아, 해당 옵션으로 동작을 제어할 수 없습니다.

### 3. Vision: 수동 실행

각 노드는 별도 터미널에서 실행합니다. 터미널마다 환경을 먼저 적용합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/cobot3_ws/install/setup.bash
export ROS_DOMAIN_ID=103
```

```bash
# [1/5] image_transport
ros2 run image_transport republish raw \
  --ros-args --remap in:=/rgb --remap out/compressed:=/rgb/compressed

# [2/5] parcel_hub_node
ros2 run cobot3 parcel_hub_node \
  --ros-args \
  -p input_topic:=/rgb/compressed \
  -p output_topic:=/hub/rgb/compressed \
  -p enable_watchdog:=true

# [3/5] detector: launch에서 target_class_ids=[0, 1] 지정
ros2 launch cobot3 parcel_detector.launch.py \
  model_path:=$(ros2 pkg prefix cobot3)/share/cobot3/models/parcel_qr_det.pt \
  rgb_topic:=/hub/rgb/compressed

# [4/5] qr_decoder_node
ros2 run cobot3 qr_decoder_node \
  --ros-args -p rgb_topic:=/hub/rgb/compressed

# [5/5] GUI
ros2 run cobot3 parcel_control_gui
```

Detector를 launch 없이 직접 실행하면 기본 모델은 `yolo11n.pt`, 클래스 ID는 `[28]`입니다. 위 launch는 포함된 `parcel_qr_det.pt`와 `[0, 1]`을 사용합니다. launch의 기본 영상 입력은 `/rgb/compressed`이며, 위 명령과 시작 스크립트는 `/hub/rgb/compressed`를 명시합니다.

---

## 🔭 남은 통합 및 검증 과제

현재 코드에서 확인한 연결 상태입니다. 아래 항목은 리팩터링으로 새로 완성된 기능에 포함하지 않습니다.

| 항목 | 현재 상태 / 필요한 작업 |
|---|---|
| GUI → detector/hub 명령 | GUI의 `/detection_enable`, `/yolo_conf_threshold`와 수신 측 `/cmd/detection_enable`, `/cmd/conf_threshold`가 다름. 토픽 통일 또는 remap 필요 |
| `/parcel_no_label` | detector는 `std_msgs/String`, GUI는 `sensor_msgs/CompressedImage`. 타입/표시 경로 정리 필요 |
| QR enable | `/cmd/qr_enable`이 상태값만 바꾸며 실제 decode 처리 중단에는 미반영 |
| `/qr_code` → gate | QR 문자열을 시뮬레이터 측 `/tmp/zone_command.txt`의 `A`/`B`로 변환하는 bridge/writer 미구현. 현재 gate는 두 구역을 처리하며 A~E 자동 분류가 아님 |
| GUI → simulation | 정지·reset·simulation control 발행 UI는 있으나 대응 consumer와 `/state/simulation` publisher가 저장소 Python에 없음 |
| Gate / box timing | 연속 같은 zone은 다른 zone/reset 전까지 중복으로 처리. gate countdown은 pause 중 멈추지만 box schedule은 wall time 사용. 실제 동작 확인 필요 |
| PatchCore | 별도 prototype. 기본 pipeline 통합과 이상 탐지 성능 평가 필요 |
| 전체 runtime | FixedJoint/PhysX, RMPFlow, USD 구성, A/B 간 timing, forklift, ROS camera/QoS, 정상·오류 종료를 smoke checklist에 따라 검증 필요 |

추가 개선 방향은 QR 기반 구역 연결, 미검출/판독 실패 박스의 물리적 예외 처리, 흡착 성공 상태 발행, Vision 의존성 버전 고정입니다. 실기 로봇 연동 및 sim-to-real 검증은 현재 구현·검증 범위 밖입니다.
