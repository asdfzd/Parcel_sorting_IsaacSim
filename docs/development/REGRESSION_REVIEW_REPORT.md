# Regression review: parcel_sorting

검수일: 2026-09-19.

## 1. 결론과 검증 범위

Sol 리팩터링은 중복 source injection을 제거하고 A/B 및 standalone/dual의
실제 차이를 보존한 공통 구현을 만들었다. 그러나 **`08f0d16`을 그대로
동작 보존 완료로 승인할 수는 없다.** 지연된 task 초기화의 A/B 설정 충돌과
같은 프레임 내 흡착 결과 전달 오류를 Python 테스트로 재현했다.
현재 구조 안에서 해당 회귀와 관련 상태 전달 문제를 수정했다.

- `STATICALLY_VERIFIED`: 원본 비교, 설정/호출 관계, 회귀 재현 및 수정 후 테스트.
- `LIKELY_CORRECT`: 원본 알고리즘과 설정이 같지만 외부 엔진 결과는 미확인.
- `NEEDS_ISAAC_SIM_TEST`: PhysX, USD composition, 실제 로봇 이동, 카메라/ROS,
  Kit 비동기 업데이트 및 실제 종료 동작.
- `POSSIBLE_REGRESSION`: 확정할 수 없는 차이 또는 보존 검토가 더 필요한 경로.

이 환경의 Python은 3.11.9이며 Isaac Sim/ROS2 런타임을 실행하지 않았다.
NumPy 2.4.6과 Pyflakes 3.4.0은 검수용 임시 디렉터리에만 설치했다.
테스트의 Isaac import/World 대역은 물리 시뮬레이터가 아니다.

## 2. Git 근거와 저장소 조사

| 대상 | 실제 기준 |
|---|---|
| Original | `08f0d16^` = `2d5589d` |
| Sol baseline | `08f0d16115e284ca9a30b68d9693265b6a1fb285` |
| Audit result | 위 HEAD 위의 미커밋 working-tree 변경 |
| 시작 상태 | clean working tree |

`git log`, `git diff 08f0d16^ 08f0d16`, `git show 08f0d16^:<path>`로
실제 원본을 읽었다. 원본 A/B뿐 아니라 dual runner의 `_CELL_A_SOURCE`,
`_CELL_B_SOURCE`도 AST 문자열 상수에서 추출해 별도로 비교했다. 원본을
추측해서 재구성하지 않았다. 원본 추출물과 일회성 분석 스크립트는 저장소
밖 임시 폴더에 두었다.

시작 시 tracked 파일은 401개, Python은 29개였다. M0609의 세 entrypoint,
공통 package 10개 모듈, 두 RMPFlow controller와 YAML/URDF 경로,
Vision detector/QR/hub/GUI/PatchCore/launch/setup/package/start script,
기존 테스트와 세 문서를 조사했다. USD/mesh/texture/model은 파일 목록과
경로·변경 여부를 확인했으며, binary 내용을 Python 소스처럼 검증했다고
주장하지 않는다. 주 환경 파일은 실제 `PXR-USDC` binary이며 조사한 `.usd`
파일에서 미다운로드 LFS pointer는 발견하지 않았다. composed stage는 미검증이다.

Sol commit이 변경한 파일은 17개이며 Vision, RMPFlow controller/YAML,
USD/URDF/texture/mesh/model은 그 commit에서 변경되지 않았다.
이번 검수에서도 asset과 Vision 코드는 수정하지 않았다.

### 비교 방법과 결과

함수/클래스 단위 AST를 비교하고, 실제 compatibility alias 이름과
docstring을 정규화한 뒤 차이가 나는 함수 및 각 main/worker 전체 흐름을
별도로 대조했다. 단순 line count 감소를 보존 증거로 사용하지 않았다.

| 원본 profile | Sol과 정규화 후 동일한 top-level 정의 | 내용 차이 | 별도 매핑 대상 |
|---|---:|---:|---|
| A standalone | 149 | 5 | `main` → bootstrap + `_worker_loop` |
| B standalone | 143 | 9 | `main` → bootstrap + `_worker_loop` |
| Embedded A dual | 147 | 5 | `make_worker_A` → `_worker_loop` |
| Embedded B dual | 140 | 9 | `make_worker_B` 및 robotBprop 이름의 helper 3개 |

B dual의 robotBprop helper는 삭제된 기능이 아니라 공통 robotaprop helper와
B config 값으로 대응된다. 동일한 AST도 global namespace가 바뀌면 동작이
달라질 수 있으므로, 위 수치는 런타임 동등성 증명이 아니다. 실제로 R1~R3가
이 한계에서 발생했다.

원본 top-level 설정을 NumPy/Path 기반으로 평가해 settings + profile overlay와
비교했다. 수정 후 A standalone 407개, B standalone 398개, dual A/B 각각
395개 대문자 설정 값이 일치했다. 경로 위치 기준도 원본 M0609 디렉터리로
맞춰 비교했다. `RMPFLOW_DIR`은 settings가 아니라 worker에서 동일하게
구성된다. `SimulationApp` 생성은 이 비교에서 실행하지 않았다.
원본 warning filter 누락은 R6으로 복원했다. 속도·가속도·threshold·좌표·
clearance·yaw target·event timing 값의 재조정은 없다.

## 3. 현재 구조와 Sol의 장점

| 모듈 | 실제 책임 / 평가 |
|---|---|
| `config.py`, `__init__.py` | frozen dataclass와 네 profile. Isaac import/World 생성 없음. 값뿐 아니라 fused yaw, lowering, forklift 정책 flag도 담는다. |
| `settings.py` | 공통 motion/physics/진단 설정. numpy 배열·파일 경로 해석은 있지만 app/World 생성 없음. |
| `scene.py` | USD 경로/transform/bbox, RG2 제거, VGC10 visual/follow, suction 판정. |
| `physics.py` | FixedJoint attach/release, rigid/kinematic/collision/velocity 관리. |
| `diagnostics.py` | pose/joint/yaw 검사. 일부 helper는 mass/inertia도 수정하므로 순수 로그 모듈은 아니다. |
| `palletizing.py` | detector, box/slot 선택, goal/waypoint, task, RMPFlow 목표 계산. |
| `forklift.py` | 실제로는 B의 home-return 병렬 pallet lowering coordinator. A forklift 전체가 이 파일로 이동한 것은 아니다. |
| `worker.py` | 공통 generator와 per-worker 상태. A forklift coroutine도 여전히 내부에 존재. 수정 후 World callback에 scoped context 적용. |
| `bootstrap.py` | standalone 확장 설정, World/reset, 반복 step, close 소유. |
| dual runner | bridge, shared World/reset, 두 factory worker, box sequencer와 gate. |

`scene → diagnostics → physics → palletizing → worker` 순서로 공통 helper 의존성이
이어진다. 실제 상대 import graph의 순환은 발견되지 않았다. bootstrap의 worker
import는 함수 내부에 있고, config/package import는 Isaac 모듈을 요구하지 않는다.
worker import는 RMPFlow 경로를 `sys.path`에 추가한다. 따라서 모든 공통 모듈이
완전히 side-effect-free라는 표현은 부정확하지만, import만으로 app/World를
생성하는 공통 모듈은 발견되지 않았다.

장점은 A/B entrypoint가 각각 20줄로 줄고, 18,000줄 이상의 embedded source가
제거되었으며, 네 profile의 차이가 명시적이고 주요 계산 함수가 재사용된다는 점이다.
공통 코드에서 `if side == "A"/"B"`를 반복하는 방식도 아니다.

다만 “책임 분리가 완료됐다”고 평가할 수는 없다. baseline 공통 모듈은 총
10,072줄이고 worker만 3,466줄이다. wildcard import, global overlay, 긴 generator,
진단과 제어의 혼합이 남아 있다. 거대한 common.py 한 파일로 옮긴 것보다는
개선됐지만, instance 기반 완전 격리 구조도 아니다. 이번에는 재설계하지 않았다.

## 4. 발견한 회귀와 최소 수정

### R1 — P0: dual의 deferred scene setup이 마지막 B 설정을 사용

`STATICALLY_VERIFIED` — 재현 후 수정.

Sol의 `PalletizingWorker.__next__`는 step 직전에만 module globals를 덮어썼다.
A/B 첫 `next`는 task 등록 후 yield하고, master가 나중에 `World.reset()`을
호출한다. 그 시점의 전역 값은 B이므로 A task도 B 로봇·box·pallet로 초기화된다.
원본은 A/B 함수가 서로 다른 namespace에 있었기 때문에 발생하지 않았다.

NVIDIA의 [Isaac Sim 5.1 World 구현](https://github.com/isaac-sim/IsaacSim/blob/v5.1.0/source/extensions/isaacsim.core.api/python/impl/world/world.py)은
`add_task`에서 등록하고 `reset`에서 `set_up_scene`을 호출한다. 또한 `step`은
각 task의 `pre_step`을 호출한다. 이 경계를 모사한 테스트에서 baseline은
`ValueError: duplicate scene object: m0609_robot_B`로 실패했다.

수정: `worker._WorkerTask`가 scene setup, observations, pre_step, post_reset을
소유 worker의 context 안에서 실행한다. `PalletizingWorker.context`는 종료와
예외 때 호출자 globals를 복구한다. 따라서 A의 내부 World.step 중 B callback이
실행돼도 이후 A 코드가 B 설정으로 계속 실행되지 않는다. 새 World나 추가 reset은 없다.

### R2 — P1: 발견된 robot/tool fallback 경로가 다른 모듈에 전달되지 않음

`STATICALLY_VERIFIED` — 경로 변경 대역 테스트로 수정 검증.

원본 `_load_usd`는 한 namespace의 `ROBOT_PRIM_PATH`, `OLD_GRIPPER_PRIM_PATH`,
`VGC10_FOLLOW_TARGET_PATH`를 갱신했다. 추출 후에는 palletizing.py의 값만
변하고 scene/physics/worker는 기존 값을 사용한다. 지연 reset 동안 변경된
값이 다음 worker 활성화 때 다시 덮일 수도 있었다.

수정: `_WorkerTask._load_usd`가 원래 로딩을 마친 즉시 해당 worker에 resolved
경로를 capture하고 helper 모듈들에 반영한다. A/B 각각 다른 fallback 경로가
setup을 계속하기 전에 전달되고 worker별로 남는 것을 테스트했다.
실제 USD의 fallback 경로 필요 여부는 `NEEDS_ISAAC_SIM_TEST`이다.

### R3 — P1: suction 판정 결과와 FixedJoint anchor 소비 시점 불일치

`STATICALLY_VERIFIED` — 성공·실패 경로 모두 재현 후 수정.

`scene.evaluate_suction_grid_on_box_top`이 `_LAST_SUCTION_GRID_INFO`를 새 dict로
재할당하면 worker는 이전 dict를 계속 읽는다. worker 종료 시 capture해도
현재 step의 `begin_grid_physics_attach`에는 늦다. 최초 attach는 fallback anchor를,
후속 판정은 이전 프레임의 anchor를 사용할 수 있다.

수정: worker와 helper가 공유하는 per-cell dict를 `clear/update`로 갱신한다.
no-bbox/no-suction-point 실패도 같은 dict에서 이전 성공 정보를 지운다.
XY clamp, top-surface epsilon, 거리/면적 조건과 FixedJoint 생성 알고리즘은 동일하다.
테스트는 최신 `[1.01, 2.0, 3.002]` anchor가 같은 호출에서 전달되고 다른 worker에
섞이지 않는 것을 확인한다.

### R4 — P2: B joint 진단에 A 이름이 하드코딩됨

`STATICALLY_VERIFIED` — 수정 및 테스트.

원본 B의 `scan_stage_joints_128`은 B robot joint를 검색했다. 공통 diagnostics는
`m0609_A`만 검사했다. 현재는 `ACTIVE_ROBOT_ROOT_PATH`로 검색한다.
box/vgc10 문자열이 없는 B 전용 robot joint도 진단 목록에 남는지 테스트했다.
제어·physics 값에는 영향이 없다.

### R5 — P3: suction 진단 함수의 counter가 A/B에 공유됨

`STATICALLY_VERIFIED` — 수정 및 테스트.

원본 namespace별 함수 속성 `_counter`가 공통 함수 하나의 속성이 되었다.
로그 간격이 양쪽 호출 수에 의해 바뀌었다. counter를 worker가 소유/전환하는
`_SUCTION_GRID_EVALUATION_COUNT`로 옮겼다. A/B/A 호출 결과가 각각 2/1이다.

### R6 — P2: 기존 Kit warning filter 누락

`STATICALLY_VERIFIED`(코드 복원), 실제 Kit 적용은 `NEEDS_ISAAC_SIM_TEST`.

원본 네 profile의 `SUPPRESS_NON_CRITICAL_KIT_WARNINGS_122=True`와 네 `/log/*`
설정 적용이 Sol에서 빠졌다. warning 폭주가 복구 대상 로그를 가릴 수 있어
원래 키/값/예외 처리만 bootstrap의 명시적 호출로 복원했다. standalone 및 dual에서
app 생성 후, scene setup 전에 적용된다. Python `print` 진단은 그대로이다.

### H1 — 종료 보완: owner main의 예외 경로

`STATICALLY_VERIFIED` — 정상 경로 소유권 유지, 오류 경로 보완.

standalone은 extension/import/World/factory가 기존 try 밖에 있었고,
dual은 전체 main을 보호하는 finally가 없어 setup 실패나 interrupt 때
close를 건너뛸 수 있었다. 원본에도 있던 취약점으로, 새 회귀라고 세지 않는다.
standalone try 범위를 넓히고 dual `main`에 단일 finally close를 두었다.
World 생성 실패 시 standalone close 1회를 테스트했다. dual reset 실패는
기존 로그/return을 유지하고 finally에서 닫는다.

entrypoint의 top-level import 자체가 실패하는 경우까지 포괄하는 재구성은
하지 않았다. 실제 Kit 종료 및 예약 coroutine 처리는 smoke test 대상이다.

## 5. A/B 차이 검토

`STATICALLY_VERIFIED`(값/branch), 실행 결과 `NEEDS_ISAAC_SIM_TEST`.

| 항목 | A standalone | B standalone | A dual | B dual |
|---|---|---|---|---|
| root/box/pallet | m0609_A / OriBoxA / APalt | m0609_B / OriBoxB / BPalt | A 경로 | B 경로 |
| ready X | -2.08983 | 1.9599 | -2.08983 | 2.08983 |
| side-only flag | True | False | True | True |
| slot marker sequence | A01,A02 | B01,B02,B01,B02 | A01,A02 | B01,B02 |
| joint-1 pattern (deg) | 220,155,220,155 | 220,155,220,120 | A와 동일 | 220,155,220,155 |
| layers / slot count | 2 / 4 | 2 / 4 | 1 / 2 | 1 / 2 |
| fused lower/settle yaw | 켬 | 끔 | 끔 | 끔 |
| missing-marker final move | 끔 | 켬 | 끔 | 끔 |
| lower after layer | 켬, 먼저 settle | 켬, home과 병렬 | 끔 | 끔 |
| virtual forklift trigger | 4개 | 4개 | 2개 | 실행 안 함 |
| diagnostic release count | 4 | 2 | 2 | 2 |
| ROS2 bridge flag | False | False | True | True |

진단 stop 자체는 원본의 비활성화 설정을 유지한다. B standalone에도 원본상
4개 뒤 BPalt virtual forklift 동작이 있고, B dual만 stack-only이다.
“B는 항상 forklift가 없다”로 단순화하지 않았다. B standalone의 A-prop fallback
목록과 특이한 box asset fallback도 원본 그대로다. 어색한 값이라는 이유로
바꾸지 않았다. A standalone marker가 둘뿐인 상태에서 4개를 처리하는 계산도
원래 marker-resolution fallback 로직을 유지한다.

## 6. lifecycle, shared state, import

- standalone: app → logging/extension/update → World/worker import → World 하나
  → task 등록 yield → owner reset 한 번 → 초기화 yield → owner step/worker → close.
- dual: app → ROS2 bridge/update → World/worker import → shared World 하나 → A/B
  task 등록 → master reset 한 번 → 각 worker 초기화 → home/VGC10 확인 → box
  sequence → master step, gate, A/B 순차 next → owner close.
- worker 내부에는 reset/close가 없다. 원본 standalone reset 소유권을 bootstrap으로
  이동한 것은 의도된 lifecycle 변경이다.
- 두 standalone entrypoint와 dual runner를 동시에 import하는 것은 지원 용도가
  아니다. 각 entrypoint는 원본처럼 top-level에서 SimulationApp을 만든다.
- generator-local completed roots, carry/home counters, forklift caches/cargo,
  B lowering coordinator는 각 worker별이다. global 호환 상태는 context에서
  연결하고 callback 종료 후 복구한다. **멀티스레드 안전성을 보장하지 않는다.**

일반 dual outer loop의 step은 하나지만 worker 초기 warm-up(30회),
`initialize_robot_for_conveyor` 내부 준비, release 관찰 루프와 일부 fallback도
같은 World를 step한다. 이는 원본 embedded code에도 존재한다. 따라서
“wrapper만 모든 World.step을 호출한다”는 README 문장을 수정했다.
이 구간의 다른 로봇 controller 정지 시간과 gate/box timer 영향은 실제 환경에서
검사해야 하며, step을 무조건 yield로 교체하면 timing이 달라져 이번에는 유지했다.

## 7. VGC10 / suction / physics / motion

`LIKELY_CORRECT`, R3의 상태 전달은 `STATICALLY_VERIFIED`, 물리 결과는
`NEEDS_ISAAC_SIM_TEST`.

RG2 제거, visual-only VGC10 reference, mount scale, tool follow, suction local
offset, center/top 판정과 clamp는 원본 함수와 비교했다. attach는 여전히
`UsdPhysics.FixedJoint`이며 body0/link, body1/box, localPos0/1, localRot0/1을
현재 world pose 기준으로 계산한다. release는 joint 제거 후 원래 선택된
drop/restore/velocity 경로로 간다. 모든 release 분기가 강제로 같은 rigid-body
toggle을 수행하는 구조로 바꾸지 않았다.

활성 carry 경로는 FixedJoint가 박스를 움직이게 하며 직접 box transform을
추종시키는 fallback은 기존 flag로 분리되어 있다. `STACK_SNAP_BOX_TO_SLOT_ON_RELEASE`
및 즉시 yaw snap은 비활성 상태를 유지한다. 별도로 **release 후 virtual forklift는
원본부터 pallet/cargo를 freeze하고 Xform으로 운반한다**. 이 기존 기능을 suction
운반이 teleport로 바뀐 것으로 오인하거나 FixedJoint 방식으로 새로 교체하지 않았다.

PickPlaceController/RMPFlow wrapper 두 파일, YAML, URDF와 controller constructor
인자·events_dt를 비교했다. stop/ready → pre-align → FixedJoint → vertical safe
lift → joint-1 reverse/swing → marker move-over → yaw/lower/settle → release →
home → next candidate의 흐름을 유지한다. B의 병렬 lowering은 기존 smoothstep과
home 160-step 설정을 coordinator로 옮긴 것으로, Pure Python 곡선 검사를 통과했다.
실제 interpolation 결과가 동일한 물리 궤적을 만든다는 보장은 아직 없다.

## 8. ROS2와 Vision 통합

Bridge import 순서는 `STATICALLY_VERIFIED`; publisher 동작은
`NEEDS_ISAAC_SIM_TEST`.

dual은 app 생성 뒤 `enable_extension("isaacsim.ros2.bridge")`와 update를
실행하고 World/helper를 import한다. standalone 기본 False는 원본과 같다.
Python은 기존 USD camera graph를 참조해 사용하며 graph 생성 구현을 새로
추가하지 않았다. `DISABLE_LEGACY_ROBOT_CAMERA_GRAPHS=False`,
`KEEP_RSD455_CAMERA_ACTIVE=True`도 유지한다. bridge enable 성공만으로
`ROS2CameraHelper` publisher 생성을 확정할 수 없다.

Vision은 원본/Sol에서 같은 코드이며 simulation Python을 import하지 않는다.
확인한 데이터 흐름은 `/rgb` → 외부 image_transport → `/rgb/compressed` → hub
→ `/hub/rgb/compressed` → YOLO package/qr_label nearest pairing → QR crop →
pyzbar/OpenCV fallback → `/qr_code`이다. QR 처리는 queue/thread, tracking과
중복 방지 상태를 갖는다. hub/watchdog와 GUI, PatchCore prototype도 별도 영역이다.

다음은 **기존 통합 문제/누락**이며 이번에 토픽·알고리즘을 바꾸지 않았다.

| 상태 | 근거와 영향 |
|---|---|
| `STATICALLY_VERIFIED` 기존 mismatch | detector `/parcel_no_label`은 `std_msgs/String`, GUI 같은 토픽은 `CompressedImage` 구독. 송장 미부착 이미지 경로가 연결되지 않는다. |
| `STATICALLY_VERIFIED` 기존 mismatch | GUI conf/detection 명령은 `/yolo_conf_threshold`, `/detection_enable`; hub/detector는 `/cmd/conf_threshold`, `/cmd/detection_enable`. 외부 remap/bridge 없이는 명령 경로가 다르다. |
| `STATICALLY_VERIFIED` 저장소 내 누락 | `/qr_code`를 `/tmp/zone_command.txt`의 `A/B`로 변환하는 writer/zone_bridge_node가 없다. `ZONE_A`를 그대로 쓰면 gate가 알 수 없는 zone으로 처리한다. 다중 PC라면 파일 위치/공유 방식도 필요하다. |
| `STATICALLY_VERIFIED` 기존 구현 차이 | QR header docstring은 JSON이라고 하나 실제 `_publish_result`는 zone 문자열을 발행한다. `_enabled`는 상태에만 반영되고 decode 경로를 중단하지 않는다. |
| `STATICALLY_VERIFIED` 기존 설정 차이 | start script의 `publish_only_on_change`는 decoder에서 선언/사용되지 않는다. launch 기본 camera 경로는 hub를 우회하지만 start script는 명시적으로 hub 토픽을 넣는다. |
| `NEEDS_ISAAC_SIM_TEST` | raw/compressed 변환, QoS, ROS domain 103, graph authored topic, 외부 simulation state/control node. Python에 `/state/simulation` publisher나 GUI simulation 명령 consumer가 없다. |
| `LIKELY_CORRECT` 별도 prototype | PatchCore는 raw `/rgb`, memory-bank/threshold 두 `.pt` 파일을 사용한다. 기본 launch/start script에는 포함되지 않는다. 외부 Python/ROS 의존성 전체 설치 여부는 미검증이다. |

README의 “Replicator 합성데이터 300장, mAP50 0.995”는 기록된 실험 결과다.
저장소 Python에서 Replicator/BasicWriter/annotator/domain randomization 생성
framework를 확인하지 못했다. 이를 구현 완료로 확장 주장하지 않았으며 모델을
재학습하거나 수치를 재측정하지 않았다.

## 9. Original vs Current feature matrix

Original A/B의 `S`는 standalone 파일, `D`는 원본 runner 내부 embedded source를
뜻한다. 아래 `PRESERVED`는 코드/설정 보존 판단이지 실제 Isaac 실행 통과가 아니다.
Current는 이번 수정 후 working tree이며 Sol 결함은 Notes에 명시한다.

| Feature | Original A | Original B | Current implementation | Status | Notes |
|---|---|---|---|---|---|
| App-before-Isaac imports | S app 먼저, D master 공유 | 동일 | 세 entrypoint + bootstrap | PRESERVED | app/extension/World 순서 확인 |
| Standalone World/reset | S main 하나씩 | S main 하나씩 | bootstrap → configured worker | CHANGED_INTENTIONALLY | 소유권 이동, reset 추가 없음 |
| Dual source injection | `_CELL_A_SOURCE` + exec | `_CELL_B_SOURCE` + exec | `create_worker` 두 번 | CHANGED_INTENTIONALLY | production embedded source/exec/compile 제거 |
| Deferred scene setup | D namespace A | D namespace B | `_WorkerTask` context | PRESERVED | Sol R1을 수정; duplicate B 재현 테스트 |
| Runtime mutable state | A globals/locals | 별도 B globals/locals | worker state + generator locals | PRESERVED | callback 격리 수정; threads 미지원 |
| Robot path discovery | S task:5072 | S task:4973 | task `_load_usd` + 경로 propagation | PRESERVED | Sol R2 수정; 실제 fallback USD는 runtime |
| SingleManipulator registration | A root | B root | task + config object name | PRESERVED | dual 이름/prim 분리 테스트 |
| RMPFlow/PickPlace | 기존 wrappers/events | 동일 wrappers/events | 변경 없는 rmpflow 파일 + worker | PRESERVED | 알고리즘/parameter 변경 없음 |
| RG2→VGC10 visual | S attach:1891 | S attach:1889 | `scene.attach_vgc10_to_link6` | PRESERVED | visual-only 처리 포함 |
| Suction visual follow | S update:2176 | S update:2174 | `scene.update_vgc10_suction_anchor` | PRESERVED | dual B tool/debug 경로 분리 |
| Suction surface test | S evaluate:2696 | S evaluate:2694 | scene evaluate + per-cell result dict | PRESERVED | Sol R3/R5 수정; 조건·epsilon 동일 |
| FixedJoint creation | S create:2806 | S create:2804 | `physics.create_physics_attach_joint` | PRESERVED | local position/rotation 및 mass 경로 유지 |
| FixedJoint physical following | 실제 link/box coupling 의도 | 동일 | worker FixedJoint branch | NEEDS_RUNTIME_TEST | joint drift/contact/관통은 정적으로 확정 불가 |
| Release/dynamic/velocity | S release:2892, restore:3068 | S release:2890, restore:3066 | physics release/restore + 원래 branch | PRESERVED | no-snap drop 경로도 보존 |
| Stop and ready zone | S detector:4260/4503 | S detector:4161/4404 | BoxStopDetector/PickZoneDetector + config | PRESERVED | B S/D ready X 차이 유지 |
| Box selection/completion | S discover:4553 | S discover:4454 | common candidate selector + prefix | PRESERVED | completed roots는 generator-local |
| Slot and stack goal | APalt marker, fallback | BPalt 반복 marker, fallback | config + core resolution/goal | PRESERVED | B S final move flag 보존 |
| Safe lift/clearance | S conveyor top:4803 | S conveyor top:4704 | common safe-height calculation | PRESERVED | numeric threshold 변경 없음 |
| Joint swing and guard | S make targets:5686 | S make targets:5587 | common joint route/link2 guard | PRESERVED | S B 마지막 120°와 D 155° 차이 |
| Cartesian carry sequence | S make targets:5808 | S make targets:5709 | common phase_sequence | PRESERVED | phase별 steps/waypoint 계산 비교 |
| Fused yaw | S만 lower/settle 중 켬 | 없음 | config + fused-yaw helper | PRESERVED | dual A에도 자동 적용하지 않음 |
| Pre-release yaw | A S fused 정책 | B 및 dual 별도 phase | RMPFlow orientation 목표 | PRESERVED | Sol의 undefined world.pause→my_world 보완도 유지 |
| Home / next box | S main:6111 | S main:6011 | `_worker_loop` | PRESERVED | sleep 위치/초기 yield 재배치는 아래 주의사항 |
| A virtual forklift | S 4개, D 2개 후 | D 실행 안 함 | worker coroutine + config flag | NEEDS_RUNTIME_TEST | 함수/속도/딜레이 보존; async update 실측 필요 |
| B pallet lower | A S blocking settle/lower | S schedule:6991, home과 병렬 | `PalletLoweringCoordinator` | PRESERVED | 160-step 곡선 검사; D lowering 끔 |
| Dual box physics schedule | A01@1, A02@13 | B01@7, B02@19 | `BoxPhysicsSequencer` | PRESERVED | collision subtree/velocity 처리도 비교 |
| Gate | A delay4/down5, Group | B delay6/down8, Group_01 | runner poll/trigger | PRESERVED | 동일 zone dedup와 pause semantics도 원본 |
| ROS2 Bridge | S False / D True | S False / D True | owner extension activation | PRESERVED | publish 성공 판정과 별개 |
| Camera publishers | authored graph 기대 | shared graph 기대 | 기존 USD + ROS2 bridge | NEEDS_RUNTIME_TEST | ROS2CameraHelper 실제 노드/토픽 검사 필요 |
| QR→zone file bridge | 원본에도 없음 | 원본에도 없음 | writer 구현 없음 | MISSING | 리팩터링 누락으로 분류하지 않음 |
| B robot joint diagnostics | A/B 각자 검색 | B 문자열 검색 | active root 기준 검색 | PRESERVED | Sol R4 수정 |
| Kit warning filter | True + 네 log 설정 | 동일 | settings + bootstrap explicit call | PRESERVED | Sol R6 누락 복구 |
| Shutdown | owner close, 오류 경로 불완전 | 동일 | main/run finally | CHANGED_INTENTIONALLY | H1; main 진입 후 오류 경로 보완 |
| Vision processing | 별도 ROS package | 동일 pipeline | Vision 원본 파일 그대로 | PRESERVED | 기존 topic/interface 문제는 위 표 |
| Synthetic generation code | 확인되지 않음 | 확인되지 않음 | framework script 없음 | MISSING | README 실험 결과와 구분 |

## 10. 수정하지 않은 의심 사항과 technical debt

1. `POSSIBLE_REGRESSION`: standalone은 sleep이 원본의 step 직후에서 worker
   실행 뒤로 이동했고, dual worker는 초기 준비 후 yield가 하나 더 있다.
   control phase step 수는 같지만 첫 제어 tick과 wall-clock callback timing은
   동일하다고 보장할 수 없다. 측정 없이 되돌리지 않았다.
2. `NEEDS_ISAAC_SIM_TEST`: forklift coroutine은 원본처럼 `next_update_async`와
   asyncio sleep을 사용한다. pallet/cargo 경로·offset은 closure별이며 shared helper의
   기본 transform 처리는 경로 인자를 받는다. 실제 pause/resume, 다른 worker와의
   interleave, 종료 시 pending task 처리는 실측 대상이다. thread 동시 실행은 금지 전제다.
3. `STATICALLY_VERIFIED` 기존 잠재 오류: JOINT_SWING의 bbox 실패 fallback에
   `attach_center`가 local assignment 전에 사용될 수 있다. 원본에도 동일한 코드가
   있으며 기본 모드는 `VERTICAL_JOINT1_REVERSE`다. 임의 fallback 좌표를 선택하면
   motion semantics를 바꾸므로 남겨 두고 진단 대상으로 기록했다.
4. `STATICALLY_VERIFIED` 기존 동작: `_last_zone`은 연속 같은 zone을 영구 중복으로
   취급하며 다른 zone 또는 reset 전까지 재트리거하지 않는다. gate timer는 pause에
   멈추고 box sequencer는 wall time을 사용한다. 새 scheduler로 교체하지 않았다.
5. `NEEDS_ISAAC_SIM_TEST`: reset 전에 box OFF를 시도해도 최초 scene이 아직
   구성되지 않았으면 해당 prim이 없다. post-reset OFF와 post-worker OFF는 유지되어
   있다. reset 내부 physics step에 의한 초기 box 미세 이동 여부는 원본부터의 검사 대상이다.
6. `STATICALLY_VERIFIED`: default box asset fallback의 `oriA/oriB` 경로 일부는
   checkout에 없다. 정상 사용은 USD pre-placed boxes이고 원본도 동일하다.
   missing-box fallback 실행은 별도 환경 확인이 필요하다.
7. `STATICALLY_VERIFIED`: legacy `_109/_145/_155/_170/_184` 이름·상수·alias가
   많이 남았다. 기능 이름으로 옮긴 공개 helper와 호환 alias는 유지했다.
   direct reference가 없는 후보(`initialize_robot`, `zero_body_velocity`,
   `_create_or_set_attr`, `get_suction_grid_world_points`, `should_script_attach`,
   `resolve_first_valid_path`, `align_box_yaw_to_slot_marker`,
   `lower_stack_support_cube_for_next_layer`, `_quat_np`, `_fit_joint_delta`)도
   외부 사용/feature flag fallback 가능성 때문에 일괄 삭제하지 않았다.
8. `STATICALLY_VERIFIED`: settings에 중복 할당(예: HOME_RETURN_STEPS), 과거 버전을
   설명하는 주석, B 실행에서도 A라고 출력하는 일부 legacy 로그가 남아 있다.
   runtime 적용 값은 마지막 할당 기준으로 비교했다. 로그 이름만으로 side를
   판정하지 말고 active root/body/box 경로를 확인해야 한다.

## 11. 변경 파일과 이유

| 파일 | 변경 이유 |
|---|---|
| `palletizing/worker.py` | R1/R2 callback context·resolved 경로 전달, R5 counter 격리 |
| `palletizing/scene.py` | R3 동일 step의 suction result 공유, R5 counter 저장 위치 |
| `palletizing/diagnostics.py` | R4 B joint filter |
| `palletizing/settings.py` | R6 원래 warning-filter flag 복구 |
| `palletizing/bootstrap.py` | R6 원래 logging 설정 복구, H1 close 범위 |
| `run_ab_dual_robot_ros_gate.py` | logging 호출, master finally close |
| `tests/test_palletizing_runtime.py` | 지연 setup·상태 격리·anchor·lowering·종료·B 진단 회귀 테스트 |
| `README.md` | worker 내부 World.step 예외를 정확히 설명, audit 링크 |
| `REFACTOR_ANALYSIS.md` | 역사적 문서임을 표시하고 잘못된 isolation 가정의 정정 링크 |
| `ISAAC_SIM_SMOKE_TEST.md` | shared lifecycle·slot·anchor·종료·Vision 경계 검증 보완 |
| `REGRESSION_REVIEW_REPORT.md` | 본 보고서 및 기능 비교표 |

M0609 아래 경로는 `isaacsim_dual_robot_palletizing/M0609/` 기준이다.
config profile, motion algorithm, RMPFlow 값, controller behavior, topic/service 이름,
Vision 모델/알고리즘 및 asset/folder 이름은 변경하지 않았다.

## 12. 실행한 검증과 남은 실행 검증

`STATICALLY_VERIFIED`:

- `python -m unittest discover -s tests -v`: **17개 통과**(기존 8 + 신규 9).
  baseline에서 신규 핵심 3개는 duplicate B 등록 1 error, stale result 2 failure로
  먼저 실패했으며 수정 후 통과했다.
- `python -m compileall -q isaacsim_dual_robot_palletizing/M0609 Vision tests`: 통과.
  새 bytecode는 임시 cache prefix를 사용했다.
- 저장소 Python 30개 AST parse: 통과. production source injection 미발견.
- common module relative import 순환 검사, Config/entrypoint/dual single reset 구조
  검사: 통과. import 대역으로 전체 common runtime 모듈을 실제 import했다.
- Pyflakes는 wildcard export를 확장해 name resolution을 보완 검사했다.
  `_LAST_SUCTION_GRID_INFO`는 context의 dynamic injection으로 공급되는 두 경고이며
  테스트로 확인했다. `attach_center` fallback 경고는 위의 기존 미해결 항목이다.
  따라서 “undefined symbol 0개”라고 보고하지 않는다.
- repository-wide reference, 원본/Sol 함수 및 profile 상수 비교, `git diff --check`:
  검사 완료. Vision lint/ROS launch/Isaac physics 테스트를 실행한 것으로 세지 않는다.

재실행은 NumPy가 있는 Python에서 위 unittest/compileall 명령을 사용한다.
감사 환경은 임시 설치 위치를 `PYTHONPATH`로 지정했으며 production dependency나
시스템 Python 설치를 변경하지 않았다. 테스트는 Isaac 엔진 대신 import 대역을 쓴다.

`NEEDS_ISAAC_SIM_TEST`: 보완한 [smoke checklist](../ISAAC_SIM_SMOKE_TEST.md)의
A standalone, B standalone, dual 순서로 실제 확인해야 한다. 특히 A/B 각 robot
등록 → 올바른 suction body0/body1 → lift/swing → marker/yaw/lower → release/home
→ repeat, A forklift, B 병렬 lower, camera publish, gate까지 완료 전에는
실행 동등성 검증 완료로 표시하면 안 된다.

Git commit/push/remote 변경, reset/clean/history rewrite는 수행하지 않았다.
baseline commit과 검수 결과를 보존했다.
