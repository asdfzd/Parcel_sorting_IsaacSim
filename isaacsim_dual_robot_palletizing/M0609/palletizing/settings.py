"""Shared M0609 thresholds and motion settings.

This module is deliberately free of Isaac/Omni imports and side effects. Cell-specific
paths and behavior switches are overlaid by :mod:`palletizing.worker` for each worker.
Numeric values are copied from the validated standalone A baseline without tuning.
"""

from pathlib import Path
import math

import numpy as np

_THIS_DIR = Path(__file__).resolve().parent.parent

# Replaced with the owning entrypoint's instance before any runtime helper runs.
simulation_app = None
SUPPRESS_NON_CRITICAL_KIT_WARNINGS_122 = True

# ╔══════════════════════════════════════════════════════════════╗
# ║  A. Task 파라미터                                             ║
# ╚══════════════════════════════════════════════════════════════╝
# 원본 M0609 USD를 로드한다.
# RG2 삭제와 VGC10 부착은 실행 중 Stage에서만 수행되므로 원본 USD 파일은 수정되지 않는다.
PROJECT_DIR = _THIS_DIR.parent
# Conveyor 프로젝트 구조 기준:
# Conveyor/
# ├── M0609/      ← 이 py 파일 위치
# ├── conveyor/   ← 환경 USD 모음 폴더
# ├── oriA/OriBoxA.usda
# └── oriB/OriBoxB.usda
#
# 절대경로를 쓰지 않는다. Conveyor 폴더 전체를 다른 PC/다른 위치로 옮겨도
# M0609/ 과 conveyor/ 의 상대 구조만 유지하면 그대로 동작한다.
def _resolve_first_existing_path(candidates, label="file"):
    for p in candidates:
        pp = Path(p)
        if pp.exists():
            return str(pp)
    return str(Path(candidates[0]))

CONVEYOR_USD_CANDIDATES = [
    # 122_: 사용자가 새로 수정한 작업환경 USD를 우선 사용한다.
    # py 위치가 .../Collected_Conveyor_lift/M0609 이면 PROJECT_DIR은 .../Collected_Conveyor_lift 이다.
    # 따라서 새 USD는 보통 ~/Videos/Collected_Conveyor_lift/Conveyor_lift.usd 에 두면 된다.
    PROJECT_DIR / "Conveyor_lift.usd",
    _THIS_DIR / "Conveyor_lift.usd",

    # fallback: 새 파일이 없으면 기존 표준 USD 사용.
    PROJECT_DIR / "Conveyor_lift.usd",
    _THIS_DIR / "Conveyor_lift.usd",
]
CONVEYOR_USD_PATH = _resolve_first_existing_path(CONVEYOR_USD_CANDIDATES, "Conveyor USD")
USD_PATH        = CONVEYOR_USD_PATH
# 40_ 핵심 변경:
# 70_ 핵심 변경:
# 새 Conveyor_lift.usd에서는 1번 로봇이 /World/m0609_A 이다.
# 지금은 1번 로봇만 동작 테스트하고, /World/m0609_B 는 idle 로봇으로 둔다.
ACTIVE_ROBOT_ROOT_PATH = "/World/m0609_A"
ROBOT_PRIM_PATH = ACTIVE_ROBOT_ROOT_PATH + "/m0609"
ROBOT_OBJECT_NAME = "m0609_robot"
EE_LINK_NAME    = "link_6"

# 기존 RG2 집게 삭제 대상. 7_ 버전은 reference prim까지 강제 제거/비활성화한다.
OLD_GRIPPER_PRIM_PATH = ROBOT_PRIM_PATH + "/onrobot_rg2ft"

# idle 로봇 설정: /World/m0609_* 중 active가 아닌 로봇에 VGC10 visual을 붙이고 제어하지 않는다.
ATTACH_VGC10_TO_IDLE_M0609_ROBOTS = False  # 122_: 새 USD 환경 보존. 2번/idle 로봇에 VGC10 visual을 새로 붙이지 않음
IDLE_M0609_ROOT_PREFIX = "m0609_"
IDLE_VGC10_ROOT_PREFIX = "/World/vgc10_visual_idle"

# 실제 VGC10 CAD를 Blender/Isaac 등에서 변환한 USDA 경로.
# .usda는 ASCII 형태의 USD라서 .usd로 변환하지 않아도 AddReference로 바로 붙일 수 있다.
# 핵심 수정: VGC10을 link_6/tool0 하위에 직접 넣지 않고 /World 아래 visual-follow 모델로 둔다.
# 이렇게 해야 CAD mesh/collider가 로봇 articulation에 섞이면서 바닥을 밀거나 집는 현상을 피할 수 있다.
VGC10_USD_PATH  = str(_THIS_DIR / "assets/gripper_vgc10_v1.usda")
VGC10_FALLBACK_PATHS = [
    str(_THIS_DIR / "assets/gripper.usda"),
    str(_THIS_DIR / "assets/gripper_vgc10.usd"),
    str(_THIS_DIR / "assets/gripper_vgc10_v1.usda"),
]
VGC10_PRIM_PATH = "/World/vgc10_visual_follow"
# scale이 확실히 적용되도록, 움직이는 root와 스케일 적용 mount를 분리한다.
# root: tool0 월드 위치/자세를 따라감
# mount: gripper 모델에 local translate/rotate/scale 적용
# model: 실제 gripper.usda reference
VGC10_MOUNT_PATH = "/World/vgc10_visual_follow/vgc10_scaled_mount"
VGC10_MODEL_PATH = "/World/vgc10_visual_follow/vgc10_scaled_mount/gripper_model"
VGC10_FOLLOW_TARGET_PATH = ROBOT_PRIM_PATH + "/" + EE_LINK_NAME + "/tool0"

# 106_: 카메라 복구 설정.
# 105번까지는 오류 회피용으로 /World/rsd455 자체를 비활성화했지만,
# 이제 rsd455가 필요하므로 카메라 prim은 active 상태로 유지한다.
RSD455_ROOT_PATH = "/World/rsd455"
KEEP_RSD455_CAMERA_ACTIVE = True
DISABLE_LEGACY_ROBOT_CAMERA_GRAPHS = False  # 122_: 새 USD에 저장한 camera_graph/환경을 비활성화하지 않음
DISABLE_WORLD_RSD455_PRIM = False

# 109_: 사용자가 Stage에서 선택한 Cube가 py 실행 후 화면에서 사라지는 문제 방지.
# 사진 기준으로 Prim Path가 /Cube로 보이므로 /Cube와 /World/Cube를 모두 강제로 visible/active 처리한다.
FORCE_SHOW_CUBE_PRIMS_109 = True
CUBE_VISIBLE_CANDIDATE_PATHS_109 = ("/Cube", "/World/Cube", "/World/APalt")  # 122_: APalt도 실행 중 active/visible 보장

# 146_: 새 USD에서 Prim Path가 Environment인 환경 오브젝트들이 145 실행 후 안 보이는 문제 방지.
# /World에 USD를 reference하면 원본 /Environment가 /World/Environment로 들어올 수 있으므로 둘 다 처리한다.
FORCE_SHOW_ENVIRONMENT_PRIMS_146 = True
ENVIRONMENT_VISIBLE_CANDIDATE_PATHS_146 = (
    "/Environment",
    "/World/Environment",
    "/World/environment",
    "/World/ENVIRONMENT",
)
ENVIRONMENT_VISIBLE_NAME_KEYWORDS_146 = ("Environment", "environment", "ENVIRONMENT")
ENVIRONMENT_FORCE_PURPOSE_DEFAULT_146 = True


# VGC10 local pose 보정값. 모델 축/크기에 따라 여기만 조정하면 된다.
# gripper.usda는 네가 확인한 기준으로 X=-90도가 제대로 보이는 방향이다.
VGC10_LOCAL_TRANSLATE = np.array([0.0, 0.0, 0.0], dtype=float)
VGC10_LOCAL_ROTATE_XYZ = np.array([-90.0, 0.0, 0.0], dtype=float)
VGC10_LOCAL_SCALE = np.array([0.001, 0.001, 0.001], dtype=float)

# VGC10 실제 흡착 기준점 보정값.
# 이제 scripted_suction_body / scripted_suction_direction_marker visual은 만들지 않는다.
# 이 값은 /World/vgc10_visual_follow 기준 local offset이며, tool0/VGC10을 따라 같이 움직인다.
# 안 붙으면 VSCode에서 이 값만 조정한다.
VGC10_SUCTION_POINT_PATH = "/World/vgc10_visual_follow/vgc10_suction_point"
# GUI에서 맞춘 suction point 위치. 이 값은 VGC10 기준 local offset이다.
VGC10_SUCTION_LOCAL_OFFSET = np.array([0.0, 0.0, 0.12476], dtype=float)

# 51_ 핵심: 단일 흡착점이 아니라 VGC10 기준 3x3 흡착점 그리드를 코드에서 만든다.
# 전체 폭은 8cm x 8cm. 박스 윗면을 너무 넓게 벗어나지 않으면서도 한 점 흡착보다 안정적으로 판정한다.
SUCTION_GRID_ENABLED = False  # 59_: 9점 판정 중지. 중심 흡착점 1개 + 넓은 흡착 반경으로 복구
SUCTION_GRID_ROOT_PATH = "/World/vgc10_visual_follow/vgc10_suction_grid"
SUCTION_GRID_SPACING_XY = 0.060  # 59_: 디버그용만. 실제 판정은 중심 흡착점 1개로 처리
SUCTION_GRID_MARKERS_VISIBLE = True
SUCTION_GRID_MARKER_RADIUS = 0.007
SUCTION_GRID_ATTACH_MIN_POINTS = 1  # 59_: 중심 흡착점 1개 판정
SUCTION_GRID_ATTACH_REQUIRE_CENTER = True
SUCTION_GRID_LOG_INTERVAL = 1  # 52_: 흡착점이 제대로 동작하는지 매 step 가까이 로그 확인
SUCTION_GRID_LOG_EVERY_STEP = True  # 52_: p00~p22 hit 여부를 확실히 확인
SUCTION_GRID_VERBOSE_POINTS = True

# 59_: 9점 흡착 판정을 중지하고, 중심 흡착점 1개를 넓은 원형 흡착 패드처럼 취급한다.
# - 실제 흡착 위치는 p_center 1개
# - XY는 박스 윗면 중심 근처인지 확인
# - Z는 실제 윗면에 거의 닿았는지 확인
CENTER_SUCTION_SINGLE_POINT_MODE = True
CENTER_SUCTION_EFFECTIVE_RADIUS_XY = 0.085  # m. 실제 패드가 넓다고 가정하는 반경
CENTER_SUCTION_CENTER_TOL_XY = 0.055        # m. 중심점이 box_top 중심에서 허용되는 오차
CENTER_SUCTION_LOG_INTERVAL = 1


# 51_ 핵심: 흡착 성공 시 박스 물리를 끄지 않고 link_6와 박스 rigid body를 FixedJoint로 연결한다.
# 실패하면 joint path/body 경로 로그가 뜬다.
PHYSICS_FIXED_JOINT_ATTACH_ENABLED = True
PHYSICS_ATTACH_JOINT_PATH = "/World/vgc10_physics_attach_fixed_joint"
PHYSICS_ATTACH_BODY0_LINK_NAME = "link_6"
PHYSICS_ATTACH_BODY1_USE_BOX_PRIM = True
PHYSICS_RELEASE_ZERO_VELOCITY = True

# 흡착 위치 확인이 필요할 때만 True로 바꾼다. 기본값 False라 화면에 빨간/흰 마커가 안 보인다.
DEBUG_SHOW_SUCTION_POINT = False
VGC10_SUCTION_DEBUG_MARKER_PATH = "/World/vgc10_suction_debug_marker"

# 29_ 정리: 목표 위치 표시용 goal_marker는 더 이상 만들지 않는다.
# USD에 이미 들어있어도 실행 시 숨김/비활성화한다.
SHOW_GOAL_MARKER = True  # 122_: 새 USD에서 직접 배치한 goal_marker를 삭제/숨김 처리하지 않음
GOAL_MARKER_PATH = "/World/goal_marker"

# VGC10 CAD/USD는 화면에만 보여주고 물리 충돌/rigid body는 제거한다.
# 그래야 VGC10 모델이 바닥이나 큐브를 밀어 물리가 꼬이지 않는다.
VGC10_VISUAL_ONLY = True

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

# 기존 finger gripper는 완전히 제거한다. 컨트롤러에는 NullGripper를 넘긴다.
CUBE_STATIC     = 1.2
CUBE_DYNAMIC    = 1.0

# 흡착 시연 로직. 실제 visual 마커는 만들지 않고, VGC10 suction point 좌표만 계산한다.
# 기존 scripted_suction_body / scripted_suction_direction_marker는 생성하지 않는다.
SUCTION_BODY_PATH = "/World/scripted_suction_body"              # 이전 버전 청소용 경로
SUCTION_MARKER_PATH = "/World/scripted_suction_direction_marker" # 이전 버전 청소용 경로
TARGET_CUBE_PATH = "/World/OriBoxA_01"  # 기존 코드 호환용 이름. 72_에서는 실제 대상도 root BOX_PRIM_PATH를 사용한다.
SUCTION_OFFSET_FROM_EE = np.array([0.0, 0.0, -0.12])  # fallback 전용
# 23_ 기준: suction_pos 계산용 안전 하한.
# 주의: 이 값은 로그/판정용 suction_pos 하한이고, VGC10 visual 자체의 실제 위치를 멈추는 값은 아니다.
SUCTION_MIN_Z = 0.070
# 네가 직접 맞춘 값. 이제 이 값은 고정하고, 흡착 타이밍은 should_script_attach()의 ok 조건만 조정한다.
SUCTION_HOLD_OFFSET = np.array([0.0, 0.0, -0.056])
# 21_ 수정: 실제 흡착은 접촉 직전에만 붙는 게 아니라 약간 떨어진 거리에서도 빨아들일 수 있다.
# 그래서 event=1 후반부터 흡착 판정을 허용한다.
# 단, hold_error 기준을 사용해서 너무 멀거나 너무 눌러 들어간 상태는 막는다.
PICK_CLOSE_EVENTS = {1, 2, 3}
RETRY_CLOSE_EVENTS = {4}
FAIL_IF_NOT_ATTACHED_EVENT = 5

# 23_ 핵심: 마지막 place 하강(event=6)에서 큐브를 계속 따라 내리면
# VGC10 visual이 큐브를 뚫고 내려가는 것처럼 보인다.
# 그래서 event=6에서 suction point가 이 높이 이하로 내려오면 바로 목표 위치에 내려놓고 정지한다.
PLACE_RELEASE_EVENT = 5  # 23_: event=5에서 반대편 XY 근처에 도착하면 바로 흡착 해제
PLACE_RELEASE_SUCTION_Z = 0.095
PLACE_APPROACH_MIN_CUBE_Z = None  # CUBE_INIT_POS 정의 후 아래에서 설정
PAUSE_AFTER_RELEASE = False
RETURN_HOME_AFTER_RELEASE = True
HOME_RETURN_STEPS = 160

# ╔══════════════════════════════════════════════════════════════╗
# ║  B. Controller 파라미터                                       ║
# ╚══════════════════════════════════════════════════════════════╝
M0609_URDF_PATH           = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

CUBE_INIT_POS = np.array([1.92428, -4.20444, -0.41169])  # fallback/screenshot local translate, 실제 판정은 bbox world 사용
CUBE_HALF_Z   = 0.12  # ori box scale 2 기준 대략 half height fallback
PLACE_APPROACH_MIN_CUBE_Z = float(CUBE_INIT_POS[2] + 0.010)
GOAL_POS      = np.array([0.0, -0.45, 0.0])  # 새 코드에서는 박스 기준 상대 offset으로 사용
# 21_ 기준 유지: 0.15는 너무 낮아서 흡착 전에 VGC10이 큐브를 뚫었다.
# 0.17부터 테스트한다. 더 내려가야 하면 0.165, 아직 뚫으면 0.175로 조정한다.
EE_OFFSET     = np.array([0.0, 0.0, 0.17])

EVENTS_DT = [
    0.008,   # 0. 접근 이동
    0.005,   # 1. 하강
    0.35,    # 2. 흡착 ON 단계: dt는 작을수록 느리다. 0.35면 약 3 step 후 다음 event
    0.80,    # 3. 흡착 유지 대기 최소화: 바로 다음 단계로 넘어가게 함
    0.025,   # 4. retry window: 흡착 실패 시 바로 출발하지 않고 근접 상태를 조금 더 유지
    0.01,    # 5. Place 위치로 이동
    0.0025,  # 6. 하강: 23_에서는 너무 낮아지기 전에 조기 release + pause로 침투를 막는다.
    0.2,     # 7. 흡착 OFF 대기 단축
    0.008,   # 8. 상승
    0.08,    # 9. 복귀
]

# ╔══════════════════════════════════════════════════════════════╗
# ║  C. Conveyor / OriBoxA 대상 설정                              ║
# ╚══════════════════════════════════════════════════════════════╝
# 사용자가 USD에서 정리한 prim path.
# 72_ 핵심: 이제 child를 선택하지 않는다. 실제 좌표/정지/흡착/이동 기준은 OriBox root이다.
# 전제 USD 구조:
#   /World/OriBoxA_01 또는 /World/OriBoxB_01 = Rigid Body 있음, 실제 물리 박스 대표 좌표
#   하위 Small_Cardboard_box = Collider만 있음, Rigid Body 없음, local translate=(0,0,0)
BOX_PRIM_PATH = "/World/OriBoxA_01"
STOP_CHECK_PRIM_PATH = BOX_PRIM_PATH
BOX_ROOT_PATH = BOX_PRIM_PATH
BOX_MOVE_PRIM_PATH = BOX_ROOT_PATH
ORI_BOX_USD_PATH = _resolve_first_existing_path([PROJECT_DIR / "oriB" / "OriBoxB.usda", PROJECT_DIR / "oriA" / "OriBoxA.usda"], "OriBox USD")

# 사진에 보이는 값. 이 값은 "참고/생성 fallback"이다.
# 실제 흡착 판정은 이 숫자를 맹신하지 않고 root BOX_PRIM_PATH의 월드 bbox를 매 프레임 계산한다.
BOX_SCREENSHOT_LOCAL_TRANSLATE = np.array([1.92428, -4.20444, -0.41169], dtype=float)
BOX_SCREENSHOT_LOCAL_ROTATE_XYZ = np.array([0.23, 0.203, -0.001], dtype=float)
BOX_SCREENSHOT_LOCAL_SCALE = np.array([2.0, 2.0, 2.0], dtype=float)

# 흡착 판정 범위.
# 10_ 핵심: 큰 박스는 "윗면 중심점"이 아니라 "윗면 면적 안"으로 들어왔는지 봐야 한다.
# 기존 xy<=0.085 방식은 suction point가 박스 윗면 위에 있어도 중심에서 8.5cm 이상이면 실패했다.
# 로그상 suction point는 박스 윗면 안쪽에 들어왔지만 xy=0.09~0.10 정도라 계속 gate=False가 났다.
BOX_ATTACH_XY_TOL = 0.085       # 이전 중심점 방식 로그용 값. 10_에서는 판정 핵심으로 쓰지 않는다.
BOX_ATTACH_Z_MIN = -0.004       # 55_: 실제 윗면 흡착. 윗면을 살짝 파고드는 오차만 허용.
BOX_ATTACH_Z_MAX = 0.018        # 55_: 윗면보다 2.6cm 이내일 때만 흡착. 10cm 위 흡착 금지.
BOX_ATTACH_DIST_TOL = 0.080     # 55_: 로그/보조용. 실제 판정은 9점 top rectangle + z_gap.
BOX_TOP_SURFACE_MARGIN_X = 0.030  # 18_: 가장자리/옆면 흡착 방지. X 가장자리 3cm 제외.
BOX_TOP_SURFACE_MARGIN_Y = 0.055  # 18_: 옆면 흡착 방지. Y 가장자리 5.5cm 제외.
BOX_ATTACH_ON_TOP_SURFACE = True
PHYSICS_ATTACH_TOP_SURFACE_EPS = 0.002  # 55_: FixedJoint anchor는 흡착점 평균 높이가 아니라 실제 박스 윗면 z+3mm에 둔다.
# 흡착 실패로 계속 하강해 박스를 관통하는 것을 막기 위한 안전 정지.
PAUSE_IF_SUCTION_PENETRATES_WITHOUT_ATTACH = True
SUCTION_PENETRATION_STOP_Z_GAP = -0.030  # 18_: 윗면 기준 3cm 이상 파고들면 바로 정지.
# 11_ debug: scripted suction이 실제로 박스 bbox를 움직였는지 매 프레임 확인한다.
DEBUG_MOVE_PARENT_AND_FOLLOW = True
FOLLOW_ERROR_WARN_TOL = 0.030
MOTION_PROBE_DELTA_Z = 0.030

# 목표 위치는 박스 현재 위치 기준 상대 이동으로 잡는다.
# 예: [0.0, -0.45, 0.0]이면 현재 박스를 Y- 방향으로 45cm 옮겨 내려놓는다.
# 21_: 목표 위치는 "로봇 중심을 기준으로 박스 현재 위치의 반대편"으로 자동 계산한다.
# 예: robot_xy를 기준으로 box_xy를 대칭 이동한 좌표에 놓는다.
GOAL_MODE = "MIRROR_ACROSS_ROBOT_CENTER"
GOAL_OFFSET_FROM_BOX_CENTER = np.array([0.0, -0.45, 0.0], dtype=float)  # fallback: robot center를 못 읽을 때만 사용
GOAL_Z_OFFSET_FROM_PICK_CENTER = 0.280  # 23_: 반대편으로 이동할 때 바닥 높이가 아니라 들어올린 높이를 목표로 둔다.
PLACE_RELEASE_XY_TOL = 0.110          # 23_: XY 기준으로 반대편 좌표 근처에 오면 release
PLACE_RELEASE_Z_TOL = 999.0           # 23_: release 판정에서 Z는 직접 쓰지 않는다. 하강을 기다리면 박스가 끌려 내려감

# 15_ 핵심: 박스 USD 원점/루트가 바닥 기준이어도 bbox로 실제 윗면을 계산한다.
# 로봇 pick 목표는 박스 중심이 아니라 top_center + clearance로 올린다.
USE_BOX_TOP_CENTER_AS_PICK_TARGET = True
BOX_PICK_TOP_CLEARANCE = 0.090  # 18_: 옆면이 아니라 윗면에서 잡도록 pick 목표 높이를 더 올림.

# 16_ 핵심: 네가 M0609/발판 위치를 다시 조정한 뒤 로그상 suction point가
# 박스 윗면 기준으로 X는 +7cm, Y는 -25cm 정도 빗나갔다.
# controller에 넣는 picking_position을 반대로 보정해서 VGC10 흡착점이 박스 윗면 중앙으로 오게 한다.
# 로그 기준: dx=suction_x-box_top_x≈+0.07, dy≈-0.25 → pick target 보정=[-0.07,+0.25,0]
PICK_TARGET_MANUAL_OFFSET = np.array([0.0, 0.0, 0.0], dtype=float)  # 53_: manual offset 대신 pre-align에서 box_top 좌표로 직접 정렬

# 흡착이 되면 place까지 가지 않고, 박스를 붙인 상태로 로봇을 초기 자세(0 joint, 일자로 뻗은 상태)로 복귀한다.
RETURN_HOME_IMMEDIATELY_AFTER_ATTACH = False
KEEP_BOX_ATTACHED_DURING_HOME_RETURN = False

# home 복귀 목표. M0609 joint 수가 6이므로 기본은 0 자세.
HOME_TARGET_JOINTS_CONFIG = None  # None이면 np.zeros_like(current_joints) 사용

# release 후 로봇 복귀
RETURN_HOME_AFTER_RELEASE = True
HOME_RETURN_STEPS = 160

# 19_ 핵심: 흡착 후 박스를 목표 위치로 텔레포트/강제 배치하지 않는다.
# VGC10이 visual-only라서 흡착 중에는 scripted carry가 필요하지만,
# 일정 높이만 들어올리면 바로 release하고 박스 물리를 다시 켠 뒤 로봇만 홈으로 복귀한다.
# 21_: 20_의 핵심은 유지한다. 잡자마자 바로 옆으로 끌지 않고 먼저 들어 올린다.
# 다만 20_처럼 수직 상승 후 바로 release하지 않고, 반대편 goal 근처에서 release한다.
RELEASE_AFTER_LIFT_DELTA_Z = 0.060
RELEASE_AFTER_ATTACH_MIN_STEPS = 8
RELEASE_AT_CURRENT_POSE = True  # goal 근처에 도달했을 때 현재 pose에서 흡착 해제. 시작/스폰 위치 강제 이동은 하지 않음.

# 23_ 핵심: 흡착 후에는 PickPlaceController event를 더 진행하지 않고,
# RMPFlow로 suction point 목표를 직접 나눈다.
# 흐름: 위로 들어올림 -> 높은 위치에서 로봇 중심 반대편 XY로 이동 -> 목표 위에서 살짝 하강 -> release -> robot home.
CUSTOM_CARRY_AFTER_ATTACH = True
CUSTOM_CARRY_SAFE_SUCTION_Z = 1.18          # 51_: 9점 물리 흡착 테스트에서도 안전 높이 기준으로 사용
CUSTOM_LIFT_DELTA_Z = 0.16                  # 49_: 너무 높게 들지 않고 BoxAprop 위로 이동 가능한 높이까지만 상승.
CUSTOM_LOWER_CLEARANCE_Z = 0.010            # release 직전 박스 윗면과 suction 사이 여유. 음수면 눌러 들어감.
CUSTOM_LIFT_TOL = 0.060  # 49_: 1.42m 근처까지 오면 다음 phase로 넘어가게 완화
CUSTOM_MOVE_XY_TOL = 0.080  # move phase 완료 기준. 너무 작으면 이동 phase가 max step으로만 넘어간다.
CUSTOM_LOWER_TOL = 0.035
CUSTOM_PHASE_MAX_STEPS = 900  # 50_: slot까지 자연스럽게 더 접근할 시간을 준다
CUSTOM_RELEASE_AFTER_LOWER_MIN_STEPS = 20
CUSTOM_LOG_INTERVAL = 55  # 128_: 불필요한 custom phase 로그 축소

# 49_ 핵심: BoxAprop 적재에서 로봇이 slot XY에 완전히 못 붙으면 lower phase가 무한 대기했다.
# lower 진입은 조금 더 넓게 허용하고, 그래도 못 내려가면 timeout 후 slot 중심으로 snap-release한다.
STACK_DIRECT_LOWER_ALLOW_XY_TOL = 0.120
STACK_RELEASE_ON_LOWER_TIMEOUT = True  # 50_: timeout이면 현재 위치에서 release, snap은 하지 않음


# 53_ 핵심: 작업영역에 들어온 박스의 bbox top_center 좌표를 읽은 뒤,
# PickPlaceController로 대략 접근만 하는 대신 suction 중심을 box_top 위로 직접 정렬한다.
# 로그에서 dx≈0.27m로 빗나간 문제가 있었기 때문에, 흡착 판정 전에 XY 정렬을 먼저 끝낸다.
PRE_ATTACH_ALIGN_ENABLED = True
PRE_ATTACH_ALIGN_XY_TOL = 0.030          # suction 중심과 box_top 중심의 XY 허용 오차
PRE_ATTACH_ALIGN_Z_TOL = 0.040           # 55_: XY 정렬 단계에서 높이 허용 오차
PRE_ATTACH_ALIGN_HEIGHT = 0.080          # 55_: XY 정렬은 box_top보다 8cm 위에서 먼저 수행
PRE_ATTACH_CONTACT_GAP = 0.010           # 55_: 실제 윗면 접촉용. box_top보다 1.8cm 위까지 접근.
PRE_ATTACH_MIN_STEPS_BEFORE_ATTACH = 8   # 너무 빠른 오검출 방지
PRE_ATTACH_MAX_STEPS = 900               # 여기까지 안 맞으면 home return 후 재시도
PRE_ATTACH_LOG_INTERVAL = 10
PRE_ATTACH_EVALUATE_GRID_EVERY_STEP = True

# 55_ 핵심 수정:
# 53_에서는 pre-align 시작 순간의 EE 방향을 고정해서, VGC10/흡착점이 하늘을 보는 자세로
# 위치 이동을 시도했다. 그 결과 box_top 좌표는 읽지만 suction XY가 계속 빗나갔다.
# 이번 버전은 pre-align 동안 orientation constraint를 빼고 위치를 먼저 맞춘다.
# 즉, 흡착 전에는 '하늘 보는 방향 고정'을 하지 않고 box_top 위 좌표로 suction 중심을 맞춘다.
PRE_ATTACH_KEEP_CURRENT_ORIENTATION = False  # 109_: pre-align은 위치 우선, attach 후 carry에서 orientation lock

# 55_: 시각/판정용 9점 흡착 그리드는 실제 USD prim의 기울어진 local pose 대신
# 현재 suction 중심 기준 world XY 평면에 가상으로 만든다.
# 목적: 상자 윗면 판정이 gripper가 하늘을 보거나 기울어진 자세에 끌려가지 않게 하기 위함.
SUCTION_GRID_EVALUATE_AS_WORLD_XY_GRID = True
SUCTION_GRID_WORLD_MARKER_ROOT_PATH = "/World/vgc10_world_xy_suction_grid_debug"

# 완전 대칭 좌표가 로봇 도달 범위를 벗어나면 박스 시작점 기준 이동 거리를 제한한다.
# 목표 좌표가 너무 멀면 goal_xy_error가 계속 1m 이상으로 남고 release가 안 되므로 안전장치로 둔다.
MIRROR_GOAL_MAX_XY_DISTANCE_FROM_PICK = 0.65
ROBOT_CENTER_CANDIDATE_PATHS = [
    # 123_: 새 USD에서는 /World/Cube가 로봇 중심 기준이 아니어서 goal_angle이 -6도만 계산됨.
    # goal_angle을 다시 쓸 때도 실제 1번 로봇 기준으로 계산되도록 Cube를 후보에서 제외한다.
    ROBOT_PRIM_PATH + "/base_link",
    ROBOT_PRIM_PATH,
    ACTIVE_ROBOT_ROOT_PATH,
]

# 24_ 핵심: 로봇/발판 큐브를 가로질러 직선으로 이동하면 박스와 팔이 관통처럼 보인다.
# 실제 충돌 회피가 아니라 RMPFlow 목표점을 여러 waypoint로 나누어 발판 주변을 돌아가게 만든다.
CARRY_AVOID_BASE_AND_ARM = False  # 49_: 3개 적재에서는 우회 waypoint가 오히려 멀어져서 비활성화
CARRY_FORBIDDEN_RADIUS_XY = 0.58        # 로봇 중심 주변 금지 반경. 큐브+로봇팔 주변을 대략 피한다.
CARRY_ROUTE_MARGIN_XY = 0.24            # 금지 반경 바깥으로 추가 여유.
CARRY_ROUTE_SIDE_SIGN = 1.0             # +1 또는 -1. 경로가 이상하면 -1로 바꿔 반대쪽으로 돌아가게 한다.
CUSTOM_MIN_SUCTION_Z_FOR_XY_MOVE = 0.0 # 59_: move_stack 중 target_suction이 현재 위치로 튀는 guard 완전 비활성화
CUSTOM_LOWER_ONLY_AT_GOAL = True        # 목표 XY 근처에 도착하기 전에는 절대 하강하지 않는다.


# 28_ 핵심: 벽에 닿아 멈춘 박스를 흡착한 뒤, 먼저 위로만 빼내되 27_보다 lift 대기 시간을 줄인다.
# 박스가 벽 높이/베이스 주변보다 충분히 올라가기 전까지 joint_1 회전을 금지한다.
# 흐름: 흡착 -> joint_2/3 수직 lift 우선 -> 높이 조건 만족 후 joint_1 반 바퀴 회전 -> 살짝 내림 -> release.
# link_1/link_2의 transform을 직접 수정하지 않는다. articulation에서는 joint target을 줘야 한다.
# 47_ 핵심: 쌓기 목표 좌표는 BoxAprop 위의 슬롯 좌표이므로
# joint swing(반 바퀴 회전) 대신 Cartesian waypoint가 task.goal_center를 따라가게 한다.
# 109_: 상자를 기울이지 않고 수직으로 들기 위해 관절 delta(JOINT_SWING)가 아니라
# RMPFlow/IK에 suction position + fixed orientation을 함께 준다.
# attach 순간의 EE orientation을 유지하므로 link_6가 기울면서 박스를 같이 기울이는 문제를 줄인다.

# =============================================================================
# 170_: 164 안정판 기반 release 직전 APalt pose 진단 전용 버전.
#       흡착/운반/release 흐름은 변경하지 않고 로그만 추가.
# =============================================================================
# 122_ / 117_ SIMPLE 3-STEP MODE - JOINT_1 ROTATE
# 목적: 복잡한 boxaprop/path_move/자동 다음 박스 반복 제거.
# 흐름: (1) 수직 상승 -> (2) joint_1/link_1 회전으로 뒤쪽 큐브 방향 이동 -> (3) 같은 높이만큼 역방향 하강 후 release.
# 주의: 114_에서 joint_2를 돌리면 팔을 접는 동작이 되어 상자가 뒤쪽으로 가지 않는다.
#      117_는 사용자가 의도한 “뒤쪽으로 보내는 최소 회전”을 joint_1 회전으로 해석한다.
# =============================================================================
CUSTOM_CARRY_MODE = "VERTICAL_JOINT1_REVERSE"  # 122_: 117 동작 유지. 117_: 3단계: 수직 상승 -> joint_1 회전 -> 역방향 하강
# 80_: 테스트 목적은 "물리 적용된 박스가 joint로 붙어서 같이 들리는지" 확인하는 것.
#      팔레타이징 이동/스윙은 일단 빼고, 흡착 -> FixedJoint -> 수직 lift -> release만 한다.
PHYSICS_FIXED_JOINT_LIFT_ONLY_TEST = False  # 108_: lift only가 아니라 lift -> swing -> release까지 진행
PHYSICS_LIFT_KEEP_ATTACHED_AND_PAUSE = False  # 108_: 들어올린 뒤 pause하지 않고 joint_1 swing 후 release까지 진행
PHYSICS_FIXED_JOINT_STABILIZE_STEPS = 55  # legacy 값. 103_ 진단 모드에서는 아래 DIAGNOSTIC_HOLD_STEPS를 사용한다.
PHYSICS_FIXED_JOINT_DIAGNOSTIC_NO_LIFT = False
PHYSICS_DIAGNOSTIC_HOLD_STEPS = 120

# 104_: 원본 USD를 저장하지 않고 실행 중에만 박스 MassAPI / inertia를 명시해 검증한다.
PHYSICS_MASS_INERTIA_DIAG_ENABLED = True
PHYSICS_MASS_INERTIA_DIAG_MASS = 2.0
PHYSICS_MASS_INERTIA_DIAG_USE_BBOX_COM = True
PHYSICS_MASS_INERTIA_DIAG_INERTIA_SCALE = 1.0
PHYSICS_DIAGNOSTIC_SAMPLE_STEPS = {0, 1, 2, 3, 5, 10, 20, 30, 40, 60, 80, 100, 120}
PHYSICS_DIAGNOSTIC_LOG_COLLIDER_TREE = True
PHYSICS_DIAGNOSTIC_LOG_EVERY_STEP_UNTIL = 5
JOINT_CARRY_LOG_INTERVAL = 10
TOP_LOCK_FOLLOW_DESIRED_PATH = False  # 75_: joint_1 회전 중에는 박스가 실제 suction을 따라가게 둔다.
TOP_LOCK_PATH_LIFT_STEPS = 150
TOP_LOCK_PATH_MOVE_STEPS = 1
TOP_LOCK_PATH_LOWER_STEPS = 1
TOP_LOCK_PATH_SETTLE_STEPS = 80

# 59_: 다시 한 단계 낮춘 테스트.
# 목표는 적재가 아니라 “상자 윗면 중심 흡착 → 실제 suction 위치를 따라 수직으로만 들어올리기” 검증이다.
VERTICAL_LIFT_ONLY_TEST = True
VERTICAL_LIFT_DELTA_Z = 0.24  # 109_: RMPFlow fixed-orientation으로 수직 lift 높이
VERTICAL_LIFT_HOLD_STEPS = 130  # 81_: 들어올린 상태 확인 시간
VERTICAL_LIFT_PAUSE_AFTER_SUCCESS = False
# 59_: 수직 리프트 후 BoxAprop 위 좌표까지 회전/이동만 테스트한다. release/stack은 하지 않는다.
VERTICAL_LIFT_THEN_CUBE_OVER_ENABLED = False  # 114_: boxaprop/path_move 사용 금지. 3단계만 수행.  # 109_: 수직 lift 후 Cube/APalt 쪽 이동/하강까지 테스트
CUBE_OVER_MOVE_STEPS = 380  # 109_: orientation 유지한 상태로 천천히 이동
CUBE_OVER_HOLD_STEPS = 40
BOXAPROP_LOWER_STEPS = 170  # 109_: 기울임/튀김 방지용 저속 하강
BOXAPROP_SETTLE_STEPS = 70
BOXAPROP_RELEASE_ON_SETTLE = True

# 117_: 사용자가 원하는 흐름
#   1) 흡착 후 최대한 수직으로 상승: RMPFlow Cartesian Z lift + fixed EE orientation
#   2) 뒤쪽 큐브 방향으로 최소 움직임: joint_1/link_1만 부드럽게 회전
#   3) 내려놓기: 1)의 반대 방향으로 현재 XY에서 수직 하강
# 이 모드는 109/110처럼 목표 XY까지 RMPFlow로 길게 끌고 가지 않는다.
# 따라서 이상한 우회/큰 움직임을 줄이고, “뒤쪽으로 보내는 회전”을 joint_1로 확인한다.
# 124_: 현재 파일은 마지막 위치 회전은 신경쓰지 않고 drop release 확인이 우선이므로 manual_delta 유지.
# APalt와 상자 사이 XY 위치가 맞지 않으면, 이 manual 회전각을 먼저 조절한다.
HYBRID_JOINT1_ROTATE_MODE = "manual_delta"  # goal_angle 대신 수동 회전량 사용
HYBRID_JOINT1_ROTATE_SIGN = 1.0  # 회전 방향이 반대면 -1.0으로 변경
HYBRID_JOINT1_ROTATE_FALLBACK_DEG = 160.0  # joint_1 실제 회전각. APalt보다 덜 가면 +, 지나치면 -
HYBRID_JOINT1_ROTATE_MAX_DEG = 360.0  # 117_: APalt 방향까지 더 회전 허용  # 117_: 115번 120도에서 뒤쪽까지 부족해서 160도까지 허용
HYBRID_JOINT1_ROTATE_STEPS = 240  # 117_: 회전량 증가에 맞춰 더 천천히 수행  # 117_: 회전량 증가에 맞춰 더 천천히 수행
HYBRID_REVERSE_LOWER_STEPS = 170  # 들어올린 높이만큼 반대로 내려놓기.
HYBRID_SETTLE_STEPS = 30  # 내려놓은 뒤 짧게 안정화.
HYBRID_RELEASE_ON_SETTLE = True

# 145_: n개 적재용 회전 패턴. 홀수 번째 박스는 160도, 짝수 번째 박스는 130도.
# 기준: 1번=왼쪽, 2번=오른쪽, 3번=왼쪽 위, 4번=오른쪽 위.
STACK_JOINT1_DEG_PATTERN_145 = (220.0, 155.0, 220.0, 155.0)
BOX_STACK_HEIGHT_145 = 0.24        # 3개 이상 적재할 때 APalt를 내릴 높이. 상자 높이에 맞춰 수정
APALT_LOWER_LOG_INTERVAL_145 = 15  # APalt lowering 중 박스 위치 로그 간격
APALT_LOWER_SPEED_NOTE_145 = "STACK_LOWER_SUPPORT_STEPS 값을 키우면 더 천천히 내려갑니다"


# =============================================================================
# 155_: APalt 정답지 큐브 기반 hybrid palletizing
# 목적:
# - APalt 위에 사용자가 직접 둔 정답지 큐브 위치를 최종 place 기준으로 사용한다.
# - 큰 이동은 기존 link_1 회전 장점을 유지하고, 마지막만 slot marker 좌표로 보정한다.
# - 컨베이어 위 다른 상자보다 충분히 높게 든 뒤 joint_1 회전을 시작한다.
# 정답지 큐브 prim path:
#   /World/APalt_slot_01
#   /World/APalt_slot_02
# 주의: 정답지 큐브는 위치/방향 기준점용이다. Rigid Body/Collider는 없어도 된다.
# =============================================================================
SLOT_MARKER_PALLETIZING_ENABLED_155 = True
SLOT_MARKER_PATHS_155 = (
    "/World/APalt_slot_01",
    "/World/APalt_slot_02",
)
# slot marker가 2개뿐일 때 3번째 박스부터 1/2번을 반복할지 여부.
SLOT_MARKER_WRAP_IF_SHORT_155 = True
# marker 중심이 실제 박스 center 목표라고 본다. 즉 marker를 박스와 같은 크기/높이로 APalt 위에 올려둔다.
SLOT_MARKER_USE_BBOX_CENTER_155 = True
# link_1 회전 후 marker 위로 갈 때, 바로 하강하지 않고 marker 위쪽에서 접근할 높이 여유.
SLOT_MARKER_APPROACH_CLEARANCE_Z_155 = 0.06  # 160_: 불필요한 재상승을 줄임. 실제 no-up은 runtime에서 한 번 더 보정
# marker 위 마지막 보정 이동 step. 너무 작으면 팔이 꺾일 수 있어 180~260 권장.
SLOT_MARKER_FINAL_MOVE_STEPS_155 = 220
SLOT_MARKER_FINAL_LOWER_STEPS_155 = 150
SLOT_MARKER_FINAL_SETTLE_STEPS_155 = 35
# 컨베이어 위 다른 OriBoxA_* 최고 높이보다 이만큼 더 높게 들어올린 뒤 swing 시작.
CONVEYOR_SAFE_LIFT_ENABLED_155 = True
CONVEYOR_SAFE_LIFT_CLEARANCE_Z_155 = 0.22
CONVEYOR_SAFE_LIFT_INCLUDE_PREFIXES_155 = ("/World/OriBoxA_",)
CONVEYOR_SAFE_LIFT_EXCLUDE_ATTACHED_155 = False  # False면 현재 잡은 박스도 높이 계산에 포함. 최고 높이 기준이라 안전함.
# 방향 정렬은 다음 단계에서 안정화한다. 이번 155는 marker 위치 기반 place를 먼저 확인한다.
SLOT_MARKER_YAW_ALIGN_ENABLED_155 = False

# 156_: joint_1 회전 기준 변경.
# 기존 155는 "흡착한 현재 joint_1 + 220도" 방식이라, 집는 위치가 바뀌면 최종 회전각도 같이 바뀌었다.
# 이번 버전은 "로봇 초기화 상태 joint_1 + 목표 각도"를 절대 목표로 사용한다.
# 예: 초기 기준에서 -10도 지점에서 집었고 목표가 +220도라면, 현재 위치에서는 +230도 이동해 최종 +220도에 도달한다.
ABSOLUTE_JOINT1_FROM_INITIAL_ENABLED_156 = True
ABSOLUTE_JOINT1_LOG_156 = True
ROBOT_INITIAL_JOINTS_156 = None

# =============================================================================
# 157_: APalt_slot 정답지 큐브 기반 joint_1 자동 회전량 계산
# 목적:
# - 220/155처럼 손으로 쓴 회전각을 쓰지 않는다.
# - 로봇 base 기준으로 현재 박스 방향(x3)과 목표 slot 방향(x1/x2)을 계산한다.
# - 현재 자세에서 필요한 회전량은 (slot 방향 - 현재 박스 방향)이다.
# - 최종 정확도는 기존처럼 APalt_slot 좌표 보정(move_over/lower/settle)이 담당한다.
# =============================================================================
AUTO_JOINT1_FROM_SLOT_MARKER_ENABLED_157 = True
AUTO_JOINT1_USE_SHORTEST_DELTA_157 = True  # True: -180~+180도 중 짧은 방향으로 회전
AUTO_JOINT1_MAX_ABS_DELTA_DEG_157 = 170.0  # 159에서 slot별 방향 강제 시 AUTO_JOINT1_MAX_ABS_DELTA_DEG_159가 우선
AUTO_JOINT1_MIN_DELTA_DEG_157 = 0.0        # 너무 작은 회전도 허용. 필요하면 5~10도 설정
AUTO_JOINT1_LOG_157 = True

# 158_: joint_1 회전 후 다음 Cartesian 보정 phase가
#       회전 전 lift_suction을 start로 다시 사용하면서 로봇이 되돌아가는 문제를 막는다.
#       slot_marker_move_over/lower/settle phase가 시작될 때 실제 현재 suction 위치를 start로 재설정한다.
PHASE_DYNAMIC_START_AFTER_JOINT1_ENABLED_158 = True
PHASE_DYNAMIC_START_NAMES_158 = (
    "slot_marker_move_over_155",
    "slot_marker_lower_155",
    "slot_marker_settle_155",
)
PHASE_DYNAMIC_START_LOG_158 = True


# =============================================================================
# 159_: slot별 회전 방향 분리 + 정답지 큐브 yaw 정렬
# 목적:
# - APalt_slot_01/02가 서로 다른 쪽으로 접근하게 하여, 이미 놓은 상자를 지나가며 치는 문제를 줄인다.
# - 상자가 컨베이어에서 가로/세로/애매한 방향으로 들어와도 release 직전에 slot marker와 같은 yaw로 맞춘다.
# =============================================================================
SLOT_ROUTE_SIGN_BY_MARKER_159 = (-1.0, +1.0)   # slot_01=음(-)방향, slot_02=양(+)방향. 3/4번은 1/2 반복.
SLOT_ROUTE_SIGN_FORCE_159 = True               # True면 shortest delta 대신 slot별 지정 방향으로 회전한다.
AUTO_JOINT1_MAX_ABS_DELTA_DEG_159 = 260.0      # slot별 방향을 강제하면 180도를 넘을 수 있으므로 157의 170 제한보다 넓게 허용.
SLOT_MARKER_YAW_ALIGN_ENABLED_159 = False  # 160_: release 직전 순간 yaw 보정 금지. 로봇 동작 검증을 위해 snap/teleport 제거.
SLOT_MARKER_YAW_ALIGN_LOG_159 = True
SLOT_MARKER_YAW_ALIGN_ZERO_VEL_159 = True

# 163_: 160 안정판 기반. 흡착 후 joint_6 yaw 보정/pose recenter는 제거한다.
# 164_: 강제 보정 없이 box/slot local X,Y축 yaw 진단을 확장하고, 2번째 release 직후 정지한다.
# 161/162에서 확인된 문제: 흡착 후 wrist 회전은 yaw는 줄여도 box center를 크게 밀었다.
# 이번 버전은 상자/slot yaw를 흡착 전과 release 전에 기록하되, 상자를 순간 회전시키지 않는다.
PRE_PICK_YAW_DIAGNOSTIC_ENABLED_163 = True
PRE_PICK_YAW_OK_TOL_DEG_163 = 5.0
PRE_PICK_YAW_WARN_TOL_DEG_163 = 15.0
POST_PICK_YAW_CORRECTION_DISABLED_163 = True

# =============================================================================
# 171_: release 직전 APalt_slot 정답지 방향 기준 RMPFlow yaw align
# - 170에서 확인된 상태: center_xy_err=0, level OK, yaw만 18.8도 틀어짐.
# - pre-align / suction / link_1 회전 / slot_marker move/lower/settle 흐름은 건드리지 않는다.
# - slot_marker_settle_155 이후, release 직전에만 현재 suction 위치를 유지하고
#   EE target orientation을 APalt yaw 오차만큼 짧게 보정한다.
# - box transform 직접 회전, snap, teleport, joint_6 단독 trim 금지.
# =============================================================================
PRE_RELEASE_YAW_ALIGN_RMPFLOW_ENABLED_171 = True
PRE_RELEASE_YAW_ALIGN_ONLY_IF_CENTER_LEVEL_OK_171 = True
PRE_RELEASE_YAW_ALIGN_STEPS_171 = 160  # 173_: 172 probe에서 검증된 것처럼 충분한 step 동안 실제 RMPFlow action 적용
PRE_RELEASE_YAW_ALIGN_MAX_DEG_171 = 25.0
PRE_RELEASE_YAW_ALIGN_MIN_DEG_171 = 2.0
PRE_RELEASE_YAW_ALIGN_CENTER_ABORT_M_171 = 0.018
PRE_RELEASE_YAW_ALIGN_LEVEL_ABORT_DEG_171 = 3.0
PRE_RELEASE_YAW_ALIGN_LOG_INTERVAL_171 = 20
PRE_RELEASE_YAW_ALIGN_KEEP_SUCTION_POSITION_171 = True
PRE_RELEASE_YAW_ALIGN_INSERT_AFTER_SETTLE_171 = True
PRE_RELEASE_YAW_ALIGN_ABORT_PAUSE_171 = True
PRE_RELEASE_YAW_ALIGN_REQUIRE_AXIS_OK_BEFORE_RELEASE_171 = True  # 173_: yaw 보정 실패 시 release 차단

# =============================================================================
# 184_: 2번째 상자 정렬 시간 단축
# 목적:
# - 기존 173은 slot_marker_settle_155 이후 release 직전에 pre_release_yaw_align_171을
#   별도 160 step 동안 수행했다.
# - 2번째 상자는 로그상 yaw 오차가 약 18.8deg이며, 별도 정렬 phase 55 step 안에 이미
#   거의 0deg까지 줄어든다. 따라서 이 yaw 정렬을 내려가는 slot_marker_lower_155 / settle
#   과정에 흡수해 별도 대기 시간을 제거한다.
# - 박스 transform 직접 회전/snap/teleport는 여전히 금지한다. RMPFlow target orientation만
#   lower/settle 동안 함께 바꾼다.
# =============================================================================
FUSED_SECOND_BOX_YAW_ALIGN_ENABLED_184 = True
FUSED_YAW_ALIGN_ALL_FOUR_NOTE_185 = "185_: 1~4번째 모두 별도 pre_release_yaw_align phase 없이 lower/settle 중 yaw 정렬"
FUSED_YAW_ALIGN_SLOT_INDICES_184 = (0, 1, 2, 3)  # 185_: zero-based. 1~4번째 상자 모두 lower/settle 중 yaw 정렬
FUSED_YAW_ALIGN_PHASE_NAMES_184 = ("slot_marker_lower_155", "slot_marker_settle_155")
FUSED_YAW_ALIGN_START_PHASE_184 = "slot_marker_lower_155"
FUSED_YAW_ALIGN_DISABLE_FINAL_PRE_RELEASE_PHASE_184 = True
FUSED_YAW_ALIGN_REUSE_IN_SETTLE_184 = True
FUSED_YAW_ALIGN_MIN_DEG_184 = 2.0
FUSED_YAW_ALIGN_MAX_DEG_184 = 25.0
FUSED_YAW_ALIGN_CENTER_TOL_M_184 = 0.080  # lower 시작 전 move_over 후에는 이 범위 안이면 orientation 보정 시작
FUSED_YAW_ALIGN_LEVEL_TOL_DEG_184 = 5.0
FUSED_YAW_ALIGN_LOG_184 = True


# 164_: 강제 보정 없이 진단만 확장.
# - box local X/Y축과 APalt_slot local X/Y축 yaw를 모두 찍는다.
# - 두 번째 상자 release 직후 정지해서 로그만 확인한다.
AXIS_YAW_DIAGNOSTIC_ENABLED_164 = True
AXIS_YAW_DIAG_LOG_BBOX_164 = True
DIAG_STOP_AFTER_RELEASE_ENABLED_164 = False  # 185_: 2번째 release 후 진단 정지 금지. FORKLIFT_TRIGGER_COUNT까지 계속 진행
DIAG_STOP_AFTER_RELEASE_COUNT_164 = 4  # 185_: 참고값. 실제 정지는 위 enabled=False라 사용하지 않음
DIAG_STOP_AFTER_RELEASE_SKIP_APALT_LOWER_164 = True

# =============================================================================
# 160_: no-snap yaw + 빠른 place + release 후 안정화 대기
# 목적:
# - 상자를 APalt_slot yaw로 순간 이동/회전시키는 보정을 제거한다.
# - yaw는 실제 로봇 경로/자세가 맞추지 못하면 로그상 실패로 남긴다.
# - joint_1 회전 후 이미 slot 근처에 있으면 move_over에서 다시 위로 살짝 들어올리지 않는다.
# - 2번째 release 직후 바로 APalt를 내리지 않고 1~2초 물리 안정화 시간을 준다.
# =============================================================================
SLOT_MARKER_YAW_SNAP_DISABLED_160 = True
SLOT_MARKER_YAW_CHECK_LOG_160 = True
SLOT_MARKER_NO_UP_BEFORE_RELEASE_160 = True
SLOT_MARKER_SKIP_MOVE_IF_NEAR_XY_160 = True
SLOT_MARKER_SKIP_MOVE_XY_TOL_160 = 0.08
POST_RELEASE_SETTLE_BEFORE_APALT_LOWER_ENABLED_160 = True
POST_RELEASE_SETTLE_BEFORE_APALT_LOWER_STEPS_160 = 120
POST_RELEASE_SETTLE_LOG_INTERVAL_160 = 30


# =============================================================================
# 124_ DROP RELEASE MODE
# 목적: 마지막 위치/회전 상태는 그대로 두고, 흡착 FixedJoint를 제거해서
#       물리 적용된 상자가 중력으로 떨어지는지 확인한다.
# 주의: link_1 Transform을 직접 돌리거나 박스를 순간이동시키지 않는다.
#       기존 117/123 흐름이 끝난 뒤 FixedJoint만 제거한다.
# =============================================================================
DROP_RELEASE_AFTER_SETTLE_124 = True
# 126_: VERTICAL_JOINT1_REVERSE 모드에서는 place_enabled가 False여도 마지막 settle 후 무조건 FixedJoint를 제거한다.
FORCE_DROP_RELEASE_AFTER_SETTLE_126 = True
# 126_: 떨어지는 장면 확인용. 0.10이면 10cm 더 높은 곳에서 흡착을 떼므로 낙하가 보인다.
#       0.00이면 APalt 위에 거의 올려진 상태에서 떼기 때문에 떨어지는 장면이 안 보일 수 있다.
DROP_RELEASE_EXTRA_Z_125 = 0.10
DROP_RELEASE_OBSERVE_STEPS_124 = 0  # 163_: release 후 바로 home return 시작      # 128_: release 후 관찰 step. 핵심 진단 로그만 출력
DROP_RELEASE_LOG_INTERVAL_124 = 60        # 128_: drop 관찰 로그 간격 축소
DROP_RELEASE_RETURN_HOME_AFTER_OBSERVE_124 = True   # 127_: release 후 home 복귀까지 실행
DROP_RELEASE_ZERO_VELOCITY_BEFORE_DETACH_124 = False # True면 떼는 순간 속도 제거 후 순수 중력 낙하

# 127_: 126에서 phase 완료 블록이 실행되지 않으면 release 코드에 못 들어가는 문제가 있었다.
# 그래서 상자가 pick 위치에서 충분히 이동했고, APalt 위쪽에서 일정 step 이상 정지/유지되면
# phase 상태와 무관하게 FixedJoint를 제거하는 안전장치(watchdog)를 둔다.
FORCE_DROP_RELEASE_WATCHDOG_127 = False  # 128_: 중간에 갑자기 흡착 해제될 수 있어 watchdog release 금지
FORCE_DROP_RELEASE_WATCHDOG_STEPS_127 = 30
FORCE_DROP_RELEASE_MIN_XY_MOVE_127 = 0.25      # pick 위치에서 XY로 이 이상 이동하면 최종 위치 근처로 판단
FORCE_DROP_RELEASE_MIN_Z_ABOVE_PICK_127 = 0.06 # pick 높이보다 이 이상 높으면 낙하 확인용 release 가능
FORCE_DROP_RELEASE_OBSERVE_STEPS_127 = 180
FORCE_DROP_RELEASE_LOG_INTERVAL_127 = 20
RETURN_HOME_AFTER_WATCHDOG_DROP_127 = True

# =============================================================================
# 128_ RELEASE DIAGNOSIS MODE
# 목적: 같은 증상이 반복되는 원인을 한 번에 분리한다.
# - 중간 이동 중에는 절대 release하지 않는다.
# - 마지막 settle 완료 지점에서만 release한다.
# - release 전/후 joint 잔존, rigid/kinematic/collision/gravity 상태를 짧게 출력한다.
# - release 직후 상자 root/subtree를 dynamic으로 복구해도 안 떨어지는지 확인한다.
# =============================================================================
RELEASE_DIAG_128_ENABLED = True
RELEASE_DIAG_SCAN_ALL_JOINTS_128 = True
RELEASE_DIAG_FORCE_DYNAMIC_AFTER_DETACH_128 = True
RELEASE_DIAG_RESTORE_COLLISION_128 = True
RELEASE_DIAG_ZERO_VELOCITY_128 = True
RELEASE_DIAG_MAX_ROWS_128 = 24
RELEASE_DIAG_OBSERVE_SAMPLES_128 = (0, 30, 60, 120, 180, 240, 299)
RELEASE_DIAG_CLEAR_DROP_TRACKER_128 = True

# 114_ 호환용 이름. 다른 코드 블록이 이 이름을 참조해도 실행되도록 유지한다.
HYBRID_LINK2_ROTATE_TARGET_DEG = -125.0
HYBRID_LINK2_ROTATE_MIN_DEG = -135.0
HYBRID_LINK2_ROTATE_STEPS = HYBRID_JOINT1_ROTATE_STEPS

# 62_: release 순간에는 PhysX/USD physics 속성 변경 금지. 현재 pose에 그대로 놓고 attach 플래그만 OFF.
BOXAPROP_SAFE_RELEASE_NO_PHYSICS_TOGGLE = True
BOXAPROP_RELEASE_MAX_SUCTION_ERR = 0.65  # 109_: RMPFlow 목표 오차 허용을 조금 넓힘  # 이보다 크면 내려놓기 실패로 보고 release하지 않음
# 67_: BoxAprop 목표로 못 가는데 step timeout만으로 lower/release까지 진행하면
#      두 번째 박스가 엉뚱한 위치에서 release abort로 멈춘다.
#      move/lower/settle 단계에서 목표 오차가 너무 크면 pause하지 않고 실패 처리 후 home 복귀한다.
BOXAPROP_ABORT_HOME_ON_TARGET_ERROR = False  # 109_: 첫 upright RMPFlow 테스트에서는 목표 오차가 있어도 로그 확인 우선
BOXAPROP_MOVE_MAX_SUCTION_ERR = 0.45
BOXAPROP_LOWER_MAX_SUCTION_ERR = 0.45
CUBE_OVER_HEIGHT_MARGIN = 0.000  # lift_suction 높이를 그대로 유지. 더 높게 지나가려면 0.03~0.05 추가.
CUBE_OVER_MAX_TARGET_ERR_WARN = 0.45
VERTICAL_LIFT_MAX_SUCTION_TARGET_ERR = 0.090
JOINT_LIFT_STEPS = 240  # 102_: 안정화 후 더 천천히, 79보다 조금 더 높게 물리 lift 확인
JOINT_SWING_STEPS = 200  # 108_: joint_1 swing을 천천히 수행
JOINT_LOWER_STEPS = 1  # 75_: joint_1 회전 후 j2/j3 lowering 금지. 급격한 꺾임/XY 튐 방지
JOINT_SETTLE_STEPS = 1  # 75_: release 전 대기 최소화

# 들어올리기 관절 보정값.
# M0609 관절 순서가 joint_1~joint_6이면 index 1=joint_2, index 2=joint_3이다.
# 만약 실행했을 때 박스가 내려가면 JOINT_LIFT_SIGN을 -1.0으로 바꿔라.
JOINT_LIFT_SIGN = 1.0
JOINT_LIFT_J2_DELTA_RAD = -0.61   # 108_: ㄱ자 리프트 목표. 초기 link_2 z=-90 기준에서 약 -125도까지 허용/이동
JOINT_LIFT_J3_DELTA_RAD =  0.12   # 108_: 106 성공값에 가깝게 보조. 높이가 부족하면 0.18~0.25로 소폭 증가
JOINT_USE_J3_FOR_LIFT = True      # link_2만 테스트하려면 False로 바꿔라.

# 108_: Stage에서 초기 link_2 Orient Z가 -90도로 보인다는 사용자의 기준을 joint_2 제한으로 변환한다.
# 계산식: link2_z_est_deg = initial_link2_z_deg + sign * degrees(current_j2 - reference_j2)
# 초기 상태에서 reference_j2를 자동으로 읽고, link2_z_est_deg >= min_link2_z_deg가 되도록 joint_2 target을 clamp한다.
LINK2_ORIENT_Z_GUARD_ENABLED = True
LINK2_ORIENT_Z_INITIAL_DEG = -90.0
LINK2_ORIENT_Z_MIN_DEG = -135.0
LINK2_ORIENT_Z_TARGET_DEG = -125.0  # 108_: 계산 목표값. ref_j2 + radians(-125 - -90) ≈ ref_j2 -0.611rad
LINK2_ORIENT_Z_SIGN = 1.0  # 실행 결과 GUI Orient Z 변화 방향이 반대면 -1.0으로 바꾼다.
LINK2_ORIENT_Z_GUARD_LOG = True

# 반대편으로 넘기는 동작. 반대방향이 이상하면 JOINT_SWING_SIGN만 -1.0으로 바꿔라.
JOINT_SWING_SIGN = 1.0
JOINT_SWING_DELTA_RAD = 3.10      # 약 178도. 거의 반 바퀴 회전. 너무 많이 돌면 2.85~3.00으로 낮춘다.
JOINT_SWING_CLAMP_RAD = 6.28      # 너무 과회전 방지

# 내려놓기. lift_delta를 몇 % 되돌릴지 결정한다.
# 1.0이면 들어올린 만큼 거의 전부 내리고, 0.55면 높은 위치에서 살짝 내린 뒤 release한다.
JOINT_LOWER_RETURN_RATIO = 0.0  # 75_: swing 후 j2/j3를 되돌리지 않음. link_1 z축 회전만 사용
JOINT_RELEASE_MIN_BOX_CENTER_Z = 0.0   # 75_: lower phase 비활성화. 높이는 APalt slot snap에서 맞춤
JOINT_RELEASE_MIN_SUCTION_Z = 0.0      # 75_: lower guard 비활성화

# 27_: 벽을 뚫지 않기 위한 핵심 조건. 이 높이 전에는 joint_1 회전 금지.
JOINT_STRICT_VERTICAL_LIFT_BEFORE_SWING = True
JOINT_SWING_START_MIN_BOX_CENTER_Z = 1.02  # 108_: joint_2 guard 때문에 lift 높이가 낮아질 수 있어 임시 기준 완화
JOINT_SWING_START_MIN_SUCTION_Z = 1.14  # 108_: joint_2 guard 상태에서 swing 시작 기준
JOINT_LIFT_EXTRA_HOLD_STEPS = 55

# 28_ 조정 요약:
# - 박스가 벽에 닿아 멈춘 상태를 기준으로, 흡착 직후에는 절대 joint_1을 돌리지 않는다.
# - box/suction 높이가 기준 이상 올라간 뒤에만 joint_1 반 바퀴 회전을 시작한다.
# - 내려놓기도 너무 낮게 하지 않아 벽/큐브/로봇 관통 느낌을 줄인다.
# 조정 팁:
# - 너무 많이 돌면 JOINT_SWING_DELTA_RAD = 2.85~3.00
# - 반대 방향이면 JOINT_SWING_SIGN = -1.0
# - 더 높이 들고 싶으면 J2=-0.54, J3=0.66 쪽으로 조금씩 증가
# - lift 후 오래 멈춘다면 아래 JOINT_SWING_START_MIN_* 값을 조금 낮춘다.

# 로봇을 reset 때 0 자세로 보낼지 여부.
# 처음 팔이 너무 팍 꺾이면 False로 바꿔서 현재 자세 유지부터 테스트해라.
RESET_ROBOT_TO_ZERO = True

# 30_ 핵심: 성공/실패 후 pause로 끝내지 않고 계속 반복한다.
# 성공: release -> home -> 다음 상자 대기
# 실패: attach 실패/관통 위험 -> home -> 현재 박스 위치를 다시 bbox로 읽고 재시도
RUN_CONTINUOUS_LOOP = True  # 135_: release 후 멈추지 않고 home 복귀/대기 루프 계속 진행. forklift 테스트용
LOOP_RETRY_AFTER_ATTACH_FAIL = True
LOOP_IGNORE_RELEASED_BOX_UNTIL_MOVED = False  # 47_: OriBoxA_* root 완료 목록으로 재집기 방지하므로 이동거리 ignore는 끈다.
LOOP_RELEASED_BOX_IGNORE_MOVE_TOL = 0.20  # release된 같은 박스를 바로 다시 집지 않기 위한 XY 이동 기준(m)
LOOP_WAIT_LOG_INTERVAL = 30

# ╔══════════════════════════════════════════════════════════════╗
# ║  D. 박스 정지 후 흡착 설정                                    ║
# ╚══════════════════════════════════════════════════════════════╝
# 핵심 변경: 박스가 움직이는 중에는 로봇이 pick-place를 시작하지 않는다.
# 8_ 수정: "사진에서 선택한 Small_Cardboard_box가 화면상 멈췄는지"를 기준으로 본다.
# 이전 7_는 physics:angularVelocity까지 정지 조건에 넣어서, bbox가 완전히 멈춰도
# angularVelocity 잔류값 때문에 stable=0/25에서 영원히 못 넘어갈 수 있었다.
WAIT_UNTIL_BOX_STOPPED_BEFORE_PICK = True
BOX_STABLE_REQUIRED_STEPS = 25      # 25 step 연속 정지 판정 후 시작. 너무 오래 기다리면 15~20으로 낮춘다.
BOX_STABLE_POS_TOL = 0.0015         # m/step. 박스 bbox 중심 이동량이 1.5mm 이하이면 정지 후보.
BOX_STABLE_LINEAR_VEL_TOL = 0.010   # m/s. 현재는 로그용 기본값. 아래 USE_LINEAR_VEL=True일 때만 gate에 사용.
BOX_STABLE_ANGULAR_VEL_TOL = 0.050  # rad/s. 현재는 로그용 기본값. 아래 USE_ANGULAR_VEL=True일 때만 gate에 사용.

# 정지 판정에 실제로 사용할 항목.
# 사용자 요청 기준: /World/OriBoxA/Small_Cardboard_box prim 하나가 "안 움직일 때"만 본다.
# 그래서 bbox 중심 이동량만 gate에 사용하고, linear/angular velocity는 정확한 원인 확인용 로그로만 출력한다.
BOX_STOP_USE_BBOX_MOVE = True
BOX_STOP_USE_LINEAR_VEL = False
BOX_STOP_USE_ANGULAR_VEL = False

BOX_FREEZE_AFTER_STOP = False       # 18_: 박스는 컨베이어가 굴려오게 둔다. 정지 판정 후에도 강제 kinematic 처리하지 않는다.
# 15_ 핵심: 흡착 후에는 실제 물리 충돌/중력과 scripted 이동이 싸우지 않도록
# 박스 subtree의 rigid body / collision을 잠시 끄고 순수 visual scripted carry로 이동한다.
# VGC10은 visual-only이므로 실제 물리로 박스를 밀 수 없다. 시연 안정성을 위해 이 방식이 가장 확실하다.
BOX_DISABLE_PHYSICS_DURING_CARRY = False  # 51_: 박스 물리 OFF/scripted carry 금지. FixedJoint 물리 연결로 테스트
BOX_REENABLE_PHYSICS_AFTER_RELEASE = False  # 66_: 적재/완료 박스는 다음 사이클에서 물리를 다시 켜지 않는다. root-child 순간이동 방지.
# 56_: FixedJoint로 잡으면 떨어진 body 사이 joint가 스냅/회전을 만들 수 있다.
# 그래서 실제 윗면 판정 후에는 top-lock kinematic carry로 조용히 들고 이동한다.
# 적재된 박스는 kinematic 상태를 유지해서 2x2 스택이 굴러가지 않게 한다.
BOX_KEEP_KINEMATIC_AFTER_RELEASE = False
BOX_STOP_LOG_INTERVAL = 10          # 69_: 새 환경에서는 로그 폭주 방지
BOX_STOP_LOG_EVERY_STEP = False

# 18_ 핵심: 박스가 시작 위치에서 잠깐 정지해 보인다고 바로 pick하지 않는다.
# 박스는 컨베이어가 굴려와서 실제 픽업 구간에 들어온 뒤 멈춰야 한다.
BOX_READY_ZONE_GATE = True
BOX_READY_MIN_Y = -4.00          # 이 y보다 작아진 뒤에만 pick 시작. 시작점 y=-2.8 근처 오검출 방지.
BOX_READY_MAX_CENTER_Z = 1.05    # 공중/초기 스폰 높이에서 정지 판정되는 것 방지. 실제 벨트 위 center z≈0.888.
RESET_BOX_TO_USD_START_ON_PLAY = False  # 20_: 박스는 컨베이어가 자연스럽게 굴려오게 둔다. 시작 시 코드로 위치를 되돌리지 않는다.


# ╔══════════════════════════════════════════════════════════════╗
# ║  E. 1번 로봇 앞 감지 영역 기반 Pick Trigger                  ║
# ╚══════════════════════════════════════════════════════════════╝
# 41_ 핵심:
# 기존 방식은 "박스가 완전히 멈춘 뒤" pick을 시작했다.
# 이번 방식은 1번 로봇 앞의 좁은 감지 영역 안으로 박스 중심이 들어오면 바로 pick 준비/동작을 시작한다.
# 카메라 인식이 아니라 USD bbox 기반의 영역 센서 방식이다.
# 영역은 m0609_ prefix와 겹치지 않는 이름으로 둔다.
# 그래야 idle 로봇 탐색에서 pick zone visual이 로봇 후보로 잘못 잡히지 않는다.
PICK_TRIGGER_MODE = "FRONT_ZONE"  # "FRONT_ZONE" 또는 "STOPPED_BOX"

PICK_ZONE_VISUAL_ENABLED = False  # 122_: 새 USD에 직접 만든 pick_ready_zone_A를 삭제 후 재생성하지 않음  # 81_: ready zone A visual은 그대로 생성/표시한다.
# 42_ 수정: GUI에서 사용자가 맞춘 pick-ready 영역 값 적용.
# Transform: translate=(-2.08983, -4.35, 0.88), rotate=(0,0,0), scale=(0.92,0.32,0.44).
PICK_ZONE_PATH = "/World/pick_ready_zone_A"

# 영역 위치/크기 조정은 여기만 수정하면 된다. 단위는 meter, world 좌표 기준.
# center: 영역 중심, size: X/Y/Z 방향 크기.
# 처음 위치가 안 맞으면 Isaac Sim에서 박스가 지나가는 위치를 보고 아래 값을 조금씩 조정해라.
PICK_ZONE_CENTER = np.array([-2.08983, -4.35, 0.88], dtype=float)
PICK_ZONE_SIZE   = np.array([0.92,  0.32, 0.44], dtype=float)

# 박스 bbox center가 영역 안에 몇 step 연속 들어와야 pick을 시작할지.
# 1~3이면 빠르고, 8~15면 오검출이 줄어든다.
PICK_ZONE_REQUIRED_STEPS = 3
PICK_ZONE_LOG_INTERVAL = 10
PICK_ZONE_LOG_EVERY_STEP = False

# True이면 영역 진입 순간 박스 velocity를 0으로 정리해서, 이동 중인 박스를 너무 놓치는 현상을 줄인다.
# 실제 컨베이어 흐름을 더 살리고 싶으면 False로 바꿔라.
PICK_ZONE_ZERO_BOX_VELOCITY_ON_PICK_START = True
# 77_: A ready zone 안에 들어온 뒤, 바로 집지 않고 bbox 기준으로 정지까지 확인한다.
PICK_ZONE_REQUIRE_STOPPED_BEFORE_PICK = True
# 77_: ready zone A에서는 OriBoxA 계열만 집는다. OriBoxB는 나중에 B zone을 따로 만들 때 사용한다.
PICK_ZONE_A_ONLY = True

# 감지 영역 visual. 물리 없음. 색만 들어간 표시용 큐브다.
PICK_ZONE_COLOR = np.array([0.05, 0.85, 0.20], dtype=float)  # RGB, 초록색
PICK_ZONE_OPACITY = 0.28

# 65_ 핵심: robotAprop_01 충돌 확인/보정.
# 주의: 현재 운반 방식은 VGC10이 실제로 박스를 밀고 가는 물리 운반이 아니라,
# 흡착된 박스 root를 suction 위치에 맞춰 직접 따라가게 하는 top-lock 방식이다.
# 그래서 collider가 있어도 직접 Xform 이동은 물리 충돌로 자동 정지하지 않는다.
# 아래 설정은 1) robotAprop_01에 static collider를 강제로 켜고,
# 2) 운반 경로의 Z 높이를 robotAprop_01 윗면보다 충분히 높여 뚫고 지나가는 장면을 피한다.
ROBOTAPROP_COLLISION_GUARD_ENABLED = False  # 77_: 예전 robotAprop/Cube 보정 로직 사용 안 함. /World/APalt Mesh를 받침 기준으로 사용
ROBOTAPROP_NAME_PREFIXES = ("robotAprop", "RobotAprop", "robotaprop", "Robotaprop")
ROBOTAPROP_EXACT_PATH_CANDIDATES = (
    "/World/robotAprop_01",
    "/World/RobotAprop_01",
    "/World/robotaprop_01",
    "/World/Robotaprop_01",
)
ROBOTAPROP_STATIC_COLLIDER_REPAIR = False  # 77_: USD에서 직접 편집한 robotAprop 물리 속성 건드리지 않음
ORIBOX_COLLIDER_REPAIR_ON_SETUP = False  # 77_: root RigidBody/child Collider는 USD에서 이미 정리했으므로 코드가 재보정하지 않음
ROBOTAPROP_CLEARANCE_Z_MARGIN = 0.080  # robotAprop_01 윗면과 박스 바닥 사이 여유 높이
ROBOTAPROP_CLEARANCE_EXTRA_SUCTION_Z = 0.015  # suction 목표 자체에 추가 여유
ROBOTAPROP_COLLISION_LOG_INTERVAL = 30
ADD_DEFAULT_GROUND_PLANE = False  # 77_: USD에 있는 환경만 사용. 코드가 default ground plane을 새로 만들지 않음


# ╔══════════════════════════════════════════════════════════════╗
# ║  F. OriBoxA_ 다중 박스 + BoxAprop 위 2x2 적재 설정              ║
# ╚══════════════════════════════════════════════════════════════╝
# 52_ 핵심:
# - /World 하위에서 이름이 OriBoxA_ 로 시작하는 박스만 대상으로 삼는다.
# - 각 root 하위의 Small_Cardboard_box bbox center가 위 PICK_ZONE 안에 들어오면 pick 대상으로 선택한다.
# - 1차 목표: BoxAprop 위에 2개 나란히 적재 = (2,1)
# - 2차 목표: 2개 적재 후 BoxAprop를 박스 높이만큼 낮추고 다시 2개 적재 = (2,2)
# - BoxAprop의 위치/크기는 사용자가 USD에서 직접 조정하면 되고, 코드는 매번 BoxAprop world bbox를 읽는다.
MULTI_ORIBOX_STACKING_ENABLED = True
ORIBOX_STACK_PARENT_PATH = "/World"
ORIBOX_STACK_NAME_PREFIX = "OriBoxA_"  # legacy 표시용
ORIBOX_STACK_NAME_PREFIXES = ("OriBoxA_", "OriBoxA")  # 77_: ready zone A에서는 A 상자만 대상
ORIBOX_STACK_BOX_MESH_NAME = "Small_cardboard_box"  # 72_: child 이름은 collider 확인/호환용. 좌표 기준은 root
ORIBOX_STACK_BOX_MESH_NAME_CANDIDATES = ("Small_cardboard_box", "Small_Cardboard_box")  # USD 대소문자 차이 fallback

STACK_SUPPORT_CUBE_PATH = "/World/APalt"  # 117_: 상자를 올려둘 받침 큐브/APalt Mesh 기준  # 77_: 예전 /World/Cube 받침 기준 제거. 팔레트 기준은 /World/APalt만 사용
STACK_SUPPORT_NAME_CANDIDATES = ("APalt",)  # 117_: fallback도 APalt만 허용  # 77_: fallback도 APalt만 허용

# 75_ 핵심: 팔레타이징 기준은 USD에 사용자가 찍어둔 Xform /World/APalt 를 사용한다.
# APalt는 "팔레트 윗면 중심" 좌표로 둔다. 실제 slot z는 APalt.z + box_height/2 로 계산한다.
USE_PALLET_ORI_A_MARKER = False  # 117_: /World/APalt Mesh의 bbox top을 받침 윗면으로 사용
PALLET_ORI_A_PATH = "/World/APalt"  # 117_: APalt를 팔레트/받침 기준으로 사용
PALLET_SLOT_AXIS = "ROBOT_X"  # 2x1 방향. 필요하면 "ROBOT_Y"로 바꿔 테스트
STACK_COLUMNS = 2  # 145_: APalt 위 좌/우 2열 기준. 홀수=160도, 짝수=130도
STACK_LAYERS = 2      # 145_: 최대 2층까지 테스트
STACK_SLOT_COUNT = 4  # 145_: 최대 4개 적재. 실제 이번 테스트 개수는 FORKLIFT_TRIGGER_COUNT가 결정
STACK_SLOT_AXIS = "ROBOT_X"  # 60_: 로봇 기준 좌우 방향으로 2개를 나란히 둔다.
STACK_SLOT_GAP = 0.550  # 79_: 화면상 붙어 보여서 오차 고려. box_x≈0.26 + gap 0.16 = slot 간격 약 0.42m
STACK_FIRST_SLOT_OFFSET = np.array([0.0, 0.0, 0.0], dtype=float)  # world 보정 fallback
# 124_ APalt와 상자 간격 조절 핵심 파라미터
# BOXAPROP_SLOT_OFFSET_ROBOT = [좌우, 앞뒤, 높이] 보정값(m), 로봇 기준.
#   첫 번째 값 + : 로봇 기준 오른쪽, - : 왼쪽
#   두 번째 값 + : 로봇 기준 앞쪽/바깥쪽, - : 뒤쪽/안쪽  (방향이 반대로 보이면 부호만 바꿔 테스트)
#   세 번째 값 + : 더 위에서 release, - : 더 낮게 release
BOXAPROP_SLOT_OFFSET_ROBOT = np.array([0.0, 0.0, 0.0], dtype=float)
# APalt 윗면과 상자 바닥 사이 높이 여유(m).
#   키우면 더 위에서 떨어짐/덜 파묻힘, 줄이면 APalt에 더 가까이 붙음.
STACK_PLACE_Z_CLEARANCE = 0.006
STACK_USE_BOX_SIZE_FOR_SLOT_STEP = True
STACK_MANUAL_SLOT_STEP = np.array([0.80, 0.0, 0.0], dtype=float)  # 79_: bbox 계산 fallback도 42cm 간격으로 고정

# 1층 2개를 놓은 뒤 BoxAprop를 박스 높이만큼 낮춘다.
# 그러면 1층 박스 윗면이 다시 원래 작업 높이 근처가 되고, 같은 방식으로 2층 2개를 쌓을 수 있다.
STACK_LOWER_SUPPORT_AFTER_EACH_LAYER = True   # 145_: 3개 이상 테스트 시 1층 2개 후 APalt를 박스 높이만큼 내림
STACK_LOWER_SUPPORT_STEPS = 180                 # 145_: APalt lowering step 수. 크면 더 천천히 내려감
STACK_LOWER_EXTRA_Z = 0.000                     # 양수면 박스 높이보다 조금 더 낮춤
STACK_LOWER_MAX_LAYERS = 1                      # 145_: 최대 1번만 내림(1층 -> 2층 준비)

# 성공한 root는 다시 집지 않는다. Stop→Play하면 이 목록은 코드 내부에서 초기화된다.
STACK_SKIP_COMPLETED_BOXES = True
STACK_STOP_WHEN_FULL = True  # 75_: 2x1 두 칸이 차면 정지
# 52_: 이번 버전은 박스 물리 OFF/scripted carry가 아니라 FixedJoint + 관절 기반 이동 우선.
STACK_USE_DIRECT_CARRY_ROUTE = True
# 순간이동 보정 금지. BoxAprop 위 slot 중심 강제 snap을 쓰지 않는다.
STACK_SNAP_BOX_TO_SLOT_ON_RELEASE = False  # 77_: 순간이동 금지. APalt 슬롯으로 강제 보정하지 않고 현재 위치에서 release
STACK_SNAP_MAX_XY_ERROR = 0.0  # 75_: snap 비활성화

# 64_ 핵심 디버그: 박스가 순간이동하는 원인을 잡기 위해 지정 박스의 world pose를 계속 추적한다.
# 기본은 네가 요청한 OriBoxA_02 / OriBoxA_03만 추적한다.
# 로그가 너무 많으면 POSE_TRACK_LOG_EVERY_STEP=False, POSE_TRACK_LOG_INTERVAL=10 정도로 바꿔라.
POSE_TRACK_ENABLED = False  # 128_: 일반 POSE_TRACK 로그 제거. release 진단 로그만 출력
POSE_TRACK_ROOT_PATHS = [
    "/World/OriBoxA_01",
    "/World/OriBoxA_02",
]
POSE_TRACK_BOX_CHILD_NAME = "Small_cardboard_box"  # 72_: log에서 child 동반 여부 확인용. 기준 좌표/bbox는 root
POSE_TRACK_LOG_EVERY_STEP = False  # 69_: 새 환경에서는 로그 폭주 방지. 정밀 추적 필요하면 True
POSE_TRACK_LOG_INTERVAL = 10
POSE_TRACK_JUMP_WARN_TOL = 0.015  # m. 이전 로그 대비 1.5cm 이상 바뀌면 JUMP 표시
_POSE_TRACK_STATE = {"step": 0, "prev": {}}

# 72_ 핵심:
# 새 Conveyor_lift.usd에서는 USD에서 root에 Rigid Body를 붙이고 child는 Collider만 남긴 구조를 사용한다.
# root가 실제 물리 박스의 대표 좌표이므로, 코드에서 child를 root에 강제로 맞추지 않는다.
ORIBOX_KEEP_CHILD_CENTER_MATCHED_TO_ROOT = False
ORIBOX_CENTER_MATCH_TOL = 0.001
ORIBOX_CENTER_MATCH_LOG = False



__all__ = [name for name in globals() if not name.startswith('__')]
