"""Extracted common M0609 runtime implementation.

This module must only be imported after ``SimulationApp`` has been created.
"""

from pathlib import Path
from contextlib import contextmanager
import math
import sys
import time

import numpy as np
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.manipulators import SingleManipulator
try:
    from isaacsim.core.utils.types import ArticulationAction
except Exception:
    from omni.isaac.core.utils.types import ArticulationAction

from .settings import *

from . import diagnostics as _diagnostics_module
from . import physics as _physics_module
from . import palletizing as _palletizing_module
from . import scene as _scene_module
from . import settings as _settings_module
from .config import RobotCellConfig
from .forklift import PalletLoweringCoordinator
from .diagnostics import *
from .physics import *
from .palletizing import *
from .scene import *

_RUNTIME_MODULES = (
    _settings_module,
    _scene_module,
    _diagnostics_module,
    _physics_module,
    _palletizing_module,
)

RMPFLOW_DIR = str(_settings_module._THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_pick_place_controller import PickPlaceController
from m0609_rmpflow_controller import RMPFlowController


def _runtime_overrides(config: RobotCellConfig, simulation_app):
    """Translate the explicit cell profile to legacy names used by the stable core."""
    robot_prim = config.robot_prim_path
    project_dir = _settings_module.PROJECT_DIR
    box_asset = _settings_module._resolve_first_existing_path(
        [project_dir / relative_path for relative_path in config.box_asset_candidates],
        "OriBox USD",
    )
    return {
        "simulation_app": simulation_app,
        "ENABLE_ROS2_BRIDGE": config.enable_ros2_bridge,
        "ACTIVE_ROBOT_ROOT_PATH": config.robot_root_path,
        "ROBOT_PRIM_PATH": robot_prim,
        "ROBOT_OBJECT_NAME": config.robot_object_name,
        "OLD_GRIPPER_PRIM_PATH": f"{robot_prim}/onrobot_rg2ft",
        "VGC10_PRIM_PATH": config.vgc10_root_path,
        "VGC10_MOUNT_PATH": config.vgc10_mount_path,
        "VGC10_MODEL_PATH": config.vgc10_model_path,
        "VGC10_FOLLOW_TARGET_PATH": f"{robot_prim}/{_settings_module.EE_LINK_NAME}/tool0",
        "VGC10_SUCTION_POINT_PATH": config.vgc10_suction_point_path,
        "SUCTION_GRID_ROOT_PATH": config.suction_grid_path,
        "SUCTION_GRID_WORLD_MARKER_ROOT_PATH": config.suction_grid_world_path,
        "SUCTION_BODY_PATH": config.suction_body_path,
        "SUCTION_MARKER_PATH": config.suction_marker_path,
        "VGC10_SUCTION_DEBUG_MARKER_PATH": config.suction_debug_marker_path,
        "PHYSICS_ATTACH_JOINT_PATH": config.physics_attach_joint_path,
        "TARGET_CUBE_PATH": config.first_box_path,
        "BOX_PRIM_PATH": config.first_box_path,
        "STOP_CHECK_PRIM_PATH": config.first_box_path,
        "BOX_ROOT_PATH": config.first_box_path,
        "BOX_MOVE_PRIM_PATH": config.first_box_path,
        "ORI_BOX_USD_PATH": box_asset,
        "CUBE_VISIBLE_CANDIDATE_PATHS_109": ("/Cube", "/World/Cube", config.pallet_path),
        "ROBOT_CENTER_CANDIDATE_PATHS": (
            f"{robot_prim}/base_link",
            robot_prim,
            config.robot_root_path,
        ),
        "PICK_ZONE_PATH": config.pick_zone_path,
        "PICK_ZONE_CENTER": np.array(config.pick_zone_center, dtype=float),
        "PICK_ZONE_A_ONLY": config.pick_zone_side_only,
        "ORIBOX_STACK_NAME_PREFIX": f"{config.box_prefix}_",
        "ORIBOX_STACK_NAME_PREFIXES": (f"{config.box_prefix}_", config.box_prefix),
        "LEGACY_BOX_WRAPPER_PATH": f"/World/{config.box_prefix}_move",
        "STACK_SUPPORT_CUBE_PATH": config.pallet_path,
        "STACK_SUPPORT_NAME_CANDIDATES": (config.pallet_name,),
        "PALLET_ORI_A_PATH": config.pallet_path,
        "SLOT_MARKER_PATHS_155": config.slot_marker_paths,
        "STACK_JOINT1_DEG_PATTERN_145": config.stack_joint1_degrees,
        "CONVEYOR_SAFE_LIFT_INCLUDE_PREFIXES_155": config.conveyor_box_prefixes,
        "POSE_TRACK_ROOT_PATHS": list(config.pose_track_root_paths),
        "ROBOTAPROP_NAME_PREFIXES": config.robot_prop_name_prefixes,
        "ROBOTAPROP_EXACT_PATH_CANDIDATES": config.robot_prop_paths,
        "STACK_LAYERS": config.stack_layers,
        "STACK_SLOT_COUNT": config.stack_slot_count,
        "STACK_LOWER_SUPPORT_AFTER_EACH_LAYER": config.lower_support_after_each_layer,
        "POST_RELEASE_SETTLE_BEFORE_APALT_LOWER_ENABLED_160": config.settle_before_pallet_lower,
        "SLOT_ROUTE_SIGN_FORCE_159": config.slot_route_sign_force,
        "DIAG_STOP_AFTER_RELEASE_COUNT_164": config.diagnostic_stop_release_count,
        "DIAG_STOP_AFTER_RELEASE_SKIP_APALT_LOWER_164": config.diagnostic_skip_pallet_lower,
        "FUSED_SECOND_BOX_YAW_ALIGN_ENABLED_184": config.fused_yaw_enabled,
        "SLOT_MARKER_FALLBACK_GOAL_ENABLE_FINAL_MOVE_179": config.marker_fallback_final_move,
        "APALT_LOWER_DURING_HOME_RETURN_180": config.pallet_lower_during_home,
        "APALT_LOWER_DURING_HOME_USE_HOME_STEPS_180": config.pallet_lower_use_home_steps,
    }


def _apply_worker_context(worker):
    values = dict(worker._overrides)
    values.update(worker._state)
    for module in _RUNTIME_MODULES:
        module.__dict__.update(values)
    globals().update(values)


def _capture_worker_context(worker):
    """Keep mutable legacy globals isolated when A and B workers are interleaved."""
    for name, module in (
        ("_POSE_TRACK_STATE", _physics_module),
        ("_PHYSICS_JOINT_DIAG_STATE", _physics_module),
        ("_LAST_SUCTION_GRID_INFO", _scene_module),
        ("_SUCTION_GRID_EVALUATION_COUNT", _scene_module),
        ("ROBOT_INITIAL_JOINTS_156", _palletizing_module),
    ):
        if name in module.__dict__:
            worker._state[name] = module.__dict__[name]
    for name in ("ROBOT_PRIM_PATH", "OLD_GRIPPER_PRIM_PATH", "VGC10_FOLLOW_TARGET_PATH"):
        if name in _palletizing_module.__dict__:
            worker._overrides[name] = _palletizing_module.__dict__[name]


class _WorkerTask(M0609ConveyorBoxTask):
    """Keep World-driven callbacks in the task owner's legacy helper context."""

    def __init__(self, name, worker):
        self._worker = worker
        super().__init__(name=name)

    def set_up_scene(self, scene):
        with self._worker.context():
            return super().set_up_scene(scene)

    def _load_usd(self):
        super()._load_usd()
        # Propagate discovered paths before scene/physics setup uses them.
        _capture_worker_context(self._worker)
        _apply_worker_context(self._worker)

    def get_observations(self):
        with self._worker.context():
            return super().get_observations()

    def pre_step(self, control_index, simulation_time):
        with self._worker.context():
            return super().pre_step(control_index, simulation_time)

    def post_reset(self):
        with self._worker.context():
            return super().post_reset()


def _worker_loop(shared_world, config, worker):
    my_world = shared_world
    task = _WorkerTask(name=config.task_name, worker=worker)
    my_world.add_task(task)
    # The lifecycle owner resets only after every worker has registered its task.
    yield

    stage = omni.usd.get_context().get_stage()

    # reset/recompose 이후에도 기존 RG2가 다시 보이는 경우를 막기 위해 한 번 더 제거한다.
    remove_old_gripper(stage, verbose=True)
    # 109_: reset/recompose 후에도 사용자가 보낸 Cube가 viewport에 남아 있도록 다시 visible 처리한다.
    force_show_cube_prims(stage, verbose=True)
    # 146_: reset/recompose 이후에도 Environment 계열 prim을 다시 visible/active로 복구한다.
    force_show_environment_prims(stage, verbose=True)
    repair_robotaprop_and_oribox_colliders(stage, completed_roots=set(), verbose=True)

    robot = my_world.scene.get_object(config.robot_object_name)
    initialize_robot_for_conveyor(robot, my_world)

    for _ in range(30):
        update_vgc10_suction_anchor(robot)
        my_world.step(render=True)

    print("\n" + "=" * 60)
    print("[C-2] PickPlaceController 생성")
    print("=" * 60)
    print(f"  Conveyor USD = {USD_PATH}")
    print(f"  box path     = {task.box_path}")
    print(f"  move path    = {task.box_move_path}")
    print(f"  box center   = {task.box_initial_center}")
    print(f"  box top      = {task.box_initial_top_center}")
    print(f"  box height   = {task.box_height}")
    print(f"  pick target  = {task.pick_center}  # controller picking_position")
    print(f"  goal center  = {task.goal_center}")
    print(f"  URDF         = {M0609_URDF_PATH}")
    print(f"  description  = {M0609_DESCRIPTION_PATH}")
    print(f"  rmpflow      = {M0609_RMPFLOW_CONFIG_PATH}")
    print(f"  events_dt    = {EVENTS_DT}")
    print(f"  EE frame     = {EE_LINK_NAME}")
    print(f"  attach mode  = top-surface-rectangle, z_gap=[{BOX_ATTACH_Z_MIN},{BOX_ATTACH_Z_MAX}], dist<={BOX_ATTACH_DIST_TOL}")
    print(f"  stop gate    = path={STOP_CHECK_PRIM_PATH}, required_steps={BOX_STABLE_REQUIRED_STEPS}, move<={BOX_STABLE_POS_TOL}m/step")
    print(f"  axis diag    = enabled={AXIS_YAW_DIAGNOSTIC_ENABLED_164}, stop_after_release={DIAG_STOP_AFTER_RELEASE_COUNT_164 if DIAG_STOP_AFTER_RELEASE_ENABLED_164 else None}")
    print(f"  stop mode    = bbox_move={BOX_STOP_USE_BBOX_MOVE}, linear_vel_gate={BOX_STOP_USE_LINEAR_VEL}, angular_vel_gate={BOX_STOP_USE_ANGULAR_VEL}")
    print(f"  carry mode   = disable_physics_during_carry={BOX_DISABLE_PHYSICS_DURING_CARRY}, reenable_after_release={BOX_REENABLE_PHYSICS_AFTER_RELEASE}")
    print(f"  debug log    = every_step={BOX_STOP_LOG_EVERY_STEP}, interval={BOX_STOP_LOG_INTERVAL}, note=linear/angular velocity are INFO_ONLY unless gate=True")

    null_gripper = NullGripper()

    controller = PickPlaceController(
        name=config.pick_place_controller_name,
        gripper=null_gripper,
        robot_articulation=robot,
        end_effector_initial_height=0.30,
        events_dt=EVENTS_DT,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )
    cart_controller = RMPFlowController(
        name=config.cartesian_controller_name,
        robot_articulation=robot,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )
    print("  [OK] PickPlaceController 생성 완료")
    print("  [OK] Custom RMPFlow carry controller 생성 완료")
    print("  [30_] infinite retry loop: joint lift -> base swing -> release -> home -> wait/retry")

    print("\n[VGC10 + Conveyor OriBox root suction 시작]\n")
    was_playing = False
    task_done = False
    attached = False
    ever_attached = False
    released = False
    returning_home = False
    home_start_joints = None
    home_target_joints = None
    home_return_step = 0
    retry_logged = False
    best_attach_reason = "none"
    best_attach_dist = 999.0

    # attach 순간의 bbox center offset. suction point를 따라 박스 bbox center를 움직일 때 사용한다.
    attached_center_offset = None
    attach_center_z = None
    attach_steps = 0

    custom_carry_active = False
    custom_carry_phase = None
    custom_phase_step = 0
    custom_targets = None
    custom_fixed_orientation = None
    custom_min_center_z = None
    custom_phase_index = 0
    release_watchdog_counter_127 = 0

    # 박스가 완전히 멈춘 뒤에만 로봇 pick-place를 시작한다.
    box_stop_detector = BoxStopDetector()
    pick_zone_detector = PickZoneDetector()
    box_stopped_for_pick = False
    pick_started = False

    # 53_: 작업영역 진입 후 흡착 전에 suction point를 box_top 좌표로 직접 정렬하는 상태값
    pre_attach_align_active = False
    pre_attach_align_phase = "idle"
    pre_attach_align_step = 0
    pre_attach_fixed_orientation = None

    # 30_: 무한 반복 상태값
    loop_cycle_index = 0
    loop_attempt_index = 0
    home_return_reason = "initial"
    home_return_is_success = False
    ignore_released_center = None
    ignore_wait_counter = 0

    # 47_: OriBoxA_01~03 다중 박스 쌓기 상태값
    completed_box_roots = set()
    zone_counts_by_root = {}
    stack_slot_index = 0
    stack_platform_lower_count = 0
    stack_complete_logged = False

    # ================================================================
    # 포크리프트 적재 시퀀스 설정  ← 여기 숫자만 바꾸면 됩니다
    # ================================================================
    # 145_: 이번 사이클에서 APalt에 실을 상자 개수.
    #       1,2,3,4 중 하나로 바꾸면 전체 흐름이 바뀐다.
    #       2 = OriBoxA_01, OriBoxA_02 release 후 카운트다운/운반
    #       3/4 = 2개 release 후 APalt를 BOX_STACK_HEIGHT_145만큼 내리고, 나머지를 쌓은 뒤 운반
    FORKLIFT_TRIGGER_COUNT = int(config.forklift_trigger_count)
    # 139_: release count가 조건에 도달해도 바로 forklift를 움직이지 않고 카운트다운 후 시작한다.
    FORKLIFT_START_DELAY_SEC_139 = 5
    # 139_: 평상시 main_loop forklift 위치 로그는 끈다. delay 중에는 5초 카운트만 출력한다.
    FORKLIFT_TRACK_MAIN_LOOP_139 = False
    # 144_: 화면/옆면 기준 좌표 표현으로 정리한다.
    #       - 코드상 X = 트럭이 앞으로 가는 방향 = USD GUI Translate Y
    #       - 코드상 Y = 리프트가 위로 올라가는 방향 = USD GUI Translate Z
    #       - USD GUI Translate X는 사용하지 않는다.
    FORKLIFT_LIFT_TARGET_Y  = 0.4    # 코드상 Y축 상승 높이. 실제로는 USD Translate Z를 움직인다.
    FORKLIFT_LIFT_TARGET_Z  = FORKLIFT_LIFT_TARGET_Y  # 기존 함수 호환용 이름. 실제 USD 축은 Z.
    FORKLIFT_MOVE_TARGET_X  = -5.0    # 코드상 X축 전진 거리. 실제로는 USD Translate Y를 움직인다.
    FORKLIFT_MOVE_TARGET_Y  = 0.0    # 144_: 코드상 Y는 상승에 사용하므로 평면 Y 이동은 기본 사용 안 함.
    FORKLIFT_LIFT_WAIT_SEC  = 3.0    # 리프트 올린 후 안정화 대기 시간 (초)
    FORKLIFT_MOVE_SPEED     = 0.5    # 코드상 X 이동 속도 (m/s). 실제 USD Translate Y 속도.
    FORKLIFT_PRIM_PATH = config.pallet_path  # 141_: lift_flat 대신 /World/APalt를 가상 지게차/팔레트로 사용
    FORKLIFT_JOINT_PATH     = ""  # 141_: 실제 lift joint 사용 안 함. /World/APalt Xform을 직접 이동

    # 137_: 현재 USD의 forklift는 drive joint가 아니라 /World/forklift/S_ForkliftFork Xform 구조다.
    #       lift drive가 없으면 fork Xform 자체의 local Z를 직접 올려서 리프트 동작을 테스트한다.
    FORKLIFT_DIRECT_FORK_LIFT_ENABLED_137 = True
    # 138_: 사용자가 Stage에서 확인한 실제 포크 lift prim.
    # 이 Xform을 local Z로 올리면 내부 node_/mesh_까지 같이 올라간다.
    FORKLIFT_DIRECT_FORK_PATHS_137 = (
        config.pallet_path,
    )

    # 137_: forklift trigger=1 테스트 중에도 stack_slot_count=1 때문에 로봇 루프가 멈추지 않게 한다.
    #       slot 계산은 기존 1번 slot을 쓰되, stack full 조건만 무시한다.
    ALLOW_CONTINUE_AFTER_STACK_FULL_137 = True
    # ================================================================

    forklift_sequence_triggered = False  # 시퀀스가 이미 실행됐는지 체크

    print(f"[VIRTUAL_PALLET_TARGET] root={config.pallet_path} (lift_flat/forklift model not used)")
    print(f"[VIRTUAL_PALLET_TARGET] side={config.side} side-view code axes: X=truck direction(USD Translate Y), Y=lift up(USD Translate Z)")

    # ================================================================
    # 136_: forklift 동작 확인용 위치 추적 + 안전 경로 자동 탐색
    # - 기존 FORKLIFT_JOINT_PATH가 USD와 다르면 null prim 오류가 났다.
    # - 이제 root / lift joint / targetPosition attribute를 stage에서 자동 탐색한다.
    # - 못 찾으면 오류로 멈추지 않고 후보 경로를 출력한 뒤 Y 이동 테스트만 계속한다.
    # ================================================================
    FORKLIFT_ROOT_CANDIDATE_PATHS_136 = (
        FORKLIFT_PRIM_PATH,
    )
    # 142_: /World/APalt 가상 지게차 모드에서는 실제 lift joint를 전혀 쓰지 않는다.
    # 빈 path("")나 예전 lift_flat 후보를 넣으면 Property/UI async 갱신 중 SdfPath 경고가 날 수 있어 제거한다.
    FORKLIFT_LIFT_JOINT_CANDIDATE_PATHS_136 = ()
    FORKLIFT_LIFT_TARGET_ATTR_NAMES_136 = (
        "drive:linear:physics:targetPosition",
        "drive:linear:targetPosition",
        "physics:targetPosition",
        "targetPosition",
    )

    FORKLIFT_POSE_TRACK_ENABLED = True
    FORKLIFT_POSE_LOG_INTERVAL = 20
    FORKLIFT_POSE_LOG_EVERY_STEP = False
    FORKLIFT_POSE_JUMP_WARN_TOL = 0.010
    _FORKLIFT_TRACK_STATE_135 = {"step": 0, "prev_root": None, "prev_center": None}
    _FORKLIFT_RESOLVE_CACHE_136 = {"root_path": None, "lift_path": None, "attr_name": None}

    # 141_: lift_flat/forklift 모델을 쓰지 않고 /World/APalt 하나를 가상 지게차/팔레트처럼 움직인다.
    # release 완료된 상자들은 APalt 이동 시작 직전에 kinematic 상태로 고정하고,
    # APalt bbox center 기준 상대 offset을 저장한 뒤 APalt 이동량을 그대로 따라가게 한다.
    VIRTUAL_APALT_FORKLIFT_ENABLED_141 = True
    VIRTUAL_APALT_PATH_141 = config.pallet_path
    VIRTUAL_APALT_CARGO_ROOTS_141 = []
    VIRTUAL_APALT_CARGO_OFFSETS_141 = {}
    VIRTUAL_APALT_CARGO_DISABLE_COLLISION_141 = False  # True로 바꾸면 이동 중 상자 충돌도 꺼짐
    VIRTUAL_APALT_LOG_INTERVAL_141 = 30
    # 142_: 지게차처럼 천천히 상승하도록 APalt Z 이동을 step 단위로 수행한다.
    VIRTUAL_APALT_LIFT_SPEED_142 = 0.08  # m/s. 0.4m 상승이면 약 5초
    VIRTUAL_APALT_LIFT_LOG_INTERVAL_142 = 15

    def _resolve_forklift_root_path_136(stage_now, verbose=False):
        """USD 안의 forklift root prim을 안전하게 찾는다."""
        cached = _FORKLIFT_RESOLVE_CACHE_136.get("root_path")
        if cached:
            try:
                prim = stage_now.GetPrimAtPath(cached)
                if prim and prim.IsValid():
                    return str(cached)
            except Exception:
                pass

        for p in FORKLIFT_ROOT_CANDIDATE_PATHS_136:
            try:
                prim = stage_now.GetPrimAtPath(p)
                if prim and prim.IsValid():
                    _FORKLIFT_RESOLVE_CACHE_136["root_path"] = str(p)
                    if verbose:
                        print(f"[FORKLIFT_RESOLVE_136] root found by candidate: {p}")
                    return str(p)
            except Exception:
                pass

        found = []
        try:
            for prim in stage_now.Traverse():
                ps = prim.GetPath().pathString
                low = ps.lower()
                if "forklift" in low:
                    found.append(ps)
            if found:
                # 가장 짧은 path를 root 후보로 사용한다.
                found_sorted = sorted(found, key=lambda x: (x.count("/"), len(x)))
                root_path = found_sorted[0]
                _FORKLIFT_RESOLVE_CACHE_136["root_path"] = root_path
                if verbose:
                    print(f"[FORKLIFT_RESOLVE_136] root auto-selected: {root_path}")
                    print(f"[FORKLIFT_RESOLVE_136] forklift candidates sample={found_sorted[:12]}")
                return root_path
        except Exception as exc:
            if verbose:
                print(f"[FORKLIFT_RESOLVE_WARN_136] root scan failed: {exc}")

        if verbose:
            print(f"[FORKLIFT_RESOLVE_FAIL_136] forklift root not found. expected={FORKLIFT_PRIM_PATH}")
        return None

    def _get_lift_drive_attr_from_prim_136(prim):
        """prim에서 lift targetPosition 계열 attribute를 찾는다."""
        if prim is None:
            return None, None
        try:
            if not prim.IsValid():
                return None, None
        except Exception:
            return None, None

        for attr_name in FORKLIFT_LIFT_TARGET_ATTR_NAMES_136:
            try:
                attr = prim.GetAttribute(attr_name)
                if attr and attr.IsValid():
                    return attr, attr_name
            except Exception:
                pass

        try:
            for attr in prim.GetAttributes():
                name = attr.GetName()
                low = name.lower()
                if "targetposition" in low and ("drive" in low or "physics" in low or "linear" in low):
                    return attr, name
        except Exception:
            pass
        return None, None

    def _resolve_forklift_lift_drive_attr_136(stage_now, verbose=False):
        """lift joint prim과 targetPosition attr를 자동 탐색한다."""
        cached_path = _FORKLIFT_RESOLVE_CACHE_136.get("lift_path")
        cached_attr = _FORKLIFT_RESOLVE_CACHE_136.get("attr_name")
        if cached_path and cached_attr:
            try:
                prim = stage_now.GetPrimAtPath(cached_path)
                attr = prim.GetAttribute(cached_attr) if prim and prim.IsValid() else None
                if attr and attr.IsValid():
                    return prim, attr, str(cached_attr)
            except Exception:
                pass

        for p in FORKLIFT_LIFT_JOINT_CANDIDATE_PATHS_136:
            try:
                prim = stage_now.GetPrimAtPath(p)
                attr, attr_name = _get_lift_drive_attr_from_prim_136(prim)
                if attr is not None:
                    _FORKLIFT_RESOLVE_CACHE_136["lift_path"] = str(p)
                    _FORKLIFT_RESOLVE_CACHE_136["attr_name"] = str(attr_name)
                    if verbose:
                        print(f"[FORKLIFT_RESOLVE_136] lift drive found by candidate: prim={p}, attr={attr_name}")
                    return prim, attr, str(attr_name)
            except Exception:
                pass

        root_path = _resolve_forklift_root_path_136(stage_now, verbose=verbose)
        found_candidates = []
        try:
            for prim in stage_now.Traverse():
                ps = prim.GetPath().pathString
                low = ps.lower()
                if root_path and not ps.startswith(root_path):
                    continue
                if ("lift" not in low) and ("fork" not in low) and ("joint" not in low):
                    continue

                attr, attr_name = _get_lift_drive_attr_from_prim_136(prim)
                if attr is not None:
                    _FORKLIFT_RESOLVE_CACHE_136["lift_path"] = ps
                    _FORKLIFT_RESOLVE_CACHE_136["attr_name"] = str(attr_name)
                    if verbose:
                        print(f"[FORKLIFT_RESOLVE_136] lift drive auto-selected: prim={ps}, attr={attr_name}")
                    return prim, attr, str(attr_name)

                # 디버그 후보: lift/joint 이름은 있지만 target attr가 없는 prim
                try:
                    target_attrs = [a.GetName() for a in prim.GetAttributes() if "target" in a.GetName().lower() or "drive" in a.GetName().lower()]
                except Exception:
                    target_attrs = []
                if len(found_candidates) < 20:
                    found_candidates.append((ps, prim.GetTypeName(), target_attrs[:8]))
        except Exception as exc:
            if verbose:
                print(f"[FORKLIFT_RESOLVE_WARN_136] lift drive scan failed: {exc}")

        if verbose:
            print(f"[FORKLIFT_RESOLVE_FAIL_136] lift drive attr not found. old_joint_path={FORKLIFT_JOINT_PATH}")
            for ps, typ, attrs in found_candidates[:12]:
                print(f"  [FORKLIFT_LIFT_CANDIDATE_136] path={ps}, type={typ}, attrs={attrs}")
        return None, None, None

    def _get_or_add_translate_op_137(xformable):
        """Xformable에서 translate op를 찾거나 없으면 만든다."""
        translate_op = None
        current_pos = None
        try:
            for op in xformable.GetOrderedXformOps():
                if "translate" in op.GetOpName():
                    translate_op = op
                    current_pos = op.Get()
                    break
        except Exception:
            pass

        if translate_op is None:
            translate_op = xformable.AddTranslateOp()
            current_pos = Gf.Vec3d(0.0, 0.0, 0.0)

        if current_pos is None:
            current_pos = Gf.Vec3d(0.0, 0.0, 0.0)
        return translate_op, current_pos

    def _resolve_forklift_direct_fork_path_137(stage_now, verbose=False):
        """drive joint가 없는 forklift에서 실제로 들어올릴 fork Xform을 찾는다."""
        cached = _FORKLIFT_RESOLVE_CACHE_136.get("direct_fork_path")
        if cached:
            try:
                prim = stage_now.GetPrimAtPath(cached)
                if prim and prim.IsValid():
                    return str(cached)
            except Exception:
                pass

        for p in FORKLIFT_DIRECT_FORK_PATHS_137:
            try:
                prim = stage_now.GetPrimAtPath(p)
                if prim and prim.IsValid() and prim.IsA(UsdGeom.Xformable):
                    _FORKLIFT_RESOLVE_CACHE_136["direct_fork_path"] = str(p)
                    if verbose:
                        print(f"[FORKLIFT_RESOLVE_137] direct fork found by candidate: {p}")
                    return str(p)
            except Exception:
                pass

        root_path = _resolve_forklift_root_path_136(stage_now, verbose=verbose)
        candidates = []
        try:
            for prim in stage_now.Traverse():
                ps = prim.GetPath().pathString
                if root_path and not ps.startswith(root_path):
                    continue
                low = ps.lower()
                if "fork" not in low:
                    continue
                try:
                    is_xformable = prim.IsA(UsdGeom.Xformable)
                except Exception:
                    is_xformable = False
                candidates.append((ps, prim.GetTypeName(), is_xformable))
                # Mesh child보다 Xform parent를 우선한다.
                if is_xformable and prim.GetTypeName() in ("Xform", "Scope"):
                    _FORKLIFT_RESOLVE_CACHE_136["direct_fork_path"] = ps
                    if verbose:
                        print(f"[FORKLIFT_RESOLVE_137] direct fork auto-selected: {ps}")
                    return ps
        except Exception as exc:
            if verbose:
                print(f"[FORKLIFT_RESOLVE_WARN_137] direct fork scan failed: {exc}")

        if verbose:
            print("[FORKLIFT_RESOLVE_FAIL_137] direct fork Xform not found.")
            for ps, typ, is_xformable in candidates[:12]:
                print(f"  [FORKLIFT_FORK_CANDIDATE_137] path={ps}, type={typ}, xformable={is_xformable}")
        return None

    def _get_forklift_direct_fork_local_z_137(stage_now):
        fork_path = _resolve_forklift_direct_fork_path_137(stage_now, verbose=False)
        if not fork_path:
            return None, None
        try:
            prim = stage_now.GetPrimAtPath(fork_path)
            xform = UsdGeom.Xformable(prim)
            _op, pos = _get_or_add_translate_op_137(xform)
            return fork_path, float(pos[2])
        except Exception:
            return fork_path, None

    def forklift_set_direct_fork_lift_137(z_pos):
        """drive attr가 없을 때 fork Xform local Z를 직접 올리는 fallback 리프트."""
        if not bool(FORKLIFT_DIRECT_FORK_LIFT_ENABLED_137):
            return False
        stage_now = omni.usd.get_context().get_stage()
        fork_path = _resolve_forklift_direct_fork_path_137(stage_now, verbose=True)
        if not fork_path:
            print("[FORKLIFT][DIRECT_LIFT_SKIP_137] fork Xform을 찾지 못했습니다.")
            return False
        try:
            prim = stage_now.GetPrimAtPath(fork_path)
            xform = UsdGeom.Xformable(prim)
            translate_op, current_pos = _get_or_add_translate_op_137(xform)
            start_z = float(current_pos[2])
            target_z = float(z_pos)
            translate_op.Set(Gf.Vec3d(float(current_pos[0]), float(current_pos[1]), target_z))
            print(f"[FORKLIFT][DIRECT_LIFT_137] fork={fork_path}, local_z={start_z:.4f} -> {target_z:.4f}")
            log_forklift_pose_135("after_direct_fork_lift_137", force=True)
            return True
        except Exception as exc:
            print(f"[FORKLIFT][DIRECT_LIFT_ERROR_137] {type(exc).__name__}: {exc}")
            return False

    def log_forklift_pose_135(label="forklift", force=False):
        if not bool(FORKLIFT_POSE_TRACK_ENABLED):
            return
        try:
            _FORKLIFT_TRACK_STATE_135["step"] = int(_FORKLIFT_TRACK_STATE_135.get("step", 0)) + 1
            step = int(_FORKLIFT_TRACK_STATE_135["step"])
            if (not force) and (not bool(FORKLIFT_POSE_LOG_EVERY_STEP)) and (step % int(FORKLIFT_POSE_LOG_INTERVAL) != 0):
                return

            stage_now = omni.usd.get_context().get_stage()
            root_path = _resolve_forklift_root_path_136(stage_now, verbose=force)
            if not root_path:
                print(f"[FORKLIFT_TRACK_136][{label}][step={step}] MISSING forklift root. expected={FORKLIFT_PRIM_PATH}")
                return

            root_t = get_world_translation(stage_now, root_path)
            root_t = np.array(root_t if root_t is not None else [np.nan, np.nan, np.nan], dtype=float)
            bbox = get_world_bbox_info(stage_now, root_path)
            if bbox is not None:
                center = np.array(bbox.get("center", [np.nan, np.nan, np.nan]), dtype=float)
                top = np.array(bbox.get("top_center", [np.nan, np.nan, np.nan]), dtype=float)
            else:
                center = np.array([np.nan, np.nan, np.nan], dtype=float)
                top = np.array([np.nan, np.nan, np.nan], dtype=float)

            prev_root = _FORKLIFT_TRACK_STATE_135.get("prev_root")
            prev_center = _FORKLIFT_TRACK_STATE_135.get("prev_center")
            d_root = 0.0
            d_center = 0.0
            jump = ""
            if prev_root is not None:
                try:
                    d_root = float(np.linalg.norm(root_t - prev_root))
                    d_center = float(np.linalg.norm(center - prev_center))
                    if max(d_root, d_center) >= float(FORKLIFT_POSE_JUMP_WARN_TOL):
                        jump = f" MOVE>= {FORKLIFT_POSE_JUMP_WARN_TOL:.3f}"
                except Exception:
                    pass
            _FORKLIFT_TRACK_STATE_135["prev_root"] = root_t.copy()
            _FORKLIFT_TRACK_STATE_135["prev_center"] = center.copy()

            lift_target = None
            lift_path_txt = "None"
            try:
                prim, attr, attr_name = _resolve_forklift_lift_drive_attr_136(stage_now, verbose=False)
                if attr is not None:
                    lift_target = attr.Get()
                    lift_path_txt = prim.GetPath().pathString
            except Exception:
                lift_target = None

            # 137_: drive attr가 없는 forklift면 fork Xform local Z를 lift 값처럼 추적한다.
            if lift_target is None:
                try:
                    fork_path_137, fork_z_137 = _get_forklift_direct_fork_local_z_137(stage_now)
                    if fork_z_137 is not None:
                        lift_target = float(fork_z_137)
                        lift_path_txt = str(fork_path_137)
                except Exception:
                    pass

            lift_txt = "None" if lift_target is None else f"{float(lift_target):+.4f}"
            print(
                f"[FORKLIFT_TRACK_136][{label}][step={step}] "
                f"root_path={root_path} "
                f"root=({root_t[0]:+.4f},{root_t[1]:+.4f},{root_t[2]:+.4f}) "
                f"bbox_center=({center[0]:+.4f},{center[1]:+.4f},{center[2]:+.4f}) "
                f"bbox_top=({top[0]:+.4f},{top[1]:+.4f},{top[2]:+.4f}) "
                f"d_root={d_root:.4f} d_center={d_center:.4f}{jump} "
                f"lift_target={lift_txt} lift_prim={lift_path_txt}"
            )
        except Exception as exc:
            print(f"[FORKLIFT_TRACK_WARN_136][{label}] {exc}")

    def _virtual_apalt_get_center_141(stage_now):
        """APalt의 bbox center를 반환한다. bbox가 안 잡히면 prim world translation을 fallback으로 사용한다."""
        path = str(VIRTUAL_APALT_PATH_141)
        bbox = get_world_bbox_info(stage_now, path)
        if bbox is not None:
            try:
                return np.array(bbox.get("center"), dtype=float)
            except Exception:
                pass
        p = get_world_translation(stage_now, path)
        if p is None:
            return None
        return np.array(p, dtype=float)

    def _virtual_apalt_set_cargo_physics_frozen_141(stage_now, box_root_path, frozen=True):
        """APalt 이동 중 상자가 물리 반응하지 않도록 kinematic/gravity/velocity를 정리한다."""
        try:
            if frozen:
                set_prim_kinematic(stage_now, box_root_path, True)
                try:
                    rb = UsdPhysics.RigidBodyAPI.Apply(stage_now.GetPrimAtPath(box_root_path))
                    # kinematic body로 두고 중력만 꺼서 코드 위치 추종이 흔들리지 않게 한다.
                    if hasattr(rb, "CreateKinematicEnabledAttr"):
                        rb.CreateKinematicEnabledAttr(True).Set(True)
                    if hasattr(rb, "CreateRigidBodyEnabledAttr"):
                        rb.CreateRigidBodyEnabledAttr(True).Set(True)
                except Exception:
                    pass
                try:
                    from pxr import PhysxSchema
                    prim = stage_now.GetPrimAtPath(box_root_path)
                    physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                    physx_rb.CreateDisableGravityAttr(True).Set(True)
                except Exception:
                    pass
                zero_subtree_velocity(stage_now, box_root_path)
                if bool(VIRTUAL_APALT_CARGO_DISABLE_COLLISION_141):
                    set_subtree_collision_enabled(stage_now, box_root_path, False)
            else:
                if bool(VIRTUAL_APALT_CARGO_DISABLE_COLLISION_141):
                    set_subtree_collision_enabled(stage_now, box_root_path, True)
                set_prim_kinematic(stage_now, box_root_path, False)
        except Exception as exc:
            print(f"[VIRTUAL_APALT_CARGO_FREEZE_WARN_141] root={box_root_path}, frozen={frozen}, err={type(exc).__name__}: {exc}")

    def _virtual_apalt_freeze_apalt_physics_145(stage_now, frozen=True):
        """최종 운반 모드에서 APalt 자체도 물리 영향 없이 Xform으로만 움직이게 정리한다."""
        apalt_path = str(VIRTUAL_APALT_PATH_141)
        try:
            prim = stage_now.GetPrimAtPath(apalt_path)
            if not prim or not prim.IsValid():
                print(f"[VIRTUAL_APALT_FREEZE_SKIP_145] APalt prim 없음: {apalt_path}")
                return False
            if frozen:
                try:
                    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
                    if hasattr(rb, "CreateRigidBodyEnabledAttr"):
                        rb.CreateRigidBodyEnabledAttr(True).Set(True)
                    if hasattr(rb, "CreateKinematicEnabledAttr"):
                        rb.CreateKinematicEnabledAttr(True).Set(True)
                except Exception:
                    pass
                try:
                    from pxr import PhysxSchema
                    physx_rb = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                    physx_rb.CreateDisableGravityAttr(True).Set(True)
                except Exception:
                    pass
                zero_subtree_velocity(stage_now, apalt_path)
            print(f"[VIRTUAL_APALT_FREEZE_145] apalt={apalt_path}, frozen={bool(frozen)}")
            return True
        except Exception as exc:
            print(f"[VIRTUAL_APALT_FREEZE_WARN_145] frozen={frozen}, err={type(exc).__name__}: {exc}")
            return False

    def _virtual_apalt_prepare_cargo_follow_141(stage_now):
        """completed_box_roots 안의 상자들을 APalt 기준 상대 offset으로 저장한다."""
        nonlocal VIRTUAL_APALT_CARGO_ROOTS_141, VIRTUAL_APALT_CARGO_OFFSETS_141
        apalt_center = _virtual_apalt_get_center_141(stage_now)
        if apalt_center is None:
            print(f"[VIRTUAL_APALT_PREP_FAIL_141] APalt center를 읽지 못했습니다: {VIRTUAL_APALT_PATH_141}")
            VIRTUAL_APALT_CARGO_ROOTS_141 = []
            VIRTUAL_APALT_CARGO_OFFSETS_141 = {}
            return False

        _virtual_apalt_freeze_apalt_physics_145(stage_now, True)

        roots = []
        offsets = {}
        for root in sorted(str(x) for x in completed_box_roots):
            prim = stage_now.GetPrimAtPath(root)
            if not prim or not prim.IsValid():
                continue
            box_pos = get_world_translation(stage_now, root)
            if box_pos is None:
                bbox = get_world_bbox_info(stage_now, root)
                if bbox is None:
                    continue
                box_pos = bbox.get("center")
            box_pos = np.array(box_pos, dtype=float)
            offsets[root] = box_pos - apalt_center
            roots.append(root)
            _virtual_apalt_set_cargo_physics_frozen_141(stage_now, root, True)
            print(
                f"[VIRTUAL_APALT_CARGO_LOCK_141] root={root}, "
                f"box=({box_pos[0]:+.4f},{box_pos[1]:+.4f},{box_pos[2]:+.4f}), "
                f"apalt_center=({apalt_center[0]:+.4f},{apalt_center[1]:+.4f},{apalt_center[2]:+.4f}), "
                f"offset=({offsets[root][0]:+.4f},{offsets[root][1]:+.4f},{offsets[root][2]:+.4f})"
            )

        VIRTUAL_APALT_CARGO_ROOTS_141 = roots
        VIRTUAL_APALT_CARGO_OFFSETS_141 = offsets
        print(f"[VIRTUAL_APALT_PREP_141] cargo_count={len(roots)}, roots={roots}")
        return True

    def _virtual_apalt_update_cargo_follow_141(stage_now, label="follow", force=False):
        """APalt 현재 center + 저장된 offset으로 cargo 상자들을 계속 따라가게 한다."""
        apalt_center = _virtual_apalt_get_center_141(stage_now)
        if apalt_center is None:
            return False
        for root in list(VIRTUAL_APALT_CARGO_ROOTS_141):
            offset = VIRTUAL_APALT_CARGO_OFFSETS_141.get(root)
            if offset is None:
                continue
            target = apalt_center + np.array(offset, dtype=float)
            set_prim_world_translation(stage_now, root, target)
            zero_subtree_velocity(stage_now, root)
        if force:
            print(
                f"[VIRTUAL_APALT_FOLLOW_141][{label}] apalt_center=({apalt_center[0]:+.4f},{apalt_center[1]:+.4f},{apalt_center[2]:+.4f}), "
                f"cargo_count={len(VIRTUAL_APALT_CARGO_ROOTS_141)}"
            )
        return True

    async def forklift_lift_smooth_142(z_offset, speed=None):
        """142_: /World/APalt를 지게차 리프트처럼 천천히 Z 방향으로 상승시킨다.
        cargo 상자들은 APalt 중심 기준 offset을 유지하며 같이 따라간다.
        simulation_app.update()를 쓰지 않고 next_update_async()만 사용해서 asyncio 재진입 오류를 피한다.
        """
        import omni.kit.app
        stage_now = omni.usd.get_context().get_stage()
        apalt_path = str(VIRTUAL_APALT_PATH_141)
        prim = stage_now.GetPrimAtPath(apalt_path)
        if not prim or not prim.IsValid():
            print(f"[VIRTUAL_APALT_LIFT_SKIP_142] APalt prim을 찾지 못했습니다: {apalt_path}")
            return False

        root_pos = get_world_translation(stage_now, apalt_path)
        center_before = _virtual_apalt_get_center_141(stage_now)
        if root_pos is None or center_before is None:
            print(f"[VIRTUAL_APALT_LIFT_SKIP_142] APalt 위치를 읽지 못했습니다: {apalt_path}")
            return False

        root_pos = np.array(root_pos, dtype=float)
        start_z = float(root_pos[2])
        target_z = start_z + float(z_offset)
        direction = 1.0 if float(z_offset) >= 0 else -1.0
        speed = float(VIRTUAL_APALT_LIFT_SPEED_142 if speed is None else speed)
        dt = 1.0 / 60.0
        step = max(1e-6, abs(speed) * dt) * direction
        current_z = start_z
        lift_i = 0

        print(
            f"[VIRTUAL_APALT_LIFT_START_142] apalt={apalt_path}, "
            f"start_z={start_z:+.4f}, target_z={target_z:+.4f}, dz={float(z_offset):+.4f}, speed={speed:.3f}m/s"
        )
        _virtual_apalt_update_cargo_follow_141(stage_now, label="lift_start_142", force=True)

        while abs(current_z - target_z) > abs(step):
            current_z += step
            root_now = root_pos.copy()
            root_now[2] = current_z
            set_prim_world_translation(stage_now, apalt_path, root_now)
            _virtual_apalt_update_cargo_follow_141(stage_now, label=f"lift:{lift_i}", force=False)
            lift_i += 1
            if lift_i % int(max(1, VIRTUAL_APALT_LIFT_LOG_INTERVAL_142)) == 0:
                _virtual_apalt_update_cargo_follow_141(stage_now, label=f"lift:{lift_i}", force=True)
                print(f"[VIRTUAL_APALT_LIFT_142] step={lift_i}, z={current_z:+.4f}/{target_z:+.4f}")
            await omni.kit.app.get_app().next_update_async()

        root_final = root_pos.copy()
        root_final[2] = target_z
        set_prim_world_translation(stage_now, apalt_path, root_final)
        _virtual_apalt_update_cargo_follow_141(stage_now, label="lift_done_142", force=True)
        center_after = _virtual_apalt_get_center_141(stage_now)
        print(
            f"[VIRTUAL_APALT_LIFT_DONE_142] apalt={apalt_path}, steps={lift_i}, "
            f"center_before=({center_before[0]:+.4f},{center_before[1]:+.4f},{center_before[2]:+.4f}), "
            f"center_after=({center_after[0]:+.4f},{center_after[1]:+.4f},{center_after[2]:+.4f})"
        )
        return True

    async def forklift_move_axis_smooth_143(axis_index, target_offset, speed=0.5, axis_name="Y"):
        """143_: /World/APalt를 지정 축(X/Y)으로 천천히 이동시킨다.
        cargo 상자들은 APalt 중심 기준 offset을 유지하며 같이 따라간다.
        axis_index: 0=X, 1=Y, 2=Z
        """
        import omni.kit.app
        stage_now = omni.usd.get_context().get_stage()
        apalt_path = str(VIRTUAL_APALT_PATH_141)
        prim = stage_now.GetPrimAtPath(apalt_path)
        if not prim or not prim.IsValid():
            print(f"[VIRTUAL_APALT_MOVE_SKIP_143] APalt prim을 찾지 못했습니다: {apalt_path}")
            return False

        target_offset = float(target_offset)
        if abs(target_offset) < 1e-9:
            print(f"[VIRTUAL_APALT_MOVE_SKIP_143] {axis_name} 이동 거리=0.0 → 건너뜀")
            _virtual_apalt_update_cargo_follow_141(stage_now, label=f"move_{axis_name.lower()}_skip_143", force=True)
            return True

        root_pos = get_world_translation(stage_now, apalt_path)
        if root_pos is None:
            print(f"[VIRTUAL_APALT_MOVE_SKIP_143] APalt 위치를 읽지 못했습니다: {apalt_path}")
            return False
        root_pos = np.array(root_pos, dtype=float)
        start_v = float(root_pos[int(axis_index)])
        target_v = start_v + target_offset
        direction = 1.0 if target_offset > 0 else -1.0
        dt = 1.0 / 60.0
        step = max(1e-6, abs(float(speed)) * dt) * direction
        current_v = start_v
        move_i = 0

        _virtual_apalt_update_cargo_follow_141(stage_now, label=f"move_{axis_name.lower()}_start_143", force=True)
        print(
            f"[VIRTUAL_APALT_MOVE_{axis_name}_START_143] "
            f"start_{axis_name.lower()}={start_v:+.4f}, target_{axis_name.lower()}={target_v:+.4f}, "
            f"offset={target_offset:+.4f}, speed={float(speed):.3f}"
        )

        while abs(current_v - target_v) > abs(step):
            current_v += step
            root_now = root_pos.copy()
            root_now[int(axis_index)] = current_v
            set_prim_world_translation(stage_now, apalt_path, root_now)
            _virtual_apalt_update_cargo_follow_141(stage_now, label=f"move_{axis_name.lower()}:{move_i}", force=False)
            move_i += 1
            if move_i % int(max(1, VIRTUAL_APALT_LOG_INTERVAL_141)) == 0:
                _virtual_apalt_update_cargo_follow_141(stage_now, label=f"move_{axis_name.lower()}:{move_i}", force=True)
                print(f"[VIRTUAL_APALT_MOVE_{axis_name}_143] step={move_i}, {axis_name.lower()}={current_v:+.4f}/{target_v:+.4f}")
            await omni.kit.app.get_app().next_update_async()

        root_final = root_pos.copy()
        root_final[int(axis_index)] = target_v
        set_prim_world_translation(stage_now, apalt_path, root_final)
        _virtual_apalt_update_cargo_follow_141(stage_now, label=f"move_{axis_name.lower()}_done_143", force=True)
        print(
            f"[VIRTUAL_APALT_MOVE_{axis_name}_DONE_143] "
            f"apalt={apalt_path}, {axis_name} 이동 완료: {start_v:.3f} -> {target_v:.3f}"
        )
        return True

    async def forklift_move_x_smooth_143(target_x_offset, speed=0.5):
        """144_: 코드상 X축 이동 = 트럭 방향 전진/후진.
        사용자가 GUI에서 확인한 실제 축은 USD Translate Y이므로 axis_index=1을 움직인다.
        cargo 상자들은 APalt 중심 offset을 유지하며 같이 따라간다.
        """
        return await forklift_move_axis_smooth_143(1, target_x_offset, speed=speed, axis_name="X")

    async def forklift_move_y_smooth(target_y_offset, speed=0.5):
        """141_: /World/APalt를 Y로 이동시키고, cargo 상자들은 APalt 중심 offset 기준으로 따라가게 한다."""
        import omni.kit.app
        stage_now = omni.usd.get_context().get_stage()
        apalt_path = str(VIRTUAL_APALT_PATH_141)
        prim = stage_now.GetPrimAtPath(apalt_path)
        if not prim or not prim.IsValid():
            print(f"[VIRTUAL_APALT_MOVE_SKIP_141] APalt prim을 찾지 못했습니다: {apalt_path}")
            return False
        root_pos = get_world_translation(stage_now, apalt_path)
        if root_pos is None:
            print(f"[VIRTUAL_APALT_MOVE_SKIP_141] APalt 위치를 읽지 못했습니다: {apalt_path}")
            return False
        root_pos = np.array(root_pos, dtype=float)
        start_y = float(root_pos[1])
        target_y = start_y + float(target_y_offset)
        direction = 1.0 if target_y_offset > 0 else -1.0
        dt = 1.0 / 60.0
        step = max(1e-6, abs(float(speed)) * dt) * direction
        current_y = start_y
        move_i = 0
        _virtual_apalt_update_cargo_follow_141(stage_now, label="move_start", force=True)
        print(f"[VIRTUAL_APALT_MOVE_141] start_y={start_y:+.4f}, target_y={target_y:+.4f}, speed={float(speed):.3f}")

        while abs(current_y - target_y) > abs(step):
            current_y += step
            root_now = root_pos.copy()
            root_now[1] = current_y
            set_prim_world_translation(stage_now, apalt_path, root_now)
            _virtual_apalt_update_cargo_follow_141(stage_now, label=f"move:{move_i}", force=False)
            move_i += 1
            if move_i % int(max(1, VIRTUAL_APALT_LOG_INTERVAL_141)) == 0:
                _virtual_apalt_update_cargo_follow_141(stage_now, label=f"move:{move_i}", force=True)
            await omni.kit.app.get_app().next_update_async()

        root_final = root_pos.copy()
        root_final[1] = target_y
        set_prim_world_translation(stage_now, apalt_path, root_final)
        _virtual_apalt_update_cargo_follow_141(stage_now, label="move_done", force=True)
        print(f"[VIRTUAL_APALT_MOVE_DONE_141] apalt={apalt_path}, Y 이동 완료: {start_y:.3f} -> {target_y:.3f}")
        return True

    async def forklift_load_sequence():
        """144_: 코드상 Y축 리프트업 → 코드상 X축 전진 시퀀스.
        실제 USD 축 매핑은 Y축 리프트업=Translate Z, X축 전진=Translate Y 이다.
        예외가 나도 asyncio task가 죽지 않게 막는다.
        """
        import asyncio as _asyncio
        try:
            print(f"[VIRTUAL_PALLET] ===== {config.side} cell forklift start ({FORKLIFT_TRIGGER_COUNT} boxes) =====")
            stage_now = omni.usd.get_context().get_stage()
            _virtual_apalt_prepare_cargo_follow_141(stage_now)
            _virtual_apalt_update_cargo_follow_141(stage_now, label="sequence_start", force=True)

            # 1단계: 코드상 Y축 상승. 실제 USD에서는 Translate Z가 변한다.
            extra_lift_145 = float(stack_platform_lower_count) * float(BOX_STACK_HEIGHT_145)
            final_lift_z_145 = float(FORKLIFT_LIFT_TARGET_Z) + extra_lift_145
            print(f"[VIRTUAL_APALT] 1단계: APalt 코드상 Y축 천천히 상승 → base_y={FORKLIFT_LIFT_TARGET_Y}, lowered_layers={stack_platform_lower_count}, extra_y={extra_lift_145:.4f}, final_y={final_lift_z_145:.4f}, actual_USD_Z, speed={VIRTUAL_APALT_LIFT_SPEED_142}m/s")
            lift_ok_136 = await forklift_lift_smooth_142(final_lift_z_145, speed=VIRTUAL_APALT_LIFT_SPEED_142)
            if lift_ok_136:
                # 상승 완료 후 짧게 안정화. sync update 사용 금지.
                await _asyncio.sleep(float(FORKLIFT_LIFT_WAIT_SEC))
                _virtual_apalt_update_cargo_follow_141(stage_now, label="after_lift_wait_142", force=True)
            else:
                await _asyncio.sleep(0.2)
                print("[VIRTUAL_APALT][LIFT_WARN_142] lift는 건너뛰고 Y 이동 테스트로 진행합니다.")

            # 2단계: 코드상 X축 전진. 실제 USD에서는 Translate Y가 변한다.
            print(f"[VIRTUAL_APALT] 2단계: APalt 코드상 X축 전진 → x={FORKLIFT_MOVE_TARGET_X}m (actual_USD_Y, 속도={FORKLIFT_MOVE_SPEED}m/s)")
            move_x_ok_143 = await forklift_move_x_smooth_143(FORKLIFT_MOVE_TARGET_X, speed=FORKLIFT_MOVE_SPEED)

            # 3단계: 144번에서는 평면 Y 이동을 사용하지 않는다. 코드상 Y는 리프트 높이로만 사용한다.
            move_y_ok_136 = True

            _virtual_apalt_update_cargo_follow_141(stage_now, label="sequence_done", force=True)
            print(
                f"[VIRTUAL_APALT] ===== 시퀀스 완료: "
                f"lift_ok={lift_ok_136}, move_x_ok={move_x_ok_143}, move_y_ok={move_y_ok_136}, "
                f"cargo={VIRTUAL_APALT_CARGO_ROOTS_141} ====="
            )
        except Exception as exc:
            print(f"[FORKLIFT_SEQUENCE_ERROR_136] {type(exc).__name__}: {exc}")
            try:
                log_forklift_pose_135("sequence_error", force=True)
            except Exception:
                pass
    import asyncio as _asyncio_module

    async def forklift_delayed_start_sequence_139(release_count):
        """139_: release count가 올라간 뒤 지정 시간 카운트다운만 출력하고 forklift sequence를 시작한다."""
        try:
            delay_sec = int(max(0, FORKLIFT_START_DELAY_SEC_139))
            for remain in range(delay_sec, 0, -1):
                print(f"[FORKLIFT_DELAY_139] release_count={release_count}/{FORKLIFT_TRIGGER_COUNT}, start_after={remain}s")
                await _asyncio_module.sleep(1.0)
            await forklift_load_sequence()
        except Exception as exc:
            print(f"[FORKLIFT_DELAY_ERROR_139] {type(exc).__name__}: {exc}")

    def check_and_trigger_forklift():
        """release count가 FORKLIFT_TRIGGER_COUNT에 도달하면 5초 카운트다운 후 forklift sequence 실행."""
        nonlocal forklift_sequence_triggered
        if not config.forklift_enabled:
            return
        release_count = len(completed_box_roots)
        if (
            not forklift_sequence_triggered
            and release_count >= FORKLIFT_TRIGGER_COUNT
        ):
            forklift_sequence_triggered = True
            _asyncio_module.ensure_future(forklift_delayed_start_sequence_139(release_count))

    def _format_pose_145(v):
        if v is None:
            return "None"
        try:
            a = np.array(v, dtype=float)
            return f"({a[0]:+.4f},{a[1]:+.4f},{a[2]:+.4f})"
        except Exception:
            return str(v)

    def log_first_two_box_pose_145(stage_now, label, baseline=None, force=False):
        """APalt lowering 중 1번/2번 박스 위치 확인용 로그."""
        ap = _virtual_apalt_get_center_141(stage_now)
        msg = [f"[APALT_LOWER_BOX_TRACK_145][{label}] apalt_center={_format_pose_145(ap)}"]
        for root in config.pose_track_root_paths[:2]:
            pos = get_world_translation(stage_now, root)
            if pos is None:
                bbox = get_world_bbox_info(stage_now, root)
                pos = bbox.get("center") if bbox is not None else None
            delta_txt = ""
            if baseline is not None and root in baseline and pos is not None:
                d = np.array(pos, dtype=float) - np.array(baseline[root], dtype=float)
                delta_txt = f", d=({d[0]:+.4f},{d[1]:+.4f},{d[2]:+.4f})"
            msg.append(f"{root}={_format_pose_145(pos)}{delta_txt}")
        print(" | ".join(msg))

    pallet_lowering = PalletLoweringCoordinator(
        enabled=config.pallet_lower_during_home,
        use_home_steps=config.pallet_lower_use_home_steps,
        pallet_path=config.pallet_path,
        get_translation=get_world_translation,
        get_center=_virtual_apalt_get_center_141,
        set_translation=set_prim_world_translation,
        log_boxes=log_first_two_box_pose_145,
        lower_steps=STACK_LOWER_SUPPORT_STEPS,
        home_steps=HOME_RETURN_STEPS,
        extra_z=STACK_LOWER_EXTRA_Z,
        log_interval=APALT_LOWER_LOG_INTERVAL_145,
    )

    def slow_lower_apalt_after_first_layer_145(stage_now, box_height=0.24):
        """3개 이상 적재할 때, 1층 2개 후 APalt를 상자 높이만큼 천천히 내린다.
        이때 1번/2번 박스 좌표를 로그로 계속 찍어서 튐/밀림 여부를 확인한다.
        """
        apalt_path = str(VIRTUAL_APALT_PATH_141)
        cur = get_world_translation(stage_now, apalt_path)
        if cur is None:
            cur_center = _virtual_apalt_get_center_141(stage_now)
            if cur_center is None:
                print(f"[APALT_LOWER_SKIP_145] APalt 위치를 읽지 못했습니다: {apalt_path}")
                return False
            cur = cur_center
        cur = np.array(cur, dtype=float)
        dz = float(max(0.01, box_height)) + float(STACK_LOWER_EXTRA_Z)
        steps = int(max(1, STACK_LOWER_SUPPORT_STEPS))
        target = cur.copy()
        target[2] -= dz

        baseline = {}
        for root in config.pose_track_root_paths[:2]:
            pos = get_world_translation(stage_now, root)
            if pos is None:
                bbox = get_world_bbox_info(stage_now, root)
                pos = bbox.get("center") if bbox is not None else None
            if pos is not None:
                baseline[root] = np.array(pos, dtype=float)

        print(
            f"[APALT_LOWER_START_145] trigger_count={FORKLIFT_TRIGGER_COUNT}, dz={dz:.4f}, steps={steps}, "
            f"apalt_start={_format_pose_145(cur)}, apalt_target={_format_pose_145(target)}, note={APALT_LOWER_SPEED_NOTE_145}"
        )
        log_first_two_box_pose_145(stage_now, "start", baseline=baseline, force=True)

        for i in range(1, steps + 1):
            alpha = float(i) / float(steps)
            now = cur * (1.0 - alpha) + target * alpha
            set_prim_world_translation(stage_now, apalt_path, now)
            if (i % int(max(1, APALT_LOWER_LOG_INTERVAL_145)) == 0) or i == 1 or i == steps:
                log_first_two_box_pose_145(stage_now, f"step={i}/{steps}", baseline=baseline, force=True)
            my_world.step(render=True)
            time.sleep(0.005)

        set_prim_world_translation(stage_now, apalt_path, target)
        log_first_two_box_pose_145(stage_now, "done", baseline=baseline, force=True)
        print(f"[APALT_LOWER_DONE_145] APalt lowered by box_height. dz={dz:.4f}, final={_format_pose_145(target)}")
        return True



    def wait_after_release_before_apalt_lower_160(stage_now, label="after_release"):
        """160_: release 직후 APalt를 바로 움직이지 않고 1~2초 정도 물리 안정화 시간을 준다."""
        if not bool(globals().get("POST_RELEASE_SETTLE_BEFORE_APALT_LOWER_ENABLED_160", True)):
            return
        steps_wait = int(max(0, globals().get("POST_RELEASE_SETTLE_BEFORE_APALT_LOWER_STEPS_160", 120)))
        log_interval = int(max(1, globals().get("POST_RELEASE_SETTLE_LOG_INTERVAL_160", 30)))
        if steps_wait <= 0:
            return
        print(f"[POST_RELEASE_SETTLE_160] start label={label}, steps={steps_wait}. APalt lower delayed until boxes settle physically.")
        for i in range(1, steps_wait + 1):
            my_world.step(render=True)
            if i == 1 or i == steps_wait or (i % log_interval) == 0:
                log_first_two_box_pose_145(stage_now, f"post_release_wait={i}/{steps_wait}", baseline=None, force=True)
            time.sleep(0.005)
        print(f"[POST_RELEASE_SETTLE_160] done label={label}, steps={steps_wait}")

    def handle_stack_release_count_145(stage_now, task, label="release"):
        """release 이후 중앙 통제 함수.
        - completed_box_roots에 root 등록
        - stack_slot_index 증가
        - FORKLIFT_TRIGGER_COUNT 기준으로 APalt lowering / 최종 운반 시작 통제
        """
        nonlocal stack_slot_index, stack_platform_lower_count
        root = get_task_completed_root_path(task)
        completed_box_roots.add(root)
        release_count = len(completed_box_roots)
        print(
            f"[RELEASE_COUNT_145][{label}] release_count={release_count}/{FORKLIFT_TRIGGER_COUNT}, "
            f"slot={stack_slot_index + 1}/{STACK_SLOT_COUNT}, root={root}, completed={sorted(completed_box_roots)}"
        )
        stack_slot_index = min(stack_slot_index + 1, int(STACK_SLOT_COUNT))

        if should_stop_after_release_count(release_count):
            print(
                f"[DIAG_STOP_AFTER_RELEASE_164] release_count={release_count} reached. "
                "Stopping after configured release-count diagnosis. APalt lower/home return skipped."
            )
            return release_count

        # 3개 이상 테스트에서는 1층 2개 완료 후 APalt를 상자 높이만큼 내린다.
        if (
            int(FORKLIFT_TRIGGER_COUNT) >= 3
            and release_count == 2
            and int(stack_platform_lower_count) < 1
            and bool(STACK_LOWER_SUPPORT_AFTER_EACH_LAYER)
        ):
            box_height = float(globals().get("BOX_STACK_HEIGHT_145", 0.24))
            baseline = {}
            for box_root in config.pose_track_root_paths[:2]:
                box_pos = get_world_translation(stage_now, box_root)
                if box_pos is None:
                    box_bbox = get_world_bbox_info(stage_now, box_root)
                    box_pos = box_bbox.get("center") if box_bbox is not None else None
                if box_pos is not None:
                    baseline[box_root] = np.array(box_pos, dtype=float)
            ok_lower = pallet_lowering.schedule(
                stage_now,
                box_height=box_height,
                baseline=baseline,
                label=f"release_count={release_count}",
            )
            if not ok_lower:
                wait_after_release_before_apalt_lower_160(stage_now, label=f"release_count={release_count}")
                print("[PALLET_LOWER_REQUEST] 1층 2개 완료 → pallet를 상자 높이만큼 내립니다.")
                ok_lower = slow_lower_apalt_after_first_layer_145(stage_now, box_height=box_height)
            if ok_lower:
                stack_platform_lower_count += 1
            else:
                print("[PALLET_LOWER_FAIL] pallet lowering 실패. 로그 확인 필요")

        zone_counts_by_root.pop(task.box_move_path, None)
        check_and_trigger_forklift()
        return release_count

    def reset_attempt_state(reason="next attempt", ignore_after_success=False):
        """
        world.pause() 없이 다음 시도 대기 상태로 되돌린다.
        박스 위치는 코드로 되돌리지 않고, 다음 stable bbox를 다시 읽어서 pick target을 갱신한다.
        """
        nonlocal attached, ever_attached, released, returning_home
        nonlocal home_start_joints, home_target_joints, home_return_step
        nonlocal retry_logged, best_attach_reason, best_attach_dist
        nonlocal attached_center_offset, attach_center_z, attach_steps
        nonlocal custom_carry_active, custom_carry_phase, custom_phase_step, custom_targets
        nonlocal custom_fixed_orientation, custom_min_center_z, custom_phase_index
        nonlocal box_stopped_for_pick, pick_started, task_done
        nonlocal pre_attach_align_active, pre_attach_align_phase, pre_attach_align_step, pre_attach_fixed_orientation
        nonlocal loop_cycle_index, loop_attempt_index, ignore_released_center, ignore_wait_counter
        nonlocal stack_platform_lower_count

        stage_now = omni.usd.get_context().get_stage()
        pallet_lowering.reset()
        log_oribox_pose_tracker(stage_now, f"reset_attempt_before_restore:{reason}", force=True)

        # 다음 사이클 시작 전 물리 joint/carry 상태 복구
        # 66_: 완료된 박스는 절대 다시 dynamic/physics restore하지 않는다.
        #      이 구간에서 child rigid body가 parent root와 분리되며 순간이동처럼 보이는 현상을 막는다.
        try:
            release_physics_attach_joint(stage_now, reason="reset_attempt_state")
            completed_id = get_task_completed_root_path(task)
            if completed_id in completed_box_roots or str(task.box_path) in completed_box_roots:
                print(f"[LOOP] completed box locked. physics restore skipped: root={completed_id}, box={task.box_path}")
                zero_subtree_velocity(stage_now, task.box_path)
            else:
                set_box_scripted_carry_mode(stage_now, task.box_move_path, False, reenable_physics=False, verbose=False)
                set_prim_kinematic(stage_now, task.stop_check_path, True)
                zero_subtree_velocity(stage_now, task.box_move_path)
        except Exception as exc:
            print(f"[LOOP_WARN] carry mode restore failed: {exc}")

        if ignore_after_success and LOOP_IGNORE_RELEASED_BOX_UNTIL_MOVED:
            bbox_now = get_world_bbox_info(stage_now, task.box_path)
            if bbox_now is not None:
                ignore_released_center = np.array(bbox_now["center"], dtype=float)
                ignore_wait_counter = 0
                print(
                    f"[LOOP] released box ignore anchor=({ignore_released_center[0]:.3f},{ignore_released_center[1]:.3f},{ignore_released_center[2]:.4f}), "
                    f"move_tol={LOOP_RELEASED_BOX_IGNORE_MOVE_TOL:.3f}m"
                )
        else:
            ignore_released_center = None
            ignore_wait_counter = 0

        controller.reset()
        try:
            cart_controller.reset()
        except Exception:
            pass
        box_stop_detector.reset()
        pick_zone_detector.reset()
        try:
            zone_counts_by_root.clear()
        except Exception:
            pass

        attached = False
        ever_attached = False
        released = False
        returning_home = False
        home_start_joints = None
        home_target_joints = None
        home_return_step = 0
        retry_logged = False
        best_attach_reason = "none"
        best_attach_dist = 999.0
        attached_center_offset = None
        attach_center_z = None
        attach_steps = 0
        custom_carry_active = False
        custom_carry_phase = None
        custom_phase_step = 0
        custom_targets = None
        custom_fixed_orientation = None
        custom_min_center_z = None
        custom_phase_index = 0
        box_stopped_for_pick = False
        pick_started = False
        pre_attach_align_active = False
        pre_attach_align_phase = "idle"
        pre_attach_align_step = 0
        pre_attach_fixed_orientation = None
        task_done = False
        loop_cycle_index += 1
        loop_attempt_index += 1

        print(
            f"[LOOP] reset attempt state. cycle={loop_cycle_index}, attempt={loop_attempt_index}, reason={reason}\n"
            "       robot is home/standing. waiting for stopped Small_Cardboard_box again."
        )
        log_oribox_pose_tracker(stage_now, f"reset_attempt_after_restore:{reason}", force=True)

    def start_home_return(reason, success=False):
        """성공/실패 모두 pause하지 않고 home 복귀 후 다음 cycle로 넘긴다."""
        nonlocal returning_home, home_start_joints, home_target_joints, home_return_step
        nonlocal custom_carry_active, custom_carry_phase, custom_phase_step, custom_targets
        nonlocal home_return_reason, home_return_is_success
        nonlocal attached, attached_center_offset, attach_center_z, attach_steps
        nonlocal task_done

        # 134_: release 후 home-return 루프가 실행되도록 task_done을 반드시 풀어준다.
        # 기존 130에서는 release 직후 task_done=True가 먼저 걸리면
        # 메인 루프의 `if is_playing and not task_done:` 조건에 막혀 초기 자세 복귀가 실행되지 않을 수 있었다.
        task_done = False

        home_return_reason = str(reason)
        home_return_is_success = bool(success)
        custom_carry_active = False
        custom_carry_phase = None
        custom_phase_step = 0
        custom_targets = None

        # 실패 루트에서 혹시 attach가 일부 켜져 있었다면 안전하게 release 상태로 돌린다.
        if not success and attached:
            stage_now = omni.usd.get_context().get_stage()
            release_physics_attach_joint(stage_now, reason="failed_home_return")
            set_box_scripted_carry_mode(stage_now, task.box_move_path, False, reenable_physics=True, verbose=True)
            zero_subtree_velocity(stage, task.box_move_path)
            attached = False
            attached_center_offset = None
            attach_center_z = None
            attach_steps = 0

        returning_home = True
        home_start_joints = np.array(robot.get_joint_positions(), dtype=float)
        home_target_joints = np.zeros_like(home_start_joints) if HOME_TARGET_JOINTS_CONFIG is None else np.array(HOME_TARGET_JOINTS_CONFIG, dtype=float)
        home_return_step = 0
        stage_now = omni.usd.get_context().get_stage()
        log_oribox_pose_tracker(stage_now, f"start_home_return:{reason}", force=True)
        print(f"[LOOP_RETURN_134] reason={home_return_reason}, success={home_return_is_success}. returning robot home/initial pose. task_done={task_done}")

    def begin_grid_physics_attach(close_reason, suction_pos):
        """53_: pre-align 또는 기존 controller 경로에서 공통으로 쓰는 물리 흡착 시작 함수."""
        nonlocal attached, ever_attached, attached_center_offset, attach_center_z, attach_steps
        nonlocal custom_carry_active, custom_carry_phase, custom_phase_step, custom_targets
        nonlocal custom_fixed_orientation, custom_min_center_z, custom_phase_index
        nonlocal task_done

        stage_now = omni.usd.get_context().get_stage()
        bbox_for_attach = get_world_bbox_info(stage_now, task.box_path)
        if bbox_for_attach is None:
            print("[PHYSICS_ATTACH_ABORT] bbox_for_attach를 읽지 못했습니다.")
            if RUN_CONTINUOUS_LOOP and LOOP_RETRY_AFTER_ATTACH_FAIL:
                start_home_return("bbox_missing_before_attach", success=False)
            else:
                task_done = True
                my_world.pause()
            return False

        attach_center = np.array(bbox_for_attach["center"], dtype=float)
        grid_attach_center = _LAST_SUCTION_GRID_INFO.get("attach_center")
        if grid_attach_center is None:
            grid_attach_center = np.array(bbox_for_attach["top_center"], dtype=float)
            grid_attach_center[2] = float(grid_attach_center[2] + PHYSICS_ATTACH_TOP_SURFACE_EPS)
        grid_attach_center = np.array(grid_attach_center, dtype=float)
        top_center_for_attach = np.array(bbox_for_attach["top_center"], dtype=float)
        top_gap_at_attach = float(np.array(suction_pos, dtype=float)[2] - top_center_for_attach[2])
        if top_gap_at_attach > float(BOX_ATTACH_Z_MAX) + 1e-6:
            print(
                f"[PHYSICS_ATTACH_ABORT] suction is not on real top surface. "
                f"z_gap={top_gap_at_attach:.4f} > {BOX_ATTACH_Z_MAX:.4f}. retrying, no joint created."
            )
            if RUN_CONTINUOUS_LOOP and LOOP_RETRY_AFTER_ATTACH_FAIL:
                start_home_return("top_surface_gap_too_large_before_attach", success=False)
            else:
                task_done = True
                my_world.pause()
            return False
        # FixedJoint는 실제 윗면 좌표에 생성하지만, carry 계산용 offset은 현재 suction 중심 기준으로 유지한다.
        attached_center_offset = attach_center - np.array(suction_pos, dtype=float)

        joint_ok = True
        if PHYSICS_FIXED_JOINT_ATTACH_ENABLED:
            joint_ok = create_physics_attach_joint(stage_now, task.box_path, grid_attach_center)
        if not joint_ok:
            print("[PHYSICS_ATTACH_ABORT] FixedJoint 생성 실패. 박스 물리를 끄는 fallback은 사용하지 않고 재시도합니다.")
            if RUN_CONTINUOUS_LOOP and LOOP_RETRY_AFTER_ATTACH_FAIL:
                start_home_return("physics_joint_attach_failed", success=False)
            else:
                task_done = True
                my_world.pause()
            return False

        if not PHYSICS_FIXED_JOINT_ATTACH_ENABLED:
            # 56_: 실제 윗면 흡착 판정 후 FixedJoint 대신 top-lock 방식으로 운반한다.
            # collision/rigidBody를 끄지 않고 kinematic만 켜서 박스가 구르거나 튀지 않게 한다.
            set_prim_kinematic(stage, task.box_path, True)
            print("  [CENTER_TOP_LOCK_ATTACH] FixedJoint OFF fallback path. 물리 테스트에서는 이 로그가 나오면 안 됩니다.")

        zero_subtree_velocity(stage, task.box_move_path)
        attached = True
        ever_attached = True
        attach_center_z = float(attach_center[2])
        attach_steps = 0
        print(f"-surface on- {config.box_prefix}_ box attach reason={close_reason}")
        print(f"             grid_attach_center={grid_attach_center}")
        print(f"             attached_center_offset={attached_center_offset}")
        try:
            _slot_idx_attach_164 = int(getattr(task, "_stack_slot_index", stack_slot_index))
            axis_yaw_diagnostic(stage_now, task.box_move_path, _slot_idx_attach_164, label="after_attach_fixed_joint")
        except Exception as _axis_attach_exc_164:
            print(f"[BOX_AXIS_YAW_164][WARN] after_attach failed: {type(_axis_attach_exc_164).__name__}: {_axis_attach_exc_164}")

        if CUSTOM_CARRY_AFTER_ATTACH:
            custom_carry_active = True
            custom_phase_step = 0
            custom_phase_index = 0
            custom_min_center_z = float(attach_center_z)

            if CUSTOM_CARRY_MODE == "JOINT_SWING":
                custom_fixed_orientation = None
                custom_targets = make_joint_swing_carry_targets(
                    stage_now,
                    robot,
                    task,
                    attach_center=attach_center,
                    attach_suction_pos=suction_pos,
                    attached_center_offset=attached_center_offset,
                )
                custom_carry_phase = custom_targets["phase_sequence"][0].get("name", "joint_lift")
                print(
                    "[JOINT_CARRY_60_CENTER_BOXAPROP_PLACE] start. "
                    f"center_path={custom_targets.get('center_path')}, "
                    f"lift_delta={custom_targets['lift_delta']}, "
                    f"swing_delta={custom_targets['swing_delta']}, "
                    f"mirror_xy_estimate={custom_targets['mirror_xy_estimate']}, "
                    f"phases={[p['name'] for p in custom_targets.get('phase_sequence', [])]}"
                )
            else:
                custom_carry_phase = "lift"
                custom_fixed_orientation = get_current_ee_pose(robot)[1]
                custom_targets = make_custom_carry_targets(
                    stage_now,
                    task,
                    attach_center=attach_center,
                    attach_suction_pos=suction_pos,
                    attached_center_offset=attached_center_offset,
                )
                if custom_targets.get("phase_sequence"):
                    custom_carry_phase = custom_targets["phase_sequence"][0].get("name", "lift")
                cart_controller.reset()
                print(
                    "[SIMPLE_3STEP_117] start. step1 vertical lift -> step2 joint_1/link_1 rotate -> step3 reverse lower. "
                    f"safe_z={custom_targets['safe_z']:.3f}, "
                    f"lift_suction={custom_targets['lift_suction']}, "
                    f"move_suction={custom_targets['move_suction']}, "
                    f"lower_suction={custom_targets['lower_suction']}, "
                    f"place_center={custom_targets['place_center']}, "
                    f"vertical_only={custom_targets.get('vertical_only', False)}, "
                    f"phases={[p['name'] for p in custom_targets.get('phase_sequence', [])]}"
                )
        return True

    # Separate post-reset construction from the first control frame. This lets both
    # standalone and dual owners step the World before advancing runtime state.
    yield
    while True:
        # The standalone/dual lifecycle owner performs the single shared World step.
        is_playing = my_world.is_playing()

        if is_playing:
            stage_for_pose_track = omni.usd.get_context().get_stage()
            log_oribox_pose_tracker(stage_for_pose_track, "main_loop", force=False)
            if bool(globals().get("FORKLIFT_TRACK_MAIN_LOOP_139", False)):
                log_forklift_pose_135("main_loop", force=False)

            # 127_ watchdog release:
            # 126에서는 DROP_RELEASE_EXTRA_Z_125=0.10으로 release 높이는 올라갔지만,
            # phase 완료 블록에 못 들어가면 FixedJoint 제거 코드가 실행되지 않았다.
            # 이 블록은 상자가 이미 pick 위치에서 충분히 이동했고 높은 위치에 머무르면
            # custom phase 상태와 무관하게 흡착 joint를 제거한다.
            if (
                bool(globals().get("FORCE_DROP_RELEASE_WATCHDOG_127", True))
                and bool(attached)
                and (not bool(released))
                and str(CUSTOM_CARRY_MODE) == "VERTICAL_JOINT1_REVERSE"
            ):
                try:
                    bbox_watch_127 = get_world_bbox_info(stage_for_pose_track, task.box_path)
                    c_watch_127 = np.array(bbox_watch_127["center"], dtype=float) if bbox_watch_127 is not None else None
                    pick_ref_127 = np.array(task.pick_center, dtype=float) if task.pick_center is not None else None
                    if c_watch_127 is not None and pick_ref_127 is not None:
                        moved_xy_127 = float(np.linalg.norm(c_watch_127[:2] - pick_ref_127[:2]))
                        z_above_pick_127 = float(c_watch_127[2] - pick_ref_127[2])
                        if (
                            moved_xy_127 >= float(FORCE_DROP_RELEASE_MIN_XY_MOVE_127)
                            and z_above_pick_127 >= float(FORCE_DROP_RELEASE_MIN_Z_ABOVE_PICK_127)
                        ):
                            release_watchdog_counter_127 += 1
                        else:
                            release_watchdog_counter_127 = 0

                        if release_watchdog_counter_127 >= int(FORCE_DROP_RELEASE_WATCHDOG_STEPS_127):
                            print(
                                f"[WATCHDOG_DROP_RELEASE_127] trigger: moved_xy={moved_xy_127:.3f}m, "
                                f"z_above_pick={z_above_pick_127:.3f}m, counter={release_watchdog_counter_127}"
                            )
                            release_ok_127 = release_physics_attach_joint(stage_for_pose_track, reason="watchdog_drop_release_127")
                            attached = False
                            released = True
                            custom_carry_active = False
                            custom_carry_phase = None
                            attached_center_offset = None
                            attach_center_z = None
                            task_done = True
                            if MULTI_ORIBOX_STACKING_ENABLED:
                                handle_stack_release_count_145(stage_for_pose_track, task, label="watchdog_drop_release_127")
                            else:
                                completed_box_roots.add(get_task_completed_root_path(task))
                                check_and_trigger_forklift()

                            print(
                                f"-surface off- WATCHDOG_DROP_RELEASE_127: FixedJoint removed={release_ok_127}. "
                                "APalt transform was NOT changed. Box should fall if it is dynamic and not supported."
                            )
                            log_oribox_pose_tracker(stage_for_pose_track, "watchdog_after_drop_release_127_begin", force=True)

                            observe_steps_127 = int(max(0, FORCE_DROP_RELEASE_OBSERVE_STEPS_127))
                            log_interval_127 = int(max(1, FORCE_DROP_RELEASE_LOG_INTERVAL_127))
                            for drop_i_127 in range(observe_steps_127):
                                try:
                                    update_vgc10_suction_anchor(robot)
                                except Exception:
                                    pass
                                my_world.step(render=True)
                                if drop_i_127 % log_interval_127 == 0 or drop_i_127 == observe_steps_127 - 1:
                                    log_oribox_pose_tracker(stage_for_pose_track, f"watchdog_drop_release_127:{drop_i_127}", force=True)

                            if bool(RETURN_HOME_AFTER_WATCHDOG_DROP_127):
                                start_home_return("watchdog_drop_release_127_after_observe", success=True)
                            was_playing = is_playing
                            yield
                            continue
                except Exception as e:
                    print(f"[WATCHDOG_DROP_RELEASE_127_WARN] {type(e).__name__}: {e}")

        if is_playing and not was_playing:
            my_world.reset()
            stage = omni.usd.get_context().get_stage()
            remove_old_gripper(stage, verbose=False)
            initialize_robot_for_conveyor(robot, my_world)

            # 18_: 이전 run에서 scripted carry 때문에 남은 kinematic/collision off 상태를 먼저 복구한다.
            set_box_scripted_carry_mode(stage, task.box_move_path, False, reenable_physics=True, verbose=True)
            set_prim_kinematic(stage, task.stop_check_path, False)
            zero_subtree_velocity(stage, task.box_move_path)

            # 20_: 박스 이동 자체는 컨베이어가 담당한다.
            # Stop→Play 때도 박스 위치를 코드로 강제 복구하지 않는다.
            if RESET_BOX_TO_USD_START_ON_PLAY:
                set_prim_world_translation(stage, task.box_move_path, task._box_initial_root_pos)
                zero_subtree_velocity(stage, task.box_move_path)

            controller.reset()
            attached = False
            ever_attached = False
            released = False
            returning_home = False
            home_start_joints = None
            home_target_joints = None
            home_return_step = 0
            retry_logged = False
            best_attach_reason = "none"
            best_attach_dist = 999.0
            attached_center_offset = None
            attach_center_z = None
            attach_steps = 0
            custom_carry_active = False
            custom_carry_phase = None
            custom_phase_step = 0
            custom_targets = None
            custom_fixed_orientation = None
            custom_min_center_z = None
            custom_phase_index = 0
            cart_controller.reset()
            box_stop_detector.reset()
            pick_zone_detector.reset()
            completed_box_roots = set()
            zone_counts_by_root = {}
            stack_slot_index = 0
            stack_platform_lower_count = 0
            stack_complete_logged = False
            box_stopped_for_pick = False
            pick_started = False
            pre_attach_align_active = False
            pre_attach_align_phase = "idle"
            pre_attach_align_step = 0
            pre_attach_fixed_orientation = None
            task_done = False

            print("-surface off- reset")
            print(f"[CELL] Conveyor_lift_test + {config.pallet_name} pallet + root-coordinate carry")
            print(f"     pick manual offset = {PICK_TARGET_MANUAL_OFFSET}  # 53_: pre-align 사용, manual offset은 기본 0")
            print(f"     pre-align = enabled={PRE_ATTACH_ALIGN_ENABLED}, xy_tol={PRE_ATTACH_ALIGN_XY_TOL}, high_gap={PRE_ATTACH_ALIGN_HEIGHT}, contact_gap={PRE_ATTACH_CONTACT_GAP}, z_gap_ok=[{BOX_ATTACH_Z_MIN},{BOX_ATTACH_Z_MAX}], keep_orientation={PRE_ATTACH_KEEP_CURRENT_ORIENTATION}")
            print(f"     suction grid eval = world_xy={SUCTION_GRID_EVALUATE_AS_WORLD_XY_GRID}, marker_root={SUCTION_GRID_WORLD_MARKER_ROOT_PATH}")
            print(f"     return home immediately after attach = {RETURN_HOME_IMMEDIATELY_AFTER_ATTACH}")
            print(f"     reset box to USD start on play = {RESET_BOX_TO_USD_START_ON_PLAY}  # False면 박스 위치를 코드로 안 움직임")
            print(f"     trigger mode = {PICK_TRIGGER_MODE} / ready_zone_{config.side}_stop={PICK_ZONE_REQUIRE_STOPPED_BEFORE_PICK}")
            print(f"     pick zone    = path={PICK_ZONE_PATH}, center={PICK_ZONE_CENTER}, size={PICK_ZONE_SIZE}, required_steps={PICK_ZONE_REQUIRED_STEPS}, zero_velocity_on_start={PICK_ZONE_ZERO_BOX_VELOCITY_ON_PICK_START}")
            print(f"     stack mode   = enabled={MULTI_ORIBOX_STACKING_ENABLED}, parent={ORIBOX_STACK_PARENT_PATH}, prefixes={ORIBOX_STACK_NAME_PREFIXES}, support={config.pallet_path}, columns={STACK_COLUMNS}, layers={STACK_LAYERS}, slots={STACK_SLOT_COUNT}, axis={STACK_SLOT_AXIS}, lower_platform={STACK_LOWER_SUPPORT_AFTER_EACH_LAYER}")
            print(f"     pose tracker = enabled={POSE_TRACK_ENABLED}, roots={POSE_TRACK_ROOT_PATHS}, every_step={POSE_TRACK_LOG_EVERY_STEP}, interval={POSE_TRACK_LOG_INTERVAL}, jump_tol={POSE_TRACK_JUMP_WARN_TOL}m")
            print(f"     legacy ready zone: y <= {BOX_READY_MIN_Y}, center_z <= {BOX_READY_MAX_CENTER_Z}")
            print(f"     custom carry mode = {CUSTOM_CARRY_MODE}, safe suction z = {CUSTOM_CARRY_SAFE_SUCTION_Z}m, lift delta={CUSTOM_LIFT_DELTA_Z}m, support mesh={config.pallet_path}, snap=False, upright_orientation_lock=True, drop_release_124=True")
            print(f"     joint carry 30_loop_28_fast_vertical_first: lift_steps={JOINT_LIFT_STEPS}, extra_hold={JOINT_LIFT_EXTRA_HOLD_STEPS}, lift_sign={JOINT_LIFT_SIGN}, j2_delta={JOINT_LIFT_J2_DELTA_RAD}, j3_delta={JOINT_LIFT_J3_DELTA_RAD if JOINT_USE_J3_FOR_LIFT else 0.0}, swing_sign={JOINT_SWING_SIGN}, swing_delta={JOINT_SWING_DELTA_RAD}, clamp={JOINT_SWING_CLAMP_RAD}")
            print(f"     target box      = {task.box_path}")
            print(f"     move wrapper    = {task.box_move_path}")
            print(f"     stop check only = {task.stop_check_path}")
            print(f"     screenshot local translate 참고 = {BOX_SCREENSHOT_LOCAL_TRANSLATE}")
            print(f"     stop gate mode  = bbox_move:{BOX_STOP_USE_BBOX_MOVE}, linear_vel:{BOX_STOP_USE_LINEAR_VEL}, angular_vel:{BOX_STOP_USE_ANGULAR_VEL}")
            print("     8_ note: ang_vel 값이 커도 기본값에서는 INFO_ONLY라 정지 판정을 막지 않음")
            log_oribox_pose_tracker(stage, "after_play_reset", force=True)

        if is_playing and not task_done:
            stage = omni.usd.get_context().get_stage()

            # ------------------------------------------------------------
            # 8_ 핵심: 선택된 /World/OriBoxA/Small_Cardboard_box prim 하나만 정지 판정 대상으로 사용한다.
            # Conveyor 위에서 박스가 이동 중이면 bbox center 변화량이 계속 크므로 여기서 대기한다.
            # 멈춘 순간의 현재 박스 위치를 pick 위치로 다시 확정한다.
            # ------------------------------------------------------------
            if WAIT_UNTIL_BOX_STOPPED_BEFORE_PICK and (not box_stopped_for_pick) and (not attached) and (not released):
                stack_candidate = None
                stack_candidate_ready = False
                stack_candidate_reason = "multi_stack_disabled"

                if MULTI_ORIBOX_STACKING_ENABLED:
                    if len(completed_box_roots) >= int(FORKLIFT_TRIGGER_COUNT):
                        hold_robot_current_pose(robot)
                        update_vgc10_suction_anchor(robot)
                        if not stack_complete_logged:
                            print(f"[STACK_TRIGGER_COMPLETE] release_count={len(completed_box_roots)}/{FORKLIFT_TRIGGER_COUNT}. {config.pallet_name} completion condition reached; further picks blocked.")
                            stack_complete_logged = True
                        was_playing = is_playing
                        yield
                        continue

                    if stack_slot_index >= int(STACK_SLOT_COUNT) and not bool(globals().get("ALLOW_CONTINUE_AFTER_STACK_FULL_137", False)):
                        hold_robot_current_pose(robot)
                        update_vgc10_suction_anchor(robot)
                        if not stack_complete_logged:
                            print(f"[STACK_COMPLETE] {STACK_COLUMNS}x{STACK_LAYERS} = {STACK_SLOT_COUNT} slots complete. No more {config.box_prefix}_ boxes will be picked.")
                            stack_complete_logged = True
                        was_playing = is_playing
                        yield
                        continue
                    elif stack_slot_index >= int(STACK_SLOT_COUNT) and bool(globals().get("ALLOW_CONTINUE_AFTER_STACK_FULL_137", False)):
                        if not stack_complete_logged:
                            print(f"[STACK_CONTINUE_137] stack_slot_index={stack_slot_index}, slot_count={STACK_SLOT_COUNT}. forklift/continuous test라서 stack full 정지를 무시합니다.")
                            stack_complete_logged = True

                    stack_candidate, stack_candidate_ready, stack_candidate_reason = select_stack_candidate_in_pick_zone(
                        stage,
                        completed_roots=completed_box_roots,
                        zone_counts_by_root=zone_counts_by_root,
                    )
                    if stack_candidate is not None:
                        task.set_active_box(
                            stack_candidate["root_path"],
                            stack_candidate["box_path"],
                            stack_slot_index=stack_slot_index,
                        )
                        bbox_wait = stack_candidate["bbox"]
                    else:
                        bbox_wait = None
                else:
                    bbox_wait = get_world_bbox_info(stage, task.stop_check_path)

                suction_wait = update_vgc10_suction_anchor(robot)
                hold_robot_current_pose(robot)

                # 30_: 성공 후 방금 내려놓은 같은 박스를 바로 다시 집지 않도록,
                # release anchor에서 충분히 멀어질 때까지 다음 pick을 막는다.
                if LOOP_IGNORE_RELEASED_BOX_UNTIL_MOVED and ignore_released_center is not None and bbox_wait is not None:
                    cur_center = np.array(bbox_wait["center"], dtype=float)
                    moved_xy = float(np.linalg.norm(cur_center[:2] - np.array(ignore_released_center, dtype=float)[:2]))
                    if moved_xy < float(LOOP_RELEASED_BOX_IGNORE_MOVE_TOL):
                        ignore_wait_counter += 1
                        box_stop_detector.reset()
                        pick_zone_detector.reset()
                        if ignore_wait_counter % LOOP_WAIT_LOG_INTERVAL == 0:
                            print(
                                f"  [loop_ignore_released_box] moved_xy={moved_xy:.4f}/{LOOP_RELEASED_BOX_IGNORE_MOVE_TOL:.4f}. "
                                "waiting until this prim moves away or a new box arrives."
                            )
                        was_playing = is_playing
                        yield
                        continue
                    else:
                        print(
                            f"[LOOP] released-box ignore cleared. moved_xy={moved_xy:.4f} >= {LOOP_RELEASED_BOX_IGNORE_MOVE_TOL:.4f}."
                        )
                        ignore_released_center = None
                        ignore_wait_counter = 0
                        box_stop_detector.reset()
                        pick_zone_detector.reset()

                if PICK_TRIGGER_MODE == "FRONT_ZONE":
                    if MULTI_ORIBOX_STACKING_ENABLED:
                        zone_ready, zone_reason = stack_candidate_ready, stack_candidate_reason
                    else:
                        zone_ready, zone_reason = pick_zone_detector.update(bbox_wait)

                    stopped_now = False
                    stop_reason = "stop_check_waiting_for_zone"
                    if zone_ready and bool(PICK_ZONE_REQUIRE_STOPPED_BEFORE_PICK):
                        stopped_now, stop_reason = box_stop_detector.update(stage, task.stop_check_path, bbox_wait)
                    elif zone_ready:
                        stopped_now = True
                        stop_reason = "stop_check_disabled"
                    else:
                        box_stop_detector.reset()

                    log_front_zone = PICK_ZONE_LOG_EVERY_STEP or (pick_zone_detector.total_steps % PICK_ZONE_LOG_INTERVAL == 0) or zone_ready
                    if log_front_zone:
                        if bbox_wait is not None:
                            c = bbox_wait["center"]
                            t = bbox_wait["top_center"]
                            print(
                                f"  [wait_ready_zone_{config.side}] {zone_reason} | {stop_reason} "
                                f"box_center=({c[0]:.3f},{c[1]:.3f},{c[2]:.4f}) "
                                f"box_top=({t[0]:.3f},{t[1]:.3f},{t[2]:.4f}) "
                                f"suction=({suction_wait[0]:.3f},{suction_wait[1]:.3f},{suction_wait[2]:.4f})"
                            )
                        else:
                            print(f"  [wait_ready_zone_{config.side}] {zone_reason} | {stop_reason}")

                    if zone_ready and stopped_now:
                        if not set_task_pick_from_current_box(task, stage, bbox_wait):
                            print(f"[FAIL] ready zone {config.side} + stopped passed but pick position update failed. pausing world.")
                            task_done = True
                            my_world.pause()
                            was_playing = is_playing
                            yield
                            continue

                        if PICK_ZONE_ZERO_BOX_VELOCITY_ON_PICK_START:
                            zero_subtree_velocity(stage, task.box_move_path)

                        controller.reset()
                        try:
                            cart_controller.reset()
                        except Exception:
                            pass
                        box_stopped_for_pick = True  # 이후 should_attach_oribox의 gate를 통과시키기 위한 내부 flag
                        pick_started = True
                        pre_attach_align_active = bool(PRE_ATTACH_ALIGN_ENABLED)
                        pre_attach_align_phase = "xy"
                        pre_attach_align_step = 0
                        pre_attach_fixed_orientation = get_current_ee_pose(robot)[1] if bool(PRE_ATTACH_KEEP_CURRENT_ORIENTATION) else None
                        try:
                            yaw_diagnostic_against_slot(stage, task.box_move_path, int(stack_slot_index), label="pre_pick_selected")
                            axis_yaw_diagnostic(stage, task.box_move_path, int(stack_slot_index), label="pre_pick_selected")
                        except Exception as _pre_yaw_exc_163:
                            print(f"[PRE_PICK_YAW_163][WARN] failed: {type(_pre_yaw_exc_163).__name__}: {_pre_yaw_exc_163}")
                        print(
                            f"-ready zone {config.side} {config.box_prefix}_ stopped target detected- pre-align enabled root={get_task_completed_root_path(task)} slot={stack_slot_index + 1}/{STACK_SLOT_COUNT} orientation_mode={'fixed_current' if pre_attach_fixed_orientation is not None else 'free_position_only'} "
                            f"box_center=({task.box_initial_center[0]:.3f},{task.box_initial_center[1]:.3f},{task.box_initial_center[2]:.4f}) "
                            f"box_top=({task.box_initial_top_center[0]:.3f},{task.box_initial_top_center[1]:.3f},{task.box_initial_top_center[2]:.4f}) "
                            f"pick_target=({task.pick_center[0]:.3f},{task.pick_center[1]:.3f},{task.pick_center[2]:.4f}) "
                            f"goal_center=({task.goal_center[0]:.3f},{task.goal_center[1]:.3f},{task.goal_center[2]:.4f})"
                        )
                        log_oribox_pose_tracker(stage, f"ready_zone_{config.side}_selected:{task.box_move_path}", force=True)

                    was_playing = is_playing
                    yield
                    continue

                stopped_now, stop_reason = box_stop_detector.update(stage, task.stop_check_path, bbox_wait)

                if BOX_STOP_LOG_EVERY_STEP or (box_stop_detector.total_steps % BOX_STOP_LOG_INTERVAL == 0) or stopped_now:
                    if bbox_wait is not None:
                        c = bbox_wait["center"]
                        t = bbox_wait["top_center"]
                        print(
                            f"  [wait_box_stop] {stop_reason} "
                            f"box_center=({c[0]:.3f},{c[1]:.3f},{c[2]:.4f}) "
                            f"box_top=({t[0]:.3f},{t[1]:.3f},{t[2]:.4f}) "
                            f"suction=({suction_wait[0]:.3f},{suction_wait[1]:.3f},{suction_wait[2]:.4f})"
                        )
                    else:
                        print(f"  [wait_box_stop] {stop_reason}")

                if stopped_now:
                    ready_ok, ready_reason = is_box_ready_for_pick_zone(bbox_wait)
                    if not ready_ok:
                        if bbox_wait is not None:
                            c = bbox_wait["center"]
                            print(
                                f"  [wait_box_ready] stable_but_not_pick_zone reason={ready_reason} "
                                f"box_center=({c[0]:.3f},{c[1]:.3f},{c[2]:.4f}). 계속 컨베이어 이동 대기."
                            )
                        box_stop_detector.reset()
                        pick_zone_detector.reset()
                        was_playing = is_playing
                        yield
                        continue

                    if not set_task_pick_from_current_box(task, stage, bbox_wait):
                        print("[FAIL] box stopped check passed but pick position update failed. pausing world.")
                        task_done = True
                        my_world.pause()
                        was_playing = is_playing
                        yield
                        continue

                    # 18_: 박스는 컨베이어가 멈춰준 상태를 그대로 사용한다. 강제 freeze는 하지 않는다.
                    zero_subtree_velocity(stage, task.box_move_path)
                    if BOX_FREEZE_AFTER_STOP:
                        set_prim_kinematic(stage, task.stop_check_path, True)
                        zero_subtree_velocity(stage, task.box_move_path)

                    controller.reset()
                    box_stopped_for_pick = True
                    pick_started = True
                    try:
                        yaw_diagnostic_against_slot(stage, task.box_move_path, int(stack_slot_index), label="pre_pick_selected_single")
                        axis_yaw_diagnostic(stage, task.box_move_path, int(stack_slot_index), label="pre_pick_selected_single")
                    except Exception as _pre_yaw_exc_163:
                        print(f"[PRE_PICK_YAW_163][WARN] failed: {type(_pre_yaw_exc_163).__name__}: {_pre_yaw_exc_163}")
                    print(
                        f"-selected Small_Cardboard_box stopped- pick enabled "
                        f"box_center=({task.box_initial_center[0]:.3f},{task.box_initial_center[1]:.3f},{task.box_initial_center[2]:.4f}) "
                        f"box_top=({task.box_initial_top_center[0]:.3f},{task.box_initial_top_center[1]:.3f},{task.box_initial_top_center[2]:.4f}) "
                        f"pick_target=({task.pick_center[0]:.3f},{task.pick_center[1]:.3f},{task.pick_center[2]:.4f}) "
                        f"goal_center=({task.goal_center[0]:.3f},{task.goal_center[1]:.3f},{task.goal_center[2]:.4f})"
                    )

                was_playing = is_playing
                yield
                continue

            if returning_home:
                # 16_ 핵심: 흡착된 상태로 로봇을 0 joint 초기 자세로 복귀한다.
                # 박스는 VGC10 suction point를 계속 따라가게 유지한다.
                if home_start_joints is None:
                    home_start_joints = np.array(robot.get_joint_positions(), dtype=float)
                if home_target_joints is None:
                    if HOME_TARGET_JOINTS_CONFIG is None:
                        home_target_joints = np.zeros_like(home_start_joints)
                    else:
                        home_target_joints = np.array(HOME_TARGET_JOINTS_CONFIG, dtype=float)

                alpha = min(1.0, float(home_return_step) / float(max(1, HOME_RETURN_STEPS)))
                smooth = alpha * alpha * (3.0 - 2.0 * alpha)
                target_joints = (1.0 - smooth) * home_start_joints + smooth * home_target_joints
                robot.apply_action(ArticulationAction(joint_positions=target_joints))

                suction_pos_now = update_vgc10_suction_anchor(robot)
                pallet_lowering.update(stage)

                if attached and KEEP_BOX_ATTACHED_DURING_HOME_RETURN:
                    if attached_center_offset is None:
                        bbox_for_follow = get_world_bbox_info(stage, task.box_path)
                        follow_center = np.array(bbox_for_follow["center"], dtype=float) if bbox_for_follow is not None else np.zeros(3)
                        attached_center_offset = follow_center - np.array(suction_pos_now, dtype=float)

                    desired_box_center = np.array(suction_pos_now, dtype=float) + np.array(attached_center_offset, dtype=float)
                    if BOX_DISABLE_PHYSICS_DURING_CARRY:
                        set_box_scripted_carry_mode(stage, task.box_move_path, True, verbose=False)
                    move_box_center_to(
                        stage,
                        task.box_move_path,
                        desired_center=desired_box_center,
                        root_to_center_offset=task.box_root_to_center_offset,
                    )
                    zero_subtree_velocity(stage, task.box_move_path)
                else:
                    zero_prim_velocity(stage, task.box_path)

                bbox_now = get_world_bbox_info(stage, task.box_path)

                if home_return_step % 10 == 0 or alpha >= 1.0:
                    box_center_now = bbox_now["center"] if bbox_now is not None else np.zeros(3)
                    print(
                        f"  [return_home_attach] step={home_return_step}/{HOME_RETURN_STEPS} "
                        f"alpha={alpha:.2f} "
                        f"box=({box_center_now[0]:.3f},{box_center_now[1]:.3f},{box_center_now[2]:.4f}) "
                        f"suction=({suction_pos_now[0]:.3f},{suction_pos_now[1]:.3f},{suction_pos_now[2]:.4f}) "
                        f"surface={'ON' if attached else 'OFF'}"
                    )
                    log_oribox_pose_tracker(stage, f"return_home:{home_return_step}", force=True)

                home_return_step += 1
                if alpha >= 1.0 and not pallet_lowering.active:
                    # 18_/30_: 어떤 경우에도 다음 실행 때 박스가 공중에 붙어 있지 않도록 carry mode를 해제한다.
                    if attached:
                        set_box_scripted_carry_mode(stage, task.box_move_path, False, reenable_physics=True, verbose=True)
                        zero_subtree_velocity(stage, task.box_move_path)
                        attached = False

                    if RUN_CONTINUOUS_LOOP:
                        print(f"[LOOP_HOME_DONE] robot returned home. reason={home_return_reason}, success={home_return_is_success}")
                        reset_attempt_state(
                            reason=f"home_done:{home_return_reason}",
                            ignore_after_success=bool(home_return_is_success),
                        )
                    else:
                        print("[완료] robot returned home. box is not kept attached in the air. pausing world.")
                        task_done = True
                        my_world.pause()

                was_playing = is_playing
                yield
                continue

            # ------------------------------------------------------------
            # 53_ 핵심: 작업영역 진입 후 box_top 좌표로 suction 중심을 먼저 맞춘다.
            # 기존 52_ 로그에서 box_top은 읽었지만 suction_x가 약 0.27m 벗어나 grid_hits=0/9였다.
            # 여기서는 PickPlaceController event로 내려가기 전에 RMPFlow로 직접
            #   1) box_top 위 높은 위치에서 XY 정렬
            #   2) 같은 XY에서 Z 접근
            #   3) 9점 흡착 판정 OK면 FixedJoint 생성
            # 순서로 처리한다.
            # ------------------------------------------------------------
            if pre_attach_align_active and (not attached) and (not released):
                bbox_now = get_world_bbox_info(stage, task.box_path)
                if bbox_now is None:
                    print("[PRE_ALIGN_FAIL] bbox를 읽지 못했습니다. home return 후 재시도합니다.")
                    if RUN_CONTINUOUS_LOOP and LOOP_RETRY_AFTER_ATTACH_FAIL:
                        start_home_return("prealign_bbox_missing", success=False)
                    else:
                        task_done = True
                        my_world.pause()
                    was_playing = is_playing
                    yield
                    continue

                suction_now = update_vgc10_suction_anchor(robot)
                top = np.array(bbox_now["top_center"], dtype=float)
                suction_now = np.array(suction_now, dtype=float)
                xy_err = float(np.linalg.norm(suction_now[:2] - top[:2]))
                z_gap = float(suction_now[2] - top[2])

                if pre_attach_align_phase == "xy":
                    desired_suction = np.array([top[0], top[1], top[2] + float(PRE_ATTACH_ALIGN_HEIGHT)], dtype=float)
                    desired_z_err = float(abs(suction_now[2] - desired_suction[2]))
                    if xy_err <= float(PRE_ATTACH_ALIGN_XY_TOL) and desired_z_err <= float(PRE_ATTACH_ALIGN_Z_TOL):
                        pre_attach_align_phase = "z"
                        try:
                            cart_controller.reset()
                        except Exception:
                            pass
                        print(
                            f"[PRE_ALIGN_PHASE] XY aligned. switching to Z approach. "
                            f"xy_err={xy_err:.4f}, z_gap={z_gap:.4f}, box_top=({top[0]:.3f},{top[1]:.3f},{top[2]:.4f})"
                        )
                else:
                    desired_suction = np.array([top[0], top[1], top[2] + float(PRE_ATTACH_CONTACT_GAP)], dtype=float)

                apply_cartesian_suction_target(
                    cart_controller,
                    robot,
                    current_suction_pos=suction_now,
                    desired_suction_pos=desired_suction,
                    fixed_orientation=pre_attach_fixed_orientation,
                )
                suction_after = update_vgc10_suction_anchor(robot)
                suction_after = np.array(suction_after, dtype=float)
                xy_err_after = float(np.linalg.norm(suction_after[:2] - top[:2]))
                z_gap_after = float(suction_after[2] - top[2])

                grid_ok, grid_summary, _grid_info = evaluate_suction_grid_on_box_top(
                    stage,
                    bbox_now,
                    event="prealign",
                    verbose=bool(PRE_ATTACH_EVALUATE_GRID_EVERY_STEP),
                )

                if (pre_attach_align_step % int(max(1, PRE_ATTACH_LOG_INTERVAL)) == 0) or grid_ok:
                    print(
                        f"  [PRE_ALIGN] step={pre_attach_align_step}/{PRE_ATTACH_MAX_STEPS}, phase={pre_attach_align_phase}, "
                        f"box_center=({bbox_now['center'][0]:.3f},{bbox_now['center'][1]:.3f},{bbox_now['center'][2]:.4f}), "
                        f"box_top=({top[0]:.3f},{top[1]:.3f},{top[2]:.4f}), "
                        f"suction=({suction_after[0]:.3f},{suction_after[1]:.3f},{suction_after[2]:.4f}), "
                        f"target=({desired_suction[0]:.3f},{desired_suction[1]:.3f},{desired_suction[2]:.4f}), "
                        f"xy_err={xy_err_after:.4f}/{PRE_ATTACH_ALIGN_XY_TOL:.4f}, "
                        f"z_gap={z_gap_after:.4f}, grid_ok={grid_ok}"
                    )

                if grid_ok and pre_attach_align_step >= int(PRE_ATTACH_MIN_STEPS_BEFORE_ATTACH):
                    pre_attach_align_active = False
                    pre_attach_align_phase = "done"
                    begin_grid_physics_attach("prealign_" + str(grid_summary), suction_after)
                    was_playing = is_playing
                    yield
                    continue

                pre_attach_align_step += 1
                if pre_attach_align_step >= int(PRE_ATTACH_MAX_STEPS):
                    print(
                        f"[PRE_ALIGN_FAIL] max steps reached. xy_err={xy_err_after:.4f}, z_gap={z_gap_after:.4f}, "
                        f"grid={grid_summary}. home return 후 재시도합니다."
                    )
                    if RUN_CONTINUOUS_LOOP and LOOP_RETRY_AFTER_ATTACH_FAIL:
                        start_home_return("prealign_timeout", success=False)
                    else:
                        task_done = True
                        my_world.pause()

                was_playing = is_playing
                yield
                continue

            if custom_carry_active and attached and not released:
                # 23_ 핵심: attach 이후 PickPlaceController event를 더 진행하지 않는다.
                # 로봇 suction point를 직접 lift -> move -> lower target으로 보낸다.
                suction_now = update_vgc10_suction_anchor(robot)
                bbox_now = get_world_bbox_info(stage, task.box_path)
                box_center_now = bbox_now["center"] if bbox_now is not None else np.zeros(3)

                if custom_targets is None:
                    custom_targets = make_custom_carry_targets(
                        stage,
                        task,
                        attach_center=box_center_now,
                        attach_suction_pos=suction_now,
                        attached_center_offset=attached_center_offset,
                    )

                # 25_ 관절 기반 운반 모드.
                # 기존 cartesian waypoint 방식은 target_suction으로 로봇을 끌고 가면서
                # 베이스/팔을 관통하거나 phase 전환 때 박스가 튀는 문제가 있었다.
                # 여기서는 joint target만 부드럽게 보간하고, 박스는 suction point를 따라오게 한다.
                if isinstance(custom_targets, dict) and custom_targets.get("mode") == "JOINT_SWING":
                    phase_sequence = custom_targets.get("phase_sequence", [])
                    custom_phase_index = int(max(0, min(custom_phase_index, len(phase_sequence) - 1)))
                    phase_info = phase_sequence[custom_phase_index]
                    custom_carry_phase = phase_info.get("name", str(custom_phase_index))
                    duration = int(max(1, phase_info.get("steps", 1)))

                    if "start_joints" not in phase_info:
                        phase_info["start_joints"] = np.array(robot.get_joint_positions(), dtype=float)
                    start_j = np.array(phase_info["start_joints"], dtype=float)
                    target_j = np.array(phase_info["target_joints"], dtype=float)
                    alpha = min(1.0, float(custom_phase_step) / float(duration))
                    smooth = alpha * alpha * (3.0 - 2.0 * alpha)
                    cmd_j = (1.0 - smooth) * start_j + smooth * target_j
                    link2_ref = float(custom_targets.get("link2_j2_ref_rad", cmd_j[1] if len(cmd_j)>1 else 0.0)) if isinstance(custom_targets, dict) else (cmd_j[1] if len(cmd_j)>1 else 0.0)
                    cmd_j = _apply_link2_orient_z_guard_to_joints(cmd_j, link2_ref, label=f"runtime_{custom_carry_phase}")
                    robot.apply_action(ArticulationAction(joint_positions=cmd_j))

                    suction_after = update_vgc10_suction_anchor(robot)
                    if BOX_DISABLE_PHYSICS_DURING_CARRY:
                        set_box_scripted_carry_mode(stage, task.box_move_path, True, verbose=False)

                    min_center_z = None
                    if custom_carry_phase in ("joint_lower", "joint_settle"):
                        min_center_z = float(JOINT_RELEASE_MIN_BOX_CENTER_Z)

                    if PHYSICS_FIXED_JOINT_ATTACH_ENABLED:
                        # 51_: 실제 물리 joint가 박스를 끌고 가게 둔다. 박스 transform 직접 이동 금지.
                        bbox_after = get_world_bbox_info(stage, task.box_path)
                        c_after = bbox_after["center"] if bbox_after is not None else np.array(attach_center, dtype=float)
                        desired_center = c_after
                        ok_follow = True
                    else:
                        ok_follow, desired_center = follow_box_to_suction(
                            stage,
                            task,
                            suction_pos=suction_after,
                            attached_center_offset=attached_center_offset,
                            min_center_z=min_center_z,
                        )
                        bbox_after = get_world_bbox_info(stage, task.box_path)
                        c_after = bbox_after["center"] if bbox_after is not None else desired_center

                    if custom_phase_step % JOINT_CARRY_LOG_INTERVAL == 0 or alpha >= 1.0:
                        link2_ref = float(custom_targets.get("link2_j2_ref_rad", cmd_j[1] if len(cmd_j)>1 else 0.0)) if isinstance(custom_targets, dict) else (cmd_j[1] if len(cmd_j)>1 else 0.0)
                        link2_z_est = _estimate_link2_orient_z_deg_from_j2(cmd_j[1], link2_ref) if len(cmd_j) > 1 else 0.0
                        print(
                            f"  [{custom_carry_phase}] step={custom_phase_step}/{duration}, "
                            f"alpha={alpha:.2f}, "
                            f"j1={cmd_j[0] if len(cmd_j)>0 else 0.0:.3f}, "
                            f"j2={cmd_j[1] if len(cmd_j)>1 else 0.0:.3f}, "
                            f"j3={cmd_j[2] if len(cmd_j)>2 else 0.0:.3f}, "
                            f"link2_z_est={link2_z_est:+.2f}deg/min={LINK2_ORIENT_Z_MIN_DEG:+.2f}, "
                            f"suction=({suction_after[0]:.3f},{suction_after[1]:.3f},{suction_after[2]:.3f}), "
                            f"box=({c_after[0]:.3f},{c_after[1]:.3f},{c_after[2]:.4f}), "
                            f"follow_ok={ok_follow}, top_lock_path={TOP_LOCK_FOLLOW_DESIRED_PATH}"
                        )

                    if bool(globals().get("PHYSICS_FIXED_JOINT_DIAGNOSTIC_NO_LIFT", False)):
                        physics_diag_log_after_attach_step(stage, custom_carry_phase, custom_phase_step)

                    custom_phase_step += 1
                    phase_done = alpha >= 1.0

                    # 27_: 벽에 붙은 박스를 먼저 충분히 위로 빼낸 뒤에만 joint_1 회전 시작.
                    # alpha가 1.0이 되어도 박스/흡착점 높이가 낮으면 joint_lift target을 유지한다.
                    if custom_carry_phase == "joint_lift" and phase_done and JOINT_STRICT_VERTICAL_LIFT_BEFORE_SWING:
                        box_z_now = float(c_after[2]) if c_after is not None else -999.0
                        suction_z_now = float(suction_after[2])
                        lift_ready = (
                            box_z_now >= float(JOINT_SWING_START_MIN_BOX_CENTER_Z)
                            and suction_z_now >= float(JOINT_SWING_START_MIN_SUCTION_Z)
                        )
                        extra_hold_done = custom_phase_step >= (duration + int(JOINT_LIFT_EXTRA_HOLD_STEPS))
                        if not lift_ready and not extra_hold_done:
                            phase_done = False
                            if custom_phase_step % JOINT_CARRY_LOG_INTERVAL == 0:
                                print(
                                    f"  [vertical_lift_gate] hold joint_1. "
                                    f"box_z={box_z_now:.3f}/{JOINT_SWING_START_MIN_BOX_CENTER_Z:.3f}, "
                                    f"suction_z={suction_z_now:.3f}/{JOINT_SWING_START_MIN_SUCTION_Z:.3f}, "
                                    f"extra={custom_phase_step-duration}/{JOINT_LIFT_EXTRA_HOLD_STEPS}"
                                )
                        elif not lift_ready and extra_hold_done:
                            # 무한 대기를 피하기 위한 안전장치. 이 로그가 뜨면 lift delta를 키우거나 threshold를 낮춰야 한다.
                            print(
                                f"  [vertical_lift_gate_WARN] height threshold not fully reached, but extra hold ended. "
                                f"box_z={box_z_now:.3f}/{JOINT_SWING_START_MIN_BOX_CENTER_Z:.3f}, "
                                f"suction_z={suction_z_now:.3f}/{JOINT_SWING_START_MIN_SUCTION_Z:.3f}; "
                                "proceeding to joint_1 swing."
                            )

                    if custom_carry_phase == "joint_lower" and float(suction_after[2]) <= float(JOINT_RELEASE_MIN_SUCTION_Z):
                        print(
                            f"  [joint_lower_guard] suction_z={suction_after[2]:.3f} <= {JOINT_RELEASE_MIN_SUCTION_Z:.3f}; "
                            "stop lowering and release soon."
                        )
                        phase_done = True

                    if phase_done:
                        if custom_phase_index < len(phase_sequence) - 1:
                            custom_phase_index += 1
                            custom_phase_step = 0
                            phase_sequence[custom_phase_index]["start_joints"] = np.array(robot.get_joint_positions(), dtype=float)
                            next_name = phase_sequence[custom_phase_index].get("name", str(custom_phase_index))
                            print(f"[JOINT_CARRY] phase done -> next={next_name}")
                        else:
                            # 102_: 물리 흡착 lift 성공 확인 모드.
                            # 여기서는 바로 release하지 않고, FixedJoint를 유지한 채 pause한다.
                            # 상자가 물리 joint로 실제로 들려 있는지 먼저 눈으로 확인한다.
                            if bool(globals().get("PHYSICS_LIFT_KEEP_ATTACHED_AND_PAUSE", False)):
                                if bool(globals().get("PHYSICS_FIXED_JOINT_DIAGNOSTIC_NO_LIFT", False)):
                                    print("[PHYSICS_DIAG_DONE_105] localRot + mass/inertia diagnostic hold finished. FixedJoint is still attached; no lift/no release/no box xform follow. Pausing now.")
                                    physics_diag_log_after_attach_step(stage, custom_carry_phase, custom_phase_step, force=True)
                                    log_oribox_pose_tracker(stage, "physics_diag_done_104", force=True)
                                else:
                                    print("[PHYSICS_LIFT_SWING_RELEASE_107] lift finished after stabilize. FixedJoint is still attached; no release, no box xform follow, no kinematic carry. Pausing now.")
                                    log_oribox_pose_tracker(stage, "physics_lift_success_hold_102", force=True)
                                my_world.pause()
                                custom_carry_active = False
                                custom_carry_phase = None
                                was_playing = is_playing
                                yield
                                continue

                            # 75_: joint_1 회전이 끝나면 현재 위치에서 바로 release한다.
                            # snap_box_to_stack_slot_if_enabled()를 호출하지 않는다. 순간이동 방지.
                            release_physics_attach_joint(stage, reason="joint_carry_release")
                            if BOX_DISABLE_PHYSICS_DURING_CARRY:
                                set_box_scripted_carry_mode(
                                    stage,
                                    task.box_move_path,
                                    False,
                                    reenable_physics=BOX_REENABLE_PHYSICS_AFTER_RELEASE,
                                    verbose=True,
                                )
                            elif not PHYSICS_FIXED_JOINT_ATTACH_ENABLED:
                                set_prim_kinematic(stage, task.box_path, False)
                            zero_subtree_velocity(stage, task.box_move_path)
                            if MULTI_ORIBOX_STACKING_ENABLED:
                                handle_stack_release_count_145(stage, task, label="joint_carry_release")
                            attached = False
                            released = True
                            custom_carry_active = False
                            custom_carry_phase = None
                            attached_center_offset = None
                            attach_center_z = None
                            print("-surface off- physics FixedJoint release after lift test. no box xform follow/no kinematic/no snap.")

                            if RETURN_HOME_AFTER_RELEASE:
                                if RUN_CONTINUOUS_LOOP:
                                    start_home_return("success_release_after_joint_carry", success=True)
                                else:
                                    returning_home = True
                                    home_start_joints = np.array(robot.get_joint_positions(), dtype=float)
                                    home_target_joints = np.zeros_like(home_start_joints)
                                    home_return_step = 0
                                    print("[RETURN] box released. returning robot to initial joint pose...")

                    was_playing = is_playing
                    yield
                    continue

                phase_sequence = custom_targets.get("phase_sequence") if isinstance(custom_targets, dict) else None
                if phase_sequence:
                    custom_phase_index = int(max(0, min(custom_phase_index, len(phase_sequence) - 1)))
                    phase_info = phase_sequence[custom_phase_index]
                    custom_carry_phase = phase_info.get("name", str(custom_phase_index))
                    phase_kind = phase_info.get("kind", "move")

                    # 117_: Hybrid mode special phase.
                    # RMPFlow로 긴 XY 이동을 하지 않고, 현재 관절 자세에서 joint_1/link_1 하나만 회전한다.
                    if phase_kind == "joint1_rotate":
                        duration = int(max(1, phase_info.get("steps", HYBRID_JOINT1_ROTATE_STEPS)))
                        if "start_joints" not in phase_info:
                            start_j = np.array(robot.get_joint_positions(), dtype=float)
                            target_j = start_j.copy()
                            ref_j1 = float(start_j[0]) if len(start_j) > 0 else 0.0

                            # joint_1 회전량은 현재 들어올린 박스 방향 -> 뒤쪽 큐브/place_center 방향의 yaw 차이로 계산한다.
                            # 156_: 단, 기본 배치 회전은 "현재 joint_1 + N도"가 아니라
                            #       "초기화 상태 joint_1 + N도"를 절대 목표로 사용한다.
                            bbox_now = get_world_bbox_info(stage, task.box_path)
                            box_center_now = np.array(bbox_now["center"], dtype=float) if bbox_now is not None else np.array(suction_now, dtype=float)
                            place_center_now = np.array(custom_targets.get("place_center", box_center_now), dtype=float) if isinstance(custom_targets, dict) else box_center_now.copy()
                            robot_center_now, center_path_now = get_robot_center_for_goal(stage)

                            mode = str(phase_info.get("mode", HYBRID_JOINT1_ROTATE_MODE))
                            sign = float(phase_info.get("sign", HYBRID_JOINT1_ROTATE_SIGN))
                            max_rad = abs(math.radians(float(phase_info.get("max_deg", HYBRID_JOINT1_ROTATE_MAX_DEG))))
                            fallback_rad = math.radians(float(phase_info.get("fallback_deg", HYBRID_JOINT1_ROTATE_FALLBACK_DEG)))
                            raw_delta = fallback_rad
                            delta_reason = "fallback_deg"
                            initial_j1 = None
                            absolute_target_j1 = None

                            if mode == "goal_angle" and robot_center_now is not None:
                                rc = np.array(robot_center_now[:2], dtype=float)
                                start_vec = box_center_now[:2] - rc
                                goal_vec = place_center_now[:2] - rc
                                if np.linalg.norm(start_vec) > 1e-6 and np.linalg.norm(goal_vec) > 1e-6:
                                    start_ang = float(np.arctan2(start_vec[1], start_vec[0]))
                                    goal_ang = float(np.arctan2(goal_vec[1], goal_vec[0]))
                                    raw_delta = float((goal_ang - start_ang + np.pi) % (2.0 * np.pi) - np.pi)
                                    delta_reason = f"goal_angle:{center_path_now}"

                            signed_delta = float(sign) * float(raw_delta)

                            # 157_: 정답지 APalt_slot 좌표를 이용해 joint_1 회전량을 자동 계산한다.
                            # x1/x2 = 로봇 base 기준 slot 방향, x3 = 로봇 base 기준 현재 박스 방향.
                            # 현재 자세에서 필요한 회전량 = (slot 방향 - 현재 박스 방향).
                            # 이 회전은 최종 위치 결정용이 아니라 APalt 방향 접근용이며,
                            # 최종 위치는 slot_marker_move_over/lower가 APalt_slot 좌표로 보정한다.
                            auto_slot_used_157 = False
                            if bool(globals().get("AUTO_JOINT1_FROM_SLOT_MARKER_ENABLED_157", False)) and robot_center_now is not None:
                                try:
                                    rc157 = np.array(robot_center_now[:2], dtype=float)
                                    box_vec157 = np.array(box_center_now[:2], dtype=float) - rc157
                                    slot_vec157 = np.array(place_center_now[:2], dtype=float) - rc157
                                    if np.linalg.norm(box_vec157) > 1e-6 and np.linalg.norm(slot_vec157) > 1e-6:
                                        x3_box_angle_157 = float(np.arctan2(box_vec157[1], box_vec157[0]))
                                        x_slot_angle_157 = float(np.arctan2(slot_vec157[1], slot_vec157[0]))
                                        raw_auto_delta_157 = float(x_slot_angle_157 - x3_box_angle_157)
                                        if bool(globals().get("AUTO_JOINT1_USE_SHORTEST_DELTA_157", True)):
                                            raw_auto_delta_157 = float((raw_auto_delta_157 + np.pi) % (2.0 * np.pi) - np.pi)

                                        # 159_: slot_01과 slot_02가 같은 방향으로 돌면 먼저 놓은 상자를 지나갈 수 있다.
                                        # 따라서 marker index별 선호 회전 방향을 강제한다.
                                        route_sign_159 = 0.0
                                        route_note_159 = "free"
                                        if bool(globals().get("SLOT_ROUTE_SIGN_FORCE_159", False)):
                                            try:
                                                route_pattern_159 = tuple(float(x) for x in globals().get("SLOT_ROUTE_SIGN_BY_MARKER_159", (-1.0, +1.0)))
                                                marker_count_159 = max(1, len(tuple(globals().get("SLOT_MARKER_PATHS_155", ("/World/APalt_slot_01", "/World/APalt_slot_02")))))
                                                marker_idx_159 = int(phase_info.get("stack_slot_index_145", 0)) % marker_count_159
                                                route_sign_159 = float(route_pattern_159[marker_idx_159 % len(route_pattern_159)]) if route_pattern_159 else 0.0
                                                if route_sign_159 > 0.0 and raw_auto_delta_157 < 0.0:
                                                    raw_auto_delta_157 = float(raw_auto_delta_157 + 2.0 * np.pi)
                                                    route_note_159 = f"force_positive:marker_idx={marker_idx_159}"
                                                elif route_sign_159 < 0.0 and raw_auto_delta_157 > 0.0:
                                                    raw_auto_delta_157 = float(raw_auto_delta_157 - 2.0 * np.pi)
                                                    route_note_159 = f"force_negative:marker_idx={marker_idx_159}"
                                                else:
                                                    route_note_159 = f"keep_sign:marker_idx={marker_idx_159},sign={route_sign_159:+.0f}"
                                            except Exception as _route_exc_159:
                                                route_note_159 = f"route_sign_failed:{_route_exc_159}"

                                        # 기존 max_rad와 별도 159/157 안전 제한 중 더 작은 값을 사용한다.
                                        max_auto_rad_157 = abs(math.radians(float(globals().get("AUTO_JOINT1_MAX_ABS_DELTA_DEG_159", globals().get("AUTO_JOINT1_MAX_ABS_DELTA_DEG_157", 170.0)))))
                                        max_use_rad_157 = min(float(max_rad), float(max_auto_rad_157))
                                        auto_delta_157 = float(sign) * float(raw_auto_delta_157)
                                        auto_delta_157 = float(np.clip(auto_delta_157, -max_use_rad_157, max_use_rad_157))
                                        min_auto_rad_157 = abs(math.radians(float(globals().get("AUTO_JOINT1_MIN_DELTA_DEG_157", 0.0))))
                                        if min_auto_rad_157 > 0.0 and abs(auto_delta_157) < min_auto_rad_157:
                                            auto_delta_157 = 0.0
                                        auto_delta_157 = _clamp_first_joint_delta(auto_delta_157)
                                        joint1_delta = float(auto_delta_157)
                                        target_j1 = float(ref_j1 + joint1_delta)
                                        signed_delta = float(joint1_delta)
                                        delta_reason = (
                                            f"auto_slot_angle_157:{center_path_now}:"
                                            f"box_angle={math.degrees(x3_box_angle_157):+.1f}deg,"
                                            f"slot_angle={math.degrees(x_slot_angle_157):+.1f}deg,"
                                            f"raw_delta={math.degrees(raw_auto_delta_157):+.1f}deg,"
                                            f"route={route_note_159}"
                                        )
                                        initial_j1 = None
                                        absolute_target_j1 = None
                                        auto_slot_used_157 = True
                                        if bool(globals().get("AUTO_JOINT1_LOG_157", True)):
                                            print(
                                                f"[AUTO_JOINT1_SLOT_157] start. robot_center=({rc157[0]:+.3f},{rc157[1]:+.3f}), "
                                                f"box_xy=({box_center_now[0]:+.3f},{box_center_now[1]:+.3f}), "
                                                f"slot_xy=({place_center_now[0]:+.3f},{place_center_now[1]:+.3f}), "
                                                f"x3_box_angle={math.degrees(x3_box_angle_157):+.1f}deg, "
                                                f"x_slot_angle={math.degrees(x_slot_angle_157):+.1f}deg, "
                                                f"delta_from_current={math.degrees(joint1_delta):+.1f}deg, "
                                                f"current_j1={math.degrees(ref_j1):+.1f}deg, "
                                                f"target_j1={math.degrees(target_j1):+.1f}deg, "
                                                f"limit={math.degrees(max_use_rad_157):.1f}deg, route={route_note_159}. "
                                                f"final placement is APalt_slot correction."
                                            )
                                except Exception as exc:
                                    print(f"[AUTO_JOINT1_SLOT_157][WARN] failed; fallback to previous mode: {exc}")

                            if not auto_slot_used_157:
                                if bool(globals().get("ABSOLUTE_JOINT1_FROM_INITIAL_ENABLED_156", False)):
                                    init_j = globals().get("ROBOT_INITIAL_JOINTS_156", None)
                                    if init_j is not None and len(init_j) > 0:
                                        initial_j1 = float(init_j[0])
                                    else:
                                        # RESET_ROBOT_TO_ZERO=True인 현재 프로젝트 기준 fallback.
                                        initial_j1 = 0.0
                                    # 여기서 signed_delta는 "초기 기준 목표각"이다.
                                    # 현재 joint_1에서 이 각도를 더하는 것이 아니라, 최종 목표를 initial_j1 + signed_delta로 고정한다.
                                    signed_delta = float(np.clip(signed_delta, -max_rad, max_rad))
                                    signed_delta = _clamp_first_joint_delta(signed_delta)
                                    absolute_target_j1 = float(initial_j1 + signed_delta)
                                    target_j1 = absolute_target_j1
                                    joint1_delta = float(target_j1 - ref_j1)
                                    delta_reason = f"abs_initial_156:{delta_reason}"
                                else:
                                    joint1_delta = float(np.clip(signed_delta, -max_rad, max_rad))
                                    joint1_delta = _clamp_first_joint_delta(joint1_delta)
                                    target_j1 = ref_j1 + joint1_delta

                            if len(target_j) > 0:
                                target_j[0] = target_j1

                            phase_info["start_joints"] = start_j
                            phase_info["target_joints"] = target_j
                            phase_info["ref_j1"] = ref_j1
                            phase_info["initial_j1_156"] = initial_j1
                            phase_info["absolute_target_j1_156"] = absolute_target_j1
                            phase_info["target_j1"] = target_j1
                            phase_info["joint1_delta"] = joint1_delta
                            phase_info["delta_reason"] = delta_reason
                            phase_info["box_center_start"] = box_center_now.copy()
                            phase_info["place_center"] = place_center_now.copy()
                            if bool(globals().get("ABSOLUTE_JOINT1_FROM_INITIAL_ENABLED_156", False)) and absolute_target_j1 is not None:
                                init_log = float(initial_j1) if initial_j1 is not None else 0.0
                                abs_log = float(absolute_target_j1) if absolute_target_j1 is not None else float(target_j1)
                                print(
                                    f"[ABS_JOINT1_PLACE_156] start. initial_j1={init_log:+.6f}rad/{math.degrees(init_log):+.1f}deg, "
                                    f"current_ref_j1={ref_j1:+.6f}rad/{math.degrees(ref_j1):+.1f}deg, "
                                    f"target_abs_j1={abs_log:+.6f}rad/{math.degrees(abs_log):+.1f}deg, "
                                    f"delta_from_current={joint1_delta:+.6f}rad/{math.degrees(joint1_delta):+.1f}deg, "
                                    f"reason={delta_reason}, only joint_1 changes. "
                                    f"box_xy=({box_center_now[0]:.3f},{box_center_now[1]:.3f}), "
                                    f"place_xy=({place_center_now[0]:.3f},{place_center_now[1]:.3f})"
                                )
                            else:
                                print(
                                    f"[SIMPLE_STEP2_JOINT1_ROTATE_117] start. ref_j1={ref_j1:+.6f}, "
                                    f"target_j1={target_j1:+.6f}, j1_delta={joint1_delta:+.6f}rad/{math.degrees(joint1_delta):+.1f}deg, "
                                    f"max={float(phase_info.get('max_deg', HYBRID_JOINT1_ROTATE_MAX_DEG)):.1f}deg, reason={delta_reason}, only joint_1 changes. "
                                    f"box_xy=({box_center_now[0]:.3f},{box_center_now[1]:.3f}), "
                                    f"place_xy=({place_center_now[0]:.3f},{place_center_now[1]:.3f})"
                                )
                        start_j = np.array(phase_info["start_joints"], dtype=float)
                        target_j = np.array(phase_info["target_joints"], dtype=float)
                        alpha_raw = float(np.clip(custom_phase_step / float(duration), 0.0, 1.0))
                        alpha = float(alpha_raw * alpha_raw * (3.0 - 2.0 * alpha_raw))
                        cmd_j = (1.0 - alpha) * start_j + alpha * target_j
                        robot.apply_action(ArticulationAction(joint_positions=cmd_j))
                        suction_after = update_vgc10_suction_anchor(robot)
                        bbox_after = get_world_bbox_info(stage, task.box_path)
                        c_after = bbox_after["center"] if bbox_after is not None else np.zeros(3)
                        root_pos = get_world_translation(stage, task.box_path)
                        root_minus_bbox = float(root_pos[2] - c_after[2]) if root_pos is not None else 999.0
                        ref_j1 = float(phase_info.get("ref_j1", start_j[0] if len(start_j)>0 else 0.0))
                        j1_delta_now = float(cmd_j[0] - ref_j1) if len(cmd_j) > 0 else 0.0
                        init_j1_log_156 = phase_info.get("initial_j1_156", None)
                        j1_from_initial_now_156 = None
                        if init_j1_log_156 is not None and len(cmd_j) > 0:
                            try:
                                j1_from_initial_now_156 = float(cmd_j[0] - float(init_j1_log_156))
                            except Exception:
                                j1_from_initial_now_156 = None
                        phase_done = bool(custom_phase_step >= duration)
                        if custom_phase_step % CUSTOM_LOG_INTERVAL == 0 or phase_done:
                            print(
                                f"  [simple_step2_joint1_rotate_117] step={custom_phase_step}/{duration}, alpha={alpha_raw:.2f}, "
                                f"j1={cmd_j[0] if len(cmd_j)>0 else 0.0:+.3f}, "
                                f"j1_delta_from_attach={j1_delta_now:+.3f}rad/{math.degrees(j1_delta_now):+.1f}deg, "
                                f"j1_from_initial={(j1_from_initial_now_156 if j1_from_initial_now_156 is not None else 0.0):+.3f}rad/{(math.degrees(j1_from_initial_now_156) if j1_from_initial_now_156 is not None else 0.0):+.1f}deg, "
                                f"j2={cmd_j[1] if len(cmd_j)>1 else 0.0:+.3f}, "
                                f"j3={cmd_j[2] if len(cmd_j)>2 else 0.0:+.3f}, "
                                f"suction=({suction_after[0]:.3f},{suction_after[1]:.3f},{suction_after[2]:.3f}), "
                                f"box=({c_after[0]:.3f},{c_after[1]:.3f},{c_after[2]:.4f}), "
                                f"upright_check_117=root_minus_bbox_z={root_minus_bbox:+.4f}"
                            )
                            log_oribox_pose_tracker(stage, f"simple_joint1_rotate_116:{custom_phase_step}", force=True)
                        custom_phase_step += 1
                        if phase_done:
                            if custom_phase_index < len(phase_sequence) - 1:
                                custom_phase_index += 1
                                custom_phase_step = 0
                                cart_controller.reset()

                                # 158_: joint_1 회전이 끝난 직후 다음 Cartesian phase의 start를
                                #       회전 전 lift_suction이 아니라 실제 현재 suction 위치로 교체한다.
                                #       이게 없으면 회전 후 APalt_slot 근처까지 갔다가 다시 컨베이어 쪽
                                #       lift 위치로 되돌아가려는 이상한 움직임이 생긴다.
                                next_phase_158 = phase_sequence[custom_phase_index]
                                next_name_158 = str(next_phase_158.get("name", ""))
                                if (
                                    bool(globals().get("PHASE_DYNAMIC_START_AFTER_JOINT1_ENABLED_158", True))
                                    and next_name_158 in tuple(globals().get("PHASE_DYNAMIC_START_NAMES_158", ()))
                                ):
                                    old_start_158 = np.array(next_phase_158.get("start", suction_after), dtype=float).copy()
                                    actual_start_158 = np.array(suction_after, dtype=float).copy()
                                    next_phase_158["start"] = actual_start_158.copy()

                                    # 160_: 회전 후 이미 slot 근처인데 move_over가 target z를 더 높게 잡으면
                                    #       release 직전에 살짝 들어올리는 동작이 생긴다. move_over에서는 z를 올리지 않는다.
                                    no_up_note_160 = ""
                                    if bool(globals().get("SLOT_MARKER_NO_UP_BEFORE_RELEASE_160", True)) and next_name_158 == "slot_marker_move_over_155":
                                        try:
                                            target_160 = np.array(next_phase_158.get("target", actual_start_158), dtype=float).copy()
                                            old_target_160 = target_160.copy()
                                            if float(target_160[2]) > float(actual_start_158[2]):
                                                target_160[2] = float(actual_start_158[2])
                                                next_phase_158["target"] = target_160.copy()
                                                no_up_note_160 += f", no_up_z:{old_target_160[2]:.3f}->{target_160[2]:.3f}"
                                            # XY가 이미 충분히 가까우면 move_over 시간을 거의 없앤다.
                                            xy_err_160 = float(np.linalg.norm(target_160[:2] - actual_start_158[:2]))
                                            if bool(globals().get("SLOT_MARKER_SKIP_MOVE_IF_NEAR_XY_160", True)) and xy_err_160 <= float(globals().get("SLOT_MARKER_SKIP_MOVE_XY_TOL_160", 0.08)):
                                                old_steps_160 = int(next_phase_158.get("steps", 1))
                                                next_phase_158["steps"] = int(min(old_steps_160, 25))
                                                no_up_note_160 += f", near_xy={xy_err_160:.4f}, steps:{old_steps_160}->{next_phase_158['steps']}"
                                        except Exception as _fix160_exc:
                                            no_up_note_160 += f", no_up_fix_warn={type(_fix160_exc).__name__}"

                                    next_phase_158["dynamic_start_set_158"] = True
                                    next_phase_158["dynamic_start_reason_158"] = "after_joint1_rotate"
                                    if bool(globals().get("PHASE_DYNAMIC_START_LOG_158", True)):
                                        _t158 = np.array(next_phase_158.get('target', actual_start_158), dtype=float)
                                        print(
                                            f"[PHASE_START_FIX_158] next={next_name_158}, reason=after_joint1_rotate, "
                                            f"old_start=({old_start_158[0]:.3f},{old_start_158[1]:.3f},{old_start_158[2]:.3f}) -> "
                                            f"actual_start=({actual_start_158[0]:.3f},{actual_start_158[1]:.3f},{actual_start_158[2]:.3f}), "
                                            f"target=({_t158[0]:.3f},{_t158[1]:.3f},{_t158[2]:.3f}){no_up_note_160}"
                                        )

                                print(f"[SIMPLE_CARRY_116] phase done -> next={phase_sequence[custom_phase_index].get('name')}")
                            else:
                                custom_carry_active = False
                        was_playing = is_playing
                        yield
                        continue

                    # legacy 114_: joint_2 회전 블록. 117_ 기본 흐름에서는 사용하지 않는다.
                    if phase_kind == "joint2_rotate":
                        duration = int(max(1, phase_info.get("steps", HYBRID_LINK2_ROTATE_STEPS)))
                        if "start_joints" not in phase_info:
                            start_j = np.array(robot.get_joint_positions(), dtype=float)
                            target_j = start_j.copy()
                            ref_j2 = float(start_j[1]) if len(start_j) > 1 else 0.0
                            target_link2_z = float(phase_info.get("target_link2_z_deg", HYBRID_LINK2_ROTATE_TARGET_DEG))
                            # 기준식: link2_z_est = -90 + degrees(j2 - ref_j2)
                            # target_j2 = ref_j2 + radians(target_link2_z - (-90))
                            target_j2 = ref_j2 + math.radians(target_link2_z - (-90.0))
                            min_link2_z = float(phase_info.get("min_link2_z_deg", HYBRID_LINK2_ROTATE_MIN_DEG))
                            min_j2 = ref_j2 + math.radians(min_link2_z - (-90.0))
                            # 너무 과하게 접히지 않게 하한 guard. target_j2가 min보다 더 작으면 min으로 제한.
                            if target_j2 < min_j2:
                                target_j2 = min_j2
                            if len(target_j) > 1:
                                target_j[1] = target_j2
                            phase_info["start_joints"] = start_j
                            phase_info["target_joints"] = target_j
                            phase_info["ref_j2"] = ref_j2
                            phase_info["target_j2"] = target_j2
                            print(
                                f"[SIMPLE_STEP2_LINK2_ROTATE_114] start. ref_j2={ref_j2:+.6f}, "
                                f"target_link2_z={target_link2_z:+.1f}deg, min_link2_z={min_link2_z:+.1f}deg, "
                                f"target_j2={target_j2:+.6f}; only joint_2 changes."
                            )
                        start_j = np.array(phase_info["start_joints"], dtype=float)
                        target_j = np.array(phase_info["target_joints"], dtype=float)
                        alpha_raw = float(np.clip(custom_phase_step / float(duration), 0.0, 1.0))
                        alpha = float(alpha_raw * alpha_raw * (3.0 - 2.0 * alpha_raw))
                        cmd_j = (1.0 - alpha) * start_j + alpha * target_j
                        robot.apply_action(ArticulationAction(joint_positions=cmd_j))
                        suction_after = update_vgc10_suction_anchor(robot)
                        bbox_after = get_world_bbox_info(stage, task.box_path)
                        c_after = bbox_after["center"] if bbox_after is not None else np.zeros(3)
                        root_pos = get_world_translation(stage, task.box_path)
                        root_minus_bbox = float(root_pos[2] - c_after[2]) if root_pos is not None else 999.0
                        ref_j2 = float(phase_info.get("ref_j2", start_j[1] if len(start_j)>1 else 0.0))
                        link2_z_est = -90.0 + math.degrees(float(cmd_j[1] - ref_j2)) if len(cmd_j) > 1 else -90.0
                        phase_done = bool(custom_phase_step >= duration)
                        if custom_phase_step % CUSTOM_LOG_INTERVAL == 0 or phase_done:
                            print(
                                f"  [simple_step2_link2_rotate_114] step={custom_phase_step}/{duration}, alpha={alpha_raw:.2f}, "
                                f"j1={cmd_j[0] if len(cmd_j)>0 else 0.0:+.3f}, "
                                f"j2={cmd_j[1] if len(cmd_j)>1 else 0.0:+.3f}, "
                                f"j3={cmd_j[2] if len(cmd_j)>2 else 0.0:+.3f}, "
                                f"link2_z_est={link2_z_est:+.2f}deg, "
                                f"suction=({suction_after[0]:.3f},{suction_after[1]:.3f},{suction_after[2]:.3f}), "
                                f"box=({c_after[0]:.3f},{c_after[1]:.3f},{c_after[2]:.4f}), "
                                f"upright_check_117=root_minus_bbox_z={root_minus_bbox:+.4f}"
                            )
                            log_oribox_pose_tracker(stage, f"hybrid_link2_rotate_111:{custom_phase_step}", force=True)
                        custom_phase_step += 1
                        if phase_done:
                            if custom_phase_index < len(phase_sequence) - 1:
                                custom_phase_index += 1
                                custom_phase_step = 0
                                cart_controller.reset()
                                print(f"[HYBRID_CARRY_111] phase done -> next={phase_sequence[custom_phase_index].get('name')}")
                            else:
                                custom_carry_active = False
                        was_playing = is_playing
                        yield
                        continue

                    # 111_: joint_2 회전이 끝난 현재 위치에서, 처음 vertical_lift의 반대 방향으로 수직 하강한다.
                    if phase_kind == "reverse_vertical_lower" and "target" not in phase_info:
                        start_suction = np.array(suction_now, dtype=float)
                        target_suction = start_suction.copy()
                        target_suction[2] = float(start_suction[2]) + float(phase_info.get("delta_z", -float(VERTICAL_LIFT_DELTA_Z)))
                        phase_info["start"] = start_suction.copy()
                        phase_info["target"] = target_suction.copy()
                        print(
                            f"[SIMPLE_STEP3_REVERSE_LOWER_117] dynamic target set. "
                            f"start=({start_suction[0]:.3f},{start_suction[1]:.3f},{start_suction[2]:.3f}), "
                            f"target=({target_suction[0]:.3f},{target_suction[1]:.3f},{target_suction[2]:.3f})"
                        )

                    # 111_: settle phase도 직전 실제 suction 위치를 유지하도록 동적으로 설정한다.
                    if phase_kind == "path_settle" and "target" not in phase_info:
                        phase_info["start"] = np.array(suction_now, dtype=float).copy()
                        phase_info["target"] = np.array(suction_now, dtype=float).copy()

                    # 171_: release 직전 APalt_slot 방향 기준 yaw만 RMPFlow target orientation으로 보정한다.
                    # target suction position은 현재 위치로 고정해서 중심/수평을 건드리지 않는 것을 우선한다.
                    if phase_kind == "pre_release_yaw_align_171":
                        prepare_pre_release_yaw_align_phase(stage, robot, task, phase_info, suction_now)

                    # 184_: 2번째 상자 yaw 정렬을 별도 pre_release phase로 하지 않고,
                    # slot_marker_lower_155 / slot_marker_settle_155가 수행되는 동안 target orientation에 섞는다.
                    if bool(phase_info.get("fused_yaw_align_184", False)):
                        prepare_fused_yaw_align_phase(stage, robot, task, phase_info, suction_now)

                    # 158_ 2차 안전장치:
                    # phase 전환 시점에 start를 갱신하지 못했거나 RMPFlow/PhysX 때문에 실제 suction 위치가
                    # 계획 위치와 달라진 경우에도, slot_marker_* phase의 첫 step에서는 반드시 현재 실제 위치에서
                    # 보간을 시작한다. 이렇게 해야 step=0 target_suction이 회전 전 lift_suction으로 되돌아가지 않는다.
                    if (
                        bool(globals().get("PHASE_DYNAMIC_START_AFTER_JOINT1_ENABLED_158", True))
                        and custom_phase_step == 0
                        and str(phase_info.get("name", "")) in tuple(globals().get("PHASE_DYNAMIC_START_NAMES_158", ()))
                        and "target" in phase_info
                    ):
                        actual_start_158b = np.array(suction_now, dtype=float).copy()
                        old_start_158b = np.array(phase_info.get("start", actual_start_158b), dtype=float).copy()
                        phase_info["start"] = actual_start_158b.copy()
                        no_up_note_160b = ""
                        if bool(globals().get("SLOT_MARKER_NO_UP_BEFORE_RELEASE_160", True)) and str(phase_info.get("name", "")) == "slot_marker_move_over_155":
                            try:
                                target_160b = np.array(phase_info.get("target", actual_start_158b), dtype=float).copy()
                                old_target_160b = target_160b.copy()
                                if float(target_160b[2]) > float(actual_start_158b[2]):
                                    target_160b[2] = float(actual_start_158b[2])
                                    phase_info["target"] = target_160b.copy()
                                    no_up_note_160b += f", no_up_z:{old_target_160b[2]:.3f}->{target_160b[2]:.3f}"
                                xy_err_160b = float(np.linalg.norm(target_160b[:2] - actual_start_158b[:2]))
                                if bool(globals().get("SLOT_MARKER_SKIP_MOVE_IF_NEAR_XY_160", True)) and xy_err_160b <= float(globals().get("SLOT_MARKER_SKIP_MOVE_XY_TOL_160", 0.08)):
                                    old_steps_160b = int(phase_info.get("steps", 1))
                                    phase_info["steps"] = int(min(old_steps_160b, 25))
                                    no_up_note_160b += f", near_xy={xy_err_160b:.4f}, steps:{old_steps_160b}->{phase_info['steps']}"
                            except Exception as _fix160b_exc:
                                no_up_note_160b += f", no_up_runtime_warn={type(_fix160b_exc).__name__}"
                        phase_info["dynamic_start_runtime_set_158"] = True
                        if bool(globals().get("PHASE_DYNAMIC_START_LOG_158", True)):
                            target_dbg_158b = np.array(phase_info.get("target", actual_start_158b), dtype=float).copy()
                            print(
                                f"[PHASE_START_RUNTIME_FIX_158] phase={phase_info.get('name')}, "
                                f"old_start=({old_start_158b[0]:.3f},{old_start_158b[1]:.3f},{old_start_158b[2]:.3f}) -> "
                                f"actual_start=({actual_start_158b[0]:.3f},{actual_start_158b[1]:.3f},{actual_start_158b[2]:.3f}), "
                                f"target=({target_dbg_158b[0]:.3f},{target_dbg_158b[1]:.3f},{target_dbg_158b[2]:.3f}){no_up_note_160b}"
                            )

                    target_final = np.array(phase_info["target"], dtype=float)
                    target_start = np.array(phase_info.get("start", target_final), dtype=float)
                    phase_steps = int(max(1, phase_info.get("steps", CUSTOM_PHASE_MAX_STEPS)))
                    alpha_raw = float(np.clip(custom_phase_step / float(phase_steps), 0.0, 1.0))
                    # 부드러운 S-curve 보간. target_suction이 갑자기 크게 변하지 않게 한다.
                    alpha = float(alpha_raw * alpha_raw * (3.0 - 2.0 * alpha_raw))
                    target_suction = target_start + (target_final - target_start) * alpha
                    phase_error = float(np.linalg.norm(np.array(suction_now, dtype=float) - target_suction))
                    phase_done = bool(custom_phase_step >= phase_steps)

                    if phase_kind in ("path_move", "path_lower", "path_lift", "path_settle", "reverse_vertical_lower"):
                        # 59_: target_suction을 현재 위치/guard 기준으로 바꾸지 않는다.
                        # 56_ 로그에서 target_suction이 중간중간 엉뚱한 좌표로 튄 원인을 제거한다.
                        pass
                    elif phase_kind == "lift":
                        phase_error = float(abs(float(suction_now[2]) - float(target_suction[2])))
                        phase_done = (phase_error <= CUSTOM_LIFT_TOL) or (custom_phase_step >= CUSTOM_PHASE_MAX_STEPS)
                    elif phase_kind == "lower":
                        phase_error = float(np.linalg.norm(np.array(suction_now, dtype=float) - target_suction))
                        phase_done = (
                            (phase_error <= CUSTOM_LOWER_TOL and custom_phase_step >= CUSTOM_RELEASE_AFTER_LOWER_MIN_STEPS)
                            or (custom_phase_step >= CUSTOM_PHASE_MAX_STEPS)
                        )
                    else:
                        phase_error = float(np.linalg.norm(np.array(suction_now[:2], dtype=float) - target_suction[:2]))
                        phase_done = (phase_error <= CUSTOM_MOVE_XY_TOL) or (custom_phase_step >= CUSTOM_PHASE_MAX_STEPS)

                    # 173_: pre_release_yaw_align_171은 target_suction을 현재 위치에 고정한 채
                    # orientation만 RMPFlow로 먹이는 phase다. 171 실패 원인은 target 위치 오차가 0이라
                    # step=0에서 phase_done=True가 되어 실제 action이 여러 step 적용되지 않은 것.
                    # 172 probe에서 검증된 것처럼 정해진 step 수 동안 반드시 action을 넣는다.
                    if phase_kind == "pre_release_yaw_align_171":
                        target_suction = np.array(phase_info.get("hold_suction_171", suction_now), dtype=float).copy()
                        if bool(phase_info.get("skip_171", False)):
                            phase_error = 0.0
                            phase_done = True
                        elif bool(phase_info.get("abort_171", False)):
                            phase_done = True
                        else:
                            phase_error = 999.0  # yaw axis error는 아래 진단 로그에서 별도로 출력
                            phase_done = bool(custom_phase_step >= int(max(1, phase_info.get("steps", PRE_RELEASE_YAW_ALIGN_STEPS_171))))

                    # 24_: min_center_z를 쓰면 suction이 실제로 못 올라갔는데 박스만 높게 튀는 순간이동이 생긴다.
                    # 그래서 박스는 항상 실제 suction_after + attach offset만 따라간다.
                    min_center_z = None
                else:
                    if custom_carry_phase == "lift":
                        target_suction = np.array(custom_targets["lift_suction"], dtype=float)
                        phase_error = float(np.linalg.norm(np.array(suction_now, dtype=float) - target_suction))
                        phase_done = (abs(float(suction_now[2]) - float(target_suction[2])) <= CUSTOM_LIFT_TOL) or (custom_phase_step >= CUSTOM_PHASE_MAX_STEPS)
                    elif custom_carry_phase == "move":
                        target_suction = np.array(custom_targets["move_suction"], dtype=float)
                        phase_error = float(np.linalg.norm(np.array(suction_now[:2], dtype=float) - target_suction[:2]))
                        phase_done = (phase_error <= CUSTOM_MOVE_XY_TOL) or (custom_phase_step >= CUSTOM_PHASE_MAX_STEPS)
                    elif custom_carry_phase == "lower":
                        target_suction = np.array(custom_targets["lower_suction"], dtype=float)
                        phase_error = float(np.linalg.norm(np.array(suction_now, dtype=float) - target_suction))
                        phase_done = (phase_error <= CUSTOM_LOWER_TOL and custom_phase_step >= CUSTOM_RELEASE_AFTER_LOWER_MIN_STEPS) or (custom_phase_step >= CUSTOM_PHASE_MAX_STEPS)
                    else:
                        target_suction = np.array(suction_now, dtype=float)
                        phase_error = 0.0
                        phase_done = True
                    min_center_z = None

                # 173_: pre_release_yaw_align_171 phase에서는 172 probe에서 성공한 방식과 동일하게
                # target_end_effector_position은 현재 위치에 가깝게 고정하고, target_end_effector_orientation만
                # APalt_slot yaw 오차로 만든 quaternion으로 넣는다.
                phase_fixed_orientation_171 = phase_info.get("fixed_orientation_171", custom_fixed_orientation) if isinstance(phase_info, dict) else custom_fixed_orientation
                apply_cartesian_suction_target(
                    cart_controller,
                    robot,
                    current_suction_pos=suction_now,
                    desired_suction_pos=target_suction,
                    fixed_orientation=phase_fixed_orientation_171,
                )

                suction_after = update_vgc10_suction_anchor(robot)
                if PHYSICS_FIXED_JOINT_ATTACH_ENABLED:
                    # 51_: 실제 물리 joint가 박스를 끌고 가게 둔다. 박스 transform 직접 이동 금지.
                    bbox_tmp = get_world_bbox_info(stage, task.box_path)
                    desired_center = bbox_tmp["center"] if bbox_tmp is not None else np.zeros(3)
                    ok_follow = True
                else:
                    if BOX_DISABLE_PHYSICS_DURING_CARRY:
                        set_box_scripted_carry_mode(stage, task.box_move_path, True, verbose=False)
                    # 59_: RMPFlow/IK가 순간적으로 흔들려도 박스가 같이 미쳐 날뛰지 않게,
                    # 운반 중 박스는 고정 waypoint의 desired suction path를 따른다.
                    # 로봇은 같은 target을 향해 움직이지만, 박스 적재 안정성을 우선한다.
                    follow_suction_for_box = target_suction if TOP_LOCK_FOLLOW_DESIRED_PATH else suction_after
                    ok_follow, desired_center = follow_box_to_suction(
                        stage,
                        task,
                        suction_pos=follow_suction_for_box,
                        attached_center_offset=attached_center_offset,
                        min_center_z=min_center_z,
                    )

                if custom_phase_step % CUSTOM_LOG_INTERVAL == 0 or phase_done:
                    bbox_after = get_world_bbox_info(stage, task.box_path)
                    c_after = bbox_after["center"] if bbox_after is not None else desired_center

                    # 110_ bug fix:
                    # 109_에서 get_world_transform()이라는 없는 함수를 호출해서 NameError가 났다.
                    # 기존 파일에 이미 정의되어 있는 get_world_translation()으로 root 위치를 읽는다.
                    box_root_pos_for_upright = get_world_translation(stage, task.box_path)
                    if box_root_pos_for_upright is not None and c_after is not None:
                        root_minus_bbox_z_110 = float(box_root_pos_for_upright[2] - c_after[2])
                    else:
                        root_minus_bbox_z_110 = 999.0

                    print(
                        f"  [custom_{custom_carry_phase}] step={custom_phase_step}, "
                        f"err={phase_error:.4f}, "
                        f"target_suction=({target_suction[0]:.3f},{target_suction[1]:.3f},{target_suction[2]:.3f}), "
                        f"suction=({suction_after[0]:.3f},{suction_after[1]:.3f},{suction_after[2]:.3f}), "
                        f"box=({c_after[0]:.3f},{c_after[1]:.3f},{c_after[2]:.4f}), "
                        f"upright_check_117=root_minus_bbox_z={root_minus_bbox_z_110:+.4f}, "
                        f"follow_ok={ok_follow}, top_lock_path={TOP_LOCK_FOLLOW_DESIRED_PATH}"
                    )
                    log_oribox_pose_tracker(stage, f"custom_{custom_carry_phase}:{custom_phase_step}", force=True)

                    if phase_kind == "pre_release_yaw_align_171":
                        try:
                            _slot_idx_171_dbg = int(getattr(task, "_stack_slot_index", 0))
                            _diag171_now = diagnose_pallet_release_pose(stage, task.box_move_path, _slot_idx_171_dbg, label=f"during_pre_release_yaw_align_173:{custom_phase_step}")
                            if isinstance(_diag171_now, dict):
                                _c_err171 = float(_diag171_now.get("center_xy_err", 999.0))
                                _lvl171 = float(_diag171_now.get("box_to_slot_z_deg", 999.0))
                                _axis171 = float(_diag171_now.get("best_axis_err", 999.0))
                                print(
                                    f"[PRE_RELEASE_YAW_ALIGN_173][CHECK] step={custom_phase_step}, "
                                    f"center_xy_err={_c_err171:.4f}, level={_lvl171:.2f}deg, axis_err={_axis171:.2f}deg"
                                )
                                if _c_err171 > float(globals().get("PRE_RELEASE_YAW_ALIGN_CENTER_ABORT_M_171", 0.018)) or _lvl171 > float(globals().get("PRE_RELEASE_YAW_ALIGN_LEVEL_ABORT_DEG_171", 3.0)):
                                    phase_info["abort_171"] = True
                                    phase_info["abort_reason_171"] = f"center_or_level_drift:center={_c_err171:.4f},level={_lvl171:.2f}"
                                    phase_done = True
                                    print(f"[PRE_RELEASE_YAW_ALIGN_173][ABORT] {phase_info['abort_reason_171']}")
                        except Exception as _align171_log_exc:
                            print(f"[PRE_RELEASE_YAW_ALIGN_173][WARN] check failed: {type(_align171_log_exc).__name__}: {_align171_log_exc}")

                custom_phase_step += 1

                if phase_done:
                    phase_sequence = custom_targets.get("phase_sequence") if isinstance(custom_targets, dict) else None

                    # 67_ 핵심 수정:
                    # path_move/path_lower/path_settle 단계가 단순 step timeout으로 끝나면,
                    # 로봇 suction이 BoxAprop 목표 근처에 실제로 도달하지 못했는데도 다음 lower/release 단계로 넘어간다.
                    # 그 결과 두 번째 박스가 현재 위치에 남은 채 [BOXAPROP_RELEASE_ABORT]로 pause된다.
                    # 목표 오차가 큰 경우에는 더 진행하지 않고 release 없이 home으로 복귀해서 멈춤/순간이동을 막는다.
                    if (
                        phase_sequence
                        and bool(custom_targets.get("boxaprop_place_enabled", custom_targets.get("cube_over_enabled", False)))
                        and bool(BOXAPROP_ABORT_HOME_ON_TARGET_ERROR)
                        and custom_carry_phase in ("boxaprop_move", "cube_over_move", "boxaprop_lower", "boxaprop_settle")
                    ):
                        max_err = float(BOXAPROP_MOVE_MAX_SUCTION_ERR) if custom_carry_phase in ("boxaprop_move", "cube_over_move") else float(BOXAPROP_LOWER_MAX_SUCTION_ERR)
                        if float(phase_error) > max_err:
                            bbox_abort = get_world_bbox_info(stage, task.box_path)
                            c_abort = bbox_abort["center"] if bbox_abort is not None else np.zeros(3)
                            print(
                                f"[BOXAPROP_MOVE_ABORT] phase={custom_carry_phase}, "
                                f"target_err={phase_error:.3f}m > {max_err:.3f}m. "
                                f"release/lower를 진행하지 않고 home 복귀합니다. "
                                f"box=({c_abort[0]:.3f},{c_abort[1]:.3f},{c_abort[2]:.4f}), "
                                f"target_suction=({target_suction[0]:.3f},{target_suction[1]:.3f},{target_suction[2]:.3f}), "
                                f"actual_suction=({suction_after[0]:.3f},{suction_after[1]:.3f},{suction_after[2]:.3f})"
                            )
                            log_oribox_pose_tracker(stage, f"boxaprop_move_abort:{custom_carry_phase}", force=True)
                            attached = False
                            custom_carry_active = False
                            custom_carry_phase = None
                            attached_center_offset = None
                            attach_center_z = None
                            start_home_return("boxaprop_move_abort_target_error", success=False)
                            was_playing = is_playing
                            yield
                            continue

                    if phase_sequence and custom_phase_index < len(phase_sequence) - 1:
                        custom_phase_index += 1
                        custom_carry_phase = phase_sequence[custom_phase_index].get("name", str(custom_phase_index))
                        custom_phase_step = 0
                        cart_controller.reset()
                        # 112_ bug fix:
                        # 111_ hybrid phases such as link2_rotate_to_back do not have a Cartesian
                        # "target" key.  After vertical_lift the old transition code tried to
                        # print phase_sequence[...]["target"], causing KeyError: 'target'.
                        next_phase_info_112 = phase_sequence[custom_phase_index]
                        if "target" in next_phase_info_112:
                            next_target = np.array(next_phase_info_112["target"], dtype=float)
                            print(
                                f"[CUSTOM_CARRY_116] phase done -> next={custom_carry_phase}, "
                                f"target=({next_target[0]:.3f},{next_target[1]:.3f},{next_target[2]:.3f})"
                            )
                        else:
                            print(
                                f"[CUSTOM_CARRY_116] phase done -> next={custom_carry_phase}, "
                                f"kind={next_phase_info_112.get('kind', 'unknown')}, no Cartesian target; dynamic/joint phase."
                            )
                    elif (not phase_sequence) and custom_carry_phase == "lift":
                        custom_carry_phase = "move"
                        custom_phase_step = 0
                        cart_controller.reset()
                        print("[CUSTOM_CARRY] lift done -> move across robot center at safe height")
                    elif (not phase_sequence) and custom_carry_phase == "move":
                        custom_carry_phase = "lower"
                        custom_phase_step = 0
                        cart_controller.reset()
                        print("[CUSTOM_CARRY] move done -> lower at opposite coordinate")
                    else:
                        if bool(VERTICAL_LIFT_ONLY_TEST) and isinstance(custom_targets, dict) and custom_targets.get("vertical_only", False):
                            bbox_after = get_world_bbox_info(stage, task.box_path)
                            c_after = bbox_after["center"] if bbox_after is not None else np.zeros(3)
                            suction_final = update_vgc10_suction_anchor(robot)
                            place_enabled = bool(custom_targets.get("boxaprop_place_enabled", custom_targets.get("cube_over_enabled", False)))
                            release_enabled = bool(custom_targets.get("boxaprop_release", False))
                            force_drop_release_126_dbg = (
                                str(CUSTOM_CARRY_MODE) == "VERTICAL_JOINT1_REVERSE"
                                and bool(globals().get("FORCE_DROP_RELEASE_AFTER_SETTLE_126", True))
                            )
                            lower_target = np.array(custom_targets.get("lower_suction", suction_final), dtype=float)
                            place_center = np.array(custom_targets.get("place_center", c_after), dtype=float)
                            final_err = float(np.linalg.norm(np.array(suction_final, dtype=float) - lower_target)) if place_enabled else 0.0
                            center_err = float(np.linalg.norm(np.array(c_after[:2], dtype=float) - place_center[:2])) if place_enabled else 0.0
                            rr_place = robot_relative_vector(stage, place_center)
                            print(
                                f"[UPRIGHT_RMPFLOW_DONE_109] box=({c_after[0]:.3f},{c_after[1]:.3f},{c_after[2]:.4f}), "
                                f"suction=({suction_final[0]:.3f},{suction_final[1]:.3f},{suction_final[2]:.3f}), "
                                f"lower_target=({lower_target[0]:.3f},{lower_target[1]:.3f},{lower_target[2]:.3f}), "
                                f"target_err={final_err:.4f}, center_xy_err={center_err:.4f}, release_enabled={release_enabled}, force_drop_release_126={force_drop_release_126_dbg}, "
                                f"place_center_world=({place_center[0]:.4f}, {place_center[1]:.4f}, {place_center[2]:.4f}), "
                                f"place_center_robot_relative=({rr_place[0]:+.4f}, {rr_place[1]:+.4f}, {rr_place[2]:+.4f})"
                            )
                            log_oribox_pose_tracker(stage, "boxaprop_place_done_before_release", force=True)

                            # 171_: release 직전 yaw-align phase가 중심/수평 drift로 중단된 경우에는 release하지 않는다.
                            # 새 문제를 만들지 않기 위해 상자를 떼지 않고 일시정지해서 로그를 확인한다.
                            if isinstance(phase_info, dict) and bool(phase_info.get("abort_171", False)):
                                print(f"[PRE_RELEASE_YAW_ALIGN_173][RELEASE_BLOCKED] {phase_info.get('abort_reason_171', 'abort')}. surface remains ON; release skipped.")
                                if bool(globals().get("PRE_RELEASE_YAW_ALIGN_ABORT_PAUSE_171", True)):
                                    try:
                                        my_world.pause()
                                    except Exception:
                                        pass
                                was_playing = is_playing
                                yield
                                continue

                            if bool(globals().get("PRE_RELEASE_YAW_ALIGN_REQUIRE_AXIS_OK_BEFORE_RELEASE_171", False)):
                                try:
                                    _slot_idx_req171 = int(getattr(task, "_stack_slot_index", 0))
                                    _diag_req171 = diagnose_pallet_release_pose(stage, task.box_move_path, _slot_idx_req171, label="after_pre_release_yaw_align_173_require_check")
                                    if isinstance(_diag_req171, dict) and not bool(_diag_req171.get("ok_axis", False)):
                                        print(f"[PRE_RELEASE_YAW_ALIGN_173][RELEASE_BLOCKED] axis still not OK: {float(_diag_req171.get('best_axis_err', 999.0)):.2f}deg")
                                        try:
                                            my_world.pause()
                                        except Exception:
                                            pass
                                        was_playing = is_playing
                                        yield
                                        continue
                                except Exception as _req171_exc:
                                    print(f"[PRE_RELEASE_YAW_ALIGN_173][WARN] require check failed: {type(_req171_exc).__name__}: {_req171_exc}")

                            if place_enabled and final_err > float(BOXAPROP_RELEASE_MAX_SUCTION_ERR):
                                print(
                                    f"[BOXAPROP_RELEASE_ABORT] suction target error too large: {final_err:.3f}m > {BOXAPROP_RELEASE_MAX_SUCTION_ERR:.3f}m. "
                                    "release하지 않고 실패 처리 후 home 복귀합니다. BoxAprop 좌표/로봇 도달 범위를 조정해야 합니다."
                                )
                                log_oribox_pose_tracker(stage, "boxaprop_release_abort_before_home", force=True)
                                attached = False
                                custom_carry_active = False
                                custom_carry_phase = None
                                attached_center_offset = None
                                attach_center_z = None
                                start_home_return("boxaprop_release_abort_target_error", success=False)
                                was_playing = is_playing
                                yield
                                continue

                            # 125_ 핵심 수정:
                            # 117/124의 VERTICAL_JOINT1_REVERSE 경로에서는 VERTICAL_LIFT_THEN_CUBE_OVER_ENABLED=False라
                            # place_enabled가 False로 남는다. 124번은 if place_enabled and release_enabled 조건 때문에
                            # hybrid_settle까지 끝나도 실제 detach 블록에 들어오지 못했다.
                            # 이제 joint_1 회전/하강/settle이 끝나면 place_enabled와 무관하게 FixedJoint를 제거한다.
                            force_drop_release_126 = (
                                str(CUSTOM_CARRY_MODE) == "VERTICAL_JOINT1_REVERSE"
                                and bool(globals().get("FORCE_DROP_RELEASE_AFTER_SETTLE_126", True))
                            )

                            if release_enabled or force_drop_release_126:
                                # 124/125_ 핵심:
                                # 마지막 위치/회전은 그대로 두고, 흡착 FixedJoint만 제거해서
                                # 물리 적용된 상자가 중력으로 떨어지는지 확인한다.
                                if bool(DROP_RELEASE_AFTER_SETTLE_124):
                                    if bool(DROP_RELEASE_ZERO_VELOCITY_BEFORE_DETACH_124):
                                        zero_subtree_velocity(stage, task.box_move_path)
                                        print("  [DROP_RELEASE_124] zero velocity before detach = True")

                                    # 160_: release 직전 상자 yaw를 순간적으로 바꾸지 않는다.
                                    #       방향이 틀리면 틀린 상태로 로그에 남겨야 실제 로봇 동작 검증이 가능하다.
                                    if bool(globals().get("SLOT_MARKER_YAW_CHECK_LOG_160", True)):
                                        try:
                                            _slot_idx_for_yaw_160 = int(getattr(task, "_stack_slot_index", 0))
                                            # 170_: 진단 전용. release 직전 APalt_slot 정답지 대비 현재 box pose만 기록한다.
                                            #       어떤 transform/joint/RMPFlow/surface 상태도 수정하지 않는다.
                                            diagnose_pallet_release_pose(stage, task.box_move_path, _slot_idx_for_yaw_160, label="before_release_no_snap")
                                            axis_yaw_diagnostic(stage, task.box_move_path, _slot_idx_for_yaw_160, label="before_release_no_snap")
                                            check_box_yaw_against_slot_marker(stage, task.box_move_path, _slot_idx_for_yaw_160, label="before_release_no_snap")
                                        except Exception as _yaw_exc_160:
                                            print(f"[YAW_CHECK_160][WARN] before release failed: {type(_yaw_exc_160).__name__}: {_yaw_exc_160}")

                                    if bool(globals().get("RELEASE_DIAG_128_ENABLED", True)):
                                        print("========== [DIAG128_RELEASE_BEGIN] ==========")
                                        compact_box_physics_state(stage, task.box_move_path, "before_release")
                                        scan_stage_joints(stage, task.box_move_path, "before_release")

                                    release_ok = release_physics_attach_joint(stage, reason="drop_release_after_final_pose_128")

                                    if bool(globals().get("RELEASE_DIAG_128_ENABLED", True)):
                                        scan_stage_joints(stage, task.box_move_path, "after_joint_remove")
                                        compact_box_physics_state(stage, task.box_move_path, "after_joint_remove_before_dynamic_restore")
                                        if bool(globals().get("RELEASE_DIAG_FORCE_DYNAMIC_AFTER_DETACH_128", True)):
                                            force_box_dynamic_after_release(stage, task.box_move_path, verbose=True)
                                        compact_box_physics_state(stage, task.box_move_path, "after_dynamic_restore")
                                        compact_drop_observe(stage, task.box_move_path, "after_release_step0")

                                    attached = False
                                    released = True
                                    custom_carry_active = False
                                    custom_carry_phase = None
                                    attached_center_offset = None
                                    attach_center_z = None
                                    print(
                                        f"-surface off- DROP_RELEASE_128: FixedJoint removed={release_ok}. "
                                        "no snap/no teleport/no kinematic toggle. Box should fall by gravity if it is dynamic."
                                    )
                                    log_oribox_pose_tracker(stage, "after_drop_release_128_begin", force=True)

                                    if MULTI_ORIBOX_STACKING_ENABLED:
                                        _release_count_after_drop_164 = handle_stack_release_count_145(stage, task, label="drop_release_128")
                                        if should_stop_after_release_count(_release_count_after_drop_164):
                                            print("[DIAG_STOP_AFTER_RELEASE_164] world.pause() after second release. Send this log for diagnosis.")
                                            task_done = True
                                            my_world.pause()
                                            was_playing = is_playing
                                            yield
                                            continue
                                    else:
                                        completed_box_roots.add(get_task_completed_root_path(task))
                                        check_and_trigger_forklift()

                                    # 바로 home으로 보내면 낙하 장면이 잘 안 보일 수 있으므로, 지정 step 동안 관찰한다.
                                    observe_steps_124 = int(max(0, DROP_RELEASE_OBSERVE_STEPS_124))
                                    log_interval_124 = int(max(1, DROP_RELEASE_LOG_INTERVAL_124))
                                    for drop_i_124 in range(observe_steps_124):
                                        try:
                                            update_vgc10_suction_anchor(robot)
                                        except Exception:
                                            pass
                                        my_world.step(render=True)
                                        if drop_i_124 % log_interval_124 == 0 or drop_i_124 == observe_steps_124 - 1:
                                            if bool(globals().get("RELEASE_DIAG_128_ENABLED", True)):
                                                compact_drop_observe(stage, task.box_move_path, f"drop_release_128:{drop_i_124}")
                                            else:
                                                log_oribox_pose_tracker(stage, f"drop_release_126:{drop_i_124}", force=True)

                                    # 134_: release 후 바로 종료 상태로 막지 않는다.
                                    # home-return을 사용할 때는 task_done=False 상태에서 start_home_return을 호출해야
                                    # 다음 프레임에서 returning_home 블록이 실행되어 로봇이 초기 자세로 돌아간다.
                                    if bool(DROP_RELEASE_RETURN_HOME_AFTER_OBSERVE_124) and bool(RETURN_HOME_AFTER_RELEASE):
                                        task_done = False
                                        print("[RESET_134_AFTER_RELEASE] drop observe done -> start home/initial return")
                                        start_home_return("drop_release_134_after_observe", success=True)
                                    else:
                                        task_done = True
                                    was_playing = is_playing
                                    yield
                                    continue

                                # fallback: 기존 safe release 방식. DROP_RELEASE_AFTER_SETTLE_124=False일 때만 사용.
                                attached = False
                                custom_carry_active = False
                                custom_carry_phase = None
                                print("-surface off- BoxAprop safe release: no physics toggle, no snap, keep current pose.")
                                log_oribox_pose_tracker(stage, "after_safe_release_before_stack_done", force=True)
                                if MULTI_ORIBOX_STACKING_ENABLED:
                                    _release_count_after_safe_164 = handle_stack_release_count_145(stage, task, label="safe_release")
                                    if should_stop_after_release_count(_release_count_after_safe_164):
                                        print("[DIAG_STOP_AFTER_RELEASE_164] world.pause() after second safe release. Send this log for diagnosis.")
                                        task_done = True
                                        my_world.pause()
                                        was_playing = is_playing
                                        yield
                                        continue
                                else:
                                    completed_box_roots.add(get_task_completed_root_path(task))
                                    check_and_trigger_forklift()
                                task_done = False
                                print("[RESET_134_AFTER_SAFE_RELEASE] start home/initial return")
                                start_home_return("boxaprop_safe_release_no_physics_toggle_134", success=True)
                                was_playing = is_playing
                                yield
                                continue

                            custom_carry_active = False
                            custom_carry_phase = None
                            task_done = True
                            if bool(VERTICAL_LIFT_PAUSE_AFTER_SUCCESS):
                                my_world.pause()
                            was_playing = is_playing
                            yield
                            continue

                        # 50_: 순간이동 방지. slot 중심 snap은 기본 비활성화되어 있으며,
                        # 실제 로봇/흡착점이 이동한 현재 위치에서만 release한다.
                        snap_box_to_stack_slot_if_enabled(stage, task)
                        release_physics_attach_joint(stage, reason="cartesian_carry_release")
                        if BOX_DISABLE_PHYSICS_DURING_CARRY:
                            set_box_scripted_carry_mode(
                                stage,
                                task.box_move_path,
                                False,
                                reenable_physics=BOX_REENABLE_PHYSICS_AFTER_RELEASE,
                                verbose=True,
                            )
                        elif not PHYSICS_FIXED_JOINT_ATTACH_ENABLED:
                            if bool(BOX_KEEP_KINEMATIC_AFTER_RELEASE):
                                set_prim_kinematic(stage, task.box_path, True)
                                print("  [CENTER_TOP_LOCK_RELEASE] placed box kept kinematic=True after vertical-only test.")
                            else:
                                set_prim_kinematic(stage, task.box_path, False)
                        zero_subtree_velocity(stage, task.box_move_path)
                        if MULTI_ORIBOX_STACKING_ENABLED:
                            handle_stack_release_count_145(stage, task, label="cartesian_carry_release")
                        attached = False
                        released = True
                        custom_carry_active = False
                        custom_carry_phase = None
                        attached_center_offset = None
                        attach_center_z = None
                        print(f"-surface off- {config.box_prefix}_ box release at current pose. no snap/no teleport.")

                        if RETURN_HOME_AFTER_RELEASE:
                            if RUN_CONTINUOUS_LOOP:
                                start_home_return("success_release_after_cartesian_carry", success=True)
                            else:
                                returning_home = True
                                home_start_joints = np.array(robot.get_joint_positions(), dtype=float)
                                home_target_joints = np.zeros_like(home_start_joints)
                                home_return_step = 0
                                print("[RETURN] box released. returning robot to initial joint pose...")

                was_playing = is_playing
                yield
                continue

            obs = task.get_observations()
            box_center = np.array(obs["oribox"]["position"], dtype=float)
            current_joints = robot.get_joint_positions()

            # controller에는 움직이는 박스 현재 위치가 아니라 초기 pick 위치를 계속 넣는다.
            # 그래야 흡착 후 박스가 따라 움직여도 로봇 목표가 흔들리지 않는다.
            actions = controller.forward(
                picking_position=task.pick_center,
                placing_position=task.goal_center,
                current_joint_positions=current_joints,
                end_effector_offset=EE_OFFSET,
            )
            robot.apply_action(actions)

            event = controller.get_current_event()
            suction_pos = update_vgc10_suction_anchor(robot)

            bbox = get_world_bbox_info(stage, task.box_path)
            close_gate, close_reason = should_attach_oribox(event, suction_pos, bbox, box_stopped=box_stopped_for_pick)

            # 10_ safety: 흡착이 안 된 상태로 VGC10이 박스 윗면을 계속 관통하면 바로 멈춘다.
            # 이 로그가 뜨면 흡착 판정이 너무 늦거나, tool0/VGC10 offset이 너무 낮은 것이다.
            if (
                PAUSE_IF_SUCTION_PENETRATES_WITHOUT_ATTACH
                and (not attached)
                and (not ever_attached)
                and bbox is not None
                and event in PICK_CLOSE_EVENTS
            ):
                z_gap_now = float(np.array(suction_pos, dtype=float)[2] - float(bbox["top_center"][2]))
                if z_gap_now < SUCTION_PENETRATION_STOP_Z_GAP:
                    print(
                        f"[STOP_PENETRATION] suction point is too far below box top before attach. "
                        f"z_gap={z_gap_now:.4f}, limit={SUCTION_PENETRATION_STOP_Z_GAP:.4f}, "
                        f"reason={close_reason}. returning home and retrying with refreshed bbox."
                    )
                    if RUN_CONTINUOUS_LOOP and LOOP_RETRY_AFTER_ATTACH_FAIL:
                        start_home_return("suction_penetration_before_attach", success=False)
                    else:
                        task_done = True
                        my_world.pause()
                    was_playing = is_playing
                    yield
                    continue

            if (not attached) and event in (PICK_CLOSE_EVENTS | RETRY_CLOSE_EVENTS):
                try:
                    top_center = bbox["top_center"] if bbox is not None else box_center
                    now_dist = float(np.linalg.norm(np.array(suction_pos, dtype=float) - np.array(top_center, dtype=float)))
                    if now_dist < best_attach_dist:
                        best_attach_dist = now_dist
                        best_attach_reason = close_reason
                except Exception:
                    pass

            if (not attached) and (event in RETRY_CLOSE_EVENTS) and (not retry_logged):
                retry_logged = True
                print(f"-surface retry- no attach at event2/3, retrying near box best={best_attach_reason}")

            if not attached and close_gate:
                # 15_ 핵심: root 기준 offset이 아니라 bbox center 기준 offset을 저장한다.
                # 이렇게 해야 박스 루트가 바닥 기준이거나 이상한 xform op를 가져도
                # 실제 보이는 박스 중심이 suction point를 따라간다.
                bbox_for_attach = get_world_bbox_info(stage, task.box_path)
                attach_center = np.array(bbox_for_attach["center"], dtype=float) if bbox_for_attach is not None else np.array(box_center, dtype=float)
                grid_attach_center = _LAST_SUCTION_GRID_INFO.get("attach_center")
                if grid_attach_center is None:
                    grid_attach_center = np.array(suction_pos, dtype=float)
                grid_attach_center = np.array(grid_attach_center, dtype=float)
                attached_center_offset = attach_center - np.array(suction_pos, dtype=float)

                # 51_: 박스 물리를 끄거나 kinematic으로 바꾸지 않는다.
                # 9점 흡착 판정 평균점에 FixedJoint를 생성해 실제 물리 연결로 들어올린다.
                joint_ok = True
                if PHYSICS_FIXED_JOINT_ATTACH_ENABLED:
                    joint_ok = create_physics_attach_joint(stage, task.box_path, grid_attach_center)
                if not joint_ok:
                    print("[PHYSICS_ATTACH_ABORT] FixedJoint 생성 실패. 박스 물리를 끄는 fallback은 사용하지 않고 재시도합니다.")
                    if RUN_CONTINUOUS_LOOP and LOOP_RETRY_AFTER_ATTACH_FAIL:
                        start_home_return("physics_joint_attach_failed", success=False)
                    else:
                        task_done = True
                        my_world.pause()
                    was_playing = is_playing
                    yield
                    continue
                zero_subtree_velocity(stage, task.box_move_path)

                attached = True
                ever_attached = True
                attach_center_z = float(attach_center[2])
                attach_steps = 0
                print(f"-surface on- {config.box_prefix}_ box attach reason={close_reason}")
                print(f"             attached_center_offset={attached_center_offset}")
                try:
                    _slot_idx_attach_164 = int(getattr(task, "_stack_slot_index", stack_slot_index))
                    axis_yaw_diagnostic(stage, task.box_move_path, _slot_idx_attach_164, label="after_attach_fixed_joint_main")
                except Exception as _axis_attach_exc_164:
                    print(f"[BOX_AXIS_YAW_164][WARN] after_attach_main failed: {type(_axis_attach_exc_164).__name__}: {_axis_attach_exc_164}")

                if CUSTOM_CARRY_AFTER_ATTACH:
                    custom_carry_active = True
                    custom_phase_step = 0
                    custom_phase_index = 0
                    custom_min_center_z = float(attach_center_z)

                    if CUSTOM_CARRY_MODE == "JOINT_SWING":
                        custom_fixed_orientation = None
                        custom_targets = make_joint_swing_carry_targets(
                            stage,
                            robot,
                            task,
                            attach_center=attach_center,
                            attach_suction_pos=suction_pos,
                            attached_center_offset=attached_center_offset,
                        )
                        custom_carry_phase = custom_targets["phase_sequence"][0].get("name", "joint_lift")
                        print(
                            "[JOINT_CARRY_27_VERTICAL_FIRST_HALF_TURN] start. "
                            f"center_path={custom_targets.get('center_path')}, "
                            f"lift_delta={custom_targets['lift_delta']}, "
                            f"swing_delta={custom_targets['swing_delta']}, "
                            f"mirror_xy_estimate={custom_targets['mirror_xy_estimate']}, "
                            f"phases={[p['name'] for p in custom_targets.get('phase_sequence', [])]}"
                        )
                    else:
                        custom_carry_phase = "lift"
                        custom_fixed_orientation = get_current_ee_pose(robot)[1]
                        custom_targets = make_custom_carry_targets(
                            stage,
                            task,
                            attach_center=attach_center,
                            attach_suction_pos=suction_pos,
                            attached_center_offset=attached_center_offset,
                        )
                        if custom_targets.get("phase_sequence"):
                            custom_carry_phase = custom_targets["phase_sequence"][0].get("name", "lift")
                        cart_controller.reset()
                        print(
                            "[HYBRID_VERTICAL_JOINT1_REVERSE_117] start. vertical RMPFlow lift + joint_1 rotate + reverse vertical lower. "
                            f"safe_z={custom_targets['safe_z']:.3f}, "
                            f"lift_suction={custom_targets['lift_suction']}, "
                            f"move_suction={custom_targets['move_suction']}, "
                            f"lower_suction={custom_targets['lower_suction']}, "
                            f"place_center={custom_targets['place_center']}, "
                            f"phases={[p['name'] for p in custom_targets.get('phase_sequence', [])]}"
                        )

                    was_playing = is_playing
                    yield
                    continue

                if RETURN_HOME_IMMEDIATELY_AFTER_ATTACH:
                    returning_home = True
                    home_start_joints = np.array(robot.get_joint_positions(), dtype=float)
                    home_target_joints = np.zeros_like(home_start_joints) if HOME_TARGET_JOINTS_CONFIG is None else np.array(HOME_TARGET_JOINTS_CONFIG, dtype=float)
                    home_return_step = 0
                    print("[RETURN] attach confirmed. returning-home-with-attached-box is disabled in 18_. continuing normal place/release.")
                    was_playing = is_playing
                    yield
                    continue

            if (not attached) and (not ever_attached) and (not released) and event >= FAIL_IF_NOT_ATTACHED_EVENT:
                print(
                    f"[FAIL] suction attach failed before departure. "
                    f"best={best_attach_reason}, best_dist={best_attach_dist:.4f}. returning home and retrying with refreshed bbox."
                )
                if RUN_CONTINUOUS_LOOP and LOOP_RETRY_AFTER_ATTACH_FAIL:
                    start_home_return("attach_failed_before_departure", success=False)
                else:
                    task_done = True
                    my_world.pause()
                was_playing = is_playing
                yield
                continue

            if attached:
                attach_steps += 1
                bbox_for_release_check = get_world_bbox_info(stage, task.box_path)
                current_center_for_release = np.array(bbox_for_release_check["center"], dtype=float) if bbox_for_release_check is not None else np.array(box_center, dtype=float)
                lifted_enough = (attach_center_z is not None) and (float(current_center_for_release[2]) >= float(attach_center_z) + RELEASE_AFTER_LIFT_DELTA_Z)
                goal_xy_error = float(np.linalg.norm(current_center_for_release[:2] - np.array(task.goal_center, dtype=float)[:2]))
                goal_z_error = float(abs(float(current_center_for_release[2]) - float(task.goal_center[2])))
                near_goal_xy = goal_xy_error <= PLACE_RELEASE_XY_TOL
                near_goal_z = goal_z_error <= PLACE_RELEASE_Z_TOL
                # 22_ 핵심:
                # 21_에서는 near_goal_z까지 기다리면서 PickPlaceController가 하강(event=5/6)해
                # 박스를 계속 아래로 끌고 갔다. 이제 Z는 기다리지 않는다.
                # 충분히 들어 올린 뒤, 반대편 XY 좌표 근처에 도착하면 바로 release한다.
                should_release_now = (
                    attach_steps >= RELEASE_AFTER_ATTACH_MIN_STEPS
                    and event >= PLACE_RELEASE_EVENT
                    and lifted_enough
                    and near_goal_xy
                )

                if should_release_now:
                    if RELEASE_AT_CURRENT_POSE:
                        # 박스를 목표점으로 강제 이동하지 않는다. 현재 들린 위치에서 release한다.
                        pass
                    else:
                        move_box_center_to(
                            stage,
                            task.box_move_path,
                            desired_center=task.goal_center,
                            root_to_center_offset=task.box_root_to_center_offset,
                        )
                    release_physics_attach_joint(stage, reason="event_goal_release")
                    if BOX_DISABLE_PHYSICS_DURING_CARRY:
                        set_box_scripted_carry_mode(
                            stage,
                            task.box_move_path,
                            False,
                            reenable_physics=BOX_REENABLE_PHYSICS_AFTER_RELEASE,
                            verbose=True,
                        )
                    elif not PHYSICS_FIXED_JOINT_ATTACH_ENABLED:
                        set_prim_kinematic(stage, task.box_path, False)
                    zero_subtree_velocity(stage, task.box_move_path)

                    if MULTI_ORIBOX_STACKING_ENABLED:
                        handle_stack_release_count_145(stage, task, label="event_goal_release")

                    attached = False
                    released = True
                    attached_center_offset = None
                    attach_center_z = None
                    attach_steps = 0

                    print(f"-surface off- {config.box_prefix}_ box release at stack goal event={event}, lifted_enough={lifted_enough}, goal_xy_error={goal_xy_error:.4f}, goal_z_error={goal_z_error:.4f}")

                    if RETURN_HOME_AFTER_RELEASE:
                        if RUN_CONTINUOUS_LOOP:
                            start_home_return("success_release_at_mirror_goal", success=True)
                        else:
                            returning_home = True
                            home_start_joints = np.array(robot.get_joint_positions(), dtype=float)
                            home_target_joints = np.zeros_like(home_start_joints)
                            home_return_step = 0
                            print("[RETURN] box released. returning robot to initial joint pose...")
                        was_playing = is_playing
                        yield
                        continue

                else:
                    if event >= PLACE_RELEASE_EVENT and attach_steps % 10 == 0:
                        print(
                            f"  [wait_release_goal] event={event}, goal_xy_error={goal_xy_error:.4f}/{PLACE_RELEASE_XY_TOL:.4f}, "
                            f"goal_z_error={goal_z_error:.4f}/{PLACE_RELEASE_Z_TOL:.4f}, lifted={lifted_enough}"
                        )
                    if attached_center_offset is None:
                        bbox_for_follow = get_world_bbox_info(stage, task.box_path)
                        follow_center = np.array(bbox_for_follow["center"], dtype=float) if bbox_for_follow is not None else np.array(box_center, dtype=float)
                        attached_center_offset = follow_center - np.array(suction_pos, dtype=float)

                    desired_box_center = np.array(suction_pos, dtype=float) + np.array(attached_center_offset, dtype=float)

                    # place 하강 중에는 박스 중심이 목표 높이보다 과도하게 낮아지지 않게 막는다.
                    # event=6에서 release 전까지 계속 아래로 끌고 내려가면 관통처럼 보이므로 높이를 clamp한다.
                    if event >= PLACE_RELEASE_EVENT:
                        # 23_: 반대편 좌표로 가는 동안 박스가 아래로 끌려 내려가지 않도록
                        # 최소 높이를 attach 시점보다 6cm 이상 높은 위치와 goal 높이 중 큰 값으로 고정한다.
                        min_carry_z = float(task.goal_center[2])
                        if attach_center_z is not None:
                            min_carry_z = max(min_carry_z, float(attach_center_z) + RELEASE_AFTER_LIFT_DELTA_Z)
                        desired_box_center[2] = max(float(desired_box_center[2]), min_carry_z)

                    if BOX_DISABLE_PHYSICS_DURING_CARRY:
                        # 매 프레임 물리 엔진이 다시 속도를 주지 못하게 carry mode와 속도를 유지한다.
                        set_box_scripted_carry_mode(stage, task.box_move_path, True, verbose=False)

                    if PHYSICS_FIXED_JOINT_ATTACH_ENABLED:
                        # 51_: 물리 연결 중에는 박스 transform을 직접 이동하지 않는다.
                        ok_follow_set = True
                    else:
                        ok_follow_set = move_box_center_to(
                            stage,
                            task.box_move_path,
                            desired_center=desired_box_center,
                            root_to_center_offset=task.box_root_to_center_offset,
                        )
                        if not BOX_DISABLE_PHYSICS_DURING_CARRY:
                            set_prim_kinematic(stage, task.box_path, True)
                        zero_subtree_velocity(stage, task.box_move_path)

                    if DEBUG_MOVE_PARENT_AND_FOLLOW:
                        bbox_after_follow = get_world_bbox_info(stage, task.box_path)
                        if bbox_after_follow is not None:
                            actual_center = np.array(bbox_after_follow["center"], dtype=float)
                            desired_center = np.array(desired_box_center, dtype=float)
                            follow_error = float(np.linalg.norm(actual_center - desired_center))
                            if follow_error > FOLLOW_ERROR_WARN_TOL:
                                print(
                                    f"[FOLLOW_WARN] move_path={task.box_move_path} ok_set={ok_follow_set} "
                                    f"desired_center=({desired_center[0]:.3f},{desired_center[1]:.3f},{desired_center[2]:.4f}) "
                                    f"actual_center=({actual_center[0]:.3f},{actual_center[1]:.3f},{actual_center[2]:.4f}) "
                                    f"follow_error={follow_error:.4f}>{FOLLOW_ERROR_WARN_TOL:.4f}"
                                )

            bbox_now = get_world_bbox_info(stage, task.box_path)
            box_center_now = bbox_now["center"] if bbox_now is not None else box_center
            top_center_now = bbox_now["top_center"] if bbox_now is not None else box_center_now

            if not np.all(np.isfinite(box_center_now)) or abs(float(box_center_now[2])) > 10.0:
                print(f"[STOP] box pose abnormal: {box_center_now}. pausing world.")
                task_done = True
                my_world.pause()

            if controller.is_done():
                if RUN_CONTINUOUS_LOOP and (not released) and (not custom_carry_active):
                    print("[LOOP_CONTROLLER_DONE] PickPlaceController ended without successful custom carry. returning home and retrying.")
                    start_home_return("controller_done_without_success", success=False)
                    was_playing = is_playing
                    yield
                    continue
                else:
                    print("[완료] Pick & Place sequence done")
                    task_done = True
                    my_world.pause()

            dist_top = float(np.linalg.norm(np.array(top_center_now) - np.array(suction_pos)))
            print(
                f"  [event={event}] "
                f"box_center=({box_center_now[0]:.3f},{box_center_now[1]:.3f},{box_center_now[2]:.4f}) "
                f"box_top=({top_center_now[0]:.3f},{top_center_now[1]:.3f},{top_center_now[2]:.4f}) "
                f"suction=({suction_pos[0]:.3f},{suction_pos[1]:.3f},{suction_pos[2]:.4f}) "
                f"dist_top={dist_top:.4f} gate={close_gate} reason={close_reason} "
                f"box_stopped={box_stopped_for_pick} pick_started={pick_started} "
                f"surface={'ON' if attached else 'OFF'} "
                f"gripped={[task.box_move_path] if attached else []}"
            )

        was_playing = is_playing
        yield

    return


class PalletizingWorker:
    """One independently stateful robot-cell worker sharing the common implementation."""

    def __init__(self, config: RobotCellConfig, shared_world, simulation_app):
        self.config = config
        self.shared_world = shared_world
        self.simulation_app = simulation_app
        self._overrides = _runtime_overrides(config, simulation_app)
        self._state = {
            "_POSE_TRACK_STATE": {"step": 0, "prev": {}},
            "_PHYSICS_JOINT_DIAG_STATE": {},
            "_LAST_SUCTION_GRID_INFO": {
                "attach_center": None,
                "hits": [],
                "summary": "not_evaluated",
            },
            "ROBOT_INITIAL_JOINTS_156": None,
            "_SUCTION_GRID_EVALUATION_COUNT": 0,
        }
        self._generator = _worker_loop(shared_world, config, self)

    def __iter__(self):
        return self

    def __next__(self):
        with self.context():
            return next(self._generator)

    @contextmanager
    def context(self):
        """Restore the caller's context after nested World callbacks or errors.

        This is for sequential/reentrant Isaac callbacks, not Python threads.
        """
        names = self._overrides.keys() | self._state.keys()
        missing = object()
        namespaces = [module.__dict__ for module in _RUNTIME_MODULES] + [globals()]
        saved = [{name: namespace.get(name, missing) for name in names} for namespace in namespaces]
        _apply_worker_context(self)
        try:
            yield
        finally:
            _capture_worker_context(self)
            for namespace, values in zip(namespaces, saved):
                for name, value in values.items():
                    if value is missing:
                        namespace.pop(name, None)
                    else:
                        namespace[name] = value

    def activate(self) -> None:
        """Activate this cell before calling shared helpers outside the worker loop."""
        _apply_worker_context(self)

    @property
    def robot(self):
        return self.shared_world.scene.get_object(self.config.robot_object_name)

    def ensure_vgc10(self, stage) -> bool:
        with self.context():
            attach_vgc10_to_link6(stage)
            robot = self.robot
            if robot is not None:
                for _ in range(5):
                    update_vgc10_suction_anchor(robot)
            root_ok = stage.GetPrimAtPath(self.config.vgc10_root_path).IsValid()
            suction_ok = stage.GetPrimAtPath(self.config.vgc10_suction_point_path).IsValid()
            return bool(root_ok and suction_ok)

    def update_vgc10(self):
        with self.context():
            robot = self.robot
            return update_vgc10_suction_anchor(robot) if robot is not None else None


def create_worker(config: RobotCellConfig, shared_world, simulation_app) -> PalletizingWorker:
    """Create an unstarted worker; first ``next`` registers its task and yields."""
    return PalletizingWorker(config, shared_world, simulation_app)


__all__ = ["PalletizingWorker", "create_worker"]
