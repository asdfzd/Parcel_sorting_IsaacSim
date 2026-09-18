"""Extracted common M0609 runtime implementation.

This module must only be imported after ``SimulationApp`` has been created.
"""

from pathlib import Path
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

from .scene import *
from .diagnostics import *
from .physics import *


def get_stack_joint1_angle(slot_index):
    """0-based slot_index 기준으로 joint_1 회전각을 반환한다.
    slot 0/2 = 160도, slot 1/3 = 130도.
    """
    try:
        idx = int(slot_index)
    except Exception:
        idx = 0
    pattern = tuple(float(x) for x in STACK_JOINT1_DEG_PATTERN_145)
    if not pattern:
        return float(HYBRID_JOINT1_ROTATE_FALLBACK_DEG)
    return float(pattern[idx % len(pattern)])

def repair_robotaprop_and_oribox_colliders(stage, completed_roots=None, verbose=True):
    """robotAprop_01은 static collider, OriBoxA_*는 collider를 명시적으로 켠다."""
    if completed_roots is None:
        completed_roots = set()

    if ROBOTAPROP_STATIC_COLLIDER_REPAIR:
        paths = find_robotaprop_paths(stage)
        if verbose:
            print(f"  [ROBOTAPROP_SEARCH] found={paths if paths else 'NONE'}")
        for p in paths:
            stats = enable_collision_on_subtree(stage, p, static=True, label="ROBOTAPROP")
            if verbose:
                print(
                    f"  [ROBOTAPROP_COLLIDER] root={p}, collision_on={stats.get('collision')}, "
                    f"rigid_removed={stats.get('rigid_removed')}, mesh_approx={stats.get('mesh_approx')}"
                )

    if ORIBOX_COLLIDER_REPAIR_ON_SETUP:
        candidates = discover_stack_candidates(stage, completed_roots=set(completed_roots)) if 'discover_stack_candidates' in globals() else []
        count = 0
        for c in candidates:
            root_path = c.get("root_path")
            if not root_path or root_path in completed_roots:
                continue
            stats = enable_collision_on_subtree(stage, root_path, static=False, label="ORIBOX")
            count += int(stats.get("collision", 0) or 0)
        if verbose:
            print(f"  [ORIBOX_COLLIDER_REPAIR] candidates={len(candidates)}, collision_on_total={count}")

class BoxStopDetector:
    """
    Small_Cardboard_box 하나가 멈췄는지 판정한다.

    8_ 수정 핵심:
    - gate 기준은 bbox center 이동량이다.
    - physics velocity는 Isaac/PhysX attr 잔류값이 남을 수 있어서 기본값으로는 gate에 쓰지 않는다.
    - 대신 매 step pos/linear/angular 각각 PASS/FAIL/IGNORED를 로그로 찍어서 왜 대기 중인지 바로 보이게 한다.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.prev_center = None
        self.stable_steps = 0
        self.total_steps = 0
        self.last_move = 999.0
        self.last_linear_vel = None
        self.last_angular_vel = None
        self.stable_center = None
        self.last_blockers = []

    @staticmethod
    def _fmt_gate(name, enabled, value, tol, ok, unit=""):
        if value is None:
            val_s = "None"
        elif abs(value) >= 100.0 or abs(value) < 0.0001:
            val_s = f"{value:.6f}"
        else:
            val_s = f"{value:.5f}"

        if not enabled:
            state = "INFO_ONLY"
        else:
            state = "OK" if ok else "FAIL"
        return f"{name}={val_s}{unit}<={tol:.5f}{unit}:{state}"

    def update(self, stage, prim_path, bbox_info):
        self.total_steps += 1

        prim = stage.GetPrimAtPath(prim_path)
        if not prim.IsValid():
            self.prev_center = None
            self.stable_steps = 0
            self.last_move = 999.0
            # 비슷한 이름을 찾아서 로그에 띄운다. 경로 오타를 바로 확인하기 위함.
            matches = []
            try:
                for p in Usd.PrimRange(stage.GetPseudoRoot()):
                    if "small_cardboard_box" in p.GetName().lower() or "small_cardboard_box" in str(p.GetPath()).lower():
                        matches.append(str(p.GetPath()))
            except Exception:
                pass
            return False, f"target_prim_invalid path={prim_path} matches={matches[:8]}"

        if bbox_info is None:
            self.prev_center = None
            self.stable_steps = 0
            self.last_move = 999.0
            return False, f"no_bbox path={prim_path} prim_valid=True"

        center = np.array(bbox_info["center"], dtype=float)
        if self.prev_center is None:
            move = 999.0
        else:
            move = float(np.linalg.norm(center - self.prev_center))

        linear_vel, angular_vel = get_prim_velocity_magnitudes(stage, prim_path)

        pos_raw_ok = move <= BOX_STABLE_POS_TOL
        linear_raw_ok = (linear_vel is None) or (linear_vel <= BOX_STABLE_LINEAR_VEL_TOL)
        angular_raw_ok = (angular_vel is None) or (angular_vel <= BOX_STABLE_ANGULAR_VEL_TOL)

        gate_parts = []
        if BOX_STOP_USE_BBOX_MOVE:
            gate_parts.append(pos_raw_ok)
        if BOX_STOP_USE_LINEAR_VEL:
            gate_parts.append(linear_raw_ok)
        if BOX_STOP_USE_ANGULAR_VEL:
            gate_parts.append(angular_raw_ok)

        # 혹시 모든 gate를 꺼도 bbox 기준으로는 최소 판정한다.
        candidate_ok = all(gate_parts) if gate_parts else pos_raw_ok

        blockers = []
        if BOX_STOP_USE_BBOX_MOVE and not pos_raw_ok:
            blockers.append("MOVE")
        if BOX_STOP_USE_LINEAR_VEL and not linear_raw_ok:
            blockers.append("LINEAR_VEL")
        if BOX_STOP_USE_ANGULAR_VEL and not angular_raw_ok:
            blockers.append("ANGULAR_VEL")

        if candidate_ok:
            self.stable_steps += 1
        else:
            self.stable_steps = 0

        self.prev_center = center.copy()
        self.last_move = move
        self.last_linear_vel = linear_vel
        self.last_angular_vel = angular_vel
        self.last_blockers = blockers

        stopped = self.stable_steps >= BOX_STABLE_REQUIRED_STEPS
        if stopped:
            self.stable_center = center.copy()

        if stopped:
            state = "STOP_CONFIRMED"
        elif candidate_ok:
            state = "COUNTING_STABLE"
        else:
            state = "WAITING_" + ("+".join(blockers) if blockers else "UNKNOWN")

        reason = (
            f"mode=bbox_only_selected_prim,"
            f"state={state},"
            f"stable={self.stable_steps}/{BOX_STABLE_REQUIRED_STEPS},"
            f"step={self.total_steps},"
            f"{self._fmt_gate('move', BOX_STOP_USE_BBOX_MOVE, move, BOX_STABLE_POS_TOL, pos_raw_ok, 'm')},"
            f"{self._fmt_gate('lin_vel', BOX_STOP_USE_LINEAR_VEL, linear_vel, BOX_STABLE_LINEAR_VEL_TOL, linear_raw_ok, 'm/s')},"
            f"{self._fmt_gate('ang_vel', BOX_STOP_USE_ANGULAR_VEL, angular_vel, BOX_STABLE_ANGULAR_VEL_TOL, angular_raw_ok, 'rad/s')},"
            f"blockers={blockers if blockers else 'NONE'}"
        )
        return stopped, reason

def hold_robot_current_pose(robot):
    """박스가 멈출 때까지 로봇 관절을 현재 위치에 유지한다."""
    try:
        current = np.array(robot.get_joint_positions(), dtype=float)
        robot.apply_action(ArticulationAction(joint_positions=current))
    except Exception:
        pass

def is_box_ready_for_pick_zone(bbox_info):
    """
    18_ 추가: 컨베이어 시작 위치/공중 스폰 상태에서 bbox가 잠깐 멈춰 보여도
    pick을 시작하지 않도록 실제 픽업 구간인지 확인한다.
    """
    if not BOX_READY_ZONE_GATE:
        return True, "ready_gate_disabled"
    if bbox_info is None:
        return False, "no_bbox"
    c = np.array(bbox_info["center"], dtype=float)
    reasons = []
    if float(c[1]) > float(BOX_READY_MIN_Y):
        reasons.append(f"Y_NOT_REACHED({c[1]:.3f}>{BOX_READY_MIN_Y:.3f})")
    if float(c[2]) > float(BOX_READY_MAX_CENTER_Z):
        reasons.append(f"Z_TOO_HIGH({c[2]:.4f}>{BOX_READY_MAX_CENTER_Z:.4f})")
    return (len(reasons) == 0), ("READY" if not reasons else "|".join(reasons))

def create_pick_zone_visual(stage):
    """
    1번 로봇 앞 pick 준비 영역을 색 있는 큐브로 표시한다.
    이 prim은 visual marker일 뿐이며 rigid body/collider를 만들지 않는다.
    """
    if not PICK_ZONE_VISUAL_ENABLED:
        # 77_: USD 직접 편집 상태를 보존한다. 기존 prim도 지우지 않고, 새 visual도 만들지 않는다.
        return False

    # 기존 영역 표시가 있으면 지우고 다시 만든다. 위치/크기 변경 시 확실히 반영된다.
    clear_prim_if_exists(stage, PICK_ZONE_PATH)

    cube = UsdGeom.Cube.Define(stage, PICK_ZONE_PATH)
    cube.CreateSizeAttr(1.0)
    prim = cube.GetPrim()

    # center/size를 바로 xform으로 적용한다.
    _set_xform_common(
        prim,
        translate=PICK_ZONE_CENTER,
        rotate=(0.0, 0.0, 0.0),
        scale=PICK_ZONE_SIZE,
    )

    # 색/투명도 설정. 물리 API는 적용하지 않는다.
    try:
        gprim = UsdGeom.Gprim(prim)
        gprim.CreateDisplayColorAttr([Gf.Vec3f(float(PICK_ZONE_COLOR[0]), float(PICK_ZONE_COLOR[1]), float(PICK_ZONE_COLOR[2]))])
        gprim.CreateDisplayOpacityAttr([float(PICK_ZONE_OPACITY)])
    except Exception:
        pass

    # 혹시 이전 실행/편집으로 physics/collision API가 남아 있으면 제거 또는 비활성화한다.
    try:
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    except Exception:
        pass
    try:
        if prim.HasAPI(UsdPhysics.MassAPI):
            prim.RemoveAPI(UsdPhysics.MassAPI)
    except Exception:
        pass
    try:
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.RemoveAPI(UsdPhysics.CollisionAPI)
    except Exception:
        pass

    print(
        f"  [PICK_ZONE_VISUAL] path={PICK_ZONE_PATH}, "
        f"center=({PICK_ZONE_CENTER[0]:.3f},{PICK_ZONE_CENTER[1]:.3f},{PICK_ZONE_CENTER[2]:.3f}), "
        f"size=({PICK_ZONE_SIZE[0]:.3f},{PICK_ZONE_SIZE[1]:.3f},{PICK_ZONE_SIZE[2]:.3f}), "
        f"opacity={PICK_ZONE_OPACITY:.2f}, physics=OFF"
    )
    return True

def is_bbox_center_inside_pick_zone(bbox_info):
    """박스 bbox center가 1번 로봇 앞 감지 영역 안에 들어왔는지 판단한다."""
    if bbox_info is None:
        return False, "no_bbox"

    c = np.array(bbox_info["center"], dtype=float)
    mn = PICK_ZONE_CENTER - PICK_ZONE_SIZE * 0.5
    mx = PICK_ZONE_CENTER + PICK_ZONE_SIZE * 0.5

    inside_xyz = (c >= mn) & (c <= mx)
    inside = bool(np.all(inside_xyz))

    blockers = []
    axes = ["X", "Y", "Z"]
    for i, axis in enumerate(axes):
        if c[i] < mn[i]:
            blockers.append(f"{axis}_LOW({c[i]:.3f}<{mn[i]:.3f})")
        elif c[i] > mx[i]:
            blockers.append(f"{axis}_HIGH({c[i]:.3f}>{mx[i]:.3f})")

    reason = (
        f"center=({c[0]:.3f},{c[1]:.3f},{c[2]:.4f}),"
        f"zone_min=({mn[0]:.3f},{mn[1]:.3f},{mn[2]:.3f}),"
        f"zone_max=({mx[0]:.3f},{mx[1]:.3f},{mx[2]:.3f}),"
        f"inside={inside},blockers={blockers if blockers else 'NONE'}"
    )
    return inside, reason

class PickZoneDetector:
    """
    1번 로봇 앞 영역 진입 감지기.
    정지 판정이 아니라, 박스 중심이 지정한 AABB 영역 안에 들어온 것을 기준으로 pick을 시작한다.
    """

    def __init__(self):
        self.reset()

    def reset(self):
        self.total_steps = 0
        self.inside_steps = 0
        self.last_reason = "not_started"

    def update(self, bbox_info):
        self.total_steps += 1
        inside, reason = is_bbox_center_inside_pick_zone(bbox_info)
        if inside:
            self.inside_steps += 1
        else:
            self.inside_steps = 0
        ready = self.inside_steps >= int(PICK_ZONE_REQUIRED_STEPS)
        state = "ZONE_READY" if ready else ("COUNTING_ZONE" if inside else "WAITING_ZONE")
        self.last_reason = (
            f"mode=front_zone,state={state},inside={self.inside_steps}/{PICK_ZONE_REQUIRED_STEPS},"
            f"step={self.total_steps},{reason}"
        )
        return ready, self.last_reason

def discover_stack_candidates(stage, completed_roots=None):
    """
    63_ 구조 대응:
    - 사용자가 OriBoxA_move wrapper Xform을 제거하고 /World/OriBoxA_01 형태로 저장한 USD를 우선 지원한다.
    - ORIBOX_STACK_PARENT_PATH는 기본 /World.
    - 혹시 예전 USD를 열었을 때도 망하지 않도록 /World/OriBoxA_move 하위도 fallback으로 추가 검색한다.
    - 각 root 하위의 Small_Cardboard_box bbox를 계산한다.
    """
    completed_roots = set() if completed_roots is None else set(completed_roots)
    results = []
    seen_roots = set()

    parent_paths = []
    for p in [
        ORIBOX_STACK_PARENT_PATH,
        "/World",
        globals().get("LEGACY_BOX_WRAPPER_PATH", "/World/OriBoxA_move"),
    ]:
        if p and p not in parent_paths:
            parent_paths.append(p)

    for parent_path in parent_paths:
        parent = stage.GetPrimAtPath(parent_path)
        if not parent.IsValid():
            continue

        for child in parent.GetChildren():
            root_name = child.GetName()
            root_path = str(child.GetPath())
            if not root_name.startswith(ORIBOX_STACK_NAME_PREFIXES):
                continue
            if root_path in seen_roots:
                continue
            seen_roots.add(root_path)
            if STACK_SKIP_COMPLETED_BOXES and root_path in completed_roots:
                continue

            # 72_: USD에서 root가 Rigid Body 대표 좌표가 되도록 정리했으므로
            # 실제 후보 bbox/좌표/이동 path는 child가 아니라 root_path를 사용한다.
            box_path = root_path
            bbox = get_world_bbox_info(stage, root_path)
            if bbox is None:
                print(f"  [STACK_SCAN_SKIP] {root_path}: root bbox 계산 실패")
                continue

            results.append({
                "root_name": root_name,
                "root_path": root_path,
                "box_path": box_path,
                "bbox": bbox,
            })

    # 이름 순서 OriBoxA_01, OriBoxA_02, OriBoxA_03 순서 우선
    results.sort(key=lambda x: x["root_name"])
    return results

def select_stack_candidate_in_pick_zone(stage, completed_roots, zone_counts_by_root):
    """
    작업영역 안에 들어온 OriBoxA_* 후보를 고른다.
    같은 root가 PICK_ZONE_REQUIRED_STEPS 만큼 연속으로 영역에 있어야 ready=True.
    """
    candidates = discover_stack_candidates(stage, completed_roots=completed_roots)
    active_roots = set()
    best_inside = None
    best_reason = "no_candidate_inside"

    for cand in candidates:
        root_path = cand["root_path"]
        active_roots.add(root_path)
        inside, reason = is_bbox_center_inside_pick_zone(cand["bbox"])
        if inside:
            zone_counts_by_root[root_path] = int(zone_counts_by_root.get(root_path, 0)) + 1
            if best_inside is None:
                best_inside = cand
                best_reason = reason
        else:
            zone_counts_by_root[root_path] = 0

    # 사라졌거나 완료된 root의 카운터 정리
    for old_root in list(zone_counts_by_root.keys()):
        if old_root not in active_roots:
            zone_counts_by_root.pop(old_root, None)

    if best_inside is None:
        return None, False, (
            f"mode=multi_oriboxa_front_zone,state=WAITING_ZONE,"
            f"candidates={len(candidates)},completed={len(completed_roots)},reason={best_reason}"
        )

    root_path = best_inside["root_path"]
    count = int(zone_counts_by_root.get(root_path, 0))
    ready = count >= int(PICK_ZONE_REQUIRED_STEPS)
    state = "ZONE_READY" if ready else "COUNTING_ZONE"
    bbox = best_inside["bbox"]
    c = bbox["center"]
    t = bbox["top_center"]
    reason = (
        f"mode=multi_oriboxa_front_zone,state={state},"
        f"target={root_path},box={best_inside['box_path']},"
        f"inside={count}/{PICK_ZONE_REQUIRED_STEPS},"
        f"center=({c[0]:.3f},{c[1]:.3f},{c[2]:.4f}),"
        f"top=({t[0]:.3f},{t[1]:.3f},{t[2]:.4f}),"
        f"{best_reason}"
    )
    return best_inside, ready, reason

def compute_stack_slot_step_from_bbox(box_bbox):
    """박스 크기를 기준으로 2열 slot 간격을 계산한다."""
    if not STACK_USE_BOX_SIZE_FOR_SLOT_STEP or box_bbox is None:
        return np.array(STACK_MANUAL_SLOT_STEP, dtype=float)
    size = np.array(box_bbox.get("size", np.array([0.30, 0.25, 0.24])), dtype=float)
    if str(STACK_SLOT_AXIS).upper() == "Y":
        return np.array([0.0, float(size[1]) + float(STACK_SLOT_GAP), 0.0], dtype=float)
    return np.array([float(size[0]) + float(STACK_SLOT_GAP), 0.0, 0.0], dtype=float)

def get_stack_slot_col_layer(slot_index):
    """slot index를 2열 x 2층 좌표로 변환한다."""
    columns = max(1, int(STACK_COLUMNS))
    slot_index = int(max(0, min(int(slot_index), int(STACK_SLOT_COUNT) - 1)))
    col = slot_index % columns
    layer = slot_index // columns
    return col, layer

def compute_stack_goal_center(stage, box_bbox, slot_index):
    """
    75_ 팔레타이징 goal 계산.

    기준:
    - APalt는 /World/APalt Xform marker.
    - APalt는 팔레트 윗면 중심 좌표라고 가정한다.
    - 2x1 테스트: slot 1, slot 2를 APalt 중심 기준으로 좌우에 배치한다.
    - 상자 이동/후보/완료 기준은 child가 아니라 OriBoxA_01/OriBoxB_01 같은 root 좌표다.
    """
    if box_bbox is None:
        raise RuntimeError("box bbox가 없어 pallet goal을 계산할 수 없습니다.")

    box_size = np.array(box_bbox.get("size", np.array([0.30, 0.25, 0.24])), dtype=float)
    box_height = max(float(box_size[2]), 0.05)
    step = compute_stack_slot_step_from_bbox(box_bbox)
    step_len = float(abs(step[0]))
    if step_len < 1e-6:
        step_len = float(max(box_size[0], box_size[1]) + STACK_SLOT_GAP)

    robot_origin, robot_right, robot_forward, robot_up = get_robot_frame_axes_for_stack(stage)

    use_ori_marker = bool(globals().get("USE_PALLET_ORI_A_MARKER", True))
    ori_path = str(globals().get("PALLET_ORI_A_PATH", "/World/APalt"))
    support_path = ori_path
    support_center = None
    support_top_z = None

    if use_ori_marker:
        ori_pos = get_world_translation(stage, ori_path)
        if ori_pos is None:
            raise RuntimeError(
                f"APalt 기준 prim을 찾지 못했습니다: {ori_path}\n"
                "USD에서 /World/APalt 를 팔레트 윗면 중심에 만든 뒤 저장하세요."
            )
        support_center = np.array(ori_pos, dtype=float)
        support_top_z = float(support_center[2])
    else:
        # fallback: 예전 Cube/BoxAprop bbox 기준
        support_path = resolve_stack_support_path(stage)
        support_bbox = get_world_bbox_info(stage, support_path)
        if support_bbox is None:
            raise RuntimeError(
                f"적재 받침을 찾지 못했습니다: {STACK_SUPPORT_CUBE_PATH} 또는 후보 이름 {STACK_SUPPORT_NAME_CANDIDATES}"
            )
        support_center = np.array(support_bbox["center"], dtype=float)
        support_top_z = float(support_bbox["max"][2])

    robot_offset = np.array(BOXAPROP_SLOT_OFFSET_ROBOT, dtype=float)
    world_offset = (
        robot_right * float(robot_offset[0])
        + robot_forward * float(robot_offset[1])
        + robot_up * float(robot_offset[2])
        + np.array(STACK_FIRST_SLOT_OFFSET, dtype=float)
    )

    slot_index = int(max(0, min(int(slot_index), int(STACK_SLOT_COUNT) - 1)))
    col, layer = get_stack_slot_col_layer(slot_index)
    columns = max(1, int(STACK_COLUMNS))

    axis_name = str(globals().get("PALLET_SLOT_AXIS", STACK_SLOT_AXIS)).upper()
    slot_axis_vec = robot_forward if axis_name in ("ROBOT_Y", "Y", "FORWARD") else robot_right

    def _slot_goal(idx):
        c, l = get_stack_slot_col_layer(idx)
        centered_col = float(c) - (float(columns) - 1.0) * 0.5
        g = support_center.copy() + world_offset
        # 2x1: APalt를 중심으로 slot 1/2를 좌우에 배치한다.
        g[:3] += slot_axis_vec * centered_col * step_len
        # APalt는 팔레트 윗면 중심이므로, 박스 root/center는 박스 높이 절반만큼 위에 둔다.
        g[2] = support_top_z + (float(l) + 0.5) * box_height + float(STACK_PLACE_Z_CLEARANCE)
        return g

    goal = _slot_goal(slot_index)

    print(
        f"  [APALT_SUPPORT_117] path={support_path}, center/top=({support_center[0]:.3f},{support_center[1]:.3f},{support_center[2]:.3f}), "
        f"top_z={support_top_z:.4f}, axis={axis_name}, columns={STACK_COLUMNS}, layers={STACK_LAYERS}, "
        f"box_size=({box_size[0]:.3f},{box_size[1]:.3f},{box_size[2]:.3f}), step_len={step_len:.3f}"
    )
    for i in range(int(STACK_SLOT_COUNT)):
        sg = _slot_goal(i)
        rr = robot_relative_vector(stage, sg)
        print(
            f"  [APALT_SLOT_TRANSLATE_117] slot={i + 1}/{STACK_SLOT_COUNT}, "
            f"world_translate=({sg[0]:.4f}, {sg[1]:.4f}, {sg[2]:.4f}), "
            f"robot_relative=({rr[0]:+.4f}, {rr[1]:+.4f}, {rr[2]:+.4f})"
        )

    rr_goal = robot_relative_vector(stage, goal)
    print(
        f"  [STACK_GOAL] slot={slot_index + 1}/{STACK_SLOT_COUNT} "
        f"(col={col + 1}/{STACK_COLUMNS}, layer={layer + 1}/{STACK_LAYERS}), "
        f"pallet_marker={support_path}, goal_center=({goal[0]:.4f}, {goal[1]:.4f}, {goal[2]:.4f}), "
        f"robot_relative=({rr_goal[0]:+.4f}, {rr_goal[1]:+.4f}, {rr_goal[2]:+.4f})"
    )
    return goal

def resolve_slot_marker_center(stage, slot_index, fallback_center=None):
    """APalt 위 정답지 큐브 bbox center를 읽어서 최종 place center로 사용한다."""
    if not bool(globals().get("SLOT_MARKER_PALLETIZING_ENABLED_155", False)):
        return None, "disabled"
    paths = tuple(str(p) for p in globals().get("SLOT_MARKER_PATHS_155", ()))
    if not paths:
        return None, "no marker paths configured"
    idx = int(max(0, slot_index))
    if idx >= len(paths):
        if bool(globals().get("SLOT_MARKER_WRAP_IF_SHORT_155", True)):
            idx = idx % len(paths)
        else:
            return None, f"slot_index={slot_index} out of marker range len={len(paths)}"
    marker_path = paths[idx]
    bbox = get_world_bbox_info(stage, marker_path)
    if bbox is None:
        return None, f"marker bbox not found: {marker_path}"
    center = np.array(bbox["center"], dtype=float)
    mn = np.array(bbox["min"], dtype=float)
    mx = np.array(bbox["max"], dtype=float)
    size = mx - mn
    print(
        f"  [SLOT_MARKER_155] slot_request={int(slot_index)+1}, marker={marker_path}, "
        f"center=({center[0]:.4f},{center[1]:.4f},{center[2]:.4f}), "
        f"size=({size[0]:.4f},{size[1]:.4f},{size[2]:.4f})"
    )
    return center, marker_path

def get_conveyor_max_box_top_z(stage, attached_box_path=None):
    """컨베이어 위 OriBoxA_*들의 최고 top_z를 읽는다. 실패하면 None."""
    if not bool(globals().get("CONVEYOR_SAFE_LIFT_ENABLED_155", False)):
        return None, "disabled"
    prefixes = tuple(str(p) for p in globals().get("CONVEYOR_SAFE_LIFT_INCLUDE_PREFIXES_155", ("/World/OriBoxA_",)))
    exclude_attached = bool(globals().get("CONVEYOR_SAFE_LIFT_EXCLUDE_ATTACHED_155", False))
    best_top = None
    best_path = None
    rows = []
    try:
        for prim in stage.Traverse():
            try:
                p = str(prim.GetPath())
            except Exception:
                continue
            if not any(p.startswith(pref) for pref in prefixes):
                continue
            # root만 본다. /World/OriBoxA_02/Small_Cardboard_box 같은 child는 제외.
            if p.count("/") != 2:
                continue
            if exclude_attached and attached_box_path and p == str(attached_box_path):
                continue
            bbox = get_world_bbox_info(stage, p)
            if bbox is None:
                continue
            top = float(bbox["max"][2])
            rows.append((p, top))
            if best_top is None or top > best_top:
                best_top = top
                best_path = p
    except Exception as e:
        return None, f"scan failed: {type(e).__name__}: {e}"
    if best_top is None:
        return None, "no OriBox top found"
    rows_txt = ", ".join([f"{pp.split('/')[-1]}:{zz:.3f}" for pp, zz in rows[:8]])
    return float(best_top), f"max_top={best_top:.4f} from {best_path}; boxes=[{rows_txt}]"

def lower_stack_support_cube_for_next_layer(stage, box_height, lower_count):
    """1층 2개 완료 후 BoxAprop를 박스 높이만큼 낮춘다."""
    if not STACK_LOWER_SUPPORT_AFTER_EACH_LAYER:
        return int(lower_count)
    lower_count = int(lower_count)
    if lower_count >= int(STACK_LOWER_MAX_LAYERS):
        return lower_count

    bbox = get_world_bbox_info(stage, STACK_SUPPORT_CUBE_PATH)
    if bbox is None:
        print(f"  [STACK_LOWER_SKIP] support cube not found: {STACK_SUPPORT_CUBE_PATH}")
        return lower_count

    box_height = max(float(box_height), 0.05)
    dz = box_height + float(STACK_LOWER_EXTRA_Z)
    cur_center = np.array(bbox["center"], dtype=float)
    new_center = cur_center.copy()
    new_center[2] -= dz

    ok = set_prim_world_translation(stage, STACK_SUPPORT_CUBE_PATH, new_center)
    for _ in range(10):
        simulation_app.update()

    new_bbox = get_world_bbox_info(stage, STACK_SUPPORT_CUBE_PATH)
    new_top = float(new_bbox["max"][2]) if new_bbox is not None else float("nan")
    print(
        f"  [STACK_PLATFORM_LOWER] layer_done={lower_count + 1}/{STACK_LOWER_MAX_LAYERS}, "
        f"dz={dz:.4f}, ok={ok}, "
        f"center_before=({cur_center[0]:.3f},{cur_center[1]:.3f},{cur_center[2]:.4f}), "
        f"center_after=({new_center[0]:.3f},{new_center[1]:.3f},{new_center[2]:.4f}), "
        f"new_top_z={new_top:.4f}"
    )
    return lower_count + 1

def get_robot_center_for_goal(stage):
    """로봇 중심/발판 중심 후보를 순서대로 찾아 반대편 좌표 계산에 사용한다."""
    for p in ROBOT_CENTER_CANDIDATE_PATHS:
        pos = get_world_translation(stage, p)
        if pos is not None and np.all(np.isfinite(pos)):
            return np.array(pos, dtype=float), p
    return None, None

def compute_mirror_goal_center(stage, box_center):
    """
    23_ 목표점 계산:
    - 박스가 컨베이어를 타고 멈춘 현재 box_center를 기준으로 한다.
    - 로봇 중심/발판 중심 후보를 읽어서 XY를 대칭 이동한다.
      goal_xy = 2*robot_xy - box_xy
    - 단, 완전 대칭점이 너무 멀면 로봇이 못 가고 release가 안 되므로 이동 거리를 제한한다.
    - Z는 실제 place 바닥 높이가 아니라 custom carry에서 별도 제어한다.
    """
    box_center = np.array(box_center, dtype=float)
    goal = box_center.copy()

    robot_center, center_path = get_robot_center_for_goal(stage)
    if robot_center is None:
        goal = box_center + np.array(GOAL_OFFSET_FROM_BOX_CENTER, dtype=float)
        goal[2] = box_center[2]
        print(f"  [WARN] robot center를 읽지 못해서 fallback goal offset 사용: goal={goal}")
        return goal

    raw_goal = box_center.copy()
    raw_goal[0] = 2.0 * robot_center[0] - box_center[0]
    raw_goal[1] = 2.0 * robot_center[1] - box_center[1]
    raw_goal[2] = box_center[2]

    delta_xy = raw_goal[:2] - box_center[:2]
    dist_xy = float(np.linalg.norm(delta_xy))
    max_dist = float(MIRROR_GOAL_MAX_XY_DISTANCE_FROM_PICK)
    goal[:] = raw_goal
    if max_dist > 0.0 and dist_xy > max_dist:
        scale = max_dist / max(dist_xy, 1e-9)
        goal[:2] = box_center[:2] + delta_xy * scale
        print(
            f"  [GOAL_CLAMP] mirror target distance {dist_xy:.3f}m > {max_dist:.3f}m. "
            f"목표를 로봇 도달 가능한 거리로 제한한다."
        )

    print(
        f"  [GOAL] mode={GOAL_MODE}, center_path={center_path}, "
        f"robot_center=({robot_center[0]:.3f},{robot_center[1]:.3f},{robot_center[2]:.3f}), "
        f"box_center=({box_center[0]:.3f},{box_center[1]:.3f},{box_center[2]:.3f}), "
        f"raw_goal=({raw_goal[0]:.3f},{raw_goal[1]:.3f},{raw_goal[2]:.3f}), "
        f"goal_center=({goal[0]:.3f},{goal[1]:.3f},{goal[2]:.3f})"
    )
    return goal

def set_task_pick_from_current_box(task, stage, bbox_info):
    """
    박스가 멈춘 순간의 bbox를 기준으로 pick 위치를 다시 확정한다.

    13_ 수정:
    - box 초기 원점/root가 바닥 기준이어도 bbox center/top_center를 다시 계산한다.
    - 로봇 controller에 넣는 picking_position은 bbox center가 아니라
      bbox top_center + BOX_PICK_TOP_CLEARANCE 를 사용한다.
    - 실제 scripted follow/release는 bbox center 기준으로 유지한다.
    """
    if bbox_info is None:
        return False

    center = np.array(bbox_info["center"], dtype=float)
    top_center = np.array(bbox_info["top_center"], dtype=float)
    height = float(bbox_info.get("height", top_center[2] - center[2]))

    root_pos = get_world_translation(stage, task.box_move_path)
    if root_pos is None:
        root_pos = center.copy()

    task._box_initial_root_pos = np.array(root_pos, dtype=float)
    task._box_initial_center = center.copy()
    task._box_initial_top_center = top_center.copy()
    task._box_height = height
    task._box_root_to_center_offset = task._box_initial_root_pos - task._box_initial_center
    try:
        root_ref = getattr(task, "active_box_root_path", None)
        if root_ref:
            root_ref_pos = get_world_translation(stage, root_ref)
            if root_ref_pos is not None:
                sep = float(np.linalg.norm(np.array(root_ref_pos, dtype=float) - np.array(task._box_initial_center, dtype=float)))
                print(
                    f"  [ROOT_CHILD_CHECK] root={root_ref}, move={task.box_move_path}, "
                    f"root_to_bbox_center={sep:.4f}m, move_path_is_child={task.box_move_path == task.box_path}"
                )
    except Exception:
        pass

    if USE_BOX_TOP_CENTER_AS_PICK_TARGET:
        task._pick_center = top_center.copy()
        task._pick_center[2] += float(BOX_PICK_TOP_CLEARANCE)
    else:
        task._pick_center = center.copy()

    # 16_ 핵심: M0609/발판 위치 변경 후 실제 VGC10 suction point가 박스보다
    # X+ / Y- 방향으로 빗나가므로 picking_position 자체를 보정한다.
    task._pick_center = np.array(task._pick_center, dtype=float) + np.array(PICK_TARGET_MANUAL_OFFSET, dtype=float)

    # 47_: goal은 기본 mirror 대신 BoxAprop 위 slot 좌표로 계산한다.
    # 박스는 흡착 전에는 절대 코드로 옮기지 않고, 감지 순간의 bbox를 기준으로만 목표를 계산한다.
    if MULTI_ORIBOX_STACKING_ENABLED:
        slot_index = int(getattr(task, "_stack_slot_index", 0))
        task._goal_center = compute_stack_goal_center(stage, bbox_info, slot_index)
    else:
        task._goal_center = compute_mirror_goal_center(stage, task._box_initial_center)
    return True

def move_box_center_to(stage, prim_path, desired_center, root_to_center_offset):
    """
    bbox center를 desired_center로 보내기 위해 root world translate를 계산해서 이동한다.
    root_to_center_offset = root_world_pos - bbox_center
    """
    desired_center = np.array(desired_center, dtype=float)
    desired_root = desired_center + np.array(root_to_center_offset, dtype=float)
    ok = set_prim_world_translation(stage, prim_path, desired_root)
    zero_subtree_velocity(stage, prim_path)
    return ok

def probe_move_path_affects_box_bbox(stage, move_path, box_path, delta_z=MOTION_PROBE_DELTA_Z):
    """
    move_path를 살짝 들어올렸을 때 box_path의 bbox가 실제로 따라오는지 확인한다.
    이 테스트가 실패하면 흡착 판정이 surface=ON이어도 박스가 안 들린다.
    """
    print("\n" + "=" * 60)
    print("[14.PROBE] 이동 prim이 실제 박스 bbox를 움직이는지 검사")
    print("=" * 60)
    print(f"  box_path  = {box_path}")
    print(f"  move_path = {move_path}")

    before_bbox = get_world_bbox_info(stage, box_path)
    before_move_pos = get_world_translation(stage, move_path)
    if before_bbox is None or before_move_pos is None:
        print("  [PROBE_FAIL] bbox 또는 move_path world translation을 읽지 못함")
        return False

    before_center = np.array(before_bbox["center"], dtype=float)
    target_move_pos = np.array(before_move_pos, dtype=float) + np.array([0.0, 0.0, float(delta_z)], dtype=float)
    ok_set = set_prim_world_translation(stage, move_path, target_move_pos)
    for _ in range(5):
        simulation_app.update()

    after_bbox = get_world_bbox_info(stage, box_path)
    after_center = np.array(after_bbox["center"], dtype=float) if after_bbox is not None else before_center.copy()
    moved = float(np.linalg.norm(after_center - before_center))
    moved_z = float(after_center[2] - before_center[2])

    # 원위치 복구
    set_prim_world_translation(stage, move_path, before_move_pos)
    for _ in range(5):
        simulation_app.update()

    success = bool(ok_set and abs(moved_z) > abs(delta_z) * 0.5)
    print(f"  [PROBE_RESULT] ok_set={ok_set}, bbox_moved={moved:.5f}m, bbox_moved_z={moved_z:.5f}m, expected_z≈{delta_z:.5f}m, success={success}")
    if not success:
        print("  [PROBE_HINT] 이 move_path로 움직여도 박스 bbox가 안 따라온다. BOX_MOVE_PRIM_PATH를 부모 prim으로 바꿔야 한다.")
    else:
        print("  [PROBE_OK] scripted suction에서 이 move_path를 움직이면 박스가 실제로 따라온다.")
    return success

def should_attach_oribox(event, suction_pos, bbox_info, box_stopped=True):
    """
    51_ 수정: 단일 suction point 대신 3x3 흡착점 그리드로 흡착 여부를 판단한다.

    조건:
    - 작업 영역에 들어온 OriBoxA_ 박스의 윗면 bbox 기준
    - 9개 점 중 SUCTION_GRID_ATTACH_MIN_POINTS개 이상이 윗면 안쪽에 있어야 함
    - 기본적으로 중심점 p11도 윗면 안쪽에 있어야 함
    """
    if bbox_info is None:
        return False, "grid_no_bbox"

    if WAIT_UNTIL_BOX_STOPPED_BEFORE_PICK and not box_stopped:
        return False, "box_not_stopped"

    stage = omni.usd.get_context().get_stage()

    if event in PICK_CLOSE_EVENTS:
        ok, summary, _info = evaluate_suction_grid_on_box_top(stage, bbox_info, event=event, verbose=False)
        return ok, summary

    if event in RETRY_CLOSE_EVENTS:
        ok, summary, _info = evaluate_suction_grid_on_box_top(stage, bbox_info, event=event, verbose=True)
        return ok, "retry_" + summary

    return False, "not_pick_event"

class M0609ConveyorBoxTask(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._task_achieved = False
        self._box_path = BOX_PRIM_PATH
        self._box_move_path = BOX_MOVE_PRIM_PATH
        self._stop_check_path = STOP_CHECK_PRIM_PATH
        self._active_box_root_name = None
        self._active_box_root_path = None  # 66_: 완료/skip 기준 root path 보관
        self._stack_slot_index = 0
        self._box_initial_root_pos = None
        self._box_initial_center = None
        self._box_initial_top_center = None
        self._box_height = None
        self._pick_center = None
        self._box_root_to_center_offset = None
        self._goal_center = None

    @property
    def box_path(self):
        return self._box_path

    @property
    def box_move_path(self):
        return self._box_move_path

    @property
    def active_box_root_path(self):
        return self._active_box_root_path

    def set_active_box(self, box_root_path, box_path=None, stack_slot_index=None):
        """현재 pick 대상으로 사용할 OriBoxA_*/OriBoxB_* root를 갱신한다.

        72_ 핵심:
        - USD에서 root에 Rigid Body를 붙이고 child는 Collider만 남긴 구조를 사용한다.
        - 완료/후보 제외, bbox/흡착/정지 판정, carry/follow 이동 모두 root 기준으로 통일한다.
        - box_path 인자는 과거 child 기반 코드 호환용으로만 받고 실제 기준에는 사용하지 않는다.
        """
        root_path = str(box_root_path)
        self._active_box_root_path = root_path
        self._box_move_path = root_path
        self._box_path = root_path
        self._stop_check_path = root_path
        self._active_box_root_name = root_path.rsplit("/", 1)[-1]
        if stack_slot_index is not None:
            self._stack_slot_index = int(stack_slot_index)
        self._box_initial_root_pos = None
        self._box_initial_center = None
        self._box_initial_top_center = None
        self._box_height = None
        self._pick_center = None
        self._box_root_to_center_offset = None
        self._goal_center = None

    @property
    def stop_check_path(self):
        return self._stop_check_path

    @property
    def goal_center(self):
        return self._goal_center

    @property
    def box_initial_center(self):
        return self._box_initial_center

    @property
    def box_initial_top_center(self):
        return self._box_initial_top_center

    @property
    def pick_center(self):
        return self._pick_center if self._pick_center is not None else self._box_initial_center

    @property
    def box_height(self):
        return self._box_height

    @property
    def box_root_to_center_offset(self):
        return self._box_root_to_center_offset

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._remove_old_gripper()
        self._discover_links()
        self._attach_vgc10()
        self._attach_idle_vgc10_robots()
        self._setup_physics()
        self._register_robot(scene)
        self._setup_existing_box(scene)
        self._setup_vgc10_suction_anchor(scene)
        print("\n  [완료] Conveyor + OriBoxA 씬 구성 성공!\n")

    def _load_usd(self):
        print("\n" + "=" * 60)
        print("[1.LOAD] Conveyor USD 로드")
        print("=" * 60)

        usd_file = Path(USD_PATH)
        if not usd_file.exists():
            raise RuntimeError(f"Conveyor USD 파일이 없습니다: {USD_PATH}")

        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

        world_prim.GetReferences().AddReference(USD_PATH)

        # 147_: 사용자가 USD에서 /World 밖(root)에 둔 /Environment는
        # /World prim에 USD를 reference하면 defaultPrim(/World)만 들어오면서 빠질 수 있다.
        # 그래서 원본 USD의 /Environment prim을 현재 stage의 /Environment로 별도 reference한다.
        try:
            env_prim = stage.GetPrimAtPath("/Environment")
            if not env_prim or not env_prim.IsValid():
                env_prim = UsdGeom.Xform.Define(stage, "/Environment").GetPrim()
            env_prim.GetReferences().AddReference(USD_PATH, "/Environment")
            print("  [ENV_REF_147] root /Environment 별도 reference 추가: /Environment <= USD:/Environment")
        except Exception as _env_ref_e:
            print(f"  [ENV_REF_147][WARN] /Environment 별도 reference 실패: {_env_ref_e}")

        for _ in range(40):
            simulation_app.update()

        # 147_: reference composition 후 root /Environment 및 하위 prim을 active/visible로 강제 복구한다.
        force_show_environment_prims(stage)

        # 29_: USD 안에 저장된 goal_marker가 있어도 실행 시 보이지 않게 정리한다.
        if not SHOW_GOAL_MARKER:
            remove_goal_marker(stage)

        # 106_: rsd455 prim은 유지하고, robot 하위 legacy camera_graph만 필요 시 비활성화한다.
        disable_legacy_camera_graphs(stage)

        # 109_: 사진으로 보낸 Cube(/Cube 또는 /World/Cube)가 실행 후에도 화면에 보이게 유지한다.
        force_show_cube_prims(stage)

        # 146_: 새 USD에서 Prim Path가 Environment인 환경 오브젝트들이 사라지지 않게 강제 표시한다.
        force_show_environment_prims(stage)

        # 77_: pick zone visual은 생성하지 않는다. 감지 계산은 PICK_ZONE_CENTER/SIZE 숫자만 사용한다.
        create_pick_zone_visual(stage)

        global ROBOT_PRIM_PATH, OLD_GRIPPER_PRIM_PATH, VGC10_FOLLOW_TARGET_PATH

        resolved_robot_path = _resolve_robot_prim_from_root(stage, ACTIVE_ROBOT_ROOT_PATH)
        if resolved_robot_path is not None:
            ROBOT_PRIM_PATH = resolved_robot_path
            OLD_GRIPPER_PRIM_PATH = ROBOT_PRIM_PATH + "/onrobot_rg2ft"
            resolved_tool_target = _find_tool_target_for_robot_prim(stage, ROBOT_PRIM_PATH)
            if resolved_tool_target is not None:
                VGC10_FOLLOW_TARGET_PATH = resolved_tool_target

        robot_prim = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
        if not robot_prim.IsValid():
            candidates = []
            try:
                world = stage.GetPrimAtPath("/World")
                if world.IsValid():
                    for child in world.GetChildren():
                        if child.GetName().startswith(IDLE_M0609_ROOT_PREFIX) or "m0609" in child.GetName().lower():
                            candidates.append(str(child.GetPath()))
            except Exception:
                pass
            raise RuntimeError(
                f"Conveyor USD는 열었지만 active 로봇 prim을 찾지 못했습니다.\n"
                f"  ACTIVE_ROBOT_ROOT_PATH={ACTIVE_ROBOT_ROOT_PATH}\n"
                f"  ROBOT_PRIM_PATH={ROBOT_PRIM_PATH}\n"
                f"  USD_PATH={USD_PATH}\n"
                f"  m0609 candidates={candidates}\n"
                f"Stage에서 실제 로봇 경로를 확인해서 ACTIVE_ROBOT_ROOT_PATH를 수정하세요."
            )

        print(f"  [OK] {USD_PATH}")
        print(f"  [OK] active robot root = {ACTIVE_ROBOT_ROOT_PATH}")
        print(f"  [OK] active robot prim = {ROBOT_PRIM_PATH}")
        print(f"  [OK] active VGC10 target = {VGC10_FOLLOW_TARGET_PATH}")

    def _remove_old_gripper(self):
        print("\n" + "=" * 60)
        print("[1-1.REMOVE] 기존 RG2 집게 삭제")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()
        remove_old_gripper(stage)
        for _ in range(5):
            simulation_app.update()

    def _discover_links(self):
        print("\n" + "=" * 60)
        print("[2.DISCOVER] 링크 경로 탐색")
        print("=" * 60)
        self._ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")
        print(f"  EE ({EE_LINK_NAME}) = {self._ee_path}")

    def _attach_vgc10(self):
        print("\n" + "=" * 60)
        print("[2-1.ATTACH] VGC10 Vacuum Gripper 부착 - active m0609_A")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()
        attach_vgc10_to_link6(stage)
        for _ in range(5):
            simulation_app.update()

    def _attach_idle_vgc10_robots(self):
        print("\n" + "=" * 60)
        print("[2-2.IDLE] 다른 m0609_ 로봇 VGC10 visual 부착 / 제어 없음")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()
        self._idle_vgc10_attached = attach_vgc10_to_idle_m0609_robots(stage)
        for _ in range(5):
            simulation_app.update()

    def _setup_physics(self):
        print("\n" + "=" * 60)
        print("[3.PHYSICS] 물리 설정")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()

        drive_count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    drive_count += 1
        print(f"  [OK] drive updated: {drive_count}")

    def _register_robot(self, scene):
        print("\n" + "=" * 60)
        print("[4.REGISTER] 로봇 등록")
        print("=" * 60)
        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name=ROBOT_OBJECT_NAME,
                end_effector_prim_path=self._ee_path,
                gripper=None,
            )
        )
        print(f"  [OK] SingleManipulator without RG2 gripper: {ROBOT_PRIM_PATH}")

    def _setup_existing_box(self, scene):
        print("\n" + "=" * 60)
        print("[5.SCENE] OriBoxA_* 다중 박스 대상 등록 / 첫 후보 확인")
        print("=" * 60)

        stage = omni.usd.get_context().get_stage()

        # 63_: /World 하위의 OriBoxA_01, OriBoxA_02, ... 후보를 우선 사용한다.
        candidates = discover_stack_candidates(stage, completed_roots=set()) if MULTI_ORIBOX_STACKING_ENABLED else []
        if candidates:
            first = candidates[0]
            self.set_active_box(first["root_path"], first["box_path"], stack_slot_index=0)
            print(f"  [OK] 초기 후보 box root = {first['root_path']}")
            print(f"  [OK] 초기 후보 box path = {first['box_path']}")
        else:
            # 72_: 호환 fallback도 root 기준 단일 OriBox를 사용한다.
            self._box_path = ensure_target_box_exists(stage)
            if not stage.GetPrimAtPath(self._box_path).IsValid():
                raise RuntimeError(
                    f"직접 배치된 OriBox root를 찾지 못했습니다. GUI에서 /World/OriBoxA_01 root가 있는지 확인하세요.\n"
                    f"  required root={BOX_ROOT_PATH}"
                )
            self.set_active_box(BOX_ROOT_PATH, self._box_path, stack_slot_index=0)
            print("  [WARN] /World/OriBoxA_/OriBoxB 후보가 없어 단일 BOX_ROOT_PATH fallback 사용")

        # 65_: robotAprop_01 / OriBoxA_* collider 상태를 실행 중 Stage에서 명시적으로 보정한다.
        # 주의: 이 보정은 물리 충돌용 collider를 켜는 것이고, top-lock으로 직접 Xform 이동 중인 박스가
        # collider에 의해 자동으로 멈추는 것은 아니다. 그래서 아래 make_custom_carry_targets()에서
        # robotAprop_01 위를 충분히 높은 Z로 지나가도록 추가 보정도 한다.
        repair_robotaprop_and_oribox_colliders(stage, completed_roots=set(), verbose=True)
        # 68_: 실행 시작 시점에 USD에서 맞춘 root/child 중심이 벌어져 있으면 다시 맞춘다.
        sync_oribox_child_centers_with_roots(stage, verbose=True)

        # 77_: USD 직접 편집 환경만 사용한다. 예전 코드처럼 default ground plane을 새로 만들지 않는다.
        if bool(globals().get("ADD_DEFAULT_GROUND_PLANE", False)):
            try:
                scene.add_default_ground_plane()
                print("  [OK] default ground plane")
            except Exception as exc:
                print(f"  [WARN] ground plane 추가 실패 또는 이미 존재: {exc}")
        else:
            print("  [CLEAN] default ground plane 생성 안 함: USD 환경 그대로 사용")

        bbox = get_world_bbox_info(stage, self._box_path)
        if bbox is None:
            raise RuntimeError(f"박스 bbox를 계산하지 못했습니다: {self._box_path}")

        root_pos = get_world_translation(stage, self._box_move_path)
        if root_pos is None:
            root_pos = bbox["center"].copy()

        self._box_initial_root_pos = np.array(root_pos, dtype=float)
        self._box_initial_center = np.array(bbox["center"], dtype=float)
        self._box_initial_top_center = np.array(bbox["top_center"], dtype=float)
        self._box_height = float(bbox.get("height", bbox["size"][2]))
        self._box_root_to_center_offset = self._box_initial_root_pos - self._box_initial_center

        if USE_BOX_TOP_CENTER_AS_PICK_TARGET:
            self._pick_center = self._box_initial_top_center.copy()
            self._pick_center[2] += float(BOX_PICK_TOP_CLEARANCE)
        else:
            self._pick_center = self._box_initial_center.copy()
        self._pick_center = np.array(self._pick_center, dtype=float) + np.array(PICK_TARGET_MANUAL_OFFSET, dtype=float)

        if MULTI_ORIBOX_STACKING_ENABLED:
            self._goal_center = compute_stack_goal_center(stage, bbox, int(self._stack_slot_index))
        else:
            self._goal_center = compute_mirror_goal_center(stage, self._box_initial_center)

        if not stage.GetPrimAtPath(self._stop_check_path).IsValid():
            raise RuntimeError(f"정지/영역 판정 대상 prim을 찾지 못했습니다: {self._stop_check_path}")

        print(f"  [OK] box path        = {self._box_path}")
        print(f"  [OK] box move path   = {self._box_move_path}")
        print(f"  [OK] stop check path = {self._stop_check_path}")
        print(f"  [OK] stack slot      = {self._stack_slot_index + 1}/{STACK_SLOT_COUNT}")
        print(f"  [OK] root world pos  = {self._box_initial_root_pos}")
        print(f"  [OK] bbox center     = {self._box_initial_center}")
        print(f"  [OK] bbox size       = {bbox['size']}")
        print(f"  [OK] bbox top center = {bbox['top_center']}")
        print(f"  [OK] bbox height     = {self._box_height:.5f} m")
        print(f"  [OK] pick target     = {self._pick_center}  # top_center + {BOX_PICK_TOP_CLEARANCE:.3f}m + manual_offset {PICK_TARGET_MANUAL_OFFSET}")
        print(f"  [OK] goal center     = {self._goal_center}")

        remove_goal_marker(stage)

        # 처음에는 일반 물리 상태로 둔다.
        set_prim_kinematic(stage, self._box_path, False)
        zero_prim_velocity(stage, self._box_path)
        if DEBUG_MOVE_PARENT_AND_FOLLOW:
            probe_move_path_affects_box_bbox(stage, self._box_move_path, self._box_path)

    def _setup_vgc10_suction_anchor(self, scene):
        print("\n" + "=" * 60)
        print("[5-1.VGC10 SUCTION] VGC10 기준 흡착점 활성화")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()

        clear_prim_if_exists(stage, SUCTION_BODY_PATH)
        clear_prim_if_exists(stage, SUCTION_MARKER_PATH)
        clear_prim_if_exists(stage, VGC10_SUCTION_DEBUG_MARKER_PATH)

        if not stage.GetPrimAtPath(VGC10_SUCTION_POINT_PATH).IsValid():
            suction_point_xform = UsdGeom.Xform.Define(stage, VGC10_SUCTION_POINT_PATH)
            suction_point_prim = suction_point_xform.GetPrim()
            suction_mat = Gf.Matrix4d(1.0)
            suction_mat.SetTranslateOnly(Gf.Vec3d(
                float(VGC10_SUCTION_LOCAL_OFFSET[0]),
                float(VGC10_SUCTION_LOCAL_OFFSET[1]),
                float(VGC10_SUCTION_LOCAL_OFFSET[2]),
            ))
            set_prim_local_matrix(suction_point_prim, suction_mat)

        print("  [OK] scripted_suction_body 제거")
        print("  [OK] scripted_suction_direction_marker 제거")
        print(f"  [OK] VGC10 suction point 활성화: {VGC10_SUCTION_POINT_PATH}")
        print(f"       offset = {VGC10_SUCTION_LOCAL_OFFSET}")
        print("       화면에 흡착점이 필요하면 DEBUG_SHOW_SUCTION_POINT=True 로 바꿔라.")

    def get_observations(self):
        stage = omni.usd.get_context().get_stage()
        bbox = get_world_bbox_info(stage, self._box_path)
        if bbox is None:
            box_center = self._box_initial_center.copy()
            box_top_center = self._box_initial_center.copy()
        else:
            box_center = bbox["center"]
            box_top_center = bbox["top_center"]

        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            },
            "oribox": {
                "position": box_center,
                "top_center": box_top_center,
                "goal_position": self._goal_center,
            },
        }

    def pre_step(self, control_index, simulation_time):
        if self._goal_center is None:
            return
        stage = omni.usd.get_context().get_stage()
        bbox = get_world_bbox_info(stage, self._box_path)
        if bbox is None:
            return
        if not self._task_achieved and np.linalg.norm(self._goal_center - bbox["center"]) < 0.08:
            self._task_achieved = True

    def post_reset(self):
        stage = omni.usd.get_context().get_stage()
        # 19_: 박스 위치는 코드로 되돌리지 않는다. 컨베이어/물리 흐름에 맡긴다.
        if RESET_BOX_TO_USD_START_ON_PLAY and self._box_initial_root_pos is not None:
            set_prim_world_translation(stage, self._box_move_path, self._box_initial_root_pos)
        set_prim_kinematic(stage, self._box_path, False)
        zero_prim_velocity(stage, self._box_path)
        self._task_achieved = False

def make_ee_target_for_desired_suction(robot, current_suction_pos, desired_suction_pos, fixed_orientation=None):
    """
    RMPFlow는 link_6 EE를 움직이고, 실제 흡착점은 VGC10 suction point다.
    현재 EE와 suction point 사이의 world offset을 유지한 채 desired_suction_pos가 되도록 EE target을 계산한다.

    fixed_orientation=None이면 orientation constraint를 걸지 않는다.
    55_에서는 pre-align 중 하늘 보는 자세가 고정되는 문제를 피하기 위해 None을 사용한다.
    """
    ee_pos, ee_quat = get_current_ee_pose(robot)
    current_suction_pos = _vec3(current_suction_pos)
    desired_suction_pos = _vec3(desired_suction_pos)
    ee_to_suction = current_suction_pos - ee_pos
    target_ee_pos = desired_suction_pos - ee_to_suction
    target_ee_quat = fixed_orientation
    return target_ee_pos, target_ee_quat

def apply_cartesian_suction_target(cart_controller, robot, current_suction_pos, desired_suction_pos, fixed_orientation=None):
    target_ee_pos, target_ee_quat = make_ee_target_for_desired_suction(
        robot,
        current_suction_pos=current_suction_pos,
        desired_suction_pos=desired_suction_pos,
        fixed_orientation=fixed_orientation,
    )
    kwargs = {"target_end_effector_position": target_ee_pos}
    if target_ee_quat is not None:
        kwargs["target_end_effector_orientation"] = target_ee_quat
    action = cart_controller.forward(**kwargs)
    robot.apply_action(action)
    return target_ee_pos

def _norm2(v, fallback=None):
    v = np.array(v, dtype=float)[:2]
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        if fallback is None:
            return np.array([1.0, 0.0], dtype=float)
        f = np.array(fallback, dtype=float)[:2]
        fn = float(np.linalg.norm(f))
        return f / max(fn, 1e-9)
    return v / n

def _push_xy_outside_robot_guard(stage, xy, center_xy=None):
    """목표 XY가 발판/로봇 중심 금지 반경 안이면 바깥으로 밀어낸다."""
    xy = np.array(xy, dtype=float)[:2]
    if not CARRY_AVOID_BASE_AND_ARM:
        return xy
    if center_xy is None:
        robot_center, _ = get_robot_center_for_goal(stage)
        if robot_center is None:
            return xy
        center_xy = np.array(robot_center, dtype=float)[:2]
    else:
        center_xy = np.array(center_xy, dtype=float)[:2]

    guard = float(CARRY_FORBIDDEN_RADIUS_XY + CARRY_ROUTE_MARGIN_XY)
    v = xy - center_xy
    d = float(np.linalg.norm(v))
    if d < guard:
        v = _norm2(v, fallback=np.array([0.0, -1.0]))
        xy = center_xy + v * guard
        print(f"  [AVOID_GOAL] place xy was inside robot/base guard. pushed outside to ({xy[0]:.3f},{xy[1]:.3f}), guard={guard:.3f}")
    return xy

def _make_avoidance_suction_waypoints(stage, attach_suction_pos, goal_suction_pos):
    """
    시작 suction XY에서 목표 suction XY까지 직선으로 가지 않고,
    로봇/발판 중심 주변 금지 원을 옆으로 돌아가는 waypoint를 만든다.
    """
    start = np.array(attach_suction_pos, dtype=float)
    goal = np.array(goal_suction_pos, dtype=float)
    safe_z = float(max(start[2], goal[2], CUSTOM_CARRY_SAFE_SUCTION_Z))
    start[2] = safe_z
    goal[2] = safe_z

    robot_center, center_path = get_robot_center_for_goal(stage)
    if (not CARRY_AVOID_BASE_AND_ARM) or robot_center is None:
        return [
            {"name": "lift", "kind": "lift", "target": start.copy()},
            {"name": "move", "kind": "move", "target": goal.copy()},
        ]

    center_xy = np.array(robot_center, dtype=float)[:2]
    start_xy = start[:2]
    goal_xy = goal[:2]

    main_dir = _norm2(goal_xy - start_xy, fallback=start_xy - center_xy)
    tangent = np.array([-main_dir[1], main_dir[0]], dtype=float) * float(CARRY_ROUTE_SIDE_SIGN)
    guard = float(CARRY_FORBIDDEN_RADIUS_XY + CARRY_ROUTE_MARGIN_XY)

    # start/goal을 main_dir 축으로 투영하고, tangent 방향으로 guard만큼 빼서 중심을 돌아간다.
    start_d = float(np.dot(start_xy - center_xy, main_dir))
    goal_d = float(np.dot(goal_xy - center_xy, main_dir))

    via1_xy = center_xy + main_dir * start_d + tangent * guard
    via2_xy = center_xy + main_dir * goal_d + tangent * guard

    # waypoint가 너무 가까우면 생략해도 되지만, 로그/디버깅을 위해 그대로 둔다.
    via1 = np.array([via1_xy[0], via1_xy[1], safe_z], dtype=float)
    via2 = np.array([via2_xy[0], via2_xy[1], safe_z], dtype=float)

    print(
        f"  [AVOID_ROUTE] center_path={center_path}, center=({center_xy[0]:.3f},{center_xy[1]:.3f}), "
        f"guard={guard:.3f}, side={CARRY_ROUTE_SIDE_SIGN}, "
        f"start=({start_xy[0]:.3f},{start_xy[1]:.3f}), "
        f"via1=({via1[0]:.3f},{via1[1]:.3f}), via2=({via2[0]:.3f},{via2[1]:.3f}), "
        f"goal=({goal_xy[0]:.3f},{goal_xy[1]:.3f}), z={safe_z:.3f}"
    )

    return [
        {"name": "lift", "kind": "lift", "target": start.copy()},
        {"name": "avoid_start", "kind": "move", "target": via1},
        {"name": "avoid_goal", "kind": "move", "target": via2},
        {"name": "approach_goal", "kind": "move", "target": goal.copy()},
    ]

def _fit_joint_delta(joints, delta):
    joints = np.array(joints, dtype=float)
    delta = np.array(delta, dtype=float)
    if len(delta) < len(joints):
        delta = np.pad(delta, (0, len(joints) - len(delta)), mode="constant")
    elif len(delta) > len(joints):
        delta = delta[:len(joints)]
    return delta

def _clamp_first_joint_delta(delta):
    delta = float(delta)
    return float(np.clip(delta, -float(JOINT_SWING_CLAMP_RAD), float(JOINT_SWING_CLAMP_RAD)))

def _estimate_link2_orient_z_deg_from_j2(j2_rad, reference_j2_rad):
    """
    Stage GUI에서 보이는 초기 link_2 Orient Z=-90도를 기준으로 joint_2 변화량을 더해 추정한다.
    이 값은 USD Transform을 직접 쓰는 값이 아니라, joint_2 목표 제한을 위한 진단/가드 값이다.
    """
    sign = float(globals().get("LINK2_ORIENT_Z_SIGN", 1.0))
    initial_deg = float(globals().get("LINK2_ORIENT_Z_INITIAL_DEG", -90.0))
    return float(initial_deg + sign * np.degrees(float(j2_rad) - float(reference_j2_rad)))

def _link2_guard_j2_boundary_rad(reference_j2_rad):
    sign = float(globals().get("LINK2_ORIENT_Z_SIGN", 1.0))
    initial_deg = float(globals().get("LINK2_ORIENT_Z_INITIAL_DEG", -90.0))
    min_deg = float(globals().get("LINK2_ORIENT_Z_MIN_DEG", -90.0))
    if abs(sign) < 1.0e-9:
        return float(reference_j2_rad)
    return float(reference_j2_rad + np.radians((min_deg - initial_deg) / sign))

def _apply_link2_orient_z_guard_to_joints(joints, reference_j2_rad, label="target"):
    arr = np.array(joints, dtype=float).copy()
    if (not bool(globals().get("LINK2_ORIENT_Z_GUARD_ENABLED", False))) or len(arr) < 2:
        return arr
    sign = float(globals().get("LINK2_ORIENT_Z_SIGN", 1.0))
    min_deg = float(globals().get("LINK2_ORIENT_Z_MIN_DEG", -90.0))
    before_j2 = float(arr[1])
    before_est = _estimate_link2_orient_z_deg_from_j2(before_j2, reference_j2_rad)
    boundary_j2 = _link2_guard_j2_boundary_rad(reference_j2_rad)

    clamped = False
    if sign >= 0.0:
        if arr[1] < boundary_j2:
            arr[1] = boundary_j2
            clamped = True
    else:
        if arr[1] > boundary_j2:
            arr[1] = boundary_j2
            clamped = True

    after_est = _estimate_link2_orient_z_deg_from_j2(float(arr[1]), reference_j2_rad)
    if bool(globals().get("LINK2_ORIENT_Z_GUARD_LOG", True)):
        print(
            f"[LINK2_ORIENT_GUARD_108][{label}] "
            f"ref_j2={reference_j2_rad:+.6f} rad, boundary_j2={boundary_j2:+.6f} rad, "
            f"j2_before={before_j2:+.6f} rad -> j2_after={float(arr[1]):+.6f} rad, "
            f"link2_z_est_before={before_est:+.2f} deg, link2_z_est_after={after_est:+.2f} deg, "
            f"min={min_deg:+.2f} deg, clamped={clamped}"
        )
    return arr

def make_joint_swing_carry_targets(stage, robot, task, attach_center, attach_suction_pos, attached_center_offset):
    """
    78_ 관절 기반: 수직 lift 후 팔레타이징 이동은 joint_1(link_1 z축) 회전만 사용. 단, lift 자세를 더 펴서 회전 원호 반지름을 넓힌다.
    - link_2 transform을 직접 바꾸지 않고 joint_2/joint_3로 들어올린다.
    - 팔레트 slot 방향 이동은 joint_1 회전으로만 처리한다.
    - 박스는 매 step suction point + attached_center_offset만 따라온다.
    """
    start_joints = np.array(robot.get_joint_positions(), dtype=float)
    dof = len(start_joints)

    lift_delta = np.zeros(dof, dtype=float)
    if dof >= 2:
        lift_delta[1] = float(JOINT_LIFT_SIGN) * float(JOINT_LIFT_J2_DELTA_RAD)
    if dof >= 3 and JOINT_USE_J3_FOR_LIFT:
        lift_delta[2] = float(JOINT_LIFT_SIGN) * float(JOINT_LIFT_J3_DELTA_RAD)

    lift_joints = start_joints + lift_delta

    # 108_: link_2 Orient Z guard. 초기 link_2 Orient Z=-90도 기준을 joint_2 제한으로 변환한다.
    link2_j2_ref_rad = float(start_joints[1]) if dof >= 2 else 0.0
    lift_joints = _apply_link2_orient_z_guard_to_joints(lift_joints, link2_j2_ref_rad, label="lift_joints")

    swing_delta = np.zeros(dof, dtype=float)
    if dof >= 1:
        swing_delta[0] = _clamp_first_joint_delta(float(JOINT_SWING_SIGN) * float(JOINT_SWING_DELTA_RAD))
    swing_joints = lift_joints + swing_delta
    swing_joints = _apply_link2_orient_z_guard_to_joints(swing_joints, link2_j2_ref_rad, label="swing_joints_initial")

    lower_joints = swing_joints - lift_delta * float(JOINT_LOWER_RETURN_RATIO)
    lower_joints = _apply_link2_orient_z_guard_to_joints(lower_joints, link2_j2_ref_rad, label="lower_joints_initial")

    robot_center, center_path = get_robot_center_for_goal(stage)
    attach_center = _vec3(attach_center)
    attach_suction_pos = _vec3(attach_suction_pos)

    # 52_: 반 바퀴 고정 회전이 아니라, 현재 박스 위치 -> BoxAprop slot 목표 위치로 가는 joint_1 회전량을 계산한다.
    # 이렇게 해야 BoxAprop scale/위치가 USD에서 바뀌어도 1번 로봇이 slot 방향으로 회전한다.
    goal_center = np.array(getattr(task, "goal_center", attach_center), dtype=float)
    if robot_center is not None:
        rc = np.array(robot_center[:2], dtype=float)
        start_vec = attach_center[:2] - rc
        goal_vec = goal_center[:2] - rc
        start_angle = float(np.arctan2(start_vec[1], start_vec[0]))
        goal_angle = float(np.arctan2(goal_vec[1], goal_vec[0]))
        delta_angle = goal_angle - start_angle
        # [-pi, pi] 정규화
        delta_angle = float((delta_angle + np.pi) % (2.0 * np.pi) - np.pi)
        computed_swing_delta = _clamp_first_joint_delta(delta_angle)
        if dof >= 1:
            swing_delta[0] = computed_swing_delta
            swing_joints = lift_joints + swing_delta
            swing_joints = _apply_link2_orient_z_guard_to_joints(swing_joints, link2_j2_ref_rad, label="swing_joints_goal")
            lower_joints = swing_joints - lift_delta * float(JOINT_LOWER_RETURN_RATIO)
            lower_joints = _apply_link2_orient_z_guard_to_joints(lower_joints, link2_j2_ref_rad, label="lower_joints_goal")
        mirror_xy = goal_center[:2]
    else:
        mirror_xy = attach_center[:2] + np.array(GOAL_OFFSET_FROM_BOX_CENTER[:2], dtype=float)
        center_path = "fallback_offset"
        computed_swing_delta = float(swing_delta[0]) if dof >= 1 else 0.0

    print(
        f"  [JOINT_SWING_GOAL] center_path={center_path}, "
        f"attach_xy=({attach_center[0]:.3f},{attach_center[1]:.3f}), "
        f"goal_xy=({goal_center[0]:.3f},{goal_center[1]:.3f}), "
        f"j1_delta={computed_swing_delta:.3f} rad, wide_radius_j2={JOINT_LIFT_J2_DELTA_RAD:.3f}, wide_radius_j3={JOINT_LIFT_J3_DELTA_RAD:.3f}"
    )
    if dof >= 2:
        print(
            f"  [LINK2_ORIENT_CALC_108] initial_link2_z={LINK2_ORIENT_Z_INITIAL_DEG:+.1f}deg, "
            f"target_link2_z={LINK2_ORIENT_Z_TARGET_DEG:+.1f}deg, min_link2_z={LINK2_ORIENT_Z_MIN_DEG:+.1f}deg, ref_j2={link2_j2_ref_rad:+.6f}rad, "
            f"start_est={_estimate_link2_orient_z_deg_from_j2(start_joints[1], link2_j2_ref_rad):+.2f}deg, "
            f"lift_est={_estimate_link2_orient_z_deg_from_j2(lift_joints[1], link2_j2_ref_rad):+.2f}deg, "
            f"swing_est={_estimate_link2_orient_z_deg_from_j2(swing_joints[1], link2_j2_ref_rad):+.2f}deg"
        )

    # 103_: 진단 모드에서는 lift를 하지 않는다.
    # FixedJoint 생성만으로 박스가 튀는지 확인해야 하므로, 로봇 관절 목표는 현재 자세 그대로 유지한다.
    if bool(globals().get("PHYSICS_FIXED_JOINT_DIAGNOSTIC_NO_LIFT", False)):
        diag_steps = int(globals().get("PHYSICS_DIAGNOSTIC_HOLD_STEPS", 120))
        phase_sequence = [
            {"name": "joint_diagnostic_hold", "kind": "joint", "target_joints": start_joints, "steps": diag_steps},
        ]
        print(f"  [PHYSICS_JOINT_DIAG_106] phases=['joint_diagnostic_hold']; should not run in 106 because no_lift=False. hold={diag_steps} steps.")
    # 80_: 물리 FixedJoint 테스트에서는 팔레트 방향 swing을 하지 않는다.
    # 박스를 코드로 따라오게 하는지 여부가 아니라, 실제 joint 연결로 lift되는지만 먼저 확인한다.
    elif bool(globals().get("PHYSICS_FIXED_JOINT_LIFT_ONLY_TEST", False)):
        stabilize_steps = int(globals().get("PHYSICS_FIXED_JOINT_STABILIZE_STEPS", 70))
        phase_sequence = [
            {"name": "joint_attach_stabilize", "kind": "joint", "target_joints": start_joints, "steps": stabilize_steps},
            {"name": "joint_lift", "kind": "joint", "target_joints": lift_joints, "steps": int(JOINT_LIFT_STEPS)},
            {"name": "joint_hold", "kind": "joint", "target_joints": lift_joints, "steps": int(VERTICAL_LIFT_HOLD_STEPS)},
        ]
        print(f"  [PHYSICS_LIFT_ONLY_TEST_107] phases=['joint_attach_stabilize','joint_lift','joint_hold']; normally disabled in 107.")
    else:
        phase_sequence = [
            {"name": "joint_attach_stabilize", "kind": "joint", "target_joints": start_joints, "steps": int(PHYSICS_FIXED_JOINT_STABILIZE_STEPS)},
            {"name": "joint_lift", "kind": "joint", "target_joints": lift_joints, "steps": int(JOINT_LIFT_STEPS)},
            {"name": "joint_swing", "kind": "joint", "target_joints": swing_joints, "steps": int(JOINT_SWING_STEPS)},
            # 75_: 여기서 joint_lower/joint_settle을 제거한다.
            # 기존에는 swing 후 j2/j3를 다시 움직이면서 suction/box XY가 크게 튀었다.
            # 팔레타이징 이동은 link_1 z축 회전까지만 하고, 그 위치에서 그대로 release한다. snap/순간이동은 하지 않는다.
        ]

    return {
        "mode": "JOINT_SWING",
        "center_path": center_path,
        "start_joints": start_joints,
        "lift_delta": lift_delta,
        "swing_delta": swing_delta,
        "lift_joints": lift_joints,
        "swing_joints": swing_joints,
        "lower_joints": lower_joints,
        "mirror_xy_estimate": mirror_xy,
        "goal_center": goal_center,
        "computed_j1_delta": computed_swing_delta,
        "attach_center": attach_center,
        "attach_suction": attach_suction_pos,
        "attached_center_offset": _vec3(attached_center_offset),
        "phase_sequence": phase_sequence,
    }

def make_custom_carry_targets(stage, task, attach_center, attach_suction_pos, attached_center_offset):
    """
    24_ 순간이동/관통 방지용 target 생성.
    - 박스 target을 직접 set하지 않고 suction point target만 만든다.
    - 박스는 매 프레임 실제 suction point + attach offset만 따라간다.
    - 로봇/발판 큐브 주변은 waypoint로 돌아간다.
    """
    attach_center = _vec3(attach_center)
    attach_suction_pos = _vec3(attach_suction_pos)
    attached_center_offset = _vec3(attached_center_offset)

    # 60_: 먼저 실제 윗면 흡착 후 Z로 들어올리고, BoxAprop 위의 현재 slot 좌표로 이동한 뒤 살포시 내려놓는다.
    # release 직전까지 박스는 실제 suction 위치 + attach offset을 따라간다.
    if bool(VERTICAL_LIFT_ONLY_TEST):
        lift_suction = attach_suction_pos.copy()
        lift_suction[2] = float(attach_suction_pos[2]) + float(VERTICAL_LIFT_DELTA_Z)

        # 65_: robotAprop_01 같은 고정 장애물을 실제 물리로 밀고 지나가는 방식이 아니므로,
        # 박스 바닥이 장애물 윗면보다 충분히 높게 지나가도록 수직 리프트 높이를 자동 상향한다.
        clear_z, clear_reason = get_robotaprop_clearance_suction_z(stage, float(task.box_height), attached_center_offset)
        if clear_z is not None and float(lift_suction[2]) < float(clear_z):
            old_z = float(lift_suction[2])
            lift_suction[2] = float(clear_z)
            print(
                f"  [ROBOTAPROP_CLEARANCE] lift_suction_z raised {old_z:.4f} -> {lift_suction[2]:.4f}; {clear_reason}"
            )
        elif clear_z is not None:
            print(
                f"  [ROBOTAPROP_CLEARANCE] current lift_suction_z={float(lift_suction[2]):.4f} ok, required={float(clear_z):.4f}; {clear_reason}"
            )
        else:
            print(f"  [ROBOTAPROP_CLEARANCE] no z raise: {clear_reason}")

        # 155_: 컨베이어 위 다른 박스보다 충분히 높게 들어 올린 뒤에만 큰 회전/link_1 이동을 시작한다.
        conv_top_z_155, conv_reason_155 = get_conveyor_max_box_top_z(stage, attached_box_path=getattr(task, "box_path", None))
        if conv_top_z_155 is not None:
            # suction 목표 높이 기준: 다른 박스 최고 윗면 + 안전 여유.
            required_suction_z_155 = float(conv_top_z_155) + float(CONVEYOR_SAFE_LIFT_CLEARANCE_Z_155)
            if float(lift_suction[2]) < required_suction_z_155:
                old_z_155 = float(lift_suction[2])
                lift_suction[2] = required_suction_z_155
                print(
                    f"  [CONVEYOR_SAFE_LIFT_155] lift_suction_z raised {old_z_155:.4f} -> {lift_suction[2]:.4f}; {conv_reason_155}; clearance={CONVEYOR_SAFE_LIFT_CLEARANCE_Z_155:.3f}"
                )
            else:
                print(
                    f"  [CONVEYOR_SAFE_LIFT_155] current lift_suction_z={float(lift_suction[2]):.4f} ok; required={required_suction_z_155:.4f}; {conv_reason_155}"
                )
        else:
            print(f"  [CONVEYOR_SAFE_LIFT_155] no z raise: {conv_reason_155}")

        # 155_: APalt 위 정답지 큐브 center를 최종 place center로 사용한다.
        slot_idx_155 = int(getattr(task, "_stack_slot_index", 0))
        marker_center_155, marker_reason_155 = resolve_slot_marker_center(stage, slot_idx_155, fallback_center=task.goal_center)
        boxaprop_place_enabled = bool(VERTICAL_LIFT_THEN_CUBE_OVER_ENABLED)
        if marker_center_155 is not None:
            place_center = marker_center_155.copy()
            boxaprop_place_enabled = True
            print(
                f"  [SLOT_MARKER_GOAL_155] using marker as final box center: {marker_reason_155}, "
                f"place_center=({place_center[0]:.4f},{place_center[1]:.4f},{place_center[2]:.4f})"
            )
        else:
            place_center = np.array(task.goal_center, dtype=float).copy() if task.goal_center is not None else attach_center.copy()
            # B standalone historically completes the final Cartesian correction even
            # when its repeated slot marker is unavailable. Other profiles preserve
            # the previous computed-goal-only fallback.
            if bool(globals().get("SLOT_MARKER_FALLBACK_GOAL_ENABLE_FINAL_MOVE_179", False)) and task.goal_center is not None:
                boxaprop_place_enabled = True
                print(
                    "  [SLOT_MARKER_GOAL_FALLBACK] computed goal_center enabled for final move: "
                    f"reason={marker_reason_155}, place_center=({place_center[0]:.4f},"
                    f"{place_center[1]:.4f},{place_center[2]:.4f})"
                )
            else:
                print(f"  [SLOT_MARKER_GOAL_155] fallback to computed goal_center; reason={marker_reason_155}")

        move_suction = lift_suction.copy()
        lower_suction = lift_suction.copy()
        if boxaprop_place_enabled:
            # place_center는 최종 박스 중심 좌표다. 박스 중심이 slot marker에 오도록 suction 목표를 역산한다.
            lower_suction = place_center - attached_center_offset
            move_suction = lower_suction.copy()
            move_suction[2] = max(
                float(lift_suction[2]),
                float(lower_suction[2]) + float(SLOT_MARKER_APPROACH_CLEARANCE_Z_155),
            )

        phase_sequence = [
            {"name": "vertical_lift", "kind": "path_lift", "start": attach_suction_pos.copy(), "target": lift_suction.copy(), "steps": int(TOP_LOCK_PATH_LIFT_STEPS)},
        ]

        if CUSTOM_CARRY_MODE == "VERTICAL_JOINT1_REVERSE":
            # 117_: 긴 Cartesian XY 이동 대신, 수직 상승 후 joint_1/link_1 하나만 회전한다.
            # joint_1은 로봇 베이스 기준 yaw라서 상자를 뒤쪽 큐브 방향으로 보내는 주 회전축이다.
            # reverse_vertical_lower의 start/target은 joint_1 회전이 끝난 시점의 실제 suction 위치로 동적 설정된다.
            phase_sequence.append({
                "name": "joint1_rotate_to_back",
                "kind": "joint1_rotate",
                "steps": int(HYBRID_JOINT1_ROTATE_STEPS),
                "mode": str(HYBRID_JOINT1_ROTATE_MODE),
                "sign": float(HYBRID_JOINT1_ROTATE_SIGN),
                "fallback_deg": float(get_stack_joint1_angle(getattr(task, "_stack_slot_index", 0))),
                "stack_slot_index_145": int(getattr(task, "_stack_slot_index", 0)),
                "max_deg": float(HYBRID_JOINT1_ROTATE_MAX_DEG),
            })
            if bool(globals().get("SLOT_MARKER_PALLETIZING_ENABLED_155", False)) and bool(boxaprop_place_enabled):
                # 155_: link_1 회전은 큰 방향 전환까지만 담당한다.
                # 최종 위치는 정답지 큐브 위로 짧게 보정한 뒤 하강한다.
                phase_sequence.append({
                    "name": "slot_marker_move_over_155",
                    "kind": "path_move",
                    "start": lift_suction.copy(),
                    "target": move_suction.copy(),
                    "steps": int(SLOT_MARKER_FINAL_MOVE_STEPS_155),
                })
                phase_sequence.append({
                    "name": "slot_marker_lower_155",
                    "kind": "path_lower",
                    "start": move_suction.copy(),
                    "target": lower_suction.copy(),
                    "steps": int(SLOT_MARKER_FINAL_LOWER_STEPS_155),
                    "fused_yaw_align_184": bool(globals().get("FUSED_SECOND_BOX_YAW_ALIGN_ENABLED_184", False)),
                })
                phase_sequence.append({
                    "name": "slot_marker_settle_155",
                    "kind": "path_settle",
                    "start": lower_suction.copy(),
                    "target": lower_suction.copy(),
                    "steps": int(SLOT_MARKER_FINAL_SETTLE_STEPS_155),
                    "fused_yaw_align_184": bool(globals().get("FUSED_SECOND_BOX_YAW_ALIGN_ENABLED_184", False)),
                })
                _insert_pre_release_yaw_184 = bool(globals().get("PRE_RELEASE_YAW_ALIGN_RMPFLOW_ENABLED_171", True)) and bool(globals().get("PRE_RELEASE_YAW_ALIGN_INSERT_AFTER_SETTLE_171", True))
                if (
                    _insert_pre_release_yaw_184
                    and bool(globals().get("FUSED_SECOND_BOX_YAW_ALIGN_ENABLED_184", False))
                    and bool(globals().get("FUSED_YAW_ALIGN_DISABLE_FINAL_PRE_RELEASE_PHASE_184", True))
                    and int(getattr(task, "_stack_slot_index", 0)) in tuple(int(x) for x in globals().get("FUSED_YAW_ALIGN_SLOT_INDICES_184", (1,)))
                ):
                    _insert_pre_release_yaw_184 = False
                    print(f"  [FUSED_YAW_ALIGN_185] slot_index={int(getattr(task, '_stack_slot_index', 0)) + 1} 상자는 slot_marker_lower/settle 중 yaw를 같이 맞추므로 별도 pre_release_yaw_align_171 phase를 생략합니다.")
                if _insert_pre_release_yaw_184:
                    phase_sequence.append({
                        "name": "pre_release_yaw_align_171",
                        "kind": "pre_release_yaw_align_171",
                        "steps": int(globals().get("PRE_RELEASE_YAW_ALIGN_STEPS_171", 70)),
                    })
                    print("  [PRE_RELEASE_YAW_ALIGN_173] phase inserted after slot_marker_settle_155 and before release - REAL RMPFlow action for full steps")
            else:
                # fallback: 154/147 방식. joint_1 회전 후 현재 위치에서 반대 방향으로 수직 하강.
                phase_sequence.append({
                    "name": "reverse_vertical_lower",
                    "kind": "reverse_vertical_lower",
                    "steps": int(HYBRID_REVERSE_LOWER_STEPS),
                    "delta_z": -float(VERTICAL_LIFT_DELTA_Z) + float(globals().get("DROP_RELEASE_EXTRA_Z_125", 0.0)),
                })
                phase_sequence.append({
                    "name": "hybrid_settle",
                    "kind": "path_settle",
                    "steps": int(HYBRID_SETTLE_STEPS),
                })
        elif boxaprop_place_enabled:
            phase_sequence.append({"name": "boxaprop_move", "kind": "path_move", "start": lift_suction.copy(), "target": move_suction.copy(), "steps": int(CUBE_OVER_MOVE_STEPS)})
            phase_sequence.append({"name": "boxaprop_lower", "kind": "path_lower", "start": move_suction.copy(), "target": lower_suction.copy(), "steps": int(BOXAPROP_LOWER_STEPS)})
            phase_sequence.append({"name": "boxaprop_settle", "kind": "path_settle", "start": lower_suction.copy(), "target": lower_suction.copy(), "steps": int(BOXAPROP_SETTLE_STEPS)})
        else:
            phase_sequence.append({"name": "vertical_hold", "kind": "path_settle", "start": lift_suction.copy(), "target": lift_suction.copy(), "steps": int(VERTICAL_LIFT_HOLD_STEPS)})

        rr_place = robot_relative_vector(stage, place_center)
        print(
            f"  [SIMPLE_3STEP_ROUTE_117] attach_suction=({attach_suction_pos[0]:.3f},{attach_suction_pos[1]:.3f},{attach_suction_pos[2]:.3f}), "
            f"lift_suction=({lift_suction[0]:.3f},{lift_suction[1]:.3f},{lift_suction[2]:.3f}), "
            f"delta_z={VERTICAL_LIFT_DELTA_Z:.3f}, drop_extra_z_125={float(globals().get('DROP_RELEASE_EXTRA_Z_125', 0.0)):.3f}, boxaprop_place_enabled={boxaprop_place_enabled}, "
            f"move_suction=({move_suction[0]:.3f},{move_suction[1]:.3f},{move_suction[2]:.3f}), "
            f"lower_suction=({lower_suction[0]:.3f},{lower_suction[1]:.3f},{lower_suction[2]:.3f}), "
            f"place_center_world=({place_center[0]:.4f}, {place_center[1]:.4f}, {place_center[2]:.4f}), "
            f"place_center_robot_relative=({rr_place[0]:+.4f}, {rr_place[1]:+.4f}, {rr_place[2]:+.4f}), "
            f"follow_actual_suction={not TOP_LOCK_FOLLOW_DESIRED_PATH}, custom_mode={CUSTOM_CARRY_MODE}, "
            f"stack_slot={int(getattr(task, '_stack_slot_index', 0)) + 1}, joint1_deg_145={get_stack_joint1_angle(getattr(task, '_stack_slot_index', 0)):.1f}"
        )
        return {
            "place_center": place_center.copy(),
            "safe_z": float(lift_suction[2]),
            "lift_suction": lift_suction,
            "move_suction": move_suction.copy(),
            "lower_suction": lower_suction.copy(),
            "phase_sequence": phase_sequence,
            "vertical_only": True,
            "boxaprop_place_enabled": boxaprop_place_enabled,
            "boxaprop_release": bool(HYBRID_RELEASE_ON_SETTLE) if CUSTOM_CARRY_MODE == "VERTICAL_JOINT1_REVERSE" else bool(BOXAPROP_RELEASE_ON_SETTLE),
            "cube_over_enabled": boxaprop_place_enabled,
            "cube_over_suction": move_suction.copy(),
            "cube_over_box_center": place_center.copy(),
        }

    place_center = np.array(task.goal_center, dtype=float).copy()
    # 56_: 2x2 적재에서는 task.goal_center의 Z가 BoxAprop 윗면/아래층 박스 높이를 반영한다.
    # 이전 방식처럼 pick 시점의 attach_center.z로 덮어쓰면 slot 높이가 무시되어 가지런히 쌓이지 않는다.
    if not MULTI_ORIBOX_STACKING_ENABLED:
        place_center[2] = float(attach_center[2])

    robot_center, _ = get_robot_center_for_goal(stage)
    # 49_: BoxAprop 위 3칸 적재에서는 목표가 BoxAprop 기준이어야 한다.
    # 기존처럼 로봇/발판 guard 밖으로 밀어내면 slot 위치가 변해서 첫 박스부터 엉뚱한 방향으로 간다.
    if robot_center is not None and not (MULTI_ORIBOX_STACKING_ENABLED and STACK_USE_DIRECT_CARRY_ROUTE):
        place_center[:2] = _push_xy_outside_robot_guard(stage, place_center[:2], center_xy=np.array(robot_center, dtype=float)[:2])

    safe_z = max(
        float(CUSTOM_CARRY_SAFE_SUCTION_Z),
        float(attach_suction_pos[2]) + float(CUSTOM_LIFT_DELTA_Z),
    )

    lift_suction = attach_suction_pos.copy()
    lift_suction[2] = safe_z

    move_suction = place_center - attached_center_offset
    move_suction[2] = safe_z

    lower_suction = place_center - attached_center_offset
    lower_suction[2] = max(float(lower_suction[2]) + float(CUSTOM_LOWER_CLEARANCE_Z), 0.10)

    if MULTI_ORIBOX_STACKING_ENABLED and STACK_USE_DIRECT_CARRY_ROUTE:
        # 49_: 적재는 단순하고 안정적인 경로로 처리한다.
        # lift -> BoxAprop slot 위로 수평 이동 -> slot 위로 하강
        # 59_: 각 phase의 목표를 한 번에 던지지 않고, 시작점->목표점을 고정 보간한다.
        # 중간에 target_suction을 재계산하지 않으므로 56_처럼 목표가 튀지 않는다.
        phase_sequence = [
            {"name": "lift", "kind": "path_lift", "start": attach_suction_pos.copy(), "target": lift_suction.copy(), "steps": int(TOP_LOCK_PATH_LIFT_STEPS)},
            {"name": "move_stack", "kind": "path_move", "start": lift_suction.copy(), "target": move_suction.copy(), "steps": int(TOP_LOCK_PATH_MOVE_STEPS)},
            {"name": "lower", "kind": "path_lower", "start": move_suction.copy(), "target": lower_suction.copy(), "steps": int(TOP_LOCK_PATH_LOWER_STEPS)},
            {"name": "settle", "kind": "path_settle", "start": lower_suction.copy(), "target": lower_suction.copy(), "steps": int(TOP_LOCK_PATH_SETTLE_STEPS)},
        ]
        print(
            f"  [STACK_DIRECT_ROUTE] start=({lift_suction[0]:.3f},{lift_suction[1]:.3f},{lift_suction[2]:.3f}), "
            f"move=({move_suction[0]:.3f},{move_suction[1]:.3f},{move_suction[2]:.3f}), "
            f"lower=({lower_suction[0]:.3f},{lower_suction[1]:.3f},{lower_suction[2]:.3f})"
        )
    else:
        phase_sequence = _make_avoidance_suction_waypoints(stage, lift_suction, move_suction)
        phase_sequence.append({"name": "lower", "kind": "lower", "target": lower_suction.copy()})

    return {
        "place_center": place_center,
        "safe_z": safe_z,
        "lift_suction": lift_suction,
        "move_suction": move_suction,
        "lower_suction": lower_suction,
        "phase_sequence": phase_sequence,
    }

def follow_box_to_suction(stage, task, suction_pos, attached_center_offset, min_center_z=None):
    """
    박스를 목표 좌표로 순간이동시키지 않는다.
    현재 suction point 위치 + attach 순간 offset만 따라가게 한다.
    """
    desired_box_center = _vec3(suction_pos) + _vec3(attached_center_offset)
    if min_center_z is not None:
        desired_box_center[2] = max(float(desired_box_center[2]), float(min_center_z))
    ok = move_box_center_to(
        stage,
        task.box_move_path,
        desired_center=desired_box_center,
        root_to_center_offset=task.box_root_to_center_offset,
    )
    zero_subtree_velocity(stage, task.box_move_path)
    return ok, desired_box_center

def snap_box_to_stack_slot_if_enabled(stage, task):
    """release 직전 slot 중심 강제 정렬. 50_에서는 기본 False라 순간이동하지 않는다."""
    if not (MULTI_ORIBOX_STACKING_ENABLED and STACK_SNAP_BOX_TO_SLOT_ON_RELEASE):
        return False
    if task is None or task.goal_center is None:
        return False
    bbox = get_world_bbox_info(stage, task.box_path)
    if bbox is None:
        return False
    current_center = np.array(bbox["center"], dtype=float)
    goal_center = np.array(task.goal_center, dtype=float)
    xy_error = float(np.linalg.norm(current_center[:2] - goal_center[:2]))
    if xy_error > float(STACK_SNAP_MAX_XY_ERROR):
        print(f"  [STACK_SNAP_SKIP] xy_error={xy_error:.3f} > {STACK_SNAP_MAX_XY_ERROR:.3f}, current={current_center}, goal={goal_center}")
        return False
    ok = move_box_center_to(
        stage,
        task.box_move_path,
        desired_center=goal_center,
        root_to_center_offset=task.box_root_to_center_offset,
    )
    zero_subtree_velocity(stage, task.box_move_path)
    print(
        f"  [STACK_SNAP] ok={ok}, xy_error_before={xy_error:.3f}, "
        f"goal_center=({goal_center[0]:.3f},{goal_center[1]:.3f},{goal_center[2]:.4f})"
    )
    return ok

def initialize_robot_for_conveyor(robot, world):
    global ROBOT_INITIAL_JOINTS_156
    robot.initialize()
    if RESET_ROBOT_TO_ZERO:
        robot.set_joint_positions(np.zeros(robot.num_dof))
    try:
        ROBOT_INITIAL_JOINTS_156 = np.array(robot.get_joint_positions(), dtype=float).copy()
        if bool(globals().get("ABSOLUTE_JOINT1_LOG_156", False)):
            j1 = float(ROBOT_INITIAL_JOINTS_156[0]) if len(ROBOT_INITIAL_JOINTS_156) > 0 else 0.0
            print(f"[ABS_JOINT1_INIT_156] captured initial joints. initial_j1={j1:+.6f} rad/{math.degrees(j1):+.1f}deg")
    except Exception as exc:
        ROBOT_INITIAL_JOINTS_156 = None
        print(f"[ABS_JOINT1_INIT_156][WARN] failed to capture initial joints: {exc}")

def _set_root_world_yaw_preserve_translation_159(stage, prim_path, yaw_rad):
    """prim root의 world translation은 유지하고, world Z yaw만 지정한다. /World 하위 root용."""
    prim = stage.GetPrimAtPath(str(prim_path))
    if not prim or not prim.IsValid():
        print(f"[SLOT_YAW_ALIGN_159][WARN] invalid prim: {prim_path}")
        return False
    try:
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        parent = prim.GetParent()
        current_world = cache.GetLocalToWorldTransform(prim)
        world_t = current_world.ExtractTranslation()

        world_mat = Gf.Matrix4d(1.0)
        world_mat.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), math.degrees(float(yaw_rad))))
        world_mat.SetTranslateOnly(world_t)

        if parent and parent.IsValid():
            parent_world = cache.GetLocalToWorldTransform(parent)
            local_mat = world_mat * parent_world.GetInverse()
        else:
            local_mat = world_mat

        op = _get_or_create_scripted_transform_op(prim)
        op.Set(local_mat)
        return True
    except Exception as exc:
        print(f"[SLOT_YAW_ALIGN_159][WARN] set yaw failed: {type(exc).__name__}: {exc}")
        return False

def align_box_yaw_to_slot_marker(stage, box_root_path, slot_index):
    """release 직전 박스 root yaw를 APalt_slot marker yaw와 동일하게 맞춘다."""
    if not bool(globals().get("SLOT_MARKER_YAW_ALIGN_ENABLED_159", False)):
        return False
    marker_path = get_slot_marker_path_for_slot(slot_index)
    if not marker_path:
        print("[SLOT_YAW_ALIGN_159][WARN] no marker path")
        return False
    target_yaw = _get_world_yaw_z_159(stage, marker_path)
    current_yaw = _get_world_yaw_z_159(stage, box_root_path)
    if target_yaw is None or current_yaw is None:
        print(f"[SLOT_YAW_ALIGN_159][WARN] yaw read failed: box={box_root_path}, marker={marker_path}")
        return False
    ok = _set_root_world_yaw_preserve_translation_159(stage, box_root_path, target_yaw)
    if bool(globals().get("SLOT_MARKER_YAW_ALIGN_LOG_159", True)):
        delta = float((target_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi)
        print(
            f"[SLOT_YAW_ALIGN_159] box={box_root_path}, marker={marker_path}, "
            f"current_yaw={math.degrees(current_yaw):+.1f}deg, target_yaw={math.degrees(target_yaw):+.1f}deg, "
            f"delta={math.degrees(delta):+.1f}deg, ok={ok}"
        )
    return bool(ok)

def _quat_wxyz_normalize_171(q):
    q = np.array(q, dtype=float).reshape(-1)
    if q.size < 4:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    q = q[:4].astype(float)
    n = float(np.linalg.norm(q))
    if n < 1.0e-9:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n

def _quat_wxyz_mul_171(q1, q2):
    """Isaac 관례인 [w,x,y,z] quaternion multiply: q = q1 * q2."""
    w1, x1, y1, z1 = _quat_wxyz_normalize_171(q1)
    w2, x2, y2, z2 = _quat_wxyz_normalize_171(q2)
    return _quat_wxyz_normalize_171(np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ], dtype=float))

def _quat_wxyz_from_world_z_yaw_deg_171(yaw_deg):
    half = math.radians(float(yaw_deg)) * 0.5
    return np.array([math.cos(half), 0.0, 0.0, math.sin(half)], dtype=float)

def _make_pre_release_yaw_target_quat_171(robot, yaw_cmd_deg):
    """
    현재 EE orientation에 world-Z yaw 보정량을 pre-multiply한다.
    상자 transform은 건드리지 않고, RMPFlow target orientation으로만 사용한다.
    """
    _ee_pos, ee_quat = get_current_ee_pose(robot)
    q_now = _quat_wxyz_normalize_171(ee_quat)
    q_yaw = _quat_wxyz_from_world_z_yaw_deg_171(float(yaw_cmd_deg))
    return _quat_wxyz_mul_171(q_yaw, q_now)

def prepare_pre_release_yaw_align_phase(stage, robot, task, phase_info, suction_now):
    """
    slot_marker_settle_155 이후 release 직전에 1회만 호출된다.
    현재 box vs APalt_slot 진단값을 보고 yaw만 RMPFlow target orientation으로 보정할지 결정한다.
    """
    if phase_info.get("prepared_171", False):
        return phase_info
    phase_info["prepared_171"] = True
    phase_info["start"] = np.array(suction_now, dtype=float).copy()
    phase_info["target"] = np.array(suction_now, dtype=float).copy()
    phase_info["steps"] = int(globals().get("PRE_RELEASE_YAW_ALIGN_STEPS_171", 70))

    slot_idx = int(getattr(task, "_stack_slot_index", 0))
    diag = diagnose_pallet_release_pose(stage, task.box_move_path, slot_idx, label="before_pre_release_yaw_align_173")
    phase_info["diag_before_171"] = diag
    if not isinstance(diag, dict):
        phase_info["skip_171"] = True
        phase_info["skip_reason_171"] = "diag_failed"
        phase_info["steps"] = 1
        print("[PRE_RELEASE_YAW_ALIGN_171] skip: diag_failed")
        return phase_info

    center_ok = bool(diag.get("ok_center", False))
    level_ok = bool(diag.get("ok_level", False))
    axis_ok = bool(diag.get("ok_axis", False))
    yaw_cmd = float(diag.get("best_axis_signed_err", 0.0))
    yaw_abs = abs(yaw_cmd)
    min_deg = float(globals().get("PRE_RELEASE_YAW_ALIGN_MIN_DEG_171", 2.0))
    max_deg = float(globals().get("PRE_RELEASE_YAW_ALIGN_MAX_DEG_171", 25.0))

    if bool(globals().get("PRE_RELEASE_YAW_ALIGN_ONLY_IF_CENTER_LEVEL_OK_171", True)) and (not center_ok or not level_ok):
        phase_info["skip_171"] = True
        phase_info["skip_reason_171"] = f"center_or_level_not_ok:center={center_ok},level={level_ok}"
        phase_info["steps"] = 1
    elif axis_ok or yaw_abs < min_deg:
        phase_info["skip_171"] = True
        phase_info["skip_reason_171"] = f"axis_already_ok_or_small:yaw={yaw_cmd:+.2f}"
        phase_info["steps"] = 1
    elif yaw_abs > max_deg:
        phase_info["skip_171"] = True
        phase_info["skip_reason_171"] = f"yaw_too_large:{yaw_cmd:+.2f}>{max_deg:.1f}"
        phase_info["steps"] = 1
    else:
        # yaw_cmd는 slot yaw - box yaw 계열이므로, EE 목표에도 같은 world-Z yaw 보정량을 준다.
        phase_info["skip_171"] = False
        phase_info["yaw_cmd_deg_171"] = yaw_cmd
        phase_info["fixed_orientation_171"] = _make_pre_release_yaw_target_quat_171(robot, yaw_cmd)
        phase_info["hold_suction_171"] = np.array(suction_now, dtype=float).copy()
        phase_info["target"] = np.array(suction_now, dtype=float).copy()
        print(
            f"[PRE_RELEASE_YAW_ALIGN_173] start. slot={slot_idx}, yaw_cmd={yaw_cmd:+.2f}deg, "
            f"best_axis={diag.get('best_axis')}:{float(diag.get('best_axis_err', 999.0)):.2f}deg, "
            f"center_xy_err={float(diag.get('center_xy_err', 999.0)):.4f}, "
            f"level={float(diag.get('box_to_slot_z_deg', 999.0)):.2f}deg, "
            f"steps={phase_info['steps']}, mode=REAL_RMPFLOW_TARGET_ORIENTATION_172_METHOD, hold_suction=True, full_steps=True, no_box_transform=True, no_joint6_trim=True"
        )

    if bool(phase_info.get("skip_171", False)):
        print(f"[PRE_RELEASE_YAW_ALIGN_171] skip_reason={phase_info.get('skip_reason_171')}")
    return phase_info

def _is_fused_yaw_slot_184(task):
    try:
        slot_idx = int(getattr(task, "_stack_slot_index", 0))
        allowed = tuple(int(x) for x in globals().get("FUSED_YAW_ALIGN_SLOT_INDICES_184", (1,)))
        return bool(slot_idx in allowed)
    except Exception:
        return False

def prepare_fused_yaw_align_phase(stage, robot, task, phase_info, suction_now):
    """
    184_: release 직전 별도 pre_release_yaw_align_171 phase를 기다리지 않고,
    slot_marker_lower_155/settle phase의 RMPFlow target orientation에 yaw 보정을 섞는다.
    위치 목표(start/target)는 절대 바꾸지 않는다. 즉 내려가면서 yaw만 같이 맞춘다.
    """
    try:
        if not bool(globals().get("FUSED_SECOND_BOX_YAW_ALIGN_ENABLED_184", False)):
            return phase_info
        if not isinstance(phase_info, dict):
            return phase_info
        phase_name = str(phase_info.get("name", ""))
        if phase_name not in tuple(globals().get("FUSED_YAW_ALIGN_PHASE_NAMES_184", ("slot_marker_lower_155", "slot_marker_settle_155"))):
            return phase_info
        if not _is_fused_yaw_slot_184(task):
            return phase_info

        # settle에서는 lower에서 계산한 orientation을 재사용한다.
        if phase_name != str(globals().get("FUSED_YAW_ALIGN_START_PHASE_184", "slot_marker_lower_155")):
            if bool(globals().get("FUSED_YAW_ALIGN_REUSE_IN_SETTLE_184", True)):
                q_reuse = getattr(task, "_fused_yaw_orientation_184", None)
                if q_reuse is not None:
                    phase_info["fixed_orientation_171"] = np.array(q_reuse, dtype=float).copy()
                    phase_info["fused_yaw_reused_184"] = True
                    if bool(globals().get("FUSED_YAW_ALIGN_LOG_184", True)) and not phase_info.get("fused_yaw_reuse_logged_184", False):
                        phase_info["fused_yaw_reuse_logged_184"] = True
                        print(f"[FUSED_YAW_ALIGN_184] reuse orientation during {phase_name}; separate pre_release_yaw_align skipped")
            return phase_info

        if phase_info.get("fused_yaw_prepared_184", False):
            return phase_info
        phase_info["fused_yaw_prepared_184"] = True

        slot_idx = int(getattr(task, "_stack_slot_index", 0))
        diag = diagnose_pallet_release_pose(stage, task.box_move_path, slot_idx, label=f"before_fused_yaw_align_184:{phase_name}")
        phase_info["fused_yaw_diag_184"] = diag
        if not isinstance(diag, dict):
            phase_info["fused_yaw_skip_184"] = "diag_failed"
            if bool(globals().get("FUSED_YAW_ALIGN_LOG_184", True)):
                print(f"[FUSED_YAW_ALIGN_184] skip: diag_failed during {phase_name}")
            return phase_info

        center_xy = float(diag.get("center_xy_err", 999.0))
        level = float(diag.get("box_to_slot_z_deg", 999.0))
        axis_ok = bool(diag.get("ok_axis", False))
        yaw_cmd = float(diag.get("best_axis_signed_err", 0.0))
        yaw_abs = abs(yaw_cmd)
        min_deg = float(globals().get("FUSED_YAW_ALIGN_MIN_DEG_184", 2.0))
        max_deg = float(globals().get("FUSED_YAW_ALIGN_MAX_DEG_184", 25.0))
        center_tol = float(globals().get("FUSED_YAW_ALIGN_CENTER_TOL_M_184", 0.080))
        level_tol = float(globals().get("FUSED_YAW_ALIGN_LEVEL_TOL_DEG_184", 5.0))

        if axis_ok or yaw_abs < min_deg:
            phase_info["fused_yaw_skip_184"] = f"axis_already_ok_or_small:yaw={yaw_cmd:+.2f}"
        elif yaw_abs > max_deg:
            phase_info["fused_yaw_skip_184"] = f"yaw_too_large:{yaw_cmd:+.2f}>{max_deg:.1f}"
        elif center_xy > center_tol:
            phase_info["fused_yaw_skip_184"] = f"center_too_far:{center_xy:.4f}>{center_tol:.4f}"
        elif level > level_tol:
            phase_info["fused_yaw_skip_184"] = f"level_not_ok:{level:.2f}>{level_tol:.2f}"
        else:
            q = _make_pre_release_yaw_target_quat_171(robot, yaw_cmd)
            phase_info["fixed_orientation_171"] = q.copy()
            phase_info["fused_yaw_cmd_deg_184"] = yaw_cmd
            phase_info["fused_yaw_active_184"] = True
            setattr(task, "_fused_yaw_orientation_184", q.copy())
            setattr(task, "_fused_yaw_cmd_deg_184", yaw_cmd)
            if bool(globals().get("FUSED_YAW_ALIGN_LOG_184", True)):
                print(
                    f"[FUSED_YAW_ALIGN_184] active during {phase_name}. slot={slot_idx}, "
                    f"yaw_cmd={yaw_cmd:+.2f}deg, center_xy_err={center_xy:.4f}, level={level:.2f}deg, "
                    f"separate_pre_release_phase={not bool(globals().get('FUSED_YAW_ALIGN_DISABLE_FINAL_PRE_RELEASE_PHASE_184', True))}, "
                    f"mode=LOWER_AND_SETTLE_RMPFLOW_TARGET_ORIENTATION"
                )
            return phase_info

        if bool(globals().get("FUSED_YAW_ALIGN_LOG_184", True)):
            print(f"[FUSED_YAW_ALIGN_184] skip during {phase_name}: {phase_info.get('fused_yaw_skip_184')}")
        return phase_info
    except Exception as exc:
        try:
            print(f"[FUSED_YAW_ALIGN_184][WARN] prepare failed: {type(exc).__name__}: {exc}")
        except Exception:
            pass
        return phase_info

# Compatibility aliases retained for external scripts during the first refactor pass.
get_stack_joint1_deg_145 = get_stack_joint1_angle
force_show_cube_prims_109 = force_show_cube_prims
force_show_environment_prims_146 = force_show_environment_prims
ensure_oribox_a_exists = ensure_target_box_exists
discover_oriboxa_stack_candidates = discover_stack_candidates
select_oriboxa_candidate_in_front_zone = select_stack_candidate_in_pick_zone
resolve_slot_marker_center_155 = resolve_slot_marker_center
get_conveyor_max_box_top_z_155 = get_conveyor_max_box_top_z
get_slot_marker_path_for_slot_159 = get_slot_marker_path_for_slot
align_box_yaw_to_slot_marker_159 = align_box_yaw_to_slot_marker
check_box_yaw_against_slot_marker_160 = check_box_yaw_against_slot_marker
yaw_diagnostic_against_slot_163 = yaw_diagnostic_against_slot
axis_yaw_diagnostic_164 = axis_yaw_diagnostic
apalt_release_pose_diagnose_only_170 = diagnose_pallet_release_pose
_prepare_pre_release_yaw_align_phase_171 = prepare_pre_release_yaw_align_phase
_prepare_fused_yaw_align_phase_184 = prepare_fused_yaw_align_phase
_diag_stop_after_release_count_reached_164 = should_stop_after_release_count
scan_stage_joints_128 = scan_stage_joints
compact_box_physics_state_128 = compact_box_physics_state
force_box_dynamic_after_release_128 = force_box_dynamic_after_release
compact_drop_observe_128 = compact_drop_observe


__all__ = [
    '_set_root_world_yaw_preserve_translation_159',
    'align_box_yaw_to_slot_marker',
    '_quat_wxyz_normalize_171',
    '_quat_wxyz_mul_171',
    '_quat_wxyz_from_world_z_yaw_deg_171',
    '_make_pre_release_yaw_target_quat_171',
    'prepare_pre_release_yaw_align_phase',
    '_is_fused_yaw_slot_184',
    'prepare_fused_yaw_align_phase',
    'get_stack_joint1_angle',
    'repair_robotaprop_and_oribox_colliders',
    'BoxStopDetector',
    'hold_robot_current_pose',
    'is_box_ready_for_pick_zone',
    'create_pick_zone_visual',
    'is_bbox_center_inside_pick_zone',
    'PickZoneDetector',
    'discover_stack_candidates',
    'select_stack_candidate_in_pick_zone',
    'compute_stack_slot_step_from_bbox',
    'get_stack_slot_col_layer',
    'compute_stack_goal_center',
    'resolve_slot_marker_center',
    'get_conveyor_max_box_top_z',
    'lower_stack_support_cube_for_next_layer',
    'get_robot_center_for_goal',
    'compute_mirror_goal_center',
    'set_task_pick_from_current_box',
    'move_box_center_to',
    'probe_move_path_affects_box_bbox',
    'should_attach_oribox',
    'M0609ConveyorBoxTask',
    'make_ee_target_for_desired_suction',
    'apply_cartesian_suction_target',
    '_norm2',
    '_push_xy_outside_robot_guard',
    '_make_avoidance_suction_waypoints',
    '_fit_joint_delta',
    '_clamp_first_joint_delta',
    '_estimate_link2_orient_z_deg_from_j2',
    '_link2_guard_j2_boundary_rad',
    '_apply_link2_orient_z_guard_to_joints',
    'make_joint_swing_carry_targets',
    'make_custom_carry_targets',
    'follow_box_to_suction',
    'snap_box_to_stack_slot_if_enabled',
    'initialize_robot_for_conveyor'
]
