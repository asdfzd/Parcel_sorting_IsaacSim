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

_LAST_SUCTION_GRID_INFO = {"attach_center": None, "hits": [], "summary": "not_evaluated"}


def set_subtree_collision_enabled(stage, root_path: str, enabled: bool) -> int:
    """Compatibility utility for the optional virtual-pallet cargo path."""
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return 0
    changed = 0
    for prim in Usd.PrimRange(root):
        try:
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            collision = UsdPhysics.CollisionAPI(prim)
            attr = collision.GetCollisionEnabledAttr()
            if not attr:
                attr = collision.CreateCollisionEnabledAttr(bool(enabled))
            attr.Set(bool(enabled))
            changed += 1
        except Exception:
            pass
    return changed


def _resolve_first_existing_path(candidates, label="file"):
    for p in candidates:
        pp = Path(p)
        if pp.exists():
            return str(pp)
    return str(Path(candidates[0]))

def find_prim_path_by_name(root_path: str, name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None

def resolve_stack_support_path(stage):
    """
    69_ FIX:
    새 Conveyor_lift 작업환경은 기존 BoxAprop 대신 /World/Cube 같은 받침 prim을 쓸 수 있다.
    우선 STACK_SUPPORT_CUBE_PATH를 확인하고, 없으면 후보 이름(Cube/BoxAprop/robotAprop_01)을 Stage 전체에서 찾는다.
    """
    preferred = str(STACK_SUPPORT_CUBE_PATH)
    prim = stage.GetPrimAtPath(preferred)
    if prim and prim.IsValid():
        return preferred

    world_prim = stage.GetPrimAtPath("/World")
    names = tuple(str(x) for x in globals().get("STACK_SUPPORT_NAME_CANDIDATES", ("Cube", "BoxAprop")))
    if world_prim and world_prim.IsValid():
        for p in Usd.PrimRange(world_prim):
            try:
                if p.GetName() in names:
                    return str(p.GetPath())
            except Exception:
                pass

    # 마지막 fallback: 기존 경로를 반환해서 이후 bbox None 경고가 제대로 뜨게 한다.
    return preferred

def _safe_normalize_vec(v, fallback):
    v = np.array(v, dtype=float)
    n = float(np.linalg.norm(v))
    if n < 1e-8:
        return np.array(fallback, dtype=float)
    return v / n

def get_robot_frame_axes_for_stack(stage):
    """
    BoxAprop 슬롯을 '로봇 기준'으로 계산하기 위한 좌표축.

    반환:
    - origin: active robot root의 world translation
    - right : robot local +X 방향
    - forward: robot local +Y 방향
    - up    : robot local +Z 방향

    로봇 transform을 못 읽으면 world 축을 fallback으로 사용한다.
    """
    origin = get_world_translation(stage, ACTIVE_ROBOT_ROOT_PATH)
    if origin is None:
        origin = get_world_translation(stage, ROBOT_PRIM_PATH)
    if origin is None:
        origin = np.array([0.0, 0.0, 0.0], dtype=float)

    mat = get_world_matrix(stage, ACTIVE_ROBOT_ROOT_PATH)
    if mat is None:
        mat = get_world_matrix(stage, ROBOT_PRIM_PATH)

    right = np.array([1.0, 0.0, 0.0], dtype=float)
    forward = np.array([0.0, 1.0, 0.0], dtype=float)
    up = np.array([0.0, 0.0, 1.0], dtype=float)

    if mat is not None:
        try:
            rx = mat.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
            ry = mat.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0))
            rz = mat.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
            right = np.array([float(rx[0]), float(rx[1]), float(rx[2])], dtype=float)
            forward = np.array([float(ry[0]), float(ry[1]), float(ry[2])], dtype=float)
            up = np.array([float(rz[0]), float(rz[1]), float(rz[2])], dtype=float)
        except Exception:
            # Gf.Matrix 환경 차이 fallback
            try:
                right = np.array([float(mat[0][0]), float(mat[0][1]), float(mat[0][2])], dtype=float)
                forward = np.array([float(mat[1][0]), float(mat[1][1]), float(mat[1][2])], dtype=float)
                up = np.array([float(mat[2][0]), float(mat[2][1]), float(mat[2][2])], dtype=float)
            except Exception:
                pass

    right = _safe_normalize_vec(right, [1.0, 0.0, 0.0])
    forward = _safe_normalize_vec(forward, [0.0, 1.0, 0.0])
    up = _safe_normalize_vec(up, [0.0, 0.0, 1.0])
    return np.array(origin, dtype=float), right, forward, up

def robot_relative_vector(stage, world_pos):
    """
    world 좌표를 active robot 기준 [right, forward, up] 성분으로 변환해 로그에 찍기 위한 함수.
    사용자가 Translate 좌표를 다시 줄 때, 이 값을 보고 어느 방향으로 보정할지 판단한다.
    """
    origin, right, forward, up = get_robot_frame_axes_for_stack(stage)
    v = np.array(world_pos, dtype=float) - np.array(origin, dtype=float)
    return np.array([
        float(np.dot(v, right)),
        float(np.dot(v, forward)),
        float(np.dot(v, up)),
    ], dtype=float)

def get_task_completed_root_path(task):
    """
    66_: 실제 이동은 OriBox root를 직접 움직이며,
    완료/재집기 방지 기준은 /World/OriBoxA_02 같은 root path로 유지한다.
    """
    root = getattr(task, "active_box_root_path", None)
    if root:
        return str(root)
    move = getattr(task, "box_move_path", None)
    return str(move) if move else ""

def find_robotaprop_paths(stage):
    """Stage 안에서 robotAprop_01 계열 prim을 찾는다."""
    found = []

    for path in ROBOTAPROP_EXACT_PATH_CANDIDATES:
        try:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                found.append(str(prim.GetPath()))
        except Exception:
            pass

    world = stage.GetPrimAtPath("/World")
    if world and world.IsValid():
        prefixes = tuple(str(x).lower() for x in ROBOTAPROP_NAME_PREFIXES)
        for prim in Usd.PrimRange(world):
            try:
                name_l = prim.GetName().lower()
                if any(name_l.startswith(p) for p in prefixes):
                    found.append(str(prim.GetPath()))
            except Exception:
                pass

    # 하위 prim이 같이 잡히면 상위 경로만 남긴다.
    unique = []
    for p in sorted(set(found), key=len):
        if not any(p.startswith(q + "/") for q in unique):
            unique.append(p)
    return unique

def _prim_is_geometry_like(prim):
    try:
        if prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube) or prim.IsA(UsdGeom.Cylinder) or prim.IsA(UsdGeom.Capsule) or prim.IsA(UsdGeom.Sphere):
            return True
    except Exception:
        pass
    return False

def enable_collision_on_subtree(stage, root_path, static=True, label="COLLIDER"):
    """
    root 하위 geometry에 CollisionAPI를 켠다.
    static=True이면 RigidBodyAPI/MassAPI를 제거해서 고정 장애물 collider처럼 둔다.
    """
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        return {"root": root_path, "valid": False, "collision": 0, "rigid_removed": 0, "mesh_approx": 0}

    collision_count = 0
    rigid_removed = 0
    mesh_approx = 0

    for prim in Usd.PrimRange(root):
        try:
            is_geom = _prim_is_geometry_like(prim)
            has_col = prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAttribute("physics:collisionEnabled")
            if is_geom or has_col:
                col_api = UsdPhysics.CollisionAPI.Apply(prim)
                attr = prim.GetAttribute("physics:collisionEnabled")
                if not attr:
                    attr = col_api.CreateCollisionEnabledAttr(True)
                attr.Set(True)
                collision_count += 1
        except Exception:
            pass

        try:
            if _prim_is_geometry_like(prim):
                mesh_col = UsdPhysics.MeshCollisionAPI.Apply(prim)
                approx_attr = prim.GetAttribute("physics:approximation")
                if not approx_attr:
                    approx_attr = mesh_col.CreateApproximationAttr()
                # Box/convexHull가 지원되는 환경이 다르므로 실패하면 그냥 CollisionAPI만 유지한다.
                try:
                    approx_attr.Set("convexHull")
                    mesh_approx += 1
                except Exception:
                    pass
        except Exception:
            pass

        if static:
            try:
                if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                    prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                    rigid_removed += 1
            except Exception:
                pass
            try:
                if prim.HasAPI(UsdPhysics.MassAPI):
                    prim.RemoveAPI(UsdPhysics.MassAPI)
            except Exception:
                pass
            try:
                rb_attr = prim.GetAttribute("physics:rigidBodyEnabled")
                if rb_attr:
                    rb_attr.Set(False)
            except Exception:
                pass
            try:
                kin_attr = prim.GetAttribute("physics:kinematicEnabled")
                if kin_attr:
                    kin_attr.Set(False)
            except Exception:
                pass

    return {"root": root_path, "valid": True, "collision": collision_count, "rigid_removed": rigid_removed, "mesh_approx": mesh_approx}

def get_robotaprop_clearance_suction_z(stage, box_height, attached_center_offset):
    """
    direct Xform/top-lock 운반은 실제 충돌로 멈추지 않으므로,
    robotAprop_01 위를 지나갈 때 박스 바닥이 robotAprop_01 top보다 위에 있도록 suction z를 올린다.
    """
    if not ROBOTAPROP_COLLISION_GUARD_ENABLED:
        return None, "disabled"

    paths = find_robotaprop_paths(stage)
    if not paths:
        return None, "robotAprop_not_found"

    max_top_z = None
    used = []
    for p in paths:
        info = get_world_bbox_info(stage, p)
        if info is None:
            continue
        top_z = float(info["max"][2])
        used.append((p, top_z))
        if max_top_z is None or top_z > max_top_z:
            max_top_z = top_z

    if max_top_z is None:
        return None, f"robotAprop_bbox_none paths={paths}"

    # box_center = suction + attached_center_offset.
    # box_bottom = box_center_z - box_height/2.
    # box_bottom >= robotAprop_top + margin 이 되도록 suction_z를 계산한다.
    offset_z = float(np.array(attached_center_offset, dtype=float)[2])
    required = (
        float(max_top_z)
        + float(box_height) * 0.5
        + float(ROBOTAPROP_CLEARANCE_Z_MARGIN)
        - offset_z
        + float(ROBOTAPROP_CLEARANCE_EXTRA_SUCTION_Z)
    )
    return float(required), f"max_top_z={max_top_z:.4f}, box_h={box_height:.4f}, offset_z={offset_z:.4f}, used={used}"

def initialize_robot(robot, world):
    robot.initialize()
    # RG2를 삭제했기 때문에 robot.gripper는 사용하지 않는다.
    robot.set_joint_positions(np.zeros(robot.num_dof))

def clear_prim_if_exists(stage, prim_path: str):
    if stage.GetPrimAtPath(prim_path).IsValid():
        stage.RemovePrim(prim_path)

def remove_goal_marker(stage, verbose=True):
    """
    /World/goal_marker는 목표 위치 확인용 visual 큐브일 뿐이라 시연에서는 제거한다.
    Conveyor.usd reference 안에 이미 들어있어 RemovePrim만으로 남는 경우를 대비해
    invisible + active=False까지 같이 처리한다.
    """
    prim = stage.GetPrimAtPath(GOAL_MARKER_PATH)
    if not prim.IsValid():
        return False

    try:
        stage.RemovePrim(GOAL_MARKER_PATH)
    except Exception:
        pass

    # reference로 compose된 prim은 RemovePrim 뒤에도 보일 수 있으므로 fallback 처리
    prim = stage.GetPrimAtPath(GOAL_MARKER_PATH)
    if prim.IsValid():
        try:
            UsdGeom.Imageable(prim).MakeInvisible()
        except Exception:
            pass
        try:
            prim.SetActive(False)
        except Exception:
            pass

    if verbose:
        still_valid = stage.GetPrimAtPath(GOAL_MARKER_PATH).IsValid()
        print(f"  [OK] goal_marker 제거/비활성화: {GOAL_MARKER_PATH}, valid_after={still_valid}")
    return True

def disable_legacy_camera_graphs(stage, verbose=True):
    """
    106_: 카메라 복구 반영.
    예전 코드에서는 오류 회피를 위해 /World/rsd455까지 SetActive(False) 했지만,
    지금은 rsd455를 사용해야 하므로 /World/rsd455는 끄지 않는다.

    - robot 하위 오래된 camera_graph만 필요 시 비활성화
    - /World/rsd455는 active=True로 유지
    """
    targets = []
    if bool(globals().get("DISABLE_LEGACY_ROBOT_CAMERA_GRAPHS", True)):
        targets.extend([
            "/World/m0609_A/Graph/camera_graph",
            "/World/m0609_B/Graph/camera_graph",
        ])
    if bool(globals().get("DISABLE_WORLD_RSD455_PRIM", False)):
        targets.append(str(globals().get("RSD455_ROOT_PATH", "/World/rsd455")))

    changed = []
    for path in targets:
        try:
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                prim.SetActive(False)
                changed.append(path)
        except Exception:
            pass

    if bool(globals().get("KEEP_RSD455_CAMERA_ACTIVE", True)):
        try:
            rsd_path = str(globals().get("RSD455_ROOT_PATH", "/World/rsd455"))
            rsd = stage.GetPrimAtPath(rsd_path)
            if rsd and rsd.IsValid():
                rsd.SetActive(True)
                try:
                    UsdGeom.Imageable(rsd).MakeVisible()
                except Exception:
                    pass
                if verbose:
                    print(f"  [CAMERA_RESTORE_106] keep active: {rsd_path}, type={rsd.GetTypeName()}, active={rsd.IsActive()}")
                # 하위 Camera prim이 실제로 있는지 확인한다.
                camera_children = []
                for p in Usd.PrimRange(rsd):
                    try:
                        if p.GetTypeName() == "Camera" or p.IsA(UsdGeom.Camera):
                            camera_children.append(str(p.GetPath()))
                    except Exception:
                        pass
                if verbose:
                    print(f"  [CAMERA_RESTORE_106] camera children={camera_children if camera_children else 'NONE_FOUND_UNDER_RSD455'}")
            elif verbose:
                print(f"  [CAMERA_RESTORE_106][WARN] rsd455 prim not found: {rsd_path}")
        except Exception as e:
            if verbose:
                print(f"  [CAMERA_RESTORE_106][WARN] failed to restore rsd455 active: {e}")

    if verbose:
        print(f"  [CLEAN] legacy camera/ROS graph disabled={changed if changed else 'NONE'}")
    return changed

def force_show_cube_prims(stage, verbose=True):
    """
    109_: USD 안에 있는 Cube가 실행 중 invisible/active=false 상태로 남는 경우를 막는다.
    사용자가 사진으로 보여준 Prim Path는 /Cube이고, 기존 코드 후보는 /World/Cube라서 둘 다 처리한다.
    원본 USD를 저장하지 않고 실행 중 Stage에서만 visible/active를 보장한다.
    """
    if not bool(globals().get("FORCE_SHOW_CUBE_PRIMS_109", True)):
        return
    paths = tuple(globals().get("CUBE_VISIBLE_CANDIDATE_PATHS_109", ("/Cube", "/World/Cube")))
    shown = []
    missing = []
    for root_path in paths:
        root = stage.GetPrimAtPath(root_path)
        if not root or not root.IsValid():
            missing.append(root_path)
            continue
        try:
            root.SetActive(True)
        except Exception:
            pass
        for prim in Usd.PrimRange(root):
            try:
                prim.SetActive(True)
            except Exception:
                pass
            try:
                if prim.IsA(UsdGeom.Imageable):
                    img = UsdGeom.Imageable(prim)
                    img.MakeVisible()
                    # 목적이 guide/proxy로 되어 있어 viewport에서 빠질 수 있으므로 default로 보정 시도
                    try:
                        img.CreatePurposeAttr().Set(UsdGeom.Tokens.default_)
                    except Exception:
                        pass
            except Exception:
                pass
        shown.append(root_path)
    if verbose:
        print(f"  [CUBE_VISIBLE_109] shown={shown if shown else 'NONE'}, missing={missing if missing else 'NONE'}")

def _iter_subtree_all_children_146(root):
    """Usd.PrimRange가 inactive prim을 건너뛰는 경우까지 대비해 GetAllChildren으로 하위 prim을 돈다."""
    if not root or not root.IsValid():
        return
    stack = [root]
    while stack:
        prim = stack.pop(0)
        yield prim
        try:
            children = list(prim.GetAllChildren())
        except Exception:
            try:
                children = list(prim.GetChildren())
            except Exception:
                children = []
        stack[0:0] = children

def _make_prim_visible_active_146(prim):
    if not prim or not prim.IsValid():
        return False
    changed = False
    try:
        prim.SetActive(True)
        changed = True
    except Exception:
        pass
    try:
        if prim.IsA(UsdGeom.Imageable):
            img = UsdGeom.Imageable(prim)
            img.MakeVisible()
            changed = True
            if bool(globals().get("ENVIRONMENT_FORCE_PURPOSE_DEFAULT_146", True)):
                try:
                    img.CreatePurposeAttr().Set(UsdGeom.Tokens.default_)
                except Exception:
                    pass
    except Exception:
        pass
    return changed

def _make_ancestors_visible_active_146(stage, path_str):
    """상위 prim이 invisible이면 자식만 visible이어도 안 보이므로 상위까지 같이 복구한다."""
    try:
        p = Sdf.Path(path_str)
    except Exception:
        return
    cur = p.GetParentPath()
    while cur and str(cur) not in ("", "/"):
        prim = stage.GetPrimAtPath(cur)
        if prim and prim.IsValid():
            _make_prim_visible_active_146(prim)
        cur = cur.GetParentPath()

def force_show_environment_prims(stage, verbose=True):
    """
    146_: 새 USD에서 Prim Path가 Environment인 환경 오브젝트가 145 실행 후 안 보이는 문제 방지.
    - 명시 후보: /Environment, /World/Environment 등
    - 추가 후보: /World 하위에서 이름에 Environment가 들어가는 prim
    - root와 모든 하위 prim을 active=True, visible, purpose=default로 보정
    원본 USD를 저장하지 않고 실행 중 Stage에서만 처리한다.
    """
    if not bool(globals().get("FORCE_SHOW_ENVIRONMENT_PRIMS_146", True)):
        return []

    candidate_paths = []
    for p in tuple(globals().get("ENVIRONMENT_VISIBLE_CANDIDATE_PATHS_146", ())):
        if p and p not in candidate_paths:
            candidate_paths.append(str(p))

    keywords = tuple(globals().get("ENVIRONMENT_VISIBLE_NAME_KEYWORDS_146", ("Environment",)))
    scan_roots = ["/World", "/"]
    for scan_root in scan_roots:
        try:
            root = stage.GetPrimAtPath(scan_root) if scan_root != "/" else stage.GetPseudoRoot()
            if not root or not root.IsValid():
                continue
            for prim in _iter_subtree_all_children_146(root):
                try:
                    name = prim.GetName()
                    path = str(prim.GetPath())
                    if any(k in name for k in keywords) and path not in candidate_paths:
                        candidate_paths.append(path)
                except Exception:
                    pass
        except Exception:
            pass

    shown = []
    missing = []
    fixed_count = 0
    for root_path in candidate_paths:
        root = stage.GetPrimAtPath(root_path)
        if not root or not root.IsValid():
            missing.append(root_path)
            continue
        _make_ancestors_visible_active_146(stage, root_path)
        local_count = 0
        for prim in _iter_subtree_all_children_146(root):
            if _make_prim_visible_active_146(prim):
                local_count += 1
        fixed_count += local_count
        shown.append(f"{root_path}({local_count})")

    if verbose:
        print(f"  [ENV_VISIBLE_146] shown={shown if shown else 'NONE'}, fixed_prims={fixed_count}, missing={missing if missing else 'NONE'}")
    return shown

def _disable_collision_and_visibility(stage, root_path: str):
    """
    지정한 prim 하위 전체를 보이지 않게 하고 물리 충돌을 끈다.
    RemovePrim이 reference prim에 바로 먹지 않는 경우를 대비한 fallback이다.
    """
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return {"hidden": 0, "collision_off": 0, "rigid_removed": 0, "mass_removed": 0}

    hidden = 0
    collision_off = 0
    rigid_removed = 0
    mass_removed = 0

    for prim in list(Usd.PrimRange(root_prim)):
        try:
            UsdGeom.Imageable(prim).MakeInvisible()
            hidden += 1
        except Exception:
            pass

        try:
            col_api = UsdPhysics.CollisionAPI.Apply(prim)
            col_api.CreateCollisionEnabledAttr(False)
            collision_off += 1
        except Exception:
            pass

        try:
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                rigid_removed += 1
        except Exception:
            pass

        try:
            if prim.HasAPI(UsdPhysics.MassAPI):
                prim.RemoveAPI(UsdPhysics.MassAPI)
                mass_removed += 1
        except Exception:
            pass

    return {
        "hidden": hidden,
        "collision_off": collision_off,
        "rigid_removed": rigid_removed,
        "mass_removed": mass_removed,
    }

def _find_old_rg2_paths(stage):
    """
    onrobot_rg2ft가 예상 경로가 아닌 곳에 남아 있어도 찾아서 처리한다.
    """
    paths = set()
    if stage.GetPrimAtPath(OLD_GRIPPER_PRIM_PATH).IsValid():
        paths.add(OLD_GRIPPER_PRIM_PATH)

    robot_root = stage.GetPrimAtPath(ROBOT_PRIM_PATH)
    if robot_root.IsValid():
        for prim in Usd.PrimRange(robot_root):
            name_l = prim.GetName().lower()
            path_s = str(prim.GetPath())
            if name_l == "onrobot_rg2ft" or "onrobot_rg2ft" in path_s.lower():
                paths.add(path_s)

    # 하위 prim이 먼저 잡혔을 때 상위 onrobot_rg2ft만 남긴다.
    cleaned = set()
    for p in paths:
        if "/onrobot_rg2ft" in p:
            cleaned.add(p.split("/onrobot_rg2ft")[0] + "/onrobot_rg2ft")
        else:
            cleaned.add(p)
    return sorted(cleaned, key=len, reverse=True)

def remove_old_gripper(stage, verbose=True):
    """
    기존 RG2 집게를 완전히 제거한다.

    주의: 원본 USD 파일을 지우는 게 아니라, 실행 중 Stage에서만 제거/비활성화한다.
    reference prim은 RemovePrim만으로 화면에 남는 경우가 있어서
    RemovePrim → invisible/collision off → SetActive(False) 순서로 강하게 처리한다.
    """
    paths = _find_old_rg2_paths(stage)
    if not paths:
        if verbose:
            print(f"  [INFO] 기존 RG2 집게 없음: {OLD_GRIPPER_PRIM_PATH}")
        return

    removed = 0
    deactivated = 0
    fallback_hidden = 0

    # 선택 표시가 남지 않도록 먼저 선택 해제
    try:
        omni.usd.get_context().get_selection().clear_selected_prim_paths()
    except Exception:
        pass

    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue

        # 1차: RemovePrim 시도
        try:
            stage.RemovePrim(path)
            for _ in range(5):
                simulation_app.update()
        except Exception:
            pass

        # RemovePrim 후 사라졌으면 성공
        if not stage.GetPrimAtPath(path).IsValid():
            removed += 1
            if verbose:
                print(f"  [OK] RG2 RemovePrim 성공: {path}")
            continue

        # 2차: 보이지 않게 하고 충돌 제거
        stats = _disable_collision_and_visibility(stage, path)
        fallback_hidden += stats.get("hidden", 0)

        # 3차: prim 자체 비활성화
        try:
            prim = stage.GetPrimAtPath(path)
            if prim.IsValid():
                prim.SetActive(False)
                deactivated += 1
                for _ in range(5):
                    simulation_app.update()
        except Exception:
            pass

        if verbose:
            still_valid = stage.GetPrimAtPath(path).IsValid()
            print(f"  [OK] RG2 fallback 처리: {path} valid_after={still_valid}")
            print(f"       hidden={stats.get('hidden', 0)} collision_off={stats.get('collision_off', 0)}")

    if verbose:
        print(f"  [SUMMARY] RG2 removed={removed}, deactivated={deactivated}, hidden_children={fallback_hidden}")

def make_subtree_visual_only(stage, root_path: str):
    """
    VGC10 CAD/USD 안에 RigidBody/Collider가 들어 있으면 link_6에 붙는 순간
    바닥, 큐브, 로봇과 충돌해서 물체를 밀거나 바닥을 들어 올리는 것처럼 보일 수 있다.
    시연에서는 VGC10은 외형만 필요하므로 해당 subtree의 물리 API/충돌을 꺼 둔다.
    """
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        print(f"  [WARN] visual-only 처리 실패: prim 없음 {root_path}")
        return

    disabled_collision = 0
    removed_rigid = 0
    removed_mass = 0
    hidden_scene_extras = 0

    # 참조 USD가 compose될 시간을 조금 준다.
    for _ in range(10):
        simulation_app.update()

    for prim in list(Usd.PrimRange(root_prim)):
        name_l = prim.GetName().lower()

        # CAD 변환 중 같이 딸려온 ground/floor 같은 장면 부속물이 있으면 숨긴다.
        # VGC10 본체 이름에 ground/floor가 들어가는 경우는 거의 없고, 있더라도 visual-only 용도라 문제 적음.
        if name_l in {"ground", "groundplane", "floor", "defaultgroundplane"} or "groundplane" in name_l:
            try:
                imageable = UsdGeom.Imageable(prim)
                imageable.MakeInvisible()
                hidden_scene_extras += 1
            except Exception:
                pass

        # Collider 비활성화. HasAPI가 false여도 Apply해서 명시적으로 꺼 둔다.
        try:
            col_api = UsdPhysics.CollisionAPI.Apply(prim)
            col_api.CreateCollisionEnabledAttr(False)
            disabled_collision += 1
        except Exception:
            pass

        # RigidBody/Mass API 제거. 실패해도 무시한다.
        try:
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                removed_rigid += 1
        except Exception:
            pass

        try:
            if prim.HasAPI(UsdPhysics.MassAPI):
                prim.RemoveAPI(UsdPhysics.MassAPI)
                removed_mass += 1
        except Exception:
            pass

    print("  [OK] VGC10 visual-only 처리")
    print(f"       collision disabled attrs = {disabled_collision}")
    print(f"       rigid body APIs removed  = {removed_rigid}")
    print(f"       mass APIs removed        = {removed_mass}")
    print(f"       hidden scene extras      = {hidden_scene_extras}")

def _resolve_vgc10_asset_path():
    candidates = [VGC10_USD_PATH] + list(VGC10_FALLBACK_PATHS)
    seen = set()
    for p in candidates:
        if p in seen:
            continue
        seen.add(p)
        if Path(p).exists():
            return p
    return VGC10_USD_PATH

def _set_xform_common(prim, translate=None, rotate=None, scale=None):
    xform_api = UsdGeom.XformCommonAPI(prim)
    if translate is not None:
        xform_api.SetTranslate(tuple(float(x) for x in translate))
    if rotate is not None:
        xform_api.SetRotate(tuple(float(x) for x in rotate))
    if scale is not None:
        xform_api.SetScale(tuple(float(x) for x in scale))

def _get_or_create_transform_op(prim):
    """prim에 matrix transform op 하나를 만들고 계속 재사용한다."""
    xformable = UsdGeom.Xformable(prim)
    ops = xformable.GetOrderedXformOps()
    if len(ops) == 1 and ops[0].GetOpType() == UsdGeom.XformOp.TypeTransform:
        return ops[0]
    xformable.ClearXformOpOrder()
    return xformable.AddTransformOp()

def set_prim_local_matrix(prim, mat):
    op = _get_or_create_transform_op(prim)
    op.Set(mat)

def make_trs_matrix(translate=None, rotate_xyz_deg=None, scale=None):
    """T * Rz * Ry * Rx * S 순서의 local transform matrix 생성."""
    translate = np.array([0.0, 0.0, 0.0] if translate is None else translate, dtype=float)
    rotate_xyz_deg = np.array([0.0, 0.0, 0.0] if rotate_xyz_deg is None else rotate_xyz_deg, dtype=float)
    scale = np.array([1.0, 1.0, 1.0] if scale is None else scale, dtype=float)

    m = Gf.Matrix4d(1.0)
    m.SetTranslateOnly(Gf.Vec3d(float(translate[0]), float(translate[1]), float(translate[2])))

    rx = Gf.Matrix4d(1.0)
    ry = Gf.Matrix4d(1.0)
    rz = Gf.Matrix4d(1.0)
    rx.SetRotateOnly(Gf.Rotation(Gf.Vec3d(1, 0, 0), float(rotate_xyz_deg[0])))
    ry.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 1, 0), float(rotate_xyz_deg[1])))
    rz.SetRotateOnly(Gf.Rotation(Gf.Vec3d(0, 0, 1), float(rotate_xyz_deg[2])))

    sm = Gf.Matrix4d(1.0)
    sm.SetScale(Gf.Vec3d(float(scale[0]), float(scale[1]), float(scale[2])))
    return m * rz * ry * rx * sm

def get_world_matrix(stage, prim_path: str):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None
    try:
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        return cache.GetLocalToWorldTransform(prim)
    except Exception:
        return None

def get_world_translation(stage, prim_path: str):
    mat = get_world_matrix(stage, prim_path)
    if mat is None:
        return None
    t = mat.ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])], dtype=float)


def _world_to_local_point(stage, prim_path, world_point):
    """Convert a world point through a prim transform for physics and diagnostics."""
    mat = get_world_matrix(stage, prim_path)
    if mat is None:
        return np.zeros(3, dtype=float)
    try:
        inv = mat.GetInverse()
        wp = np.array(world_point, dtype=float)
        lp = inv.Transform(Gf.Vec3d(float(wp[0]), float(wp[1]), float(wp[2])))
        return np.array([float(lp[0]), float(lp[1]), float(lp[2])], dtype=float)
    except Exception:
        return np.zeros(3, dtype=float)


def _local_to_world_point(stage, prim_path, local_point):
    """Convert a prim-local point to world coordinates."""
    mat = get_world_matrix(stage, prim_path)
    if mat is None:
        return np.zeros(3, dtype=float)
    try:
        lp = np.array(local_point, dtype=float)
        wp = mat.Transform(Gf.Vec3d(float(lp[0]), float(lp[1]), float(lp[2])))
        return np.array([float(wp[0]), float(wp[1]), float(wp[2])], dtype=float)
    except Exception:
        return np.zeros(3, dtype=float)

def update_vgc10_visual_follow_pose(stage, robot=None):
    """
    15_ 핵심 수정:
    - VGC10 root는 /World 아래에서 tool0 월드 transform을 따라간다.
    - 실제 gripper.usda reference는 root 바로 밑이 아니라 vgc10_scaled_mount 아래에 둔다.
    - scale은 mount prim에만 한 번 적용한다.
    - 매 프레임 root pose만 갱신하므로 scale이 set_world_pose/update에 의해 사라지지 않는다.
    """
    root_prim = stage.GetPrimAtPath(VGC10_PRIM_PATH)
    if not root_prim.IsValid():
        return None

    target_mat = get_world_matrix(stage, VGC10_FOLLOW_TARGET_PATH)
    if target_mat is None and robot is not None:
        try:
            ee_pos, _ = robot.end_effector.get_world_pose()
            target_mat = Gf.Matrix4d(1.0)
            target_mat.SetTranslateOnly(Gf.Vec3d(float(ee_pos[0]), float(ee_pos[1]), float(ee_pos[2])))
        except Exception:
            target_mat = None

    if target_mat is None:
        return None

    set_prim_local_matrix(root_prim, target_mat)
    t = target_mat.ExtractTranslation()
    return np.array([float(t[0]), float(t[1]), float(t[2])], dtype=float)

def attach_vgc10_to_link6(stage):
    # 중요: VGC10 CAD는 articulation 하위에 직접 붙이지 않는다.
    # /World/vgc10_visual_follow(root)가 tool0를 따라가고, 그 아래 mount에 scale을 적용한다.
    clear_prim_if_exists(stage, VGC10_PRIM_PATH)

    vgc10_asset = _resolve_vgc10_asset_path()

    root_xform = UsdGeom.Xform.Define(stage, VGC10_PRIM_PATH)
    root_prim = root_xform.GetPrim()

    mount_xform = UsdGeom.Xform.Define(stage, VGC10_MOUNT_PATH)
    mount_prim = mount_xform.GetPrim()

    model_xform = UsdGeom.Xform.Define(stage, VGC10_MODEL_PATH)
    model_prim = model_xform.GetPrim()

    # 실제 흡착 기준점. 화면에는 보이지 않는 Xform이다.
    # VGC10 root 하위에 두므로 root가 tool0를 따라갈 때 같이 따라간다.
    suction_point_xform = UsdGeom.Xform.Define(stage, VGC10_SUCTION_POINT_PATH)
    suction_point_prim = suction_point_xform.GetPrim()

    if Path(vgc10_asset).exists():
        model_prim.GetReferences().AddReference(str(vgc10_asset))
        print(f"  [OK] 실제 VGC10 USDA/USD reference 부착: {vgc10_asset}")
    else:
        print(f"  [WARN] 실제 VGC10 파일을 찾지 못함: {vgc10_asset}")
        print("         임시 visual suction pad만 생성한다. assets/gripper_vgc10_v1.usda 또는 assets/gripper.usda 경로를 확인해라.")
        UsdGeom.Cylinder.Define(stage, f"{VGC10_MODEL_PATH}/fallback_suction_pad")

    # root는 follow용이라 단위 transform으로 시작한다.
    set_prim_local_matrix(root_prim, Gf.Matrix4d(1.0))

    # scale/rotate/translate는 mount에만 적용한다. 이 방식이면 reference 내부 xform이나 set_world_pose가 scale을 덮어쓰지 못한다.
    mount_mat = make_trs_matrix(
        translate=VGC10_LOCAL_TRANSLATE,
        rotate_xyz_deg=VGC10_LOCAL_ROTATE_XYZ,
        scale=VGC10_LOCAL_SCALE,
    )
    set_prim_local_matrix(mount_prim, mount_mat)

    # model 자체는 transform 없이 reference만 둔다.
    set_prim_local_matrix(model_prim, Gf.Matrix4d(1.0))

    # suction point는 VGC10 root 기준 local offset으로 둔다.
    # 이 값은 meter 단위이며, 안 붙으면 VGC10_SUCTION_LOCAL_OFFSET만 조정한다.
    suction_mat = Gf.Matrix4d(1.0)
    suction_mat.SetTranslateOnly(Gf.Vec3d(
        float(VGC10_SUCTION_LOCAL_OFFSET[0]),
        float(VGC10_SUCTION_LOCAL_OFFSET[1]),
        float(VGC10_SUCTION_LOCAL_OFFSET[2]),
    ))
    set_prim_local_matrix(suction_point_prim, suction_mat)

    # 51_: 3x3 흡착점 그리드를 VGC10 root 하위에 생성한다.
    # p11이 중심점이고, p00~p22가 박스 윗면 접촉 판정에 사용된다.
    if SUCTION_GRID_ENABLED:
        try:
            clear_prim_if_exists(stage, SUCTION_GRID_ROOT_PATH)
            grid_root = UsdGeom.Xform.Define(stage, SUCTION_GRID_ROOT_PATH).GetPrim()
            set_prim_local_matrix(grid_root, Gf.Matrix4d(1.0))
            spacing = float(SUCTION_GRID_SPACING_XY)
            labels = []
            for iy, y_mul in enumerate([-1.0, 0.0, 1.0]):
                for ix, x_mul in enumerate([-1.0, 0.0, 1.0]):
                    label = f"p{iy}{ix}"
                    path = f"{SUCTION_GRID_ROOT_PATH}/{label}"
                    if SUCTION_GRID_MARKERS_VISIBLE:
                        point_prim = UsdGeom.Sphere.Define(stage, path).GetPrim()
                        try:
                            UsdGeom.Sphere(point_prim).CreateRadiusAttr(float(SUCTION_GRID_MARKER_RADIUS))
                            UsdGeom.Gprim(point_prim).CreateDisplayColorAttr([Gf.Vec3f(0.0, 1.0, 0.0)])
                        except Exception:
                            pass
                    else:
                        point_prim = UsdGeom.Xform.Define(stage, path).GetPrim()
                    point_mat = Gf.Matrix4d(1.0)
                    point_mat.SetTranslateOnly(Gf.Vec3d(
                        float(VGC10_SUCTION_LOCAL_OFFSET[0] + x_mul * spacing),
                        float(VGC10_SUCTION_LOCAL_OFFSET[1] + y_mul * spacing),
                        float(VGC10_SUCTION_LOCAL_OFFSET[2]),
                    ))
                    set_prim_local_matrix(point_prim, point_mat)
                    # marker 자체가 충돌하지 않도록 명시적으로 물리 제거/비활성화
                    try:
                        if point_prim.HasAPI(UsdPhysics.CollisionAPI):
                            point_prim.RemoveAPI(UsdPhysics.CollisionAPI)
                    except Exception:
                        pass
                    try:
                        if point_prim.HasAPI(UsdPhysics.RigidBodyAPI):
                            point_prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                    except Exception:
                        pass
                    labels.append(label)
            print(
                f"  [SUCTION_GRID_CREATE] path={SUCTION_GRID_ROOT_PATH}, points={labels}, "
                f"spacing={spacing:.3f}m, total_width={2*spacing:.3f}m, visible={SUCTION_GRID_MARKERS_VISIBLE}"
            )
        except Exception as e:
            print(f"  [SUCTION_GRID_CREATE_FAIL] {e}")

    for _ in range(10):
        simulation_app.update()

    if VGC10_VISUAL_ONLY:
        make_subtree_visual_only(stage, VGC10_PRIM_PATH)

    update_vgc10_visual_follow_pose(stage, robot=None)

    try:
        omni.usd.get_context().get_selection().clear_selected_prim_paths()
    except Exception:
        pass

    print(f"  [OK] VGC10 root path  = {VGC10_PRIM_PATH}")
    print(f"  [OK] VGC10 mount path = {VGC10_MOUNT_PATH}")
    print(f"  [OK] VGC10 model path = {VGC10_MODEL_PATH}")
    print(f"       follow target   = {VGC10_FOLLOW_TARGET_PATH}")
    print(f"       local translate = {VGC10_LOCAL_TRANSLATE}")
    print(f"       local rotateXYZ = {VGC10_LOCAL_ROTATE_XYZ}")
    print(f"       local scale     = {VGC10_LOCAL_SCALE}")
    print(f"       suction point   = {VGC10_SUCTION_POINT_PATH}")
    print(f"       suction offset  = {VGC10_SUCTION_LOCAL_OFFSET}")

def _safe_token_from_path(path: str):
    token = str(path).strip("/").replace("/", "_")
    return "".join(ch if (ch.isalnum() or ch == "_") else "_" for ch in token)

def _find_tool_target_for_robot_prim(stage, robot_prim_path: str):
    """robot_prim_path 아래에서 link_6/tool0를 찾는다. tool0가 없으면 link_6를 사용한다."""
    if not stage.GetPrimAtPath(robot_prim_path).IsValid():
        return None
    link6_path = find_prim_path_by_name(robot_prim_path, EE_LINK_NAME)
    if link6_path is None:
        return None
    tool0_path = link6_path + "/tool0"
    if stage.GetPrimAtPath(tool0_path).IsValid():
        return tool0_path
    return link6_path

def _resolve_robot_prim_from_root(stage, root_path: str):
    """/World/m0609_01/m0609 구조와 /World/m0609_01 자체 articulation 구조를 둘 다 허용한다."""
    candidates = [root_path + "/m0609", root_path]
    for p in candidates:
        if _find_tool_target_for_robot_prim(stage, p) is not None:
            return p
    return None

def _discover_idle_m0609_robot_prims(stage):
    """/World 아래 m0609_ 로 시작하는 active 외 로봇을 찾는다."""
    results = []
    world = stage.GetPrimAtPath("/World")
    if not world.IsValid():
        return results
    for child in world.GetChildren():
        name = child.GetName()
        root_path = str(child.GetPath())
        if not name.startswith(IDLE_M0609_ROOT_PREFIX):
            continue
        if root_path == ACTIVE_ROBOT_ROOT_PATH:
            continue
        robot_path = _resolve_robot_prim_from_root(stage, root_path)
        if robot_path is not None:
            results.append(robot_path)
    return sorted(set(results))

def _create_vgc10_visual_for_target(stage, target_path: str, root_path: str, label: str):
    """idle 로봇에 active 로봇과 같은 VGC10 visual을 붙여 보이게 한다. 제어는 하지 않는다."""
    if target_path is None or not stage.GetPrimAtPath(target_path).IsValid():
        print(f"  [IDLE_VGC10_SKIP] target 없음: label={label}, target={target_path}")
        return False

    clear_prim_if_exists(stage, root_path)
    asset = _resolve_vgc10_asset_path()
    mount_path = root_path + "/vgc10_scaled_mount"
    model_path = mount_path + "/gripper_model"
    suction_path = root_path + "/vgc10_suction_point"

    root_prim = UsdGeom.Xform.Define(stage, root_path).GetPrim()
    mount_prim = UsdGeom.Xform.Define(stage, mount_path).GetPrim()
    model_prim = UsdGeom.Xform.Define(stage, model_path).GetPrim()
    suction_prim = UsdGeom.Xform.Define(stage, suction_path).GetPrim()

    if Path(asset).exists():
        model_prim.GetReferences().AddReference(str(asset))
    else:
        print(f"  [IDLE_VGC10_WARN] VGC10 asset 없음: {asset}")
        UsdGeom.Cylinder.Define(stage, model_path + "/fallback_suction_pad")

    target_mat = get_world_matrix(stage, target_path)
    if target_mat is None:
        target_mat = Gf.Matrix4d(1.0)
    set_prim_local_matrix(root_prim, target_mat)

    mount_mat = make_trs_matrix(
        translate=VGC10_LOCAL_TRANSLATE,
        rotate_xyz_deg=VGC10_LOCAL_ROTATE_XYZ,
        scale=VGC10_LOCAL_SCALE,
    )
    set_prim_local_matrix(mount_prim, mount_mat)
    set_prim_local_matrix(model_prim, Gf.Matrix4d(1.0))

    suction_mat = Gf.Matrix4d(1.0)
    suction_mat.SetTranslateOnly(Gf.Vec3d(
        float(VGC10_SUCTION_LOCAL_OFFSET[0]),
        float(VGC10_SUCTION_LOCAL_OFFSET[1]),
        float(VGC10_SUCTION_LOCAL_OFFSET[2]),
    ))
    set_prim_local_matrix(suction_prim, suction_mat)

    for _ in range(10):
        simulation_app.update()
    if VGC10_VISUAL_ONLY:
        make_subtree_visual_only(stage, root_path)

    print(f"  [IDLE_VGC10_OK] {label}: root={root_path}")
    print(f"                  follow target={target_path}")
    return True

def attach_vgc10_to_idle_m0609_robots(stage):
    if not ATTACH_VGC10_TO_IDLE_M0609_ROBOTS:
        return []
    idle_robot_prims = _discover_idle_m0609_robot_prims(stage)
    attached = []
    if not idle_robot_prims:
        print("  [IDLE_VGC10] active 외 m0609_ 로봇을 찾지 못함")
        return attached
    for robot_path in idle_robot_prims:
        target = _find_tool_target_for_robot_prim(stage, robot_path)
        root_candidate = robot_path.rsplit("/m0609", 1)[0] if robot_path.endswith("/m0609") else robot_path
        token = _safe_token_from_path(root_candidate)
        vgc10_root = f"{IDLE_VGC10_ROOT_PREFIX}_{token}"
        if _create_vgc10_visual_for_target(stage, target, vgc10_root, label=root_candidate):
            attached.append((robot_path, vgc10_root, target))
    print(f"  [IDLE_VGC10_SUMMARY] idle robots with VGC10 visual = {len(attached)}")
    return attached

def zero_body_velocity(obj):
    try:
        obj.set_linear_velocity(np.zeros(3))
        obj.set_angular_velocity(np.zeros(3))
    except Exception:
        pass

class NullGripper:
    """
    기존 RG2 finger_joint를 삭제한 뒤에도 PickPlaceController가 gripper.forward()를
    호출할 수 있게 해주는 더미 그리퍼다. 실제 관절 명령은 만들지 않는다.
    """

    joint_opened_positions = np.array([], dtype=float)
    joint_closed_positions = np.array([], dtype=float)

    def initialize(self, *args, **kwargs):
        return None

    def reset(self):
        return None

    def set_joint_positions(self, *args, **kwargs):
        return None

    def get_joint_positions(self):
        return np.array([], dtype=float)

    def forward(self, action=None):
        return ArticulationAction(
            joint_positions=np.array([], dtype=float),
            joint_indices=np.array([], dtype=np.int32),
        )

    def open(self):
        return self.forward(action="open")

    def close(self):
        return self.forward(action="close")

def update_vgc10_suction_anchor(robot):
    """
    VGC10 기준 흡착 좌표만 계산한다.
    기존 scripted_suction_body / scripted_suction_direction_marker visual은 더 이상 움직이거나 생성하지 않는다.
    """
    stage = omni.usd.get_context().get_stage()

    # 먼저 VGC10 visual-follow root를 tool0 위치/자세로 갱신한다.
    update_vgc10_visual_follow_pose(stage, robot=robot)

    suction_pos = get_world_translation(stage, VGC10_SUCTION_POINT_PATH)
    if suction_pos is None:
        # fallback: VGC10 suction point를 못 찾으면 기존 EE 기준으로 계산
        ee_pos, _ = robot.end_effector.get_world_pose()
        suction_pos = np.array(ee_pos, dtype=float) + SUCTION_OFFSET_FROM_EE

    suction_pos = np.array(suction_pos, dtype=float)
    suction_pos[2] = max(float(suction_pos[2]), float(SUCTION_MIN_Z))

    # 디버그가 필요할 때만 작은 녹색 마커를 보여준다.
    if DEBUG_SHOW_SUCTION_POINT:
        marker_prim = stage.GetPrimAtPath(VGC10_SUCTION_DEBUG_MARKER_PATH)
        if not marker_prim.IsValid():
            marker = UsdGeom.Sphere.Define(stage, VGC10_SUCTION_DEBUG_MARKER_PATH)
            marker.CreateRadiusAttr(0.015)
            marker_prim = marker.GetPrim()
        marker_mat = Gf.Matrix4d(1.0)
        marker_mat.SetTranslateOnly(Gf.Vec3d(float(suction_pos[0]), float(suction_pos[1]), float(suction_pos[2])))
        set_prim_local_matrix(marker_prim, marker_mat)
    else:
        clear_prim_if_exists(stage, VGC10_SUCTION_DEBUG_MARKER_PATH)

    return suction_pos

def get_suction_grid_world_points(stage):
    """
    55_: 3x3 흡착점 world 좌표를 얻는다.

    기존 51~53_ 방식은 /World/vgc10_visual_follow/vgc10_suction_grid 하위 prim의
    실제 world pose를 읽었다. 그런데 gripper/tool0 방향이 하늘을 보거나 기울면
    9점 배열도 같이 기울어져 상자 윗면 판정이 불안정해졌다.

    이번 버전은 기본적으로 현재 suction 중심을 기준으로 world XY 평면에
    3x3 가상 흡착점을 만든다. 그래서 '상자 윗면을 제대로 덮는가'만 확인한다.
    """
    result = []

    center_pos = get_world_translation(stage, VGC10_SUCTION_POINT_PATH)
    if center_pos is None:
        center_pos = get_world_translation(stage, VGC10_PRIM_PATH)
    if center_pos is not None and bool(SUCTION_GRID_EVALUATE_AS_WORLD_XY_GRID):
        center_pos = np.array(center_pos, dtype=float)
        spacing = float(SUCTION_GRID_SPACING_XY)
        for iy, y_mul in enumerate([-1.0, 0.0, 1.0]):
            for ix, x_mul in enumerate([-1.0, 0.0, 1.0]):
                label = f"p{iy}{ix}"
                pos = np.array([
                    center_pos[0] + x_mul * spacing,
                    center_pos[1] + y_mul * spacing,
                    center_pos[2],
                ], dtype=float)
                result.append({
                    "label": label,
                    "path": f"{SUCTION_GRID_WORLD_MARKER_ROOT_PATH}/{label}",
                    "pos": pos,
                    "center": (label == "p11"),
                })

        # 화면에서도 확인할 수 있도록 world XY 기준 마커를 갱신한다.
        # 이 마커들은 물리 없음/판정용 표시 전용이다.
        if SUCTION_GRID_MARKERS_VISIBLE:
            try:
                root = UsdGeom.Xform.Define(stage, SUCTION_GRID_WORLD_MARKER_ROOT_PATH).GetPrim()
                set_prim_local_matrix(root, Gf.Matrix4d(1.0))
                for pt in result:
                    prim = stage.GetPrimAtPath(pt["path"])
                    if not prim.IsValid():
                        prim = UsdGeom.Sphere.Define(stage, pt["path"]).GetPrim()
                        try:
                            UsdGeom.Sphere(prim).CreateRadiusAttr(float(SUCTION_GRID_MARKER_RADIUS))
                            UsdGeom.Gprim(prim).CreateDisplayColorAttr([Gf.Vec3f(0.0, 0.6, 1.0)])
                        except Exception:
                            pass
                    mat = Gf.Matrix4d(1.0)
                    pp = np.array(pt["pos"], dtype=float)
                    mat.SetTranslateOnly(Gf.Vec3d(float(pp[0]), float(pp[1]), float(pp[2])))
                    set_prim_local_matrix(prim, mat)
                    try:
                        if prim.HasAPI(UsdPhysics.CollisionAPI):
                            prim.RemoveAPI(UsdPhysics.CollisionAPI)
                    except Exception:
                        pass
                    try:
                        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
                    except Exception:
                        pass
            except Exception as e:
                try:
                    print(f"  [SUCTION_WORLD_GRID_MARKER_FAIL] {e}")
                except Exception:
                    pass
        return result

    if SUCTION_GRID_ENABLED:
        for iy in range(3):
            for ix in range(3):
                label = f"p{iy}{ix}"
                path = f"{SUCTION_GRID_ROOT_PATH}/{label}"
                pos = get_world_translation(stage, path)
                if pos is not None:
                    result.append({"label": label, "path": path, "pos": np.array(pos, dtype=float), "center": (label == "p11")})
    if not result:
        pos = get_world_translation(stage, VGC10_SUCTION_POINT_PATH)
        if pos is not None:
            result.append({"label": "p11_fallback", "path": VGC10_SUCTION_POINT_PATH, "pos": np.array(pos, dtype=float), "center": True})
    return result

def evaluate_suction_grid_on_box_top(stage, bbox_info, event=None, verbose=False):
    """
    59_: 중심 흡착점 1개만 사용한다.
    9점 평균/부분 hit 방식은 상자 자세와 carry offset을 흔들 수 있어서 중지한다.

    판정 기준:
    - 중심 흡착점 p_center가 박스 윗면 중심 근처에 있어야 함
    - 중심 흡착점 z가 실제 box_top_z 근처여야 함
    - attach_center는 항상 실제 박스 윗면 z + eps로 투영
    """
    global _LAST_SUCTION_GRID_INFO
    if bbox_info is None:
        _LAST_SUCTION_GRID_INFO = {"attach_center": None, "hits": [], "summary": "no_bbox"}
        return False, "center_no_bbox", _LAST_SUCTION_GRID_INFO

    pos = get_world_translation(stage, VGC10_SUCTION_POINT_PATH)
    if pos is None:
        _LAST_SUCTION_GRID_INFO = {"attach_center": None, "hits": [], "summary": "no_center_suction_point"}
        return False, "center_no_suction_point", _LAST_SUCTION_GRID_INFO

    top_center = np.array(bbox_info["top_center"], dtype=float)
    size = np.array(bbox_info["size"], dtype=float)
    half_x = max(float(size[0]) * 0.5, 0.0)
    half_y = max(float(size[1]) * 0.5, 0.0)
    allowed_x = max(0.0, half_x - BOX_TOP_SURFACE_MARGIN_X)
    allowed_y = max(0.0, half_y - BOX_TOP_SURFACE_MARGIN_Y)

    pos = np.array(pos, dtype=float)
    dx = float(pos[0] - top_center[0])
    dy = float(pos[1] - top_center[1])
    xy_err = float(np.linalg.norm(pos[:2] - top_center[:2]))
    z_gap = float(pos[2] - top_center[2])

    inside_rect = (abs(dx) <= allowed_x) and (abs(dy) <= allowed_y)
    within_center_tol = xy_err <= float(CENTER_SUCTION_CENTER_TOL_XY)
    within_effective_radius = xy_err <= float(CENTER_SUCTION_EFFECTIVE_RADIUS_XY)
    z_ok = BOX_ATTACH_Z_MIN <= z_gap <= BOX_ATTACH_Z_MAX

    ok = bool(inside_rect and within_center_tol and within_effective_radius and z_ok)

    attach_center = np.array([pos[0], pos[1], top_center[2] + PHYSICS_ATTACH_TOP_SURFACE_EPS], dtype=float)
    # 중심 흡착 방식에서는 attach_center XY를 box top 중심 쪽으로 약간 끌어와서 offset이 한쪽으로 치우치지 않게 한다.
    attach_center[:2] = top_center[:2] + np.clip(pos[:2] - top_center[:2], -0.020, 0.020)

    blockers = []
    if not inside_rect:
        blockers.append("CENTER_OUTSIDE_TOP_RECT")
    if not within_center_tol:
        blockers.append(f"CENTER_XY_ERR({xy_err:.3f}>{CENTER_SUCTION_CENTER_TOL_XY:.3f})")
    if not within_effective_radius:
        blockers.append(f"RADIUS({xy_err:.3f}>{CENTER_SUCTION_EFFECTIVE_RADIUS_XY:.3f})")
    if not z_ok:
        blockers.append(f"Z_GAP({z_gap:.3f} not in [{BOX_ATTACH_Z_MIN:.3f},{BOX_ATTACH_Z_MAX:.3f}])")

    summary = (
        f"center_mode=True,ok={ok},"
        f"dx={dx:+.4f},dy={dy:+.4f},xy_err={xy_err:.4f},"
        f"z_gap={z_gap:+.4f},z_gap_ok=[{BOX_ATTACH_Z_MIN:.3f},{BOX_ATTACH_Z_MAX:.3f}],"
        f"radius={CENTER_SUCTION_EFFECTIVE_RADIUS_XY:.3f},center_tol={CENTER_SUCTION_CENTER_TOL_XY:.3f},"
        f"top=({top_center[0]:.3f},{top_center[1]:.3f},{top_center[2]:.4f}),"
        f"allowed_rect=({allowed_x:.4f},{allowed_y:.4f}),"
        f"attach_center=({attach_center[0]:.3f},{attach_center[1]:.3f},{attach_center[2]:.4f}),"
        f"blockers={'|'.join(blockers) if blockers else 'NONE'}"
    )

    hit_infos = []
    if ok:
        hit_infos.append({"label": "p_center", "pos": pos, "dx": dx, "dy": dy, "z_gap": z_gap})

    point_logs = [
        f"p_center:hit={int(ok)},dx={dx:+.3f},dy={dy:+.3f},xy={xy_err:.3f},zgap={z_gap:+.3f},radius={CENTER_SUCTION_EFFECTIVE_RADIUS_XY:.3f}"
    ]

    _LAST_SUCTION_GRID_INFO = {
        "attach_center": attach_center,
        "hits": hit_infos,
        "hit_count": 1 if ok else 0,
        "center_ok": ok,
        "summary": summary,
        "point_logs": point_logs,
        "ok": ok,
    }

    try:
        evaluate_suction_grid_on_box_top._counter += 1
    except Exception:
        evaluate_suction_grid_on_box_top._counter = 1
    counter = int(evaluate_suction_grid_on_box_top._counter)
    do_log = bool(verbose or ok or SUCTION_GRID_LOG_EVERY_STEP or counter % int(max(1, CENTER_SUCTION_LOG_INTERVAL)) == 0)
    if do_log:
        print(f"  [CENTER_SUCTION] event={event}, {summary}")
        if SUCTION_GRID_VERBOSE_POINTS:
            print("                 " + " | ".join(point_logs))

    return ok, summary, _LAST_SUCTION_GRID_INFO

def resolve_first_valid_path(stage, paths):
    for p in paths:
        if stage.GetPrimAtPath(p).IsValid():
            return p
    return None

def ensure_target_box_exists(stage):  # legacy name: now also supports OriBoxB fallback
    """
    72_: Conveyor_lift.usd에서는 root(/World/OriBoxA_01)를 실제 박스 기준으로 사용한다.
    root가 이미 있으면 root path를 반환한다. fallback 생성 시에도 reference는 root에 붙인다.
    """
    if stage.GetPrimAtPath(BOX_ROOT_PATH).IsValid():
        print(f"  [OK] 기존 root 박스 사용: {BOX_ROOT_PATH}")
        return BOX_ROOT_PATH

    if not Path(ORI_BOX_USD_PATH).exists():
        raise RuntimeError(
            f"박스 root prim이 없고 oriA/oriB USD도 찾지 못했습니다.\n"
            f"  BOX_ROOT_PATH={BOX_ROOT_PATH}\n"
            f"  ORI_BOX_USD_PATH={ORI_BOX_USD_PATH}"
        )

    box_xform = UsdGeom.Xform.Define(stage, BOX_ROOT_PATH)
    box_prim = box_xform.GetPrim()
    box_prim.GetReferences().AddReference(str(ORI_BOX_USD_PATH))

    # fallback 초기 위치. 실제 Conveyor_lift.usd를 쓰면 보통 이 경로는 실행되지 않는다.
    _set_xform_common(
        box_prim,
        translate=BOX_SCREENSHOT_LOCAL_TRANSLATE,
        rotate=BOX_SCREENSHOT_LOCAL_ROTATE_XYZ,
        scale=BOX_SCREENSHOT_LOCAL_SCALE,
    )

    for _ in range(20):
        simulation_app.update()

    print(f"  [OK] OriBox fallback reference 생성(root): {BOX_ROOT_PATH}")
    print(f"       source = {ORI_BOX_USD_PATH}")
    return BOX_ROOT_PATH

def get_world_bbox_info(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None

    purposes = [
        UsdGeom.Tokens.default_,
        UsdGeom.Tokens.render,
        UsdGeom.Tokens.proxy,
        UsdGeom.Tokens.guide,
    ]
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), purposes)
    try:
        aligned = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        mn = aligned.GetMin()
        mx = aligned.GetMax()
        min_v = np.array([float(mn[0]), float(mn[1]), float(mn[2])], dtype=float)
        max_v = np.array([float(mx[0]), float(mx[1]), float(mx[2])], dtype=float)
        center = (min_v + max_v) * 0.5
        size = max_v - min_v
        top_center = np.array([center[0], center[1], max_v[2]], dtype=float)
        return {
            "min": min_v,
            "max": max_v,
            "center": center,
            "size": size,
            "top_center": top_center,
            "top_z": float(max_v[2]),
            "height": float(size[2]),
        }
    except Exception as exc:
        print(f"  [WARN] bbox 계산 실패: {prim_path} / {exc}")
        root_pos = get_world_translation(stage, prim_path)
        if root_pos is None:
            return None
        return {
            "min": root_pos.copy(),
            "max": root_pos.copy(),
            "center": root_pos.copy(),
            "size": np.zeros(3),
            "top_center": root_pos.copy(),
            "top_z": float(root_pos[2]),
            "height": 0.0,
        }

def find_named_child_or_descendant(stage, root_path: str, name: str):
    """root_path 하위에서 지정 child prim path를 찾는다.

    71_: 사용자가 말한 경로는 Small_cardboard_box이고, 기존 USD들은 Small_Cardboard_box를 쓰기도 했다.
    그래서 name 하나만 보지 않고 ORIBOX_STACK_BOX_MESH_NAME_CANDIDATES를 함께 확인한다.
    """
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None
    names = []
    for n in [name] + list(globals().get("ORIBOX_STACK_BOX_MESH_NAME_CANDIDATES", ())):
        n = str(n)
        if n and n not in names:
            names.append(n)
    for prim in Usd.PrimRange(root):
        if prim.GetName() in names:
            return str(prim.GetPath())
    return None

def _vec3(arr):
    arr = np.array(arr, dtype=float)
    return np.array([float(arr[0]), float(arr[1]), float(arr[2])], dtype=float)

def _quat_np(q):
    try:
        return np.array(q, dtype=float)
    except Exception:
        return q

def get_current_ee_pose(robot):
    ee_pos, ee_quat = robot.end_effector.get_world_pose()
    return _vec3(ee_pos), ee_quat

__all__ = [
    'set_subtree_collision_enabled',
    '_resolve_first_existing_path',
    'find_prim_path_by_name',
    'resolve_stack_support_path',
    '_safe_normalize_vec',
    'get_robot_frame_axes_for_stack',
    'robot_relative_vector',
    'get_task_completed_root_path',
    'find_robotaprop_paths',
    '_prim_is_geometry_like',
    'enable_collision_on_subtree',
    'get_robotaprop_clearance_suction_z',
    'initialize_robot',
    'clear_prim_if_exists',
    'remove_goal_marker',
    'disable_legacy_camera_graphs',
    'force_show_cube_prims',
    '_iter_subtree_all_children_146',
    '_make_prim_visible_active_146',
    '_make_ancestors_visible_active_146',
    'force_show_environment_prims',
    '_disable_collision_and_visibility',
    '_find_old_rg2_paths',
    'remove_old_gripper',
    'make_subtree_visual_only',
    '_resolve_vgc10_asset_path',
    '_set_xform_common',
    '_get_or_create_transform_op',
    'set_prim_local_matrix',
    'make_trs_matrix',
    'get_world_matrix',
    'get_world_translation',
    '_world_to_local_point',
    '_local_to_world_point',
    'update_vgc10_visual_follow_pose',
    'attach_vgc10_to_link6',
    '_safe_token_from_path',
    '_find_tool_target_for_robot_prim',
    '_resolve_robot_prim_from_root',
    '_discover_idle_m0609_robot_prims',
    '_create_vgc10_visual_for_target',
    'attach_vgc10_to_idle_m0609_robots',
    'zero_body_velocity',
    'NullGripper',
    'update_vgc10_suction_anchor',
    'get_suction_grid_world_points',
    'evaluate_suction_grid_on_box_top',
    'resolve_first_valid_path',
    'ensure_target_box_exists',
    'get_world_bbox_info',
    'find_named_child_or_descendant',
    '_vec3',
    '_quat_np',
    'get_current_ee_pose'
]
