"""Box physics and FixedJoint control.

Import only after ``SimulationApp`` has been created.
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

_PHYSICS_JOINT_DIAG_STATE = {}


def _sdf_path(path):
    return Sdf.Path(str(path))

def _gf_vec3_from_np(v):
    v = np.array(v, dtype=float)
    return Gf.Vec3f(float(v[0]), float(v[1]), float(v[2]))

def _find_physics_body0_for_attach(stage):
    # link_6를 우선 사용한다. tool0는 Xform일 가능성이 높다.
    link_path = find_prim_path_by_name(ROBOT_PRIM_PATH, PHYSICS_ATTACH_BODY0_LINK_NAME)
    candidates = []
    if link_path:
        candidates.append(link_path)
    candidates.extend([VGC10_FOLLOW_TARGET_PATH, ROBOT_PRIM_PATH])
    for path in candidates:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            return path
    return None

def create_physics_attach_joint(stage, box_body_path, attach_world_point):
    """105_: MassAPI/inertia를 유지하고 FixedJoint localRot0/localRot1을 현재 body 자세 기준으로 정렬한 뒤 진단한다."""
    global _PHYSICS_JOINT_DIAG_STATE
    if not PHYSICS_FIXED_JOINT_ATTACH_ENABLED:
        return False
    clear_prim_if_exists(stage, PHYSICS_ATTACH_JOINT_PATH)

    body0 = _find_physics_body0_for_attach(stage)
    body1 = str(box_body_path)
    if body0 is None or not stage.GetPrimAtPath(body1).IsValid():
        print(f"  [PHYSICS_ATTACH_FAIL] body0={body0}, body1={body1}")
        return False

    print("\n========== [PHYSICS_DIAG_ATTACH_BEGIN_105] ==========")
    print(f"[PHYSICS_DIAG_INPUT] body0={body0}, body1={body1}, requested_attach_world={_fmt_vec(attach_world_point)}")
    _physics_diag_inspect_body(stage, body0, "body0_link")
    _physics_diag_inspect_body(stage, body1, "body1_box")
    if bool(globals().get("PHYSICS_DIAGNOSTIC_LOG_COLLIDER_TREE", True)):
        _physics_diag_inspect_colliders(stage, body1, "body1_box")

    # 104_: 원인 분리를 위해 FixedJoint 생성 전에 박스 mass / centerOfMass / inertia를 현재 stage에만 임시 명시한다.
    _physics_diag_apply_mass_inertia_104(stage, body1)
    _physics_diag_inspect_body(stage, body1, "body1_box_after_mass_inertia_104")

    # 박스 rigid/collision이 없으면 적용한다. 단, 비활성화하지 않는다.
    try:
        UsdPhysics.RigidBodyAPI.Apply(stage.GetPrimAtPath(body1))
        UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(body1)).CreateCollisionEnabledAttr(True)
    except Exception as e:
        print(f"  [PHYSICS_ATTACH_WARN] body1 physics apply warning: {e}")

    wp = np.array(attach_world_point, dtype=float)
    local0 = _world_to_local_point(stage, body0, wp)
    local1 = _world_to_local_point(stage, body1, wp)
    anchor0_pre = _local_to_world_point(stage, body0, local0)
    anchor1_pre = _local_to_world_point(stage, body1, local1)
    before_cap = _physics_diag_capture_pose(stage, body0, body1, local0, local1, wp, label="before_joint_define")
    _physics_diag_print_capture("[before_joint_define]", before_cap)
    print(
        f"[PHYSICS_DIAG_LOCAL] local0={_fmt_vec(local0)}, local1={_fmt_vec(local1)}, "
        f"pre_anchor0={_fmt_vec(anchor0_pre)}, pre_anchor1={_fmt_vec(anchor1_pre)}, "
        f"pre_anchor_err={float(np.linalg.norm(anchor0_pre-anchor1_pre)):.6f}"
    )
    # 105_: 위치 anchor는 이미 맞는 것으로 확인됐으므로, 이번에는 회전 joint frame을 현재 box 방향 기준으로 맞춘다.
    rot_diag_105 = _physics_diag_compute_joint_local_rots_105(stage, body0, body1)

    try:
        joint = UsdPhysics.FixedJoint.Define(stage, PHYSICS_ATTACH_JOINT_PATH)
        joint.CreateBody0Rel().SetTargets([_sdf_path(body0)])
        joint.CreateBody1Rel().SetTargets([_sdf_path(body1)])
        joint.CreateLocalPos0Attr(_gf_vec3_from_np(local0))
        joint.CreateLocalPos1Attr(_gf_vec3_from_np(local1))
        try:
            joint.CreateLocalRot0Attr(rot_diag_105.get("q0", Gf.Quatf(1.0)))
            joint.CreateLocalRot1Attr(rot_diag_105.get("q1", Gf.Quatf(1.0)))
        except Exception as e:
            print(f"  [PHYSICS_LOCALROT_WARN_105] failed to set localRot attrs: {e}")
        print(
            f"  [PHYSICS_ATTACH_OK] joint={PHYSICS_ATTACH_JOINT_PATH}, body0={body0}, body1={body1}, "
            f"world=({wp[0]:.3f},{wp[1]:.3f},{wp[2]:.4f}), "
            f"local0=({local0[0]:.3f},{local0[1]:.3f},{local0[2]:.3f}), "
            f"local1=({local1[0]:.3f},{local1[1]:.3f},{local1[2]:.3f}), "
            f"localRotMode=preserve_body1_box_world_orientation, rot_err={rot_diag_105.get('rot_err', -1.0):.9f}"
        )
        after_define_cap = _physics_diag_capture_pose(stage, body0, body1, local0, local1, wp, label="after_joint_define_before_step")
        _physics_diag_print_capture("[after_joint_define_before_step]", after_define_cap, base=before_cap)
        _PHYSICS_JOINT_DIAG_STATE.clear()
        _PHYSICS_JOINT_DIAG_STATE.update({
            "active": True,
            "body0": body0,
            "body1": body1,
            "local0": np.array(local0, dtype=float),
            "local1": np.array(local1, dtype=float),
            "attach_world": np.array(wp, dtype=float),
            "localRot0": rot_diag_105.get("q0", Gf.Quatf(1.0)),
            "localRot1": rot_diag_105.get("q1", Gf.Quatf(1.0)),
            "rot_diag_105": rot_diag_105,
            "before": before_cap,
            "after_define": after_define_cap,
        })
        print("========== [PHYSICS_DIAG_ATTACH_END_105] ==========\n")
        return True
    except Exception as e:
        print(f"  [PHYSICS_ATTACH_FAIL] create joint error: {e}")
        print("========== [PHYSICS_DIAG_ATTACH_FAIL_105] ==========\n")
        return False

def release_physics_attach_joint(stage, reason="release"):
    if stage.GetPrimAtPath(PHYSICS_ATTACH_JOINT_PATH).IsValid():
        try:
            stage.RemovePrim(PHYSICS_ATTACH_JOINT_PATH)
            print(f"  [PHYSICS_RELEASE_OK] joint removed: {PHYSICS_ATTACH_JOINT_PATH}, reason={reason}")
            return True
        except Exception as e:
            print(f"  [PHYSICS_RELEASE_FAIL] {e}")
    return False

def force_box_dynamic_after_release(stage, root_path, verbose=True):
    """release 직후에만 호출. 중간 운반 중에는 호출 금지."""
    root_path = str(root_path)
    root = stage.GetPrimAtPath(root_path)
    stats = {"rb_apply": 0, "rb_on": 0, "kin_off": 0, "col_on": 0, "gravity_off": 0, "vel_zero": 0}
    if not root or not root.IsValid():
        print(f"[DIAG128_DYNAMIC_RESTORE_FAIL] invalid root={root_path}")
        return stats

    # root는 반드시 rigid body 후보로 본다. child는 기존에 rigid 관련 흔적이 있는 경우만 건드린다.
    for prim in Usd.PrimRange(root):
        try:
            path = str(prim.GetPath())
            is_root = path == root_path
            has_rb_hint = (
                is_root
                or prim.HasAPI(UsdPhysics.RigidBodyAPI)
                or _diag_has_attr_128(prim, "physics:rigidBodyEnabled", "physics:kinematicEnabled", "physics:velocity", "physics:angularVelocity")
            )
            if has_rb_hint:
                try:
                    UsdPhysics.RigidBodyAPI.Apply(prim)
                    stats["rb_apply"] += 1
                except Exception:
                    pass
                if _diag_set_bool_attr_128(prim, "physics:rigidBodyEnabled", True):
                    stats["rb_on"] += 1
                if _diag_set_bool_attr_128(prim, "physics:kinematicEnabled", False):
                    stats["kin_off"] += 1
                for gname in _diag_attr_names_with_128(prim, "gravity"):
                    # disableGravity 계열이면 False가 중력 ON이다.
                    if "disable" in gname.lower() and _diag_set_bool_attr_128(prim, gname, False):
                        stats["gravity_off"] += 1
                if bool(globals().get("RELEASE_DIAG_ZERO_VELOCITY_128", True)):
                    if _diag_set_vec3_attr_128(prim, "physics:velocity", (0, 0, 0)):
                        stats["vel_zero"] += 1
                    if _diag_set_vec3_attr_128(prim, "physics:angularVelocity", (0, 0, 0)):
                        stats["vel_zero"] += 1
        except Exception:
            pass

    if bool(globals().get("RELEASE_DIAG_RESTORE_COLLISION_128", True)):
        for prim in Usd.PrimRange(root):
            try:
                is_geom_like = _prim_is_geometry_like(prim)
                has_col_hint = prim.HasAPI(UsdPhysics.CollisionAPI) or _diag_has_attr_128(prim, "physics:collisionEnabled")
                if is_geom_like or has_col_hint:
                    try:
                        UsdPhysics.CollisionAPI.Apply(prim)
                    except Exception:
                        pass
                    if _diag_set_bool_attr_128(prim, "physics:collisionEnabled", True):
                        stats["col_on"] += 1
            except Exception:
                pass

    if verbose:
        print(f"[DIAG128_DYNAMIC_RESTORE] root={root_path}, stats={stats}")
    return stats

def should_script_attach(event, suction_pos, cube_pos):
    """
    흡착 ON 판정.

    20_ 핵심 수정:
    19_의 gap_top 방식은 큐브 윗면 기준으로는 직관적이지만,
    현재 사용자가 직접 맞춘 SUCTION_HOLD_OFFSET=-0.056과 맞지 않았다.

    지금 큐브가 화면상 맞게 붙는 조건은:
        cube_center ≈ suction_pos + SUCTION_HOLD_OFFSET
    이다.

    그래서 이제는 hold_error를 본다.
        hold_error_z = (suction_z + SUCTION_HOLD_OFFSET_Z) - cube_z

    hold_error_z가 0에 가까우면, 지금 흡착해도 큐브가 튀거나 끌려 올라가지 않는다.
    hold_error_z가 음수면 VGC10이 이미 너무 내려간 상태라 큐브를 뚫는 방향이다.
    hold_error_z가 양수면 아직 멀어서 일찍 잡는 방향이다.
    """
    suction_pos = np.array(suction_pos, dtype=float)
    cube_pos = np.array(cube_pos, dtype=float)

    cube_top_z = float(cube_pos[2] + CUBE_HALF_Z)
    gap_top = float(suction_pos[2] - cube_top_z)

    dz = float(suction_pos[2] - cube_pos[2])
    xy = float(np.linalg.norm(suction_pos[:2] - cube_pos[:2]))
    dist = float(np.linalg.norm(suction_pos - cube_pos))

    held_target_pos = suction_pos + SUCTION_HOLD_OFFSET
    hold_error_vec = held_target_pos - cube_pos
    hold_error_z = float(hold_error_vec[2])
    hold_error_xy = float(np.linalg.norm(hold_error_vec[:2]))
    hold_error_dist = float(np.linalg.norm(hold_error_vec))

    metric = (
        f"hold_z={hold_error_z:.3f},hold_xy={hold_error_xy:.3f},hold_dist={hold_error_dist:.3f},"
        f"gap_top={gap_top:.3f},dz={dz:.3f},xy={xy:.3f},dist={dist:.3f}"
    )

    if event in PICK_CLOSE_EVENTS:
        # 1차 흡착 조건:
        # hold_z가 0 근처일 때만 붙인다.
        # hold_z < 0이면 이미 너무 내려간 상태라 큐브를 뚫는 방향이므로 금지한다.
        # hold_z > 0이 너무 크면 아직 멀어서 일찍 붙는 느낌이 나므로 금지한다.
        ok = (-0.003 <= hold_error_z <= 0.006) and (hold_error_xy <= 0.05) and (hold_error_dist <= 0.012)
        return ok, metric

    if event in RETRY_CLOSE_EVENTS:
        # 재시도 조건: 1차에서 약간 놓쳤을 때만 허용한다.
        # 너무 빨리 붙으면 오른쪽 0.010을 0.006으로 낮추고,
        # 뚫으면 왼쪽 -0.006을 0.000 쪽으로 올린다.
        ok = (-0.006 <= hold_error_z <= 0.010) and (hold_error_xy <= 0.05) and (hold_error_dist <= 0.016)
        return ok, "retry_" + metric

    return False, "not_pick_event"

def _as_matrix4d(value):
    """USD Python 버전에 따라 GetLocalTransformation 반환 형태가 달라지는 것을 흡수한다."""
    if isinstance(value, tuple):
        value = value[0]
    try:
        return Gf.Matrix4d(value)
    except Exception:
        return Gf.Matrix4d(1.0)

def _get_or_create_scripted_transform_op(prim):
    """
    XformCommonAPI.SetTranslate가 실패하는 incompatible xformOp 스택을 피하기 위해
    강제로 단일 matrix transform op를 사용한다.

    로그에 아래 경고가 뜨면 이 방식이 필요하다.
      Could not determine xform ops for incompatible xformable
    """
    xformable = UsdGeom.Xformable(prim)
    attr_name = "xformOp:transform:scripted_carry"
    attr = prim.GetAttribute(attr_name)
    if attr and attr.IsValid():
        op = UsdGeom.XformOp(attr)
    else:
        try:
            op = xformable.AddTransformOp(UsdGeom.XformOp.PrecisionDouble, "scripted_carry")
        except Exception:
            # 혹시 같은 이름 attribute가 이미 있는데 op wrapper 생성만 실패한 경우 fallback
            attr = prim.GetAttribute(attr_name)
            if attr and attr.IsValid():
                op = UsdGeom.XformOp(attr)
            else:
                raise

    # 기존 translate/rotate/scale op order가 incompatible이면 XformCommonAPI가 못 다루므로
    # scripted_carry matrix op 하나만 compose되게 만든다.
    try:
        xformable.SetXformOpOrder([op])
    except Exception:
        xformable.ClearXformOpOrder()
        xformable.SetXformOpOrder([op])
    return op

def set_prim_world_translation(stage, prim_path, world_pos, _sync_oribox_child=True):
    """
    prim을 원하는 world translation으로 보낸다.

    15_ 핵심:
    - XformCommonAPI.SetTranslate를 쓰지 않는다.
    - 기존 local rotate/scale을 matrix로 보존한 뒤 translation만 바꾼다.
    - /World/OriBoxA 또는 Small_Cardboard_box에서 뜨던 incompatible xformable 경고를 우회한다.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"  [WARN] set_prim_world_translation 실패: prim 없음 {prim_path}")
        return False

    world_pos = np.array(world_pos, dtype=float)

    try:
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        parent = prim.GetParent()
        if parent and parent.IsValid():
            parent_world = cache.GetLocalToWorldTransform(parent)
            local_v = parent_world.GetInverse().Transform(
                Gf.Vec3d(float(world_pos[0]), float(world_pos[1]), float(world_pos[2]))
            )
        else:
            local_v = Gf.Vec3d(float(world_pos[0]), float(world_pos[1]), float(world_pos[2]))

        xformable = UsdGeom.Xformable(prim)
        local_mat = _as_matrix4d(xformable.GetLocalTransformation())
        # 기존 local rotation/scale은 보존하고 translate만 교체한다.
        local_mat.SetTranslateOnly(local_v)

        op = _get_or_create_scripted_transform_op(prim)
        op.Set(local_mat)

        # 68_ 핵심: OriBoxA_* root를 움직인 경우, 하위 Small_Cardboard_box의 world translate도
        # root와 같게 다시 맞춰 parent/child 중심 좌표가 벌어지지 않게 한다.
        try:
            if (
                _sync_oribox_child
                and bool(ORIBOX_KEEP_CHILD_CENTER_MATCHED_TO_ROOT)
                and any(str(prim_path).startswith(str(ORIBOX_STACK_PARENT_PATH).rstrip("/") + "/" + pref) for pref in ORIBOX_STACK_NAME_PREFIXES)
                and not str(prim_path).rstrip("/").endswith("/" + str(ORIBOX_STACK_BOX_MESH_NAME))
            ):
                child_path = str(prim_path).rstrip("/") + "/" + str(ORIBOX_STACK_BOX_MESH_NAME)
                child_prim = stage.GetPrimAtPath(child_path)
                if child_prim and child_prim.IsValid():
                    child_pos = get_world_translation(stage, child_path)
                    root_pos = np.array(world_pos, dtype=float)
                    child_delta = None if child_pos is None else float(np.linalg.norm(np.array(child_pos, dtype=float) - root_pos))
                    if child_delta is None or child_delta > float(ORIBOX_CENTER_MATCH_TOL):
                        set_prim_world_translation(stage, child_path, root_pos, _sync_oribox_child=False)
                        if bool(ORIBOX_CENTER_MATCH_LOG):
                            print(
                                f"  [ORIBOX_CENTER_SYNC] root={prim_path} child={child_path} "
                                f"delta_before={child_delta} -> child world translate matched to root"
                            )
        except Exception as sync_exc:
            print(f"  [WARN] ORIBOX_CENTER_SYNC 실패: {prim_path} / {sync_exc}")

        return True
    except Exception as exc:
        print(f"  [WARN] set_prim_world_translation matrix 실패: {prim_path} / {exc}")
        return False

def sync_oribox_child_centers_with_roots(stage, roots=None, verbose=True):
    """68_: USD에서 맞춘 OriBoxA_* root와 Small_Cardboard_box의 world translate 중심을 실행 중에도 맞춘다."""
    if not bool(ORIBOX_KEEP_CHILD_CENTER_MATCHED_TO_ROOT):
        return

    if roots is None:
        roots = []
        try:
            parent = stage.GetPrimAtPath(ORIBOX_STACK_PARENT_PATH)
            if parent and parent.IsValid():
                for child in parent.GetChildren():
                    if child.GetName().startswith(ORIBOX_STACK_NAME_PREFIXES):
                        roots.append(str(child.GetPath()))
        except Exception:
            roots = []

    for root_path in roots:
        try:
            root_path = str(root_path)
            box_path = root_path.rstrip("/") + "/" + str(ORIBOX_STACK_BOX_MESH_NAME)
            root_prim = stage.GetPrimAtPath(root_path)
            box_prim = stage.GetPrimAtPath(box_path)
            if not (root_prim and root_prim.IsValid() and box_prim and box_prim.IsValid()):
                continue
            root_t = get_world_translation(stage, root_path)
            box_t = get_world_translation(stage, box_path)
            if root_t is None or box_t is None:
                continue
            delta = float(np.linalg.norm(np.array(box_t, dtype=float) - np.array(root_t, dtype=float)))
            if delta > float(ORIBOX_CENTER_MATCH_TOL):
                set_prim_world_translation(stage, box_path, root_t, _sync_oribox_child=False)
                if verbose and bool(ORIBOX_CENTER_MATCH_LOG):
                    print(
                        f"  [ORIBOX_CENTER_SETUP_SYNC] {root_path}: "
                        f"root_t={np.array(root_t)}, child_t_before={np.array(box_t)}, delta={delta:.6f} -> child=root"
                    )
            elif verbose and bool(ORIBOX_CENTER_MATCH_LOG):
                print(f"  [ORIBOX_CENTER_SETUP_OK] {root_path}: root/child world translate delta={delta:.6f}")
        except Exception as exc:
            print(f"  [WARN] ORIBOX_CENTER_SETUP_SYNC 실패: {root_path} / {exc}")

def set_prim_kinematic(stage, prim_path, enabled=True):
    """
    scripted suction 중에는 박스를 kinematic으로 만들어 물리 엔진과 싸우지 않게 한다.
    release 때 False로 돌린다.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return

    try:
        UsdPhysics.RigidBodyAPI.Apply(prim)
    except Exception:
        pass

    attr = prim.GetAttribute("physics:kinematicEnabled")
    if not attr:
        attr = prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, custom=False)
    try:
        attr.Set(bool(enabled))
    except Exception:
        pass

def zero_prim_velocity(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    for attr_name in ["physics:velocity", "physics:angularVelocity"]:
        try:
            attr = prim.GetAttribute(attr_name)
            if attr:
                attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
        except Exception:
            pass

def zero_subtree_velocity(stage, root_path):
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return 0
    count = 0
    for prim in Usd.PrimRange(root):
        for attr_name in ["physics:velocity", "physics:angularVelocity"]:
            try:
                attr = prim.GetAttribute(attr_name)
                if attr:
                    attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
                    count += 1
            except Exception:
                pass
    return count

def set_box_scripted_carry_mode(stage, root_path, enabled=True, reenable_physics=False, verbose=True):
    """
    15_ 핵심.
    VGC10은 visual-only라 물리 충돌로 박스를 밀 수 없다.
    scripted suction으로 박스를 들어올릴 때는 박스 subtree의 물리 rigid/collision을 잠시 끄고
    parent Xform을 직접 움직여야 허우적거림과 관통 느낌이 줄어든다.

    enabled=True  : carry 중. rigidBodyEnabled=False, collisionEnabled=False, kinematic=True, velocity=0
    enabled=False : release 후. 기본은 reenable_physics=False라 목표 위치에 안정적으로 고정한다.
                    dynamic으로 되돌리고 싶으면 BOX_REENABLE_PHYSICS_AFTER_RELEASE=True.
    """
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        if verbose:
            print(f"  [CARRY_MODE_FAIL] root not found: {root_path}")
        return {"rigid": 0, "collision": 0, "kinematic": 0, "velocity": 0}

    stats = {"rigid": 0, "collision": 0, "kinematic": 0, "velocity": 0}
    for prim in Usd.PrimRange(root):
        # 속도 제거
        for attr_name in ["physics:velocity", "physics:angularVelocity"]:
            try:
                attr = prim.GetAttribute(attr_name)
                if attr:
                    attr.Set(Gf.Vec3f(0.0, 0.0, 0.0))
                    stats["velocity"] += 1
            except Exception:
                pass

        # rigid body 제어
        try:
            has_rb = prim.HasAPI(UsdPhysics.RigidBodyAPI) or prim.HasAttribute("physics:rigidBodyEnabled") or prim.HasAttribute("physics:kinematicEnabled")
            if has_rb:
                UsdPhysics.RigidBodyAPI.Apply(prim)
                kin_attr = prim.GetAttribute("physics:kinematicEnabled")
                if not kin_attr:
                    kin_attr = prim.CreateAttribute("physics:kinematicEnabled", Sdf.ValueTypeNames.Bool, custom=False)
                kin_attr.Set(True if enabled else (not reenable_physics))
                stats["kinematic"] += 1

                rb_attr = prim.GetAttribute("physics:rigidBodyEnabled")
                if rb_attr:
                    rb_attr.Set(False if enabled else bool(reenable_physics))
                    stats["rigid"] += 1
        except Exception:
            pass

        # collision 제어
        try:
            has_col = prim.HasAPI(UsdPhysics.CollisionAPI) or prim.HasAttribute("physics:collisionEnabled")
            if has_col:
                col_api = UsdPhysics.CollisionAPI.Apply(prim)
                col_attr = prim.GetAttribute("physics:collisionEnabled")
                if not col_attr:
                    col_attr = col_api.CreateCollisionEnabledAttr()
                col_attr.Set(False if enabled else bool(reenable_physics))
                stats["collision"] += 1
        except Exception:
            pass

    if verbose:
        print(f"  [CARRY_MODE] enabled={enabled}, reenable_physics={reenable_physics}, root={root_path}, stats={stats}")
    return stats

__all__ = [
    '_sdf_path',
    '_gf_vec3_from_np',
    '_find_physics_body0_for_attach',
    'create_physics_attach_joint',
    'release_physics_attach_joint',
    'force_box_dynamic_after_release',
    'should_script_attach',
    '_as_matrix4d',
    '_get_or_create_scripted_transform_op',
    'set_prim_world_translation',
    'sync_oribox_child_centers_with_roots',
    'set_prim_kinematic',
    'zero_prim_velocity',
    'zero_subtree_velocity',
    'set_box_scripted_carry_mode'
]
