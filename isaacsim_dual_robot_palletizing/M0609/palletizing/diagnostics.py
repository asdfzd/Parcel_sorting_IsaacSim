"""Runtime diagnostics and non-mutating pose/yaw inspection.

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

_PHYSICS_JOINT_DIAG_STATE = {}


def _fmt_vec(v, prec=4):
    try:
        v = np.array(v, dtype=float)
        return "(" + ",".join([f"{x:.{prec}f}" for x in v[:3]]) + ")"
    except Exception:
        return "(nan,nan,nan)"

def _safe_attr_value(prim, attr_name, default="<missing>"):
    try:
        attr = prim.GetAttribute(attr_name)
        if attr and attr.IsValid():
            v = attr.Get()
            return v if v is not None else "<None>"
    except Exception:
        pass
    return default

def _has_api_name(prim, api_cls):
    try:
        return bool(prim.HasAPI(api_cls))
    except Exception:
        return False

def _physics_diag_inspect_body(stage, path, label):
    prim = stage.GetPrimAtPath(str(path))
    print(f"[PHYSICS_DIAG_BODY][{label}] path={path}, valid={bool(prim and prim.IsValid())}")
    if not prim or not prim.IsValid():
        return
    pos = get_world_translation(stage, str(path))
    bbox = get_world_bbox_info(stage, str(path))
    print(
        f"[PHYSICS_DIAG_BODY][{label}] world_pos={_fmt_vec(pos)}, "
        f"RigidBodyAPI={_has_api_name(prim, UsdPhysics.RigidBodyAPI)}, "
        f"CollisionAPI={_has_api_name(prim, UsdPhysics.CollisionAPI)}, "
        f"MassAPI={_has_api_name(prim, UsdPhysics.MassAPI)}, "
        f"physics:rigidBodyEnabled={_safe_attr_value(prim, 'physics:rigidBodyEnabled')}, "
        f"physics:kinematicEnabled={_safe_attr_value(prim, 'physics:kinematicEnabled')}, "
        f"physics:collisionEnabled={_safe_attr_value(prim, 'physics:collisionEnabled')}"
    )
    print(
        f"[PHYSICS_DIAG_MASS][{label}] "
        f"mass={_safe_attr_value(prim, 'physics:mass')}, "
        f"density={_safe_attr_value(prim, 'physics:density')}, "
        f"centerOfMass={_safe_attr_value(prim, 'physics:centerOfMass')}, "
        f"diagonalInertia={_safe_attr_value(prim, 'physics:diagonalInertia')}, "
        f"principalAxes={_safe_attr_value(prim, 'physics:principalAxes')}"
    )
    if bbox is not None:
        print(
            f"[PHYSICS_DIAG_BBOX][{label}] center={_fmt_vec(bbox['center'])}, top={_fmt_vec(bbox['top_center'])}, "
            f"min={_fmt_vec(bbox['min'])}, max={_fmt_vec(bbox['max'])}, size={_fmt_vec(bbox['size'])}, height={bbox['height']:.4f}"
        )

def _physics_diag_inspect_colliders(stage, root_path, label="box", max_rows=60):
    root = stage.GetPrimAtPath(str(root_path))
    if not root or not root.IsValid():
        print(f"[PHYSICS_DIAG_COLLIDERS][{label}] root invalid: {root_path}")
        return
    rows = []
    for prim in Usd.PrimRange(root):
        try:
            has_col = prim.HasAPI(UsdPhysics.CollisionAPI) or bool(prim.GetAttribute("physics:collisionEnabled"))
            is_geom = _prim_is_geometry_like(prim)
            has_mesh_col = prim.HasAPI(UsdPhysics.MeshCollisionAPI) or bool(prim.GetAttribute("physics:approximation"))
            if has_col or is_geom or has_mesh_col:
                rows.append((str(prim.GetPath()), prim.GetTypeName(), has_col, has_mesh_col, _safe_attr_value(prim, "physics:collisionEnabled"), _safe_attr_value(prim, "physics:approximation")))
        except Exception as e:
            rows.append((str(prim.GetPath()), "<err>", False, False, f"err={e}", "<err>"))
    print(f"[PHYSICS_DIAG_COLLIDERS][{label}] count={len(rows)}, root={root_path}")
    for i, row in enumerate(rows[:max_rows]):
        p, typ, has_col, has_mesh_col, col_enabled, approx = row
        print(f"  [COLLIDER_ROW {i:02d}] path={p}, type={typ}, CollisionAPI={has_col}, MeshCollisionAPI={has_mesh_col}, enabled={col_enabled}, approximation={approx}")
    if len(rows) > max_rows:
        print(f"  [COLLIDER_ROW] ... truncated {len(rows)-max_rows} more rows")

def _create_or_set_attr(api_or_prim, attr_name, value):
    """UsdPhysics API attr를 생성/갱신한다. Isaac/USD 버전별 Create*Attr 차이를 피하기 위한 작은 helper."""
    try:
        # api_or_prim이 API 객체면 GetPrim이 있고, prim이면 그대로 쓴다.
        prim = api_or_prim.GetPrim() if hasattr(api_or_prim, "GetPrim") else api_or_prim
        attr = prim.GetAttribute(attr_name)
        if not attr or not attr.IsValid():
            attr = prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Float)
        attr.Set(value)
        return True
    except Exception:
        return False

def _physics_diag_apply_mass_inertia_104(stage, box_path):
    """104_: 원본 USD 저장 없이 현재 stage에서만 박스 MassAPI/inertia를 명시한다."""
    if not bool(globals().get("PHYSICS_MASS_INERTIA_DIAG_ENABLED", True)):
        print("[PHYSICS_MASS_INERTIA_104] disabled")
        return False

    prim = stage.GetPrimAtPath(str(box_path))
    if not prim or not prim.IsValid():
        print(f"[PHYSICS_MASS_INERTIA_104] box invalid: {box_path}")
        return False

    bbox = get_world_bbox_info(stage, str(box_path))
    root = get_world_translation(stage, str(box_path))
    if bbox is None or root is None:
        print(f"[PHYSICS_MASS_INERTIA_104] bbox/root unavailable: {box_path}")
        return False

    size = np.array(bbox.get("size", [0.26, 0.20, 0.24]), dtype=float)
    size = np.maximum(size, np.array([0.01, 0.01, 0.01], dtype=float))
    mass = float(globals().get("PHYSICS_MASS_INERTIA_DIAG_MASS", 2.0))
    scale = float(globals().get("PHYSICS_MASS_INERTIA_DIAG_INERTIA_SCALE", 1.0))

    # root 기준 local COM. 현재 OriBox root가 bbox min z에 가까우므로 보통 (0,0,0.12) 근처가 된다.
    try:
        com_local = _world_to_local_point(stage, str(box_path), np.array(bbox["center"], dtype=float))
    except Exception:
        com_local = np.array([0.0, 0.0, float(size[2]) * 0.5], dtype=float)

    hx, hy, hz = float(size[0]), float(size[1]), float(size[2])
    ixx = scale * (mass / 12.0) * (hy * hy + hz * hz)
    iyy = scale * (mass / 12.0) * (hx * hx + hz * hz)
    izz = scale * (mass / 12.0) * (hx * hx + hy * hy)

    print("\n========== [PHYSICS_MASS_INERTIA_APPLY_BEGIN_104] ==========")
    print(
        f"[PHYSICS_MASS_INERTIA_104][before] path={box_path}, "
        f"MassAPI={_has_api_name(prim, UsdPhysics.MassAPI)}, "
        f"mass={_safe_attr_value(prim, 'physics:mass')}, "
        f"centerOfMass={_safe_attr_value(prim, 'physics:centerOfMass')}, "
        f"diagonalInertia={_safe_attr_value(prim, 'physics:diagonalInertia')}, "
        f"principalAxes={_safe_attr_value(prim, 'physics:principalAxes')}"
    )
    print(
        f"[PHYSICS_MASS_INERTIA_104][computed] bbox_size={_fmt_vec(size)}, root={_fmt_vec(root)}, "
        f"bbox_center={_fmt_vec(bbox['center'])}, com_local={_fmt_vec(com_local)}, "
        f"mass={mass:.4f}, inertia=({ixx:.6f},{iyy:.6f},{izz:.6f}), scale={scale:.3f}"
    )

    try:
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        try:
            mass_api.CreateMassAttr(float(mass)).Set(float(mass))
        except Exception:
            prim.CreateAttribute("physics:mass", Sdf.ValueTypeNames.Float).Set(float(mass))
        try:
            mass_api.CreateCenterOfMassAttr(Gf.Vec3f(float(com_local[0]), float(com_local[1]), float(com_local[2]))).Set(Gf.Vec3f(float(com_local[0]), float(com_local[1]), float(com_local[2])))
        except Exception:
            prim.CreateAttribute("physics:centerOfMass", Sdf.ValueTypeNames.Vector3f).Set(Gf.Vec3f(float(com_local[0]), float(com_local[1]), float(com_local[2])))
        try:
            mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(float(ixx), float(iyy), float(izz))).Set(Gf.Vec3f(float(ixx), float(iyy), float(izz)))
        except Exception:
            prim.CreateAttribute("physics:diagonalInertia", Sdf.ValueTypeNames.Vector3f).Set(Gf.Vec3f(float(ixx), float(iyy), float(izz)))
        try:
            mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1.0)).Set(Gf.Quatf(1.0))
        except Exception:
            prim.CreateAttribute("physics:principalAxes", Sdf.ValueTypeNames.Quatf).Set(Gf.Quatf(1.0))
    except Exception as e:
        print(f"[PHYSICS_MASS_INERTIA_104][ERROR] apply failed: {e}")
        print("========== [PHYSICS_MASS_INERTIA_APPLY_FAIL_104] ==========\n")
        return False

    print(
        f"[PHYSICS_MASS_INERTIA_104][after] path={box_path}, "
        f"MassAPI={_has_api_name(prim, UsdPhysics.MassAPI)}, "
        f"mass={_safe_attr_value(prim, 'physics:mass')}, "
        f"centerOfMass={_safe_attr_value(prim, 'physics:centerOfMass')}, "
        f"diagonalInertia={_safe_attr_value(prim, 'physics:diagonalInertia')}, "
        f"principalAxes={_safe_attr_value(prim, 'physics:principalAxes')}"
    )
    print("========== [PHYSICS_MASS_INERTIA_APPLY_END_104] ==========\n")
    return True

def _physics_diag_rot3_from_world_matrix_105(stage, prim_path):
    mat = get_world_matrix(stage, prim_path)
    if mat is None:
        return np.eye(3, dtype=float)
    cols = []
    for axis in [(1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)]:
        try:
            v = mat.TransformDir(Gf.Vec3d(float(axis[0]), float(axis[1]), float(axis[2])))
            cols.append(np.array([float(v[0]), float(v[1]), float(v[2])], dtype=float))
        except Exception:
            cols.append(np.array(axis, dtype=float))
    R = np.column_stack(cols)
    try:
        # scale/shear가 섞여도 회전 행렬만 뽑기 위해 직교화한다.
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1.0
            R = U @ Vt
    except Exception:
        R = np.eye(3, dtype=float)
    return R

def _physics_diag_quatf_from_rot3_105(R):
    R = np.array(R, dtype=float)
    try:
        tr = float(R[0, 0] + R[1, 1] + R[2, 2])
        if tr > 0.0:
            S = (tr + 1.0) ** 0.5 * 2.0
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = (1.0 + R[0, 0] - R[1, 1] - R[2, 2]) ** 0.5 * 2.0
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = (1.0 + R[1, 1] - R[0, 0] - R[2, 2]) ** 0.5 * 2.0
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = (1.0 + R[2, 2] - R[0, 0] - R[1, 1]) ** 0.5 * 2.0
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S
        n = max((w*w + x*x + y*y + z*z) ** 0.5, 1.0e-12)
        w, x, y, z = w/n, x/n, y/n, z/n
        return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))
    except Exception:
        return Gf.Quatf(1.0)

def _physics_diag_rot_desc_105(R):
    R = np.array(R, dtype=float)
    return (
        f"x={_fmt_vec(R[:,0])}, y={_fmt_vec(R[:,1])}, z={_fmt_vec(R[:,2])}, "
        f"det={float(np.linalg.det(R)):.6f}"
    )

def _physics_diag_compute_joint_local_rots_105(stage, body0, body1):
    R0 = _physics_diag_rot3_from_world_matrix_105(stage, body0)
    R1 = _physics_diag_rot3_from_world_matrix_105(stage, body1)
    # box의 현재 방향을 joint world frame으로 채택한다.
    # 그러면 body1 localRot1은 거의 identity가 되고, box의 현재 up 방향을 보존하는 쪽으로 constraint가 생성된다.
    Rj = R1.copy()
    L0 = R0.T @ Rj
    L1 = R1.T @ Rj
    W0 = R0 @ L0
    W1 = R1 @ L1
    rot_err = float(np.linalg.norm(W0 - W1))
    q0 = _physics_diag_quatf_from_rot3_105(L0)
    q1 = _physics_diag_quatf_from_rot3_105(L1)
    print("[PHYSICS_DIAG_LOCALROT_105] strategy=preserve_body1_box_world_orientation")
    print(f"[PHYSICS_DIAG_ROT_WORLD_105][body0] {_physics_diag_rot_desc_105(R0)}")
    print(f"[PHYSICS_DIAG_ROT_WORLD_105][body1] {_physics_diag_rot_desc_105(R1)}")
    print(f"[PHYSICS_DIAG_ROT_LOCAL_105][localRot0] {_physics_diag_rot_desc_105(L0)}")
    print(f"[PHYSICS_DIAG_ROT_LOCAL_105][localRot1] {_physics_diag_rot_desc_105(L1)}")
    print(f"[PHYSICS_DIAG_ROT_CHECK_105] world_rot_err={rot_err:.9f}, body1_up_z_before={float(R1[2,2]):.6f}")
    return {"q0": q0, "q1": q1, "R0": R0, "R1": R1, "L0": L0, "L1": L1, "rot_err": rot_err}

def _physics_diag_body_up_z_105(stage, prim_path):
    R = _physics_diag_rot3_from_world_matrix_105(stage, prim_path)
    return float(R[2, 2]), R

def _physics_diag_capture_pose(stage, body0, body1, local0, local1, attach_world, label="capture"):
    bbox = get_world_bbox_info(stage, body1)
    root = get_world_translation(stage, body1)
    anchor0 = _local_to_world_point(stage, body0, local0)
    anchor1 = _local_to_world_point(stage, body1, local1)
    suction = get_world_translation(stage, VGC10_SUCTION_POINT_PATH)
    body1_up_z, _body1_R = _physics_diag_body_up_z_105(stage, body1)
    body0_up_z, _body0_R = _physics_diag_body_up_z_105(stage, body0)
    root_above_bbox = None
    try:
        if root is not None and bbox is not None:
            root_above_bbox = float(root[2] - bbox["center"][2])
    except Exception:
        root_above_bbox = None
    return {
        "label": label,
        "body0": str(body0),
        "body1": str(body1),
        "local0": np.array(local0, dtype=float),
        "local1": np.array(local1, dtype=float),
        "attach_world": np.array(attach_world, dtype=float),
        "anchor0": anchor0,
        "anchor1": anchor1,
        "anchor_err": float(np.linalg.norm(anchor0 - anchor1)),
        "anchor0_attach_err": float(np.linalg.norm(anchor0 - np.array(attach_world, dtype=float))),
        "anchor1_attach_err": float(np.linalg.norm(anchor1 - np.array(attach_world, dtype=float))),
        "root": root,
        "bbox_center": bbox["center"] if bbox is not None else None,
        "bbox_top": bbox["top_center"] if bbox is not None else None,
        "bbox_size": bbox["size"] if bbox is not None else None,
        "suction": suction,
        "body0_up_z": body0_up_z,
        "body1_up_z": body1_up_z,
        "root_minus_bbox_center_z": root_above_bbox,
    }

def _physics_diag_print_capture(prefix, cap, base=None):
    if not cap:
        print(f"[PHYSICS_DIAG]{prefix} <no capture>")
        return
    root_delta = bbox_delta = top_delta = suction_delta = None
    if base:
        try:
            if cap.get("root") is not None and base.get("root") is not None:
                root_delta = float(np.linalg.norm(np.array(cap["root"]) - np.array(base["root"])))
            if cap.get("bbox_center") is not None and base.get("bbox_center") is not None:
                bbox_delta = float(np.linalg.norm(np.array(cap["bbox_center"]) - np.array(base["bbox_center"])))
            if cap.get("bbox_top") is not None and base.get("bbox_top") is not None:
                top_delta = float(np.linalg.norm(np.array(cap["bbox_top"]) - np.array(base["bbox_top"])))
            if cap.get("suction") is not None and base.get("suction") is not None:
                suction_delta = float(np.linalg.norm(np.array(cap["suction"]) - np.array(base["suction"])))
        except Exception:
            pass
    print(
        f"[PHYSICS_DIAG]{prefix} "
        f"attach={_fmt_vec(cap.get('attach_world'))}, anchor0={_fmt_vec(cap.get('anchor0'))}, anchor1={_fmt_vec(cap.get('anchor1'))}, "
        f"anchor_err={cap.get('anchor_err', float('nan')):.6f}, "
        f"a0_attach_err={cap.get('anchor0_attach_err', float('nan')):.6f}, a1_attach_err={cap.get('anchor1_attach_err', float('nan')):.6f}, "
        f"root={_fmt_vec(cap.get('root'))}, bbox_center={_fmt_vec(cap.get('bbox_center'))}, bbox_top={_fmt_vec(cap.get('bbox_top'))}, "
        f"suction={_fmt_vec(cap.get('suction'))}, "
        f"body1_up_z={cap.get('body1_up_z', float('nan')):.6f}, "
        f"root_minus_bbox_z={cap.get('root_minus_bbox_center_z') if cap.get('root_minus_bbox_center_z') is not None else float('nan'):.6f}, "
        f"d_root={root_delta if root_delta is not None else -1:.6f}, "
        f"d_bbox={bbox_delta if bbox_delta is not None else -1:.6f}, "
        f"d_top={top_delta if top_delta is not None else -1:.6f}, "
        f"d_suction={suction_delta if suction_delta is not None else -1:.6f}"
    )

def physics_diag_log_after_attach_step(stage, phase_name, step, force=False):
    global _PHYSICS_JOINT_DIAG_STATE
    st = _PHYSICS_JOINT_DIAG_STATE or {}
    if not st.get("active"):
        return
    sample_steps = set(globals().get("PHYSICS_DIAGNOSTIC_SAMPLE_STEPS", {0, 1, 2, 5, 10}))
    every_until = int(globals().get("PHYSICS_DIAGNOSTIC_LOG_EVERY_STEP_UNTIL", 5))
    if not force and not (int(step) <= every_until or int(step) in sample_steps):
        return
    cap = _physics_diag_capture_pose(stage, st["body0"], st["body1"], st["local0"], st["local1"], st["attach_world"], label=f"{phase_name}:{step}")
    _physics_diag_print_capture(f"[after_attach phase={phase_name} step={step}]", cap, base=st.get("before"))

def _diag_bool_attr_128(prim, attr_name):
    try:
        attr = prim.GetAttribute(attr_name)
        if attr:
            return attr.Get()
    except Exception:
        pass
    return None

def _diag_set_bool_attr_128(prim, attr_name, value):
    try:
        attr = prim.GetAttribute(attr_name)
        if not attr:
            attr = prim.CreateAttribute(attr_name, Sdf.ValueTypeNames.Bool, custom=False)
        attr.Set(bool(value))
        return True
    except Exception:
        return False

def _diag_set_vec3_attr_128(prim, attr_name, value):
    try:
        attr = prim.GetAttribute(attr_name)
        if attr:
            attr.Set(Gf.Vec3f(float(value[0]), float(value[1]), float(value[2])))
            return True
    except Exception:
        pass
    return False

def _diag_has_attr_128(prim, *names):
    for name in names:
        try:
            if prim.HasAttribute(name) or prim.GetAttribute(name):
                return True
        except Exception:
            pass
    return False

def _diag_attr_names_with_128(prim, text):
    out = []
    try:
        low = str(text).lower()
        for attr in prim.GetAttributes():
            name = attr.GetName()
            if low in name.lower():
                out.append(name)
    except Exception:
        pass
    return out

def _diag_rel_targets_128(prim, rel_name):
    try:
        rel = prim.GetRelationship(rel_name)
        if rel:
            return [str(t) for t in rel.GetTargets()]
    except Exception:
        pass
    return []

def scan_stage_joints(stage, box_root_path, label="scan"):
    """Stage 전체 joint를 짧게 스캔해서 box/root/vgc10/m0609와 연결된 joint가 남았는지 확인."""
    if not bool(globals().get("RELEASE_DIAG_SCAN_ALL_JOINTS_128", True)):
        return []
    box_root_path = str(box_root_path)
    rows = []
    try:
        for prim in stage.Traverse():
            try:
                typ = str(prim.GetTypeName())
                path = str(prim.GetPath())
                has_body_rel = bool(prim.GetRelationship("physics:body0")) or bool(prim.GetRelationship("physics:body1"))
                if ("Joint" not in typ) and ("joint" not in path.lower()) and (not has_body_rel):
                    continue
                b0 = _diag_rel_targets_128(prim, "physics:body0")
                b1 = _diag_rel_targets_128(prim, "physics:body1")
                joined = " ".join([path, typ] + b0 + b1)
                relevant = (
                    box_root_path in joined
                    or "/OriBox" in joined
                    or "vgc10" in joined.lower()
                    or ACTIVE_ROBOT_ROOT_PATH in joined
                    or path == str(PHYSICS_ATTACH_JOINT_PATH)
                )
                if relevant:
                    rows.append((path, typ, b0, b1))
            except Exception:
                pass
    except Exception as exc:
        print(f"[DIAG128_JOINT_SCAN_WARN][{label}] {type(exc).__name__}: {exc}")
        return []

    print(f"[DIAG128_JOINT_SCAN][{label}] relevant_joint_count={len(rows)}")
    max_rows = int(globals().get("RELEASE_DIAG_MAX_ROWS_128", 24))
    for i, (path, typ, b0, b1) in enumerate(rows[:max_rows]):
        print(f"  [J{i:02d}] path={path}, type={typ}, body0={b0}, body1={b1}")
    if len(rows) > max_rows:
        print(f"  [J...] truncated={len(rows) - max_rows}")
    return rows

def compact_box_physics_state(stage, root_path, label="state"):
    """상자 root/subtree의 physics 상태를 핵심만 출력."""
    root_path = str(root_path)
    root = stage.GetPrimAtPath(root_path)
    print(f"[DIAG128_BODY_STATE][{label}] root={root_path}, valid={bool(root and root.IsValid())}")
    if not root or not root.IsValid():
        return []

    bbox = get_world_bbox_info(stage, root_path)
    if bbox is not None:
        c = np.array(bbox["center"], dtype=float)
        mn = np.array(bbox["min"], dtype=float)
        mx = np.array(bbox["max"], dtype=float)
        print(
            f"[DIAG128_BBOX][{label}] center=({c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f}), "
            f"bottom_z={mn[2]:+.4f}, top_z={mx[2]:+.4f}, height={float(mx[2]-mn[2]):.4f}"
        )

    rows = []
    for prim in Usd.PrimRange(root):
        try:
            path = str(prim.GetPath())
            is_root = path == root_path
            rb_api = prim.HasAPI(UsdPhysics.RigidBodyAPI)
            col_api = prim.HasAPI(UsdPhysics.CollisionAPI)
            mass_api = prim.HasAPI(UsdPhysics.MassAPI)
            has_physics_attr = any(str(a.GetName()).startswith("physics:") or str(a.GetName()).startswith("physx") for a in prim.GetAttributes())
            if not (is_root or rb_api or col_api or mass_api or has_physics_attr):
                continue
            gravity_attrs = _diag_attr_names_with_128(prim, "gravity")
            rows.append({
                "path": path,
                "type": str(prim.GetTypeName()),
                "rb_api": rb_api,
                "col_api": col_api,
                "mass_api": mass_api,
                "rb_enabled": _diag_bool_attr_128(prim, "physics:rigidBodyEnabled"),
                "kinematic": _diag_bool_attr_128(prim, "physics:kinematicEnabled"),
                "collision": _diag_bool_attr_128(prim, "physics:collisionEnabled"),
                "velocity": _safe_attr_value(prim, "physics:velocity"),
                "angular_velocity": _safe_attr_value(prim, "physics:angularVelocity"),
                "gravity_attrs": {name: _safe_attr_value(prim, name) for name in gravity_attrs[:6]},
            })
        except Exception:
            pass

    print(f"[DIAG128_BODY_ROWS][{label}] rows={len(rows)}")
    max_rows = int(globals().get("RELEASE_DIAG_MAX_ROWS_128", 24))
    for i, r in enumerate(rows[:max_rows]):
        print(
            f"  [B{i:02d}] path={r['path']}, type={r['type']}, "
            f"RBAPI={r['rb_api']}, rbEnabled={r['rb_enabled']}, kin={r['kinematic']}, "
            f"COLAPI={r['col_api']}, colEnabled={r['collision']}, gravity={r['gravity_attrs']}"
        )
    if len(rows) > max_rows:
        print(f"  [B...] truncated={len(rows) - max_rows}")
    return rows

def compact_drop_observe(stage, root_path, label="observe"):
    bbox = get_world_bbox_info(stage, str(root_path))
    root_t = get_world_translation(stage, str(root_path))
    if bbox is None:
        print(f"[DIAG128_DROP_OBS][{label}] bbox=None, root_t={_fmt_vec(root_t)}")
        return
    c = np.array(bbox["center"], dtype=float)
    mn = np.array(bbox["min"], dtype=float)
    mx = np.array(bbox["max"], dtype=float)
    print(
        f"[DIAG128_DROP_OBS][{label}] root={_fmt_vec(root_t)}, "
        f"center=({c[0]:+.4f},{c[1]:+.4f},{c[2]:+.4f}), bottom_z={mn[2]:+.4f}, top_z={mx[2]:+.4f}"
    )

def log_oribox_pose_tracker(stage, label="pose", force=False):
    """
    64_ 디버그 전용.
    OriBoxA_02 / OriBoxA_03의 root world translate와 실제 Small_Cardboard_box bbox center/top을 계속 찍는다.
    - root_translate: 사용자가 말한 Prim의 Translate가 world에서 어디로 해석되는지 확인
    - bbox_center/top: 실제 박스 mesh가 어디에 있는지 확인
    - d_root/d_center: 직전 로그 대비 이동량. 갑자기 커지면 순간이동/재배치 발생 지점
    """
    if not bool(POSE_TRACK_ENABLED):
        return
    try:
        _POSE_TRACK_STATE["step"] = int(_POSE_TRACK_STATE.get("step", 0)) + 1
        step = int(_POSE_TRACK_STATE["step"])
        if (not force) and (not bool(POSE_TRACK_LOG_EVERY_STEP)) and (step % int(POSE_TRACK_LOG_INTERVAL) != 0):
            return

        prev = _POSE_TRACK_STATE.setdefault("prev", {})
        parts = []
        for root_path in POSE_TRACK_ROOT_PATHS:
            root_prim = stage.GetPrimAtPath(root_path)
            if not root_prim.IsValid():
                parts.append(f"{root_path}:MISSING")
                continue

            # 72_: 추적 기준도 root이다. child는 실제로 root에 붙어 따라오는지 확인용으로만 별도 출력한다.
            box_path = root_path
            child_path = find_named_child_or_descendant(stage, root_path, POSE_TRACK_BOX_CHILD_NAME)
            root_t = get_world_translation(stage, root_path)
            box_t = get_world_translation(stage, box_path)
            child_t = get_world_translation(stage, child_path) if child_path else None
            bbox = get_world_bbox_info(stage, root_path)

            root_t = np.array(root_t if root_t is not None else [np.nan, np.nan, np.nan], dtype=float)
            box_t = np.array(box_t if box_t is not None else [np.nan, np.nan, np.nan], dtype=float)
            if bbox is not None:
                center = np.array(bbox["center"], dtype=float)
                top = np.array(bbox["top_center"], dtype=float)
            else:
                center = np.array([np.nan, np.nan, np.nan], dtype=float)
                top = np.array([np.nan, np.nan, np.nan], dtype=float)

            old = prev.get(root_path)
            d_root = 0.0
            d_center = 0.0
            jump = ""
            if old is not None:
                try:
                    d_root = float(np.linalg.norm(root_t - old["root_t"]))
                    d_center = float(np.linalg.norm(center - old["center"]))
                    if max(d_root, d_center) >= float(POSE_TRACK_JUMP_WARN_TOL):
                        jump = f" JUMP>= {POSE_TRACK_JUMP_WARN_TOL:.3f}"
                except Exception:
                    pass

            prev[root_path] = {"root_t": root_t.copy(), "center": center.copy(), "box_t": box_t.copy(), "top": top.copy()}
            child_txt = ""
            try:
                if child_t is not None:
                    child_t_arr = np.array(child_t, dtype=float)
                    child_txt = f" child=({child_t_arr[0]:+.4f},{child_t_arr[1]:+.4f},{child_t_arr[2]:+.4f})"
            except Exception:
                child_txt = ""
            parts.append(
                f"{root_path}: root=({root_t[0]:+.4f},{root_t[1]:+.4f},{root_t[2]:+.4f}) "
                f"used_xform=({box_t[0]:+.4f},{box_t[1]:+.4f},{box_t[2]:+.4f}) "
                f"bbox_center=({center[0]:+.4f},{center[1]:+.4f},{center[2]:+.4f}) "
                f"bbox_top=({top[0]:+.4f},{top[1]:+.4f},{top[2]:+.4f}) "
                f"d_root={d_root:.4f} d_center={d_center:.4f}{jump}{child_txt}"
            )

        print(f"[POSE_TRACK][{label}][step={step}] " + " | ".join(parts))
    except Exception as exc:
        print(f"[POSE_TRACK_WARN][{label}] {exc}")

def _get_world_yaw_z_159(stage, prim_path):
    """prim의 local +X축이 world XY에서 바라보는 yaw(rad)를 반환한다."""
    try:
        prim = stage.GetPrimAtPath(str(prim_path))
        if not prim or not prim.IsValid():
            return None
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        mat = cache.GetLocalToWorldTransform(prim)
        x_axis = mat.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
        return float(math.atan2(float(x_axis[1]), float(x_axis[0])))
    except Exception:
        return None

def get_slot_marker_path_for_slot(slot_index):
    paths = tuple(globals().get("SLOT_MARKER_PATHS_155", ()))
    if not paths:
        return None
    idx = int(slot_index) % len(paths)
    return str(paths[idx])

def check_box_yaw_against_slot_marker(stage, box_root_path, slot_index, label="before_release"):
    """160_: release 직전 yaw를 강제로 바꾸지 않고, marker와 차이만 로그로 남긴다."""
    marker_path = get_slot_marker_path_for_slot(slot_index)
    if not marker_path:
        print(f"[YAW_CHECK_160][WARN] no marker path. box={box_root_path}, slot_index={slot_index}, label={label}")
        return None
    target_yaw = _get_world_yaw_z_159(stage, marker_path)
    current_yaw = _get_world_yaw_z_159(stage, box_root_path)
    if target_yaw is None or current_yaw is None:
        print(f"[YAW_CHECK_160][WARN] yaw read failed. box={box_root_path}, marker={marker_path}, label={label}")
        return None
    delta = float((target_yaw - current_yaw + math.pi) % (2.0 * math.pi) - math.pi)
    ok = abs(math.degrees(delta)) <= 5.0
    print(
        f"[YAW_CHECK_160] label={label}, box={box_root_path}, marker={marker_path}, "
        f"box_yaw={math.degrees(current_yaw):+.1f}deg, marker_yaw={math.degrees(target_yaw):+.1f}deg, "
        f"yaw_err={math.degrees(delta):+.1f}deg, ok={ok}, snap_disabled=True"
    )
    return delta

def yaw_diagnostic_against_slot(stage, box_root_path, slot_index, label="pre_pick"):
    """163_: 상자를 돌리지 않고 현재 box yaw와 APalt_slot yaw 차이만 기록한다."""
    if not bool(globals().get("PRE_PICK_YAW_DIAGNOSTIC_ENABLED_163", True)):
        return None
    marker_path = get_slot_marker_path_for_slot(slot_index)
    if not marker_path:
        print(f"[PRE_PICK_YAW_163][WARN] no marker path. box={box_root_path}, slot_index={slot_index}, label={label}")
        return None
    box_yaw = _get_world_yaw_z_159(stage, box_root_path)
    marker_yaw = _get_world_yaw_z_159(stage, marker_path)
    if box_yaw is None or marker_yaw is None:
        print(f"[PRE_PICK_YAW_163][WARN] yaw read failed. box={box_root_path}, marker={marker_path}, label={label}")
        return None
    yaw_err = float((marker_yaw - box_yaw + math.pi) % (2.0 * math.pi) - math.pi)
    yaw_err_deg = float(math.degrees(yaw_err))
    abs_err = abs(yaw_err_deg)
    ok_tol = float(globals().get("PRE_PICK_YAW_OK_TOL_DEG_163", 5.0))
    warn_tol = float(globals().get("PRE_PICK_YAW_WARN_TOL_DEG_163", 15.0))
    status = "OK" if abs_err <= ok_tol else ("WARN" if abs_err <= warn_tol else "BAD")
    print(
        f"[PRE_PICK_YAW_163] label={label}, box={box_root_path}, marker={marker_path}, "
        f"box_yaw={math.degrees(box_yaw):+.1f}deg, marker_yaw={math.degrees(marker_yaw):+.1f}deg, "
        f"yaw_err={yaw_err_deg:+.1f}deg, status={status}, "
        f"post_pick_yaw_correction_disabled={bool(globals().get('POST_PICK_YAW_CORRECTION_DISABLED_163', True))}"
    )
    return yaw_err

def _wrap_deg_164(deg):
    """[-180, 180) 범위로 각도 정규화."""
    try:
        return float((float(deg) + 180.0) % 360.0 - 180.0)
    except Exception:
        return 0.0

def _axis_equiv_err_deg_164(deg):
    """직사각형의 축 방향만 볼 때 180도 동일 방향으로 취급한 최소 오차."""
    d = abs(_wrap_deg_164(deg))
    return float(min(d, abs(180.0 - d)))

def _get_world_axis_yaws_z_164(stage, prim_path):
    """prim local X/Y축이 world XY에서 바라보는 yaw(deg)를 함께 반환한다."""
    try:
        prim = stage.GetPrimAtPath(str(prim_path))
        if not prim or not prim.IsValid():
            return None
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        mat = cache.GetLocalToWorldTransform(prim)
        t = mat.ExtractTranslation()
        x_axis = mat.TransformDir(Gf.Vec3d(1.0, 0.0, 0.0))
        y_axis = mat.TransformDir(Gf.Vec3d(0.0, 1.0, 0.0))
        z_axis = mat.TransformDir(Gf.Vec3d(0.0, 0.0, 1.0))
        x_yaw = math.degrees(math.atan2(float(x_axis[1]), float(x_axis[0])))
        y_yaw = math.degrees(math.atan2(float(y_axis[1]), float(y_axis[0])))
        return {
            "path": str(prim_path),
            "pos": (float(t[0]), float(t[1]), float(t[2])),
            "x_yaw": _wrap_deg_164(x_yaw),
            "y_yaw": _wrap_deg_164(y_yaw),
            "x_axis": (float(x_axis[0]), float(x_axis[1]), float(x_axis[2])),
            "y_axis": (float(y_axis[0]), float(y_axis[1]), float(y_axis[2])),
            "z_axis": (float(z_axis[0]), float(z_axis[1]), float(z_axis[2])),
        }
    except Exception:
        return None

def axis_yaw_diagnostic(stage, box_root_path, slot_index, label="axis_yaw"):
    """164_: 강제 보정 없이 box와 APalt_slot의 local X/Y축 방향을 모두 진단한다."""
    if not bool(globals().get("AXIS_YAW_DIAGNOSTIC_ENABLED_164", True)):
        return None
    marker_path = get_slot_marker_path_for_slot(slot_index)
    if not marker_path:
        print(f"[BOX_AXIS_YAW_164][WARN] no marker path. label={label}, box={box_root_path}, slot_index={slot_index}")
        return None
    b = _get_world_axis_yaws_z_164(stage, box_root_path)
    m = _get_world_axis_yaws_z_164(stage, marker_path)
    if b is None or m is None:
        print(f"[BOX_AXIS_YAW_164][WARN] axis read failed. label={label}, box={box_root_path}, marker={marker_path}")
        return None

    err_xx = _wrap_deg_164(m["x_yaw"] - b["x_yaw"])
    err_xy = _wrap_deg_164(m["x_yaw"] - b["y_yaw"])
    err_yx = _wrap_deg_164(m["y_yaw"] - b["x_yaw"])
    err_yy = _wrap_deg_164(m["y_yaw"] - b["y_yaw"])

    axis_candidates = {
        "boxX_to_slotX": _axis_equiv_err_deg_164(err_xx),
        "boxY_to_slotX": _axis_equiv_err_deg_164(err_xy),
        "boxX_to_slotY": _axis_equiv_err_deg_164(err_yx),
        "boxY_to_slotY": _axis_equiv_err_deg_164(err_yy),
    }
    best_axis = min(axis_candidates, key=axis_candidates.get)
    best_axis_err = float(axis_candidates[best_axis])

    bbox_txt = ""
    if bool(globals().get("AXIS_YAW_DIAG_LOG_BBOX_164", True)):
        try:
            bb = get_world_bbox_info(stage, box_root_path)
            if bb is not None:
                c = bb.get("center")
                s = bb.get("size")
                bbox_txt = (
                    f", bbox_center=({float(c[0]):+.4f},{float(c[1]):+.4f},{float(c[2]):+.4f})"
                    f", bbox_size=({float(s[0]):.4f},{float(s[1]):.4f},{float(s[2]):.4f})"
                )
        except Exception:
            bbox_txt = ""

    print(
        f"[BOX_AXIS_YAW_164] label={label}, slot_index={slot_index}, "
        f"box={box_root_path}, marker={marker_path}, "
        f"box_x_yaw={b['x_yaw']:+.1f}deg, box_y_yaw={b['y_yaw']:+.1f}deg, "
        f"slot_x_yaw={m['x_yaw']:+.1f}deg, slot_y_yaw={m['y_yaw']:+.1f}deg, "
        f"direct_errs: slotX-boxX={err_xx:+.1f}, slotX-boxY={err_xy:+.1f}, "
        f"slotY-boxX={err_yx:+.1f}, slotY-boxY={err_yy:+.1f}, "
        f"axis180_best={best_axis}:{best_axis_err:.1f}deg"
        f"{bbox_txt}"
    )
    print(
        f"[BOX_AXIS_VECTOR_164] label={label}, "
        f"boxX=({b['x_axis'][0]:+.4f},{b['x_axis'][1]:+.4f},{b['x_axis'][2]:+.4f}), "
        f"boxY=({b['y_axis'][0]:+.4f},{b['y_axis'][1]:+.4f},{b['y_axis'][2]:+.4f}), "
        f"slotX=({m['x_axis'][0]:+.4f},{m['x_axis'][1]:+.4f},{m['x_axis'][2]:+.4f}), "
        f"slotY=({m['y_axis'][0]:+.4f},{m['y_axis'][1]:+.4f},{m['y_axis'][2]:+.4f})"
    )
    return {
        "box": b,
        "marker": m,
        "direct_errs": {"slotX-boxX": err_xx, "slotX-boxY": err_xy, "slotY-boxX": err_yx, "slotY-boxY": err_yy},
        "axis180_best": best_axis,
        "axis180_best_err": best_axis_err,
    }

def _vec3_np_170(v):
    try:
        return np.array([float(v[0]), float(v[1]), float(v[2])], dtype=float)
    except Exception:
        return np.zeros(3, dtype=float)

def _angle_between_deg_170(a, b):
    """두 3D 벡터의 각도(deg). 진단용이며 어떤 pose도 수정하지 않는다."""
    try:
        av = _vec3_np_170(a)
        bv = _vec3_np_170(b)
        an = float(np.linalg.norm(av))
        bn = float(np.linalg.norm(bv))
        if an < 1.0e-9 or bn < 1.0e-9:
            return None
        c = float(np.clip(np.dot(av, bv) / (an * bn), -1.0, 1.0))
        return float(math.degrees(math.acos(c)))
    except Exception:
        return None

def diagnose_pallet_release_pose(stage, box_root_path, slot_index, label="before_release"):
    """
    170_: release 직전 진단 전용.
    - 상자/slot transform을 절대 수정하지 않는다.
    - joint/RMPFlow/surface 상태를 절대 변경하지 않는다.
    - APalt_slot을 정답지로 보고 현재 box 중심/수평/축 방향 차이만 출력한다.
    """
    if not bool(globals().get("APALT_RELEASE_POSE_DIAG_ONLY_170", True)):
        return None

    marker_path = get_slot_marker_path_for_slot(slot_index)
    if not marker_path:
        print(f"[APALT_RELEASE_POSE_DIAG_170][WARN] no marker path. label={label}, box={box_root_path}, slot_index={slot_index}")
        return None

    b = _get_world_axis_yaws_z_164(stage, box_root_path)
    m = _get_world_axis_yaws_z_164(stage, marker_path)
    if b is None or m is None:
        print(f"[APALT_RELEASE_POSE_DIAG_170][WARN] axis read failed. label={label}, box={box_root_path}, marker={marker_path}")
        return None

    bb = None
    mb = None
    try:
        bb = get_world_bbox_info(stage, box_root_path)
    except Exception:
        bb = None
    try:
        mb = get_world_bbox_info(stage, marker_path)
    except Exception:
        mb = None

    box_center = _vec3_np_170(bb.get("center")) if isinstance(bb, dict) and bb.get("center") is not None else _vec3_np_170(b.get("pos"))
    slot_center = _vec3_np_170(mb.get("center")) if isinstance(mb, dict) and mb.get("center") is not None else _vec3_np_170(m.get("pos"))
    center_xy_err = float(np.linalg.norm(box_center[:2] - slot_center[:2]))
    center_z_err = float(box_center[2] - slot_center[2])

    # 방향 후보는 기존 axis_yaw_diagnostic와 같은 방식으로 계산한다.
    err_xx = _wrap_deg_164(m["x_yaw"] - b["x_yaw"])
    err_xy = _wrap_deg_164(m["x_yaw"] - b["y_yaw"])
    err_yx = _wrap_deg_164(m["y_yaw"] - b["x_yaw"])
    err_yy = _wrap_deg_164(m["y_yaw"] - b["y_yaw"])
    axis_candidates = {
        "boxX_to_slotX": _axis_equiv_err_deg_164(err_xx),
        "boxY_to_slotX": _axis_equiv_err_deg_164(err_xy),
        "boxX_to_slotY": _axis_equiv_err_deg_164(err_yx),
        "boxY_to_slotY": _axis_equiv_err_deg_164(err_yy),
    }
    axis_signed_errors = {
        "boxX_to_slotX": float(err_xx),
        "boxY_to_slotX": float(err_xy),
        "boxX_to_slotY": float(err_yx),
        "boxY_to_slotY": float(err_yy),
    }
    best_axis = min(axis_candidates, key=axis_candidates.get)
    best_axis_err = float(axis_candidates[best_axis])
    best_axis_signed_err = float(axis_signed_errors.get(best_axis, 0.0))
    if abs(best_axis_signed_err) > 90.0:
        # 축은 180도 동치로 보므로, 실제 보정 명령도 ±90도 안쪽의 가장 가까운 방향으로 접는다.
        best_axis_signed_err = _wrap_deg_164(best_axis_signed_err + (180.0 if best_axis_signed_err < 0 else -180.0))

    box_z = _vec3_np_170(b.get("z_axis"))
    slot_z = _vec3_np_170(m.get("z_axis"))
    world_up = np.array([0.0, 0.0, 1.0], dtype=float)
    box_to_slot_z_deg = _angle_between_deg_170(box_z, slot_z)
    box_to_world_up_deg = _angle_between_deg_170(box_z, world_up)
    slot_to_world_up_deg = _angle_between_deg_170(slot_z, world_up)

    # 높이/크기 로그: 기울면 bbox height가 커지는 경향이 있어 같이 기록한다.
    bbox_size_txt = ""
    try:
        if isinstance(bb, dict) and bb.get("size") is not None:
            s = bb.get("size")
            bbox_size_txt = f", box_bbox_size=({float(s[0]):.4f},{float(s[1]):.4f},{float(s[2]):.4f})"
    except Exception:
        bbox_size_txt = ""

    ok_center = center_xy_err <= float(globals().get("APALT_RELEASE_CENTER_OK_TOL_170", 0.010))
    ok_axis = best_axis_err <= float(globals().get("APALT_RELEASE_AXIS_OK_TOL_DEG_170", 5.0))
    ok_level = (box_to_slot_z_deg is not None) and (box_to_slot_z_deg <= float(globals().get("APALT_RELEASE_LEVEL_OK_TOL_DEG_170", 5.0)))

    print(
        f"[APALT_RELEASE_POSE_DIAG_170] label={label}, box={box_root_path}, marker={marker_path}, "
        f"center_xy_err={center_xy_err:.4f}, center_z_err={center_z_err:+.4f}, "
        f"best_axis={best_axis}:{best_axis_err:.1f}deg, signed_yaw_cmd={best_axis_signed_err:+.1f}deg, "
        f"box_to_slot_z={box_to_slot_z_deg if box_to_slot_z_deg is not None else float('nan'):.2f}deg, "
        f"box_to_world_up={box_to_world_up_deg if box_to_world_up_deg is not None else float('nan'):.2f}deg, "
        f"slot_to_world_up={slot_to_world_up_deg if slot_to_world_up_deg is not None else float('nan'):.2f}deg, "
        f"ok_center={ok_center}, ok_axis={ok_axis}, ok_level={ok_level}, "
        f"NO_MOVE=True, NO_RMPFLOW_ALIGN=True, NO_JOINT_TRIM=True, NO_SNAP=True{bbox_size_txt}"
    )
    print(
        f"[APALT_RELEASE_AXIS_CANDIDATES_170] label={label}, "
        f"slotX-boxX={err_xx:+.1f}/{axis_candidates['boxX_to_slotX']:.1f}, "
        f"slotX-boxY={err_xy:+.1f}/{axis_candidates['boxY_to_slotX']:.1f}, "
        f"slotY-boxX={err_yx:+.1f}/{axis_candidates['boxX_to_slotY']:.1f}, "
        f"slotY-boxY={err_yy:+.1f}/{axis_candidates['boxY_to_slotY']:.1f}"
    )
    return {
        "center_xy_err": center_xy_err,
        "center_z_err": center_z_err,
        "best_axis": best_axis,
        "best_axis_err": best_axis_err,
        "best_axis_signed_err": best_axis_signed_err,
        "box_to_slot_z_deg": box_to_slot_z_deg,
        "box_to_world_up_deg": box_to_world_up_deg,
        "slot_to_world_up_deg": slot_to_world_up_deg,
        "ok_center": ok_center,
        "ok_axis": ok_axis,
        "ok_level": ok_level,
    }

def should_stop_after_release_count(release_count):
    try:
        return bool(globals().get("DIAG_STOP_AFTER_RELEASE_ENABLED_164", True)) and int(release_count) >= int(globals().get("DIAG_STOP_AFTER_RELEASE_COUNT_164", 2))
    except Exception:
        return False

def _vec_mag(value):
    if value is None:
        return None
    try:
        arr = np.array([float(value[0]), float(value[1]), float(value[2])], dtype=float)
        return float(np.linalg.norm(arr))
    except Exception:
        return None

def get_prim_velocity_magnitudes(stage, prim_path):
    """
    USD/PhysX 속성에서 linear/angular velocity 크기를 읽는다.
    런타임에서 attr이 비어 있으면 None을 반환하고, 이 경우 bbox 이동량으로 정지 여부를 판단한다.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return None, None

    linear_mag = None
    angular_mag = None
    try:
        attr = prim.GetAttribute("physics:velocity")
        if attr:
            linear_mag = _vec_mag(attr.Get())
    except Exception:
        linear_mag = None

    try:
        attr = prim.GetAttribute("physics:angularVelocity")
        if attr:
            angular_mag = _vec_mag(attr.Get())
    except Exception:
        angular_mag = None

    return linear_mag, angular_mag

__all__ = [
    '_fmt_vec',
    '_safe_attr_value',
    '_has_api_name',
    '_physics_diag_inspect_body',
    '_physics_diag_inspect_colliders',
    '_create_or_set_attr',
    '_physics_diag_apply_mass_inertia_104',
    '_physics_diag_rot3_from_world_matrix_105',
    '_physics_diag_quatf_from_rot3_105',
    '_physics_diag_rot_desc_105',
    '_physics_diag_compute_joint_local_rots_105',
    '_physics_diag_body_up_z_105',
    '_physics_diag_capture_pose',
    '_physics_diag_print_capture',
    'physics_diag_log_after_attach_step',
    '_diag_bool_attr_128',
    '_diag_set_bool_attr_128',
    '_diag_set_vec3_attr_128',
    '_diag_has_attr_128',
    '_diag_attr_names_with_128',
    '_diag_rel_targets_128',
    'scan_stage_joints',
    'compact_box_physics_state',
    'compact_drop_observe',
    'log_oribox_pose_tracker',
    '_get_world_yaw_z_159',
    'get_slot_marker_path_for_slot',
    'check_box_yaw_against_slot_marker',
    'yaw_diagnostic_against_slot',
    '_wrap_deg_164',
    '_axis_equiv_err_deg_164',
    '_get_world_axis_yaws_z_164',
    'axis_yaw_diagnostic',
    '_vec3_np_170',
    '_angle_between_deg_170',
    'diagnose_pallet_release_pose',
    'should_stop_after_release_count',
    '_vec_mag',
    'get_prim_velocity_magnitudes'
]
