"""Run Robot A and B against one SimulationApp, one World, and one reset."""

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})

# The camera graph needs the bridge before World and worker modules are imported.
try:
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()
    print("[ROS2_BRIDGE] isaacsim.ros2.bridge enabled at dual-wrapper level")
except Exception as ros_bridge_error:
    print(f"[ROS2_BRIDGE][WARN] failed to enable ROS2 bridge: {ros_bridge_error}")

import os
import time
import traceback

import omni.usd
from pxr import Gf, UsdGeom, UsdPhysics
from isaacsim.core.api import World

from palletizing.config import A_DUAL_CONFIG, B_DUAL_CONFIG
from palletizing.worker import create_worker


ZONE_FILE = "/tmp/zone_command.txt"
GATE_Z_DOWN = -0.05
GATE_Z_UP = 0.25
GATE_DOWN_DURATION = {"A": 5, "B": 8}
ZONE_DELAY = {"A": 4, "B": 6}
GATE_GROUPS = {"A": ("/World/Group",), "B": ("/World/Group_01",)}
GATE_FIXED_XY = {
    "/World/Group": (-3.85, 0.0),
    "/World/Group_01": (0.15, 0.0),
}

_last_zone = [None]
_gate_pending = []
_gate_last_poll_time = [None]


def set_gate_z(stage, path, z):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"[GATE][ERROR] Not found: {path}")
        return False
    x, y = GATE_FIXED_XY.get(path, (0.0, 0.0))
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(float(x), float(y), float(z)))
    print(f"[GATE] {path} -> ({x}, {y}, {z})")
    return True


def trigger_gate(zone: str):
    """Schedule the existing delayed down/up gate sequence for zone A or B."""
    zone = str(zone).strip().upper()
    if zone not in GATE_GROUPS:
        print(f"[GATE] Unknown zone: {zone}")
        return
    group_path = GATE_GROUPS[zone][0]
    if any(pending_path == group_path for _, pending_path, _ in _gate_pending):
        print(f"[GATE] SKIP — already pending: {group_path}")
        return
    delay = ZONE_DELAY.get(zone, 0)
    duration = GATE_DOWN_DURATION.get(zone, 2)
    _gate_pending.append((float(delay), group_path, "down"))
    _gate_pending.append((float(delay + duration), group_path, "up"))
    _gate_last_poll_time[0] = None
    print(f"[GATE] Zone {zone} -> {group_path} DOWN in {delay}s, UP in {delay + duration}s")


def poll_gate(stage):
    """Advance gate wall-clock timers only while the shared World is playing."""
    if not _gate_pending:
        _gate_last_poll_time[0] = None
        return
    now = time.time()
    if _gate_last_poll_time[0] is None:
        _gate_last_poll_time[0] = now
        return
    elapsed = now - _gate_last_poll_time[0]
    _gate_last_poll_time[0] = now
    remaining_actions = []
    for remaining, group_path, action in _gate_pending:
        remaining -= elapsed
        if remaining > 0:
            remaining_actions.append((remaining, group_path, action))
            continue
        target_z = GATE_Z_DOWN if action == "down" else GATE_Z_UP
        set_gate_z(stage, group_path, target_z)
        print(f"[GATE] {action.upper()}: {group_path}")
    _gate_pending[:] = remaining_actions


def poll_zone_file(stage):
    """Consume the Vision/ROS zone bridge command file once per command."""
    if not os.path.exists(ZONE_FILE):
        return
    try:
        with open(ZONE_FILE, "r", encoding="utf-8") as zone_file:
            zone = zone_file.read().strip().upper()
        if not zone:
            return
        try:
            os.remove(ZONE_FILE)
        except Exception:
            pass
        if zone != _last_zone[0]:
            _last_zone[0] = zone
            trigger_gate(zone)
            print(f"[ZONE] Received: {zone}")
        else:
            print(f"[ZONE] SKIP duplicate: {zone}")
    except Exception as error:
        print(f"[GATE][POLL_ERROR] {error}")


def reset_all_gates(stage):
    for zone_paths in GATE_GROUPS.values():
        for path in zone_paths:
            set_gate_z(stage, path, GATE_Z_UP)
    _gate_pending.clear()
    _gate_last_poll_time[0] = None
    _last_zone[0] = None
    print(f"[GATE] initialized open at Z={GATE_Z_UP}")


def _safe_next(label, worker, phase="step"):
    try:
        next(worker)
        return True
    except StopIteration:
        print(f"[DUAL][{label}][STOP] worker stopped during {phase}")
    except Exception as error:
        print(f"[DUAL][{label}][STEP_ERROR][{phase}] {type(error).__name__}: {error}")
        traceback.print_exc()
    return False


def _check_required_paths(stage):
    required = (
        "/World/m0609_A",
        "/World/m0609_B",
        "/World/APalt",
        "/World/BPalt",
        "/World/APalt_slot_01",
        "/World/APalt_slot_02",
        "/World/BPalt_slot_01",
        "/World/BPalt_slot_02",
        "/World/OriBoxA_01",
        "/World/OriBoxA_02",
        "/World/OriBoxB_01",
        "/World/OriBoxB_02",
    )
    for path in required:
        try:
            if not stage.GetPrimAtPath(path).IsValid():
                print(f"[DUAL][PATH_WARN] missing or invalid: {path}")
        except Exception as error:
            print(f"[DUAL][PATH_WARN] check failed: {path}: {error}")


def _force_robot_home_pose(label, robot):
    if robot is None:
        print(f"[STANDBY][{label}][WARN] robot object is None")
        return False
    try:
        import numpy as np

        try:
            robot.initialize()
        except Exception:
            pass
        dof = int(getattr(robot, "num_dof", 6))
        home = np.zeros(dof, dtype=float)
        robot.set_joint_positions(home)
        try:
            robot.set_joint_velocities(np.zeros(dof, dtype=float))
        except Exception:
            pass
        print(f"[STANDBY][{label}] home/initial zero pose forced. dof={dof}")
        return True
    except Exception as error:
        print(f"[STANDBY][{label}][WARN] home pose failed: {type(error).__name__}: {error}")
        traceback.print_exc()
        return False


class BoxPhysicsSequencer:
    """Preserve the dual wrapper's staggered A01/B01/A02/B02 release order."""

    sequence = (
        (1.0, "/World/OriBoxA_01"),
        (7.0, "/World/OriBoxB_01"),
        (13.0, "/World/OriBoxA_02"),
        (19.0, "/World/OriBoxB_02"),
    )

    def __init__(self, stage):
        self.stage = stage
        self.start_wall_time = None
        self.next_index = 0
        self.initialized = False
        self.enabled_paths = set()

    @staticmethod
    def _subtree(root_prim):
        stack = [root_prim]
        while stack:
            prim = stack.pop()
            yield prim
            try:
                stack.extend(list(prim.GetChildren()))
            except Exception:
                pass

    @staticmethod
    def _zero_velocity(prim):
        zero = Gf.Vec3f(0.0, 0.0, 0.0)
        for attribute_name in ("physics:velocity", "physics:angularVelocity"):
            try:
                attribute = prim.GetAttribute(attribute_name)
                if attribute:
                    attribute.Set(zero)
            except Exception:
                pass

    def _set_collision_enabled(self, root_prim, enabled):
        changed = 0
        for prim in self._subtree(root_prim):
            try:
                if not prim.HasAPI(UsdPhysics.CollisionAPI):
                    continue
                collision_api = UsdPhysics.CollisionAPI(prim)
                attribute = collision_api.GetCollisionEnabledAttr()
                if not attribute:
                    attribute = collision_api.CreateCollisionEnabledAttr(bool(enabled))
                attribute.Set(bool(enabled))
                changed += 1
            except Exception as error:
                print(f"[BOX_SEQUENCE][WARN] collision {prim.GetPath()}: {error}")
        return changed

    def _set_box_physics(self, root_path, enabled, reason):
        prim = self.stage.GetPrimAtPath(root_path)
        if not prim or not prim.IsValid():
            print(f"[BOX_SEQUENCE][WARN] missing box prim: {root_path}")
            return False
        try:
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
            attribute = rigid_body.GetRigidBodyEnabledAttr()
            if not attribute:
                attribute = rigid_body.CreateRigidBodyEnabledAttr(bool(enabled))
            attribute.Set(bool(enabled))
            kinematic = prim.GetAttribute("physics:kinematicEnabled")
            if kinematic:
                kinematic.Set(False)
            if not enabled:
                self._zero_velocity(prim)
            collision_count = self._set_collision_enabled(prim, enabled)
            if enabled:
                self.enabled_paths.add(root_path)
            else:
                self.enabled_paths.discard(root_path)
            print(
                f"[BOX_SEQUENCE] {'ON' if enabled else 'OFF'} {root_path} reason={reason}, "
                f"collision_subtree_count={collision_count}"
            )
            return True
        except Exception as error:
            print(f"[BOX_SEQUENCE][ERROR] {root_path}: {type(error).__name__}: {error}")
            traceback.print_exc()
            return False

    def force_all_off(self, reason="force_all_off"):
        print(f"[BOX_SEQUENCE] force all pre-placed boxes OFF reason={reason}")
        for path in dict.fromkeys(path for _, path in self.sequence):
            self._set_box_physics(path, False, reason)

    def initialize(self):
        self.force_all_off("sequence_initialize_all_off")
        self.start_wall_time = time.time()
        self.next_index = 0
        self.initialized = True
        print("[BOX_SEQUENCE] A01@1s -> B01@7s -> A02@13s -> B02@19s")

    def update(self):
        if not self.initialized:
            return
        elapsed = time.time() - float(self.start_wall_time)
        while self.next_index < len(self.sequence):
            delay_seconds, root_path = self.sequence[self.next_index]
            if elapsed < delay_seconds:
                break
            self._set_box_physics(root_path, True, f"scheduled_release_t={elapsed:.2f}s")
            self.next_index += 1


def main():
    shared_world = World(stage_units_in_meters=1.0)
    workers = [
        ["A", create_worker(A_DUAL_CONFIG, shared_world, simulation_app), True],
        ["B", create_worker(B_DUAL_CONFIG, shared_world, simulation_app), True],
    ]

    # Phase 1: register both tasks before the one master reset.
    for row in workers:
        print(f"[DUAL][{row[0]}] phase1 add_task")
        row[2] = _safe_next(row[0], row[1], phase="add_task")

    stage = omni.usd.get_context().get_stage()
    box_sequence = BoxPhysicsSequencer(stage)
    box_sequence.force_all_off("pre_shared_reset_usd_authored_physics_off")

    print("[DUAL] one shared_world.reset() after both tasks were added")
    try:
        shared_world.reset()
    except Exception as error:
        print(f"[DUAL][SHARED_RESET_ERROR] {type(error).__name__}: {error}")
        traceback.print_exc()
        simulation_app.close()
        return

    stage = omni.usd.get_context().get_stage()
    reset_all_gates(stage)
    box_sequence.force_all_off("post_shared_reset_usd_authored_physics_off")

    # Phase 2: each worker initializes its own robot/controllers after the reset.
    for row in workers:
        if row[2]:
            print(f"[DUAL][{row[0]}] phase2 init after shared reset")
            row[2] = _safe_next(row[0], row[1], phase="post_reset_init")

    try:
        for label, worker, alive in workers:
            if not alive:
                continue
            _force_robot_home_pose(label, worker.robot)
            ok = worker.ensure_vgc10(stage)
            print(f"[VGC10][{label}] root={worker.config.vgc10_root_path}, ok={ok}")
        for _ in range(10):
            for _, worker, alive in workers:
                if alive:
                    worker.update_vgc10()
            shared_world.step(render=True)
        print("[STANDBY] A/B home pose and VGC10 visuals ensured")
    except Exception as error:
        print(f"[STANDBY][WARN] initialization failed: {type(error).__name__}: {error}")
        traceback.print_exc()

    _check_required_paths(stage)
    box_sequence.force_all_off("post_worker_init_strict_all_off")
    box_sequence.initialize()

    print("[DUAL] loop: box schedule -> one World.step -> gates -> A/B workers")
    while simulation_app.is_running():
        try:
            box_sequence.update()
            shared_world.step(render=True)
        except Exception as error:
            print(f"[DUAL][WORLD_STEP_ERROR] {type(error).__name__}: {error}")
            traceback.print_exc()
            break

        try:
            if shared_world.is_playing():
                stage = omni.usd.get_context().get_stage()
                poll_zone_file(stage)
                poll_gate(stage)
            else:
                _gate_last_poll_time[0] = None
        except Exception as error:
            print(f"[GATE][STEP_WARN] {type(error).__name__}: {error}")
            traceback.print_exc()

        for row in workers:
            if row[2]:
                row[2] = _safe_next(row[0], row[1], phase="loop")
        time.sleep(0.01)

    print("[DUAL] closing shared SimulationApp")
    simulation_app.close()


if __name__ == "__main__":
    main()
