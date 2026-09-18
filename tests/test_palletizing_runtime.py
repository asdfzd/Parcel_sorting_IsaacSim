"""Regression tests with real helpers and small Isaac import/World doubles.

These tests require NumPy; they do not simulate PhysX or verify robot motion.
"""

import contextlib
import importlib
import io
import sys
import types
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

M0609_DIR = Path(__file__).resolve().parents[1] / "isaacsim_dual_robot_palletizing" / "M0609"
sys.path.insert(0, str(M0609_DIR))


class FakeBaseTask:
    def __init__(self, name, offset=None):
        self.name = name

    def set_up_scene(self, scene):
        pass


class FakeManipulator:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class FakeScene:
    def __init__(self):
        self.objects = {}

    def add(self, obj):
        if obj.name in self.objects:
            raise ValueError(f"duplicate scene object: {obj.name}")
        self.objects[obj.name] = obj
        return obj

    def get_object(self, name):
        return self.objects.get(name)


class DeferredWorld:
    """Model only add_task registration and reset-time task scene setup."""

    def __init__(self):
        self.tasks = []
        self.scene = FakeScene()

    def add_task(self, task):
        self.tasks.append(task)

    def reset(self):
        for task in self.tasks:
            task.set_up_scene(self.scene)


def isaac_import_doubles():
    modules = {}

    def module(name, **attrs):
        result = modules.setdefault(name, types.ModuleType(name))
        result.__dict__.update(attrs)
        if "." in name:
            parent, child = name.rsplit(".", 1)
            setattr(module(parent), child, result)
        return result

    module("omni.usd")
    module("pxr", **{name: types.SimpleNamespace() for name in ("Gf", "Sdf", "Usd", "UsdGeom", "UsdPhysics")})
    module("isaacsim.core.api", World=DeferredWorld)
    module("isaacsim.core.api.materials.physics_material", PhysicsMaterial=object)
    module("isaacsim.core.api.objects", DynamicCuboid=object)
    module("isaacsim.core.api.tasks", BaseTask=FakeBaseTask)
    module("isaacsim.core.prims", SingleGeometryPrim=object)
    module("isaacsim.robot.manipulators.manipulators", SingleManipulator=FakeManipulator)
    module("isaacsim.core.utils.types", ArticulationAction=object)
    module("m0609_pick_place_controller", PickPlaceController=object)
    module("m0609_rmpflow_controller", RMPFlowController=object)
    return modules


class RuntimeRegressionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module_patch = patch.dict(sys.modules, isaac_import_doubles())
        cls.module_patch.start()
        cls.worker = importlib.import_module("palletizing.worker")
        cls.scene = importlib.import_module("palletizing.scene")
        cls.core = importlib.import_module("palletizing.palletizing")
        cls.config = importlib.import_module("palletizing.config")

    @classmethod
    def tearDownClass(cls):
        cls.module_patch.stop()

    def setUp(self):
        self.quiet = contextlib.redirect_stdout(io.StringIO())
        self.quiet.__enter__()

    def tearDown(self):
        self.quiet.__exit__(None, None, None)

    def make_workers(self):
        world = DeferredWorld()
        return world, [self.worker.create_worker(config, world, object()) for config in
                       (self.config.A_DUAL_CONFIG, self.config.B_DUAL_CONFIG)]

    def test_deferred_scene_setup_uses_each_cells_robot_and_box(self):
        world, workers = self.make_workers()
        for worker in workers:
            next(worker)
        core = self.core
        seen = []

        def setup_box(task, scene):
            seen.append((task.name, core.BOX_PRIM_PATH, core.STACK_SUPPORT_CUBE_PATH))

        with contextlib.ExitStack() as stack:
            for method in ("_load_usd", "_remove_old_gripper", "_attach_vgc10", "_attach_idle_vgc10_robots", "_setup_physics"):
                stack.enter_context(patch.object(core.M0609ConveyorBoxTask, method, lambda task: None))
            stack.enter_context(patch.object(core.M0609ConveyorBoxTask, "_discover_links", lambda task: setattr(task, "_ee_path", core.ROBOT_PRIM_PATH + "/link_6")))
            stack.enter_context(patch.object(core.M0609ConveyorBoxTask, "_setup_existing_box", setup_box))
            stack.enter_context(patch.object(core.M0609ConveyorBoxTask, "_setup_vgc10_suction_anchor", lambda task, scene: None))
            world.reset()
        self.assertEqual(set(world.scene.objects), {"m0609_robot_A", "m0609_robot_B"})
        for side, worker in zip("AB", workers):
            self.assertEqual(worker.robot.prim_path, f"/World/m0609_{side}/m0609")
        self.assertEqual(seen, [("A_m0609_conveyor_oribox_task", "/World/OriBoxA_01", "/World/APalt"),
                                ("B_m0609_conveyor_oribox_task", "/World/OriBoxB_01", "/World/BPalt")])

    def test_suction_result_visible_to_attach_in_same_worker_step(self):
        _, workers = self.make_workers()
        worker = workers[0]
        worker.activate()
        bbox = {"top_center": np.array([1.0, 2.0, 3.0]), "size": np.array([0.4, 0.4, 0.2])}
        with patch.object(self.scene, "get_world_translation", return_value=np.array([1.01, 2.0, 3.0])):
            ok, _, info = self.scene.evaluate_suction_grid_on_box_top(None, bbox)
        self.assertTrue(ok)
        self.assertIs(self.worker._LAST_SUCTION_GRID_INFO, info)
        np.testing.assert_allclose(info["attach_center"], [1.01, 2.0, 3.0 + self.scene.PHYSICS_ATTACH_TOP_SURFACE_EPS])
        self.assertIsNone(workers[1]._state["_LAST_SUCTION_GRID_INFO"]["attach_center"])

    def test_failed_suction_clears_previous_success_in_all_consumers(self):
        _, workers = self.make_workers()
        workers[0].activate()
        original = self.worker._LAST_SUCTION_GRID_INFO
        original["attach_center"] = np.array([1.0, 2.0, 3.0])
        ok, _, info = self.scene.evaluate_suction_grid_on_box_top(None, None)
        self.assertFalse(ok)
        self.assertIs(original, info)
        self.assertIsNone(original["attach_center"])

    def test_nested_world_callback_restores_callers_context_on_error(self):
        world, workers = self.make_workers()
        for worker in workers:
            next(worker)
        seen = []

        def callback(task, control_index, simulation_time):
            seen.append(self.core.ROBOT_PRIM_PATH)
            raise RuntimeError("callback failure")

        with workers[0].context():
            with patch.object(self.core.M0609ConveyorBoxTask, "pre_step", callback):
                with self.assertRaisesRegex(RuntimeError, "callback failure"):
                    world.tasks[1].pre_step(1, 0.1)
            self.assertEqual(self.core.ROBOT_PRIM_PATH, self.config.A_DUAL_CONFIG.robot_prim_path)
            self.assertIs(self.scene._LAST_SUCTION_GRID_INFO, workers[0]._state["_LAST_SUCTION_GRID_INFO"])
        self.assertEqual(seen, [self.config.B_DUAL_CONFIG.robot_prim_path])

    def test_resolved_robot_paths_reach_scene_helpers_before_setup_continues(self):
        world, workers = self.make_workers()
        for worker in workers:
            next(worker)

        def resolve(task):
            root = self.core.ACTIVE_ROBOT_ROOT_PATH
            self.core.ROBOT_PRIM_PATH = root + "/resolved"
            self.core.OLD_GRIPPER_PRIM_PATH = root + "/resolved/rg2"
            self.core.VGC10_FOLLOW_TARGET_PATH = root + "/resolved/tool"

        with patch.object(self.core.M0609ConveyorBoxTask, "_load_usd", resolve):
            for task, worker in zip(world.tasks, workers):
                with worker.context():
                    task._load_usd()
                    self.assertEqual(self.scene.ROBOT_PRIM_PATH, worker.config.robot_root_path + "/resolved")
                    self.assertEqual(self.scene.VGC10_FOLLOW_TARGET_PATH, worker.config.robot_root_path + "/resolved/tool")
        self.assertNotEqual(workers[0]._overrides["ROBOT_PRIM_PATH"], workers[1]._overrides["ROBOT_PRIM_PATH"])

    def test_suction_diagnostic_counter_is_per_worker(self):
        _, workers = self.make_workers()
        bbox = {"top_center": np.array([0.0, 0.0, 1.0]), "size": np.array([0.4, 0.4, 0.2])}
        with patch.object(self.scene, "get_world_translation", return_value=np.array([0.0, 0.0, 1.0])):
            for index in (0, 1, 0):
                with workers[index].context():
                    self.scene.evaluate_suction_grid_on_box_top(None, bbox)
        self.assertEqual([w._state["_SUCTION_GRID_EVALUATION_COUNT"] for w in workers], [2, 1])

    def test_standalone_closes_app_if_world_creation_fails(self):
        bootstrap = importlib.import_module("palletizing.bootstrap")
        from unittest.mock import Mock
        app = Mock()
        with patch.object(bootstrap, "configure_runtime_logging"), patch("isaacsim.core.api.World", side_effect=RuntimeError("world failed")):
            with self.assertRaisesRegex(RuntimeError, "world failed"):
                bootstrap.run_standalone(self.config.A_STANDALONE_CONFIG, app)
        app.close.assert_called_once_with()

    def test_b_pallet_lowering_follows_original_home_step_curve(self):
        from palletizing.forklift import PalletLoweringCoordinator
        positions = []
        coordinator = PalletLoweringCoordinator(
            enabled=True, use_home_steps=True, pallet_path="/World/BPalt",
            get_translation=lambda stage, path: np.array([1.0, 2.0, 3.0]),
            get_center=lambda stage: None,
            set_translation=lambda stage, path, pos: positions.append((path, pos.copy())),
            log_boxes=lambda *args, **kwargs: None,
            lower_steps=180, home_steps=160, extra_z=0.0, log_interval=30,
        )
        self.assertTrue(coordinator.schedule(None, 0.24, {}, "second_release"))
        for step in range(1, 161):
            self.assertTrue(coordinator.update(None))
            alpha = step / 160.0
            expected_z = 3.0 - 0.24 * alpha * alpha * (3.0 - 2.0 * alpha)
            self.assertAlmostEqual(positions[-1][1][2], expected_z)
            self.assertEqual(coordinator.active, step < 160)
        self.assertFalse(coordinator.update(None))
        self.assertTrue(all(path == "/World/BPalt" for path, _ in positions))

    def test_b_diagnostics_include_b_only_robot_joints(self):
        _, workers = self.make_workers()
        diagnostics = importlib.import_module("palletizing.diagnostics")
        relation = types.SimpleNamespace(GetTargets=lambda: ["/World/m0609_B/m0609/link_6"])
        prim = types.SimpleNamespace(GetTypeName=lambda: "RevoluteJoint",
                                     GetPath=lambda: "/World/arm_joint",
                                     GetRelationship=lambda name: relation)
        stage = types.SimpleNamespace(Traverse=lambda: [prim])
        with workers[1].context():
            rows = diagnostics.scan_stage_joints(stage, "/World/unrelated_box")
        self.assertEqual([row[0] for row in rows], ["/World/arm_joint"])


if __name__ == "__main__":
    unittest.main()
