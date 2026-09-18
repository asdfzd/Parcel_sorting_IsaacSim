import ast
import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
M0609_DIR = PROJECT_ROOT / "Collected_Conveyor_lift_test_01" / "M0609"
if str(M0609_DIR) not in sys.path:
    sys.path.insert(0, str(M0609_DIR))

from palletizing.config import (  # noqa: E402
    A_DUAL_CONFIG,
    A_STANDALONE_CONFIG,
    B_DUAL_CONFIG,
    B_STANDALONE_CONFIG,
    CONFIGS,
)


class RobotCellConfigTest(unittest.TestCase):
    def test_profiles_are_complete_and_unique(self):
        self.assertEqual(len(CONFIGS), 4)
        for config in CONFIGS.values():
            self.assertTrue(config.robot_prim_path.startswith("/World/m0609_"))
            self.assertTrue(config.first_box_path.startswith("/World/OriBox"))
            self.assertTrue(config.slot_marker_paths)
            self.assertGreater(config.stack_slot_count, 0)

    def test_a_b_paths_are_isolated(self):
        self.assertNotEqual(A_DUAL_CONFIG.robot_prim_path, B_DUAL_CONFIG.robot_prim_path)
        self.assertNotEqual(A_DUAL_CONFIG.first_box_path, B_DUAL_CONFIG.first_box_path)
        self.assertNotEqual(A_DUAL_CONFIG.pallet_path, B_DUAL_CONFIG.pallet_path)
        self.assertNotEqual(A_DUAL_CONFIG.vgc10_root_path, B_DUAL_CONFIG.vgc10_root_path)
        self.assertNotEqual(
            A_DUAL_CONFIG.physics_attach_joint_path,
            B_DUAL_CONFIG.physics_attach_joint_path,
        )

    def test_standalone_specific_behaviors_are_preserved(self):
        self.assertTrue(A_STANDALONE_CONFIG.fused_yaw_enabled)
        self.assertTrue(B_STANDALONE_CONFIG.marker_fallback_final_move)
        self.assertTrue(B_STANDALONE_CONFIG.pallet_lower_during_home)
        self.assertEqual(B_STANDALONE_CONFIG.slot_marker_paths[0:2], B_STANDALONE_CONFIG.slot_marker_paths[2:4])

    def test_dual_profiles_preserve_two_slot_behavior(self):
        self.assertEqual(A_DUAL_CONFIG.stack_slot_count, 2)
        self.assertEqual(B_DUAL_CONFIG.stack_slot_count, 2)
        self.assertFalse(A_DUAL_CONFIG.lower_support_after_each_layer)
        self.assertFalse(B_DUAL_CONFIG.lower_support_after_each_layer)
        self.assertTrue(A_DUAL_CONFIG.forklift_enabled)
        self.assertFalse(B_DUAL_CONFIG.forklift_enabled)

    def test_entrypoints_construct_simulation_app_before_palletizing_import(self):
        for filename in (
            "robot_a_palletizing_forklift.py",
            "robot_b_palletizing_forklift.py",
            "run_ab_dual_robot_ros_gate.py",
        ):
            source = (M0609_DIR / filename).read_text(encoding="utf-8")
            app_index = source.index("SimulationApp(")
            package_index = source.index("from palletizing")
            self.assertLess(app_index, package_index, filename)

    def test_dual_wrapper_has_no_embedded_source_execution(self):
        path = M0609_DIR / "run_ab_dual_robot_ros_gate.py"
        source = path.read_text(encoding="utf-8")
        tree = ast.parse(source)
        self.assertNotIn("_CELL_A_SOURCE", source)
        self.assertNotIn("_CELL_B_SOURCE", source)
        forbidden_calls = {
            node.func.id
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
        }
        self.assertNotIn("exec", forbidden_calls)
        self.assertNotIn("compile", forbidden_calls)
        reset_calls = [
            node
            for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "reset"
            and isinstance(node.func.value, ast.Name)
            and node.func.value.id == "shared_world"
        ]
        self.assertEqual(len(reset_calls), 1)

    def test_dual_gate_and_box_schedule_are_preserved(self):
        path = M0609_DIR / "run_ab_dual_robot_ros_gate.py"
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        constants = {}
        box_sequence = None
        for node in tree.body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1:
                target = node.targets[0]
                if isinstance(target, ast.Name):
                    try:
                        constants[target.id] = ast.literal_eval(node.value)
                    except (ValueError, TypeError):
                        pass
            if isinstance(node, ast.ClassDef) and node.name == "BoxPhysicsSequencer":
                for child in node.body:
                    if (
                        isinstance(child, ast.Assign)
                        and len(child.targets) == 1
                        and isinstance(child.targets[0], ast.Name)
                        and child.targets[0].id == "sequence"
                    ):
                        box_sequence = ast.literal_eval(child.value)

        self.assertEqual(constants["ZONE_FILE"], "/tmp/zone_command.txt")
        self.assertEqual(constants["ZONE_DELAY"], {"A": 4, "B": 6})
        self.assertEqual(constants["GATE_DOWN_DURATION"], {"A": 5, "B": 8})
        self.assertEqual(
            box_sequence,
            (
                (1.0, "/World/OriBoxA_01"),
                (7.0, "/World/OriBoxB_01"),
                (13.0, "/World/OriBoxA_02"),
                (19.0, "/World/OriBoxB_02"),
            ),
        )

    def test_common_module_import_graph_is_acyclic(self):
        package_dir = M0609_DIR / "palletizing"
        module_names = {path.stem for path in package_dir.glob("*.py")}
        graph = {name: set() for name in module_names}

        for path in package_dir.glob("*.py"):
            tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
            for node in ast.walk(tree):
                if not isinstance(node, ast.ImportFrom) or node.level != 1:
                    continue
                if node.module and node.module in module_names:
                    graph[path.stem].add(node.module)
                elif node.module is None:
                    graph[path.stem].update(
                        alias.name for alias in node.names if alias.name in module_names
                    )

        visiting = set()
        visited = set()

        def visit(module_name):
            if module_name in visiting:
                self.fail(f"circular import detected at {module_name}: {graph}")
            if module_name in visited:
                return
            visiting.add(module_name)
            for dependency in graph[module_name]:
                visit(dependency)
            visiting.remove(module_name)
            visited.add(module_name)

        for module_name in graph:
            visit(module_name)


if __name__ == "__main__":
    unittest.main()
