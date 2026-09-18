"""Side-effect-free configuration for standalone and shared-world robot cells."""

from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Tuple


Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class RobotCellConfig:
    """Values that genuinely differ by robot cell or execution profile."""

    profile_name: str
    side: str
    robot_root_path: str
    robot_object_name: str
    task_name: str
    pick_place_controller_name: str
    cartesian_controller_name: str
    box_prefix: str
    box_asset_candidates: Tuple[str, ...]
    pick_zone_path: str
    pick_zone_center: Vec3
    pick_zone_side_only: bool
    pallet_path: str
    slot_marker_paths: Tuple[str, ...]
    stack_joint1_degrees: Tuple[float, ...]
    conveyor_box_prefixes: Tuple[str, ...]
    pose_track_root_paths: Tuple[str, ...]
    robot_prop_name_prefixes: Tuple[str, ...]
    robot_prop_paths: Tuple[str, ...]
    vgc10_root_path: str = "/World/vgc10_visual_follow"
    suction_grid_world_path: str = "/World/vgc10_world_xy_suction_grid_debug"
    suction_body_path: str = "/World/scripted_suction_body"
    suction_marker_path: str = "/World/scripted_suction_direction_marker"
    suction_debug_marker_path: str = "/World/vgc10_suction_debug_marker"
    physics_attach_joint_path: str = "/World/vgc10_physics_attach_fixed_joint"
    enable_ros2_bridge: bool = False
    stack_layers: int = 2
    stack_slot_count: int = 4
    lower_support_after_each_layer: bool = True
    settle_before_pallet_lower: bool = True
    slot_route_sign_force: bool = True
    diagnostic_stop_release_count: int = 4
    diagnostic_skip_pallet_lower: bool = True
    forklift_enabled: bool = True
    forklift_trigger_count: int = 4
    fused_yaw_enabled: bool = False
    marker_fallback_final_move: bool = False
    pallet_lower_during_home: bool = False
    pallet_lower_use_home_steps: bool = True

    def __post_init__(self) -> None:
        side = self.side.upper()
        if side not in {"A", "B"}:
            raise ValueError(f"Unsupported robot side: {self.side!r}")
        if not self.robot_root_path.startswith("/World/"):
            raise ValueError("robot_root_path must be an absolute /World prim path")
        if not self.box_prefix or not self.slot_marker_paths:
            raise ValueError("box_prefix and slot_marker_paths are required")
        if self.stack_slot_count < 1 or self.forklift_trigger_count < 1:
            raise ValueError("stack/forklift counts must be positive")

    @property
    def robot_prim_path(self) -> str:
        return f"{self.robot_root_path}/m0609"

    @property
    def first_box_path(self) -> str:
        return f"/World/{self.box_prefix}_01"

    @property
    def pallet_name(self) -> str:
        return self.pallet_path.rsplit("/", 1)[-1]

    @property
    def vgc10_mount_path(self) -> str:
        return f"{self.vgc10_root_path}/vgc10_scaled_mount"

    @property
    def vgc10_model_path(self) -> str:
        return f"{self.vgc10_mount_path}/gripper_model"

    @property
    def vgc10_suction_point_path(self) -> str:
        return f"{self.vgc10_root_path}/vgc10_suction_point"

    @property
    def suction_grid_path(self) -> str:
        return f"{self.vgc10_root_path}/vgc10_suction_grid"


_A_PROPS = ("robotAprop", "RobotAprop", "robotaprop", "Robotaprop")
_A_PROP_PATHS = (
    "/World/robotAprop_01",
    "/World/RobotAprop_01",
    "/World/robotaprop_01",
    "/World/Robotaprop_01",
)
_B_PROPS = ("robotBprop", "RobotBprop", "robotbprop", "Robotbprop")
_B_PROP_PATHS = (
    "/World/robotBprop_01",
    "/World/RobotBprop_01",
    "/World/robotbprop_01",
    "/World/Robotbprop_01",
)


A_STANDALONE_CONFIG = RobotCellConfig(
    profile_name="A-standalone",
    side="A",
    robot_root_path="/World/m0609_A",
    robot_object_name="m0609_robot",
    task_name="m0609_conveyor_oribox_task",
    pick_place_controller_name="m0609_pick_place_controller",
    cartesian_controller_name="m0609_custom_carry_controller",
    box_prefix="OriBoxA",
    box_asset_candidates=("oriB/OriBoxB.usda", "oriA/OriBoxA.usda"),
    pick_zone_path="/World/pick_ready_zone_A",
    pick_zone_center=(-2.08983, -4.35, 0.88),
    pick_zone_side_only=True,
    pallet_path="/World/APalt",
    slot_marker_paths=("/World/APalt_slot_01", "/World/APalt_slot_02"),
    stack_joint1_degrees=(220.0, 155.0, 220.0, 155.0),
    conveyor_box_prefixes=("/World/OriBoxA_",),
    pose_track_root_paths=("/World/OriBoxA_01", "/World/OriBoxA_02"),
    robot_prop_name_prefixes=_A_PROPS,
    robot_prop_paths=_A_PROP_PATHS,
    fused_yaw_enabled=True,
)


B_STANDALONE_CONFIG = RobotCellConfig(
    profile_name="B-standalone",
    side="B",
    robot_root_path="/World/m0609_B",
    robot_object_name="m0609_robot",
    task_name="m0609_conveyor_oribox_task",
    pick_place_controller_name="m0609_pick_place_controller",
    cartesian_controller_name="m0609_custom_carry_controller",
    box_prefix="OriBoxB",
    box_asset_candidates=("oriB/OriBoxB.usda", "oriA/OriBoxB.usda"),
    pick_zone_path="/World/pick_ready_zone_B",
    pick_zone_center=(1.9599, -4.35, 0.88),
    pick_zone_side_only=False,
    pallet_path="/World/BPalt",
    slot_marker_paths=(
        "/World/BPalt_slot_01",
        "/World/BPalt_slot_02",
        "/World/BPalt_slot_01",
        "/World/BPalt_slot_02",
    ),
    stack_joint1_degrees=(220.0, 155.0, 220.0, 120.0),
    conveyor_box_prefixes=("/World/OriBoxB_",),
    pose_track_root_paths=("/World/OriBoxB_01", "/World/OriBoxB_02"),
    # Standalone B historically retained the A-prop fallback list; preserve it.
    robot_prop_name_prefixes=_A_PROPS,
    robot_prop_paths=_A_PROP_PATHS,
    diagnostic_stop_release_count=2,
    settle_before_pallet_lower=False,
    marker_fallback_final_move=True,
    pallet_lower_during_home=True,
)


A_DUAL_CONFIG = replace(
    A_STANDALONE_CONFIG,
    profile_name="A-dual",
    robot_object_name="m0609_robot_A",
    task_name="A_m0609_conveyor_oribox_task",
    pick_place_controller_name="m0609_pick_place_controller_A",
    cartesian_controller_name="m0609_custom_carry_controller_A",
    enable_ros2_bridge=True,
    stack_layers=1,
    stack_slot_count=2,
    lower_support_after_each_layer=False,
    diagnostic_stop_release_count=2,
    diagnostic_skip_pallet_lower=False,
    forklift_trigger_count=2,
    fused_yaw_enabled=False,
)


B_DUAL_CONFIG = replace(
    B_STANDALONE_CONFIG,
    profile_name="B-dual",
    robot_object_name="m0609_robot_B",
    task_name="B_m0609_conveyor_oribox_task",
    pick_place_controller_name="m0609_pick_place_controller_B",
    cartesian_controller_name="m0609_custom_carry_controller_B",
    box_asset_candidates=("oriB/OriBoxB.usda", "oriA/OriBoxA.usda"),
    pick_zone_center=(2.08983, -4.35, 0.88),
    pick_zone_side_only=True,
    slot_marker_paths=("/World/BPalt_slot_01", "/World/BPalt_slot_02"),
    stack_joint1_degrees=(220.0, 155.0, 220.0, 155.0),
    robot_prop_name_prefixes=_B_PROPS,
    robot_prop_paths=_B_PROP_PATHS,
    vgc10_root_path="/World/vgc10_visual_follow_B",
    suction_grid_world_path="/World/vgc10_world_xy_suction_grid_debug_B",
    suction_body_path="/World/scripted_suction_body_B",
    suction_marker_path="/World/scripted_suction_direction_marker_B",
    suction_debug_marker_path="/World/vgc10_suction_debug_marker_B",
    physics_attach_joint_path="/World/vgc10_physics_attach_fixed_joint_B",
    enable_ros2_bridge=True,
    stack_layers=1,
    stack_slot_count=2,
    lower_support_after_each_layer=False,
    settle_before_pallet_lower=True,
    slot_route_sign_force=False,
    diagnostic_skip_pallet_lower=False,
    forklift_enabled=False,
    forklift_trigger_count=2,
    marker_fallback_final_move=False,
    pallet_lower_during_home=False,
)


CONFIGS = {
    config.profile_name: config
    for config in (A_STANDALONE_CONFIG, B_STANDALONE_CONFIG, A_DUAL_CONFIG, B_DUAL_CONFIG)
}

