# M0609 Palletizing Refactor Analysis

> Historical Sol refactor notes. The follow-up audit in
> [ASTRA_REVIEW_REPORT.md](ASTRA_REVIEW_REPORT.md) uses the actual Git baseline
> `08f0d16^` → `08f0d16`. It supersedes the isolation claim below: sequential
> worker stepping alone did not cover World's deferred task scene setup or
> callback reentry. The audit adds scoped task contexts and regression tests.

## Scope and baseline

This analysis was made before changing the runtime implementation. The workspace is an extracted project tree rather than a Git checkout: there is no `.git` directory at the project root, so existing uncommitted changes and history cannot be inspected here. No USD, URDF, mesh, texture, model-weight, or Vision behavior is in scope for modification.

The repository contains two runtime areas:

- `Collected_Conveyor_lift_test_01/M0609`: Isaac Sim M0609 palletizing, RMPFlow configuration/controllers, robot/gripper assets, and the dual-cell wrapper.
- `Vision`: a ROS 2 Python package containing the YOLO parcel detector, QR decoder, central hub, PatchCore anomaly node, PyQt control GUI, launch file, models, and package metadata.

The non-Python Isaac tree is predominantly simulation data: 56 USD files, 6 URDF files, 35 DAE meshes, 24 STL meshes, 172 PNG textures, 32 MDL materials, and collected asset mappings. The only M0609 Python dependencies outside the three target scripts are:

- `rmpflow/m0609_rmpflow_controller.py`: builds the M0609 Lula RMPFlow motion policy and resets its base pose.
- `rmpflow/m0609_pick_place_controller.py`: supplies the existing Isaac `PickPlaceController` event timing and delegates Cartesian control to the RMPFlow controller.

Baseline logical line counts (Python `splitlines`) are:

| File | Lines | Notes |
|---|---:|---|
| `robot_a_palletizing_forklift.py` | 9,298 | Standalone A implementation |
| `robot_b_palletizing_forklift.py` | 9,312 | Standalone B implementation |
| `run_ab_dual_robot_ros_gate.py` | 541 physical | 1,014,425 bytes because two escaped source strings contain another 9,201 A lines and 9,211 B lines |

## Existing execution and import flow

Standalone A and B currently perform the following at module import/run time:

1. Import and construct one `SimulationApp`.
2. Import the Isaac/Omni/PXR modules after app creation.
3. Optionally enable the ROS 2 bridge, then call `simulation_app.update()`.
4. Construct one `World`, add one `M0609ConveyorBoxTask`, and reset the world.
5. Resolve the robot, VGC10 visual/suction prims, RMPFlow and PickPlace controllers.
6. Poll for a stopped box in the configured ready zone.
7. Pre-align, create the physics `FixedJoint`, lift, joint-swing/carry, approach the pallet slot, align yaw, lower, release, return home, and select the next box.
8. Close the app from the standalone owner.

The dual wrapper constructs one `SimulationApp`, enables the ROS 2 bridge, compiles and executes `_CELL_A_SOURCE` and `_CELL_B_SOURCE` into isolated dictionaries, and obtains generator workers from those namespaces. Isolation was used for two reasons: the copied programs have many same-named globals, and the dual wrapper needs to alter standalone lifecycle code so both tasks are added before one master `World.reset()`. The master then advances both generators once per shared-world frame. This preserves independent generator-local state, but embeds roughly 18,400 lines of duplicated source and makes standalone/dual behavior drift inevitable.

## Dependency and responsibility map

| Responsibility | Existing symbols/areas | Runtime dependency |
|---|---|---|
| Scene/USD | prim lookup, transforms, visibility, collider repair, bbox | PXR `Usd`, `UsdGeom`, `Gf`; current stage |
| VGC10 | visual reference, follow transform, suction marker/grid | Scene/USD utilities; robot/tool prim paths |
| Box physics | rigid/kinematic flags, velocity reset, `FixedJoint` attach/release | PXR `UsdPhysics`; VGC10 suction point |
| Diagnostics | mass/inertia, joint frames, pose/bbox/yaw logging | Scene and box-physics data; must retain safety/error logging |
| Palletizing | stop/zone detectors, candidate selection, slot goal, task, carry waypoints | Scene, VGC10, physics, RMPFlow/PickPlace |
| Forklift/pallet lowering | completed-box tracking, support lowering, A virtual forklift, B concurrent lowering | Worker-local state and cell policy |
| Dual-only | one app/world/reset, box release schedule, gates, ROS bridge | Both independent workers and shared stage |
| Vision/ROS 2 | camera/detection/QR/hub/GUI topics | Separate ROS 2 package; dual bridge exposes Isaac camera graph |

The Vision package does not import the M0609 scripts directly. Coupling is through ROS 2 topics and the dual gate command file (`/tmp/zone_command.txt`). Consequently the Vision implementation should remain unchanged during this refactor.

## A/B comparison

AST comparison found 155 shared top-level definitions. Of these, 131 are byte-for-byte AST-identical. After normalizing cell path/name strings, every shared definition except `make_custom_carry_targets` and `main` is semantically identical. A has two additional fused-yaw helpers. The meaningful differences are below.

| Concern | Robot A standalone | Robot B standalone | Refactor classification |
|---|---|---|---|
| Robot | `/World/m0609_A/m0609` | `/World/m0609_B/m0609` | Config |
| Box roots | `OriBoxA_*` | `OriBoxB_*` | Config |
| Ready zone | `/World/pick_ready_zone_A`, center `(-2.08983,-4.35,0.88)` | `/World/pick_ready_zone_B`, center `(1.9599,-4.35,0.88)` | Config |
| Pallet/support | `/World/APalt` | `/World/BPalt` | Config |
| Slot sequence | APalt 01,02 | BPalt 01,02,01,02 | Config |
| Joint-1 pattern | 220,155,220,155 deg | 220,155,220,120 deg | Config |
| Candidate restriction | A-only enabled | generic side restriction flag disabled in standalone source | Config, preserve exact profile |
| A behavior | Fused RMPFlow yaw during lower/settle for all four slots | Not present | Strategy/feature policy |
| B behavior | Not present | If a marker fallback goal exists, perform final move; after second release lower BPalt concurrently with home return | Strategy/feature policy |
| Release settling | enabled before APalt lower | disabled for B concurrent lower | Config |
| Diagnostics stop count | 4 (diagnostic stop itself disabled) | 2 (diagnostic stop itself disabled) | Config |

Dual mode is intentionally not identical to either standalone profile. Its embedded variants use two slots/one layer per cell, no pallet lowering, A forklift after two boxes, scheduled A01/B01/A02/B02 physics enablement, separate B VGC10/debug/fixed-joint paths, B ready-zone center X `2.08983`, and ROS2 Bridge enabled. These values must be represented by explicit dual A/B configs rather than silently replaced by standalone values.

## Common, config, strategy, and retained candidates

### Move to one common implementation

- Prim lookup, transforms, visibility and bbox helpers.
- VGC10 asset resolution, visual-follow pose, suction point/grid.
- Rigid-body/collision/velocity and `FixedJoint` attach/release helpers.
- Stop and pick-zone detectors.
- Candidate discovery, slot marker resolution and stack-goal calculations.
- RMPFlow Cartesian command, joint swing/carry target calculation, home/release flow.
- Runtime diagnostics and safety checks.
- `M0609ConveyorBoxTask` and the worker loop.

### Move to explicit `RobotCellConfig`

- Cell side/name and robot/object/task/controller names.
- Robot, box, ready-zone, pallet/support, slot, VGC10, suction/debug, and fixed-joint paths.
- Side-specific slot sequence, joint pattern, route sign, stack size/layers, and box prefix.
- Standalone versus dual feature policy: ROS bridge, forklift trigger, fused yaw, fallback final move, and concurrent pallet lowering.

### Keep as small strategy/policy behavior

- A fused-yaw preparation during lower/settle.
- B marker-missing final-position fallback.
- B concurrent pallet lowering during home return, including the blocking fallback.
- Dual scheduled box-physics release and gate/zone polling.

### Candidate for later removal (retained in this pass)

Static in-file analysis found these top-level helpers with no direct load in either standalone file: `initialize_robot`, `zero_body_velocity`, `_create_or_set_attr`, `get_suction_grid_world_points`, `should_script_attach`, `resolve_first_valid_path`, `align_box_yaw_to_slot_marker_159`, `lower_stack_support_cube_for_next_layer`, `_quat_np`, and `_fit_joint_delta`. They are retained because some are compatibility/fallback or diagnostic entry points and the current workspace has no Git history or Isaac smoke-test capability to prove safe deletion.

Likewise, literal-false feature flags are not dead-code proof. Several protect prior physics fallbacks (`BOX_DISABLE_PHYSICS_DURING_CARRY`, watchdog/drop modes, collision repair, legacy camera handling) and are deliberately preserved.

## Version-suffixed names

Suffixes such as `_109`, `_145`, `_155`, `_159`, `_164`, `_170`, `_171`, `_180`, `_184`, `_185`, and `_190` encode edit history rather than responsibility. Active paths use many of them, so deleting by suffix is unsafe. In the first behavior-preserving pass, public/config-facing concepts should receive stable names while compatibility aliases are retained where internal calls still depend on old names. Once Isaac smoke tests establish equivalence, aliases and unused generations can be removed separately.

## Target architecture and lifecycle constraints

The target package separates side-effect-free configuration from Isaac-dependent modules:

```text
M0609/
  robot_a_palletizing_forklift.py   # thin standalone lifecycle owner
  robot_b_palletizing_forklift.py   # thin standalone lifecycle owner
  run_ab_dual_robot_ros_gate.py     # one app/world/reset + gates + two workers
  palletizing/
    config.py                       # immutable standalone/dual A/B profiles
    settings.py                     # unchanged shared thresholds/timing/motion values
    scene.py                        # USD scene, transforms, bbox, VGC10 visual
    diagnostics.py                  # physics/pose/yaw diagnostics and safety logs
    physics.py                      # box physics and attach/release control
    palletizing.py                  # detection, goals, task, motion planning helpers
    forklift.py                     # pallet-lowering policy/state
    worker.py                       # per-cell generator state and orchestration
    bootstrap.py                    # standalone/shared lifecycle helpers
```

Importing `config.py` must not create a SimulationApp, World, prim, extension, or scene mutation. Entrypoints remain responsible for creating `SimulationApp` first; only afterward do they import Isaac-dependent package modules. Standalone owns its world/reset/close. Dual owns its one shared world, performs one reset after both tasks are registered, and closes the one app.

## Validation boundary

Available here: source/AST reference analysis, syntax compilation, pure-Python config tests, searches for embedded source and `exec(compile(...))`, and import-boundary inspection. Not available here: launching Isaac Sim, PhysX behavior, RMPFlow convergence, USD visual placement, ROS 2 publisher creation, camera topics, or real timing. Those require the separate smoke-test checklist after implementation.

## Implementation outcome and retained technical debt

The implemented package follows the target architecture above. The A and B standalone scripts are now 20-line lifecycle entrypoints, and the dual wrapper is a normal 389-line module with two configured workers. Using the same Python `splitlines` measure as the baseline, the shared implementation totals 10,072 lines across focused modules instead of being copied into both standalone scripts and embedded again in the dual wrapper.

The worker deliberately remains a cooperative generator rather than being rewritten as a new state machine: changing the existing event order would be too risky without Isaac Sim. Each `PalletizingWorker` owns the mutable state that was formerly global, while a compatibility context projects that state and the selected config onto the extracted legacy helpers immediately before each worker step. This is safe for the dual wrapper's sequential, single-threaded stepping model, but it is not designed for concurrent calls from multiple Python threads.

Several active version-suffixed internals and compatibility aliases remain. They preserve call relationships and diagnostic/fallback paths that cannot be proven equivalent without the runtime smoke tests. A later cleanup should remove these only after A standalone, B standalone, and dual A+B complete the checklist in `ISAAC_SIM_SMOKE_TEST.md`.
