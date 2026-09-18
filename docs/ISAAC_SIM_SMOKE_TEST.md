# Isaac Sim M0609 Smoke Test

These checks require the real Isaac Sim/PhysX/ROS 2 environment. They were not run in the refactoring workspace. Run the scripts from `isaacsim_dual_robot_palletizing/M0609` so relative RMPFlow and asset paths resolve as documented.

Audit baseline: original `08f0d16^`, Sol refactor `08f0d16`; see
[REGRESSION_REVIEW_REPORT.md](development/REGRESSION_REVIEW_REPORT.md). All rows below remain
`NEEDS_ISAAC_SIM_TEST`. NumPy/World-double tests do not validate PhysX or ROS.
Standalone profiles leave ROS2 Bridge disabled, as in the original; the dual
runner enables it. Profile-name strings are not normal startup success logs;
check the actual active robot root and registered object instead.

Before testing, keep the original `Conveyor_lift.usd`, robot assets, RMPFlow YAML/URDF, ROS domain/environment, and Vision nodes unchanged. Stop after the first unexpected transform, collision, or timing change and capture the first traceback plus the preceding palletizing log.

| # | Check | Expected result | Log to confirm | First place to inspect on failure |
|---:|---|---|---|---|
| 1 | Robot A standalone | One app/world starts and A is the active robot | `[OK] active robot root = /World/m0609_A`, successful SingleManipulator registration | `robot_a_palletizing_forklift.py`, `A_STANDALONE_CONFIG` |
| 2 | Robot B standalone | One app/world starts and B is the active robot | `[OK] active robot root = /World/m0609_B`, successful SingleManipulator registration | `robot_b_palletizing_forklift.py`, `B_STANDALONE_CONFIG` |
| 3 | Dual A+B | Both tasks initialize in one shared World | `phase1 add_task`, one shared reset, then both `phase2` lines | `run_ab_dual_robot_ros_gate.py`, `A_DUAL_CONFIG`, `B_DUAL_CONFIG` |
| 4 | M0609 A/B spawn | Both articulations are visible at their authored transforms | active robot/path `OK` messages; no `PATH_WARN` | `Conveyor_lift.usd`, `robot_root_path` |
| 5 | VGC10 visual | A and B tools show one correctly scaled visual each | dual `[VGC10][A/B] ... ok=True` | `scene.py`, config `vgc10_root_path`, VGC10 asset path |
| 6 | Suction point | Marker/follow point stays at the configured tool0 offset | VGC10 suction-point setup/follow logs | `VGC10_SUCTION_LOCAL_OFFSET`, `VGC10_FOLLOW_TARGET_PATH` |
| 7 | Box pick detection | Only the cell's box becomes ready after zone entry and stop | ready-zone and `box_stopped` reasons | config box prefix/zone center; `BoxStopDetector`, `PickZoneDetector` |
| 8 | FixedJoint attach | Contact creates one cell-specific joint without teleport | `PHYSICS_ATTACH` success and surface ON | `physics.py`, `physics_attach_joint_path`, body0/body1 paths |
| 9 | Box lift | Box follows the tool through vertical lift without falling or snapping | joint/physics diagnostic samples; lift phase progress | RMPFlow config, FixedJoint frames, lift constants in `settings.py` |
| 10 | Swing/carry | Joint-1 swing begins only after safe lift and clears the robot/support | joint-carry phase and link-2 guard logs | `make_joint_swing_carry_targets`, guard/safe-height settings |
| 11 | Pallet slot approach | Box approaches the intended APalt/BPalt marker | slot-marker resolution and move-over/lower phase logs | config `slot_marker_paths`; `resolve_slot_marker_center` |
| 12 | Yaw alignment | Box axis approaches slot axis without direct transform snap | yaw diagnostic and pre-release/fused-yaw logs | `palletizing.py` yaw helpers, `diagnostics.py`; A `fused_yaw_enabled` policy |
| 13 | Lower | Box descends at the existing step count/clearance with no penetration | lower/settle progress and target error | lower/settle constants in `settings.py` |
| 14 | Release | FixedJoint disappears and box remains dynamic on the pallet | surface OFF, release reason, compact physics state | `release_physics_attach_joint`; rigid-body/collision state |
| 15 | Home return | Robot reaches the original zero/home joints | `LOOP_HOME_DONE` and return-home progress | `HOME_RETURN_STEPS`, initial-joint capture, worker return block |
| 16 | Next-box repeat | Completed root is skipped and the next side-matching box is selected | release count, next cycle/attempt, selected root | `discover_stack_candidates`, completed-root state |
| 17 | A forklift | In dual, A moves APalt after two boxes; standalone A uses its four-box profile | forklift delay then virtual APalt lift/move logs | config `forklift_enabled/trigger_count`; worker forklift callbacks |
| 18 | B pallet lowering | Standalone B lowers BPalt during home return after the second release | `PALLET_LOWER_PARALLEL START/DONE` interleaved with home logs | `PalletLoweringCoordinator`, B standalone config; sync fallback |
| 19 | ROS2 Bridge | Dual bridge extension loads and graph publishers appear | `[ROS2_BRIDGE] ... enabled`; no bridge warning | sourced ROS 2 environment, extension availability, camera graph |
| 20 | Camera/ROS2 topic | Existing camera/image topics publish with their unchanged names/types | ROS 2 topic list/rate and Vision subscriber state | `/World/Graph/ROS_Camera`, ROS domain/network, Vision launch |
| 21 | Gate A | Zone A command waits 4 s, closes Group for 5 s, then opens | `Zone A`, `DOWN /World/Group`, then `UP` | `/tmp/zone_command.txt`, `GATE_GROUPS`, Group prim |
| 22 | Gate B | Zone B command waits 6 s, closes Group_01 for 8 s, then opens | `Zone B`, `DOWN /World/Group_01`, then `UP` | command file, `GATE_GROUPS`, Group_01 prim |
| 23 | Shared World and deferred scene setup | One app, one World and one master reset; A registers `m0609_robot_A`, B registers `m0609_robot_B`; each uses its own boxes/tool/joint | One shared-reset line; `[OK] active robot root` first A then B; no duplicate-object exception or `SHARED_RESET_ERROR` | `worker._WorkerTask`, `PalletizingWorker.context`, task `_register_robot` |
| 24 | Slot sequence across repeat cycles | A standalone uses APalt 01/02 with its original slot-index fallback; B uses BPalt 01/02/01/02 and joint-1 angles 220/155/220/120; dual uses two slots each | `SLOT_MARKER_GOAL_155`, `SIMPLE_3STEP_ROUTE_117`, selected slot/joint1 degree | `config.py`, `resolve_slot_marker_center`, `make_custom_carry_targets` |
| 25 | Same-frame suction anchor and A/B isolation | Attach uses the latest evaluated top-surface anchor, including the original XY clamp and Z epsilon; B cannot reuse A's anchor or joint | `CENTER_SUCTION ... attach_center`, `grid_attach_center`, `PHYSICS_ATTACH_OK body0/body1` on both cells | `scene.evaluate_suction_grid_on_box_top`, worker state/context, `physics.create_physics_attach_joint` |
| 26 | Nested callbacks and resolved robot/tool paths | Scene/physics helpers use the same resolved path printed by `_load_usd`; a B callback during an A step returns to A's context | Compare active robot/VGC10 target lines against attach body0 and later A/B selection logs | `_WorkerTask._load_usd`, `PalletizingWorker.context`; do not edit the USD to force a test |
| 27 | Shutdown and initialization failure | Normal exit, interrupt and World setup/reset failure close the owning app once; workers never close it | Dual `closing shared SimulationApp` even after `SHARED_RESET_ERROR`; process exits without a live Kit window | `bootstrap.run_standalone`, dual `main` finally block |
| 28 | Vision-to-gate integration | An externally supplied bridge converts QR text such as `ZONE_A`/`ZONE_B` into file commands `A`/`B`; Vision and simulator must agree on file location/host | `/qr_code` output, file consumption, `[ZONE] Received`; absent bridge means this test fails independently of palletizing | External zone bridge (not present in this repository), `poll_zone_file` |

The ordinary dual loop steps once before advancing A/B, but startup warm-up,
release observation and optional fallback loops also step the shared World.
This is inherited behavior. During A's release observation verify that B stays
physically stable and that gates/box scheduling resume correctly; do not interpret
the one-reset guarantee as one physics step per every worker operation.

For camera validation, inspect `/World/Graph/ROS_Camera` and its
`ROS2CameraHelper` in the composed stage, then verify `/rgb` (`sensor_msgs/Image`).
`Vision/scripts/start_vision.sh` supplies the external image-transport republisher
for `/rgb/compressed`; the hub forwards `/hub/rgb/compressed`. Match ROS domain
103 and QoS across the actual machines. No Python camera-graph constructor or
zone-file writer was found in the repository; enabling the bridge alone does
not prove either integration works.

Inherited gate semantics also need observation: repeated identical zones are
discarded by `_last_zone` until another zone intervenes; gate countdown pauses,
whereas the box-release sequencer uses elapsed wall time. Record these results
without treating them as newly introduced refactor behavior.

## Recommended run order

1. Run A standalone through at least one complete attach/release/home cycle.
2. Restart Isaac Sim and run B standalone through four boxes, verifying repeated slots and concurrent BPalt lowering.
3. Restart Isaac Sim and run dual mode through A01/B01/A02/B02 in the scheduled order.
4. With dual still running, validate ROS 2 camera traffic and send one A then one B gate command.

Do not treat syntax/unit-test success as a substitute for these checks. In particular, visual alignment, contact stability, RMPFlow convergence, asynchronous forklift timing, and ROS graph creation can only be established in Isaac Sim.
