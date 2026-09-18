"""Lifecycle helpers imported only after an Isaac Sim ``SimulationApp`` exists."""

from __future__ import annotations

import time

from .config import RobotCellConfig


def configure_runtime_logging() -> None:
    """Restore the original Kit warning filter after SimulationApp creation."""
    from .settings import SUPPRESS_NON_CRITICAL_KIT_WARNINGS_122

    if SUPPRESS_NON_CRITICAL_KIT_WARNINGS_122:
        try:
            import carb

            settings = carb.settings.get_settings()
            for key in ("/log/level", "/log/outputStreamLevel", "/log/fileLogLevel", "/log/consoleLogLevel"):
                try:
                    settings.set(key, "error")
                except Exception:
                    pass
        except Exception as error:
            print(f"[122_LOG_FILTER][WARN] log filter setup skipped: {error}")


def enable_requested_extensions(config: RobotCellConfig, simulation_app) -> None:
    """Preserve the original extension-before-World import/update ordering."""
    if config.enable_ros2_bridge:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
    simulation_app.update()


def run_standalone(config: RobotCellConfig, simulation_app) -> None:
    """Own one app, one world, one reset, and one configured cell worker."""
    try:
        configure_runtime_logging()
        enable_requested_extensions(config, simulation_app)

        # These imports intentionally follow app creation and extension setup.
        from isaacsim.core.api import World

        from .worker import create_worker

        world = World(stage_units_in_meters=1.0)
        worker = create_worker(config=config, shared_world=world, simulation_app=simulation_app)
        next(worker)  # register task only
        world.reset()
        next(worker)  # post-reset initialization and first cooperative frame
        while simulation_app.is_running():
            world.step(render=True)
            next(worker)
            time.sleep(0.01)
    except StopIteration:
        print(f"[PALLETIZING][{config.profile_name}] worker stopped")
    finally:
        simulation_app.close()


__all__ = ["configure_runtime_logging", "enable_requested_extensions", "run_standalone"]
