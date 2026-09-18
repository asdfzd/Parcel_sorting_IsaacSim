"""Standalone Robot A entrypoint.

Keep ``SimulationApp`` construction before every Isaac/Omni-dependent import.
"""

from isaacsim import SimulationApp


simulation_app = SimulationApp({"headless": False})

from palletizing.bootstrap import run_standalone
from palletizing.config import A_STANDALONE_CONFIG


def main() -> None:
    run_standalone(A_STANDALONE_CONFIG, simulation_app)


if __name__ == "__main__":
    main()
