import rclpy
import code
from agimus_demo_05_pick_and_place.orchestrator import Orchestrator


if __name__ == "__main__":
    rclpy.init()
    orchestrator = Orchestrator()
    code.interact(local=locals())  # Start an interactive shell
