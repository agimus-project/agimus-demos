from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String
from agimus_demo_07_deburring.controller.deburring_controller import (
    ControllerImpl as ControllerImplBase,
)


class ControllerImpl(ControllerImplBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def setup(self, robot_description: String, environment_description: String):
        package_share_directory = Path(
            get_package_share_directory("agimus_demo_07_deburring")
        )
        config_file = (
            package_share_directory / "config" / "metal" / "pytroller_params.yaml"
        )
        super().setup(robot_description, environment_description, config_file)
