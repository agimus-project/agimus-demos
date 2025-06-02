from launch_ros.actions import Node
from agimus_demos_common.launch_utils import get_use_sim_time


def mpc_debugger_node(
    frame: str,
    parent_frame: None | str = None,
    marker_size: None | float = None,
    node_kwargs={},
) -> Node:
    args = [
        "--frame",
        frame,
    ]
    if parent_frame is not None:
        args += [
            "--parent-frame",
            parent_frame,
        ]
    if marker_size is not None:
        args.extend(["--marker-size", str(marker_size)])
    node = Node(
        package="agimus_controller_ros",
        executable="mpc_debugger_node",
        parameters=[get_use_sim_time()],
        arguments=args,
        **node_kwargs,
    )
    return node
