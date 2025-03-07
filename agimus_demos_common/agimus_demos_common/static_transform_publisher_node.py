from launch_ros.actions import Node
import typing as T

def static_transform_publisher_node(
    frame_id: str,
    child_frame_id: str,
    xyz: T.Optional[T.Tuple[float, float, float]] = None,
    rot_xyzw: T.Optional[T.Tuple[float, float, float, float]] = None,
    rot_rpy: T.Optional[T.Tuple[float, float, float]] = None,
    node_kwargs = {},
) -> Node:
    assert rot_rpy is None or rot_xyzw is None
    args = [
        "--frame-id", frame_id,
        "--child-frame-id", child_frame_id,
    ]
    labels = ("x", "y", "z", "w")
    rpy_labels = ("roll", "pitch", "yaw")
    if xyz is not None:
        for l, v in zip(labels, xyz):
            args.extend([f"--{l}", v])
    if rot_xyzw is not None:
        for l, v in zip(labels, rot_xyzw):
            args.extend([f"--q{l}", v])
    if rot_rpy is not None:
        for l, v in zip(rpy_labels, rot_rpy):
            args.extend([f"--{l}", v])
    node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=args,
        **node_kwargs,
    )
    return node