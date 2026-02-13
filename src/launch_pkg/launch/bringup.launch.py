from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_a = Node(
        package="pca9685_driver",
        executable="pca9685_node",   # must match what you install/build
        name="node_a",
        output="screen",
        # parameters=[{"rate_hz": 50}],
        # remappings=[
        #     ("/cmd", "/robot/cmd"),
        # ],
    )

    node_b = Node(
        package="control_pkg",
        executable="control_node",
        name="node_b",
        output="screen",
        # namespace="robot",       # optional
        # arguments=["--ros-args", "--log-level", "info"],  # optional
    )

    return LaunchDescription([node_a, node_b])