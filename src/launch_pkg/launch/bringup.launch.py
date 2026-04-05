from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os

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

    # Get the control parameters config
    config_dir = os.path.join(get_package_share_directory('control_pkg'), 'config')
    param_file = os.path.join(config_dir, 'control_params.yaml')
    print(param_file)

    node_b = Node(
        package="control_pkg",
        executable="control_node",
        name="node_b",
        output="screen",
        parameters=[param_file]
        # namespace="robot",       # optional
        # arguments=["--ros-args", "--log-level", "info"],  # optional
    )

    return LaunchDescription([node_a, node_b])