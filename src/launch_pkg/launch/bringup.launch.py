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
    motor_param_file = os.path.join(config_dir, 'motor_params.yaml')
    gait_param_file = os.path.join(config_dir, 'gait_params.yaml')

    node_b = Node(
        package="control_pkg",
        executable="control_node",
        name="node_b",
        output="screen",
        parameters=[motor_param_file, gait_param_file]
        # namespace="robot",       # optional
        # arguments=["--ros-args", "--log-level", "info"],  # optional
    )

    node_c = Node(
        package="joy",
        executable="joy_node",
        name="node_c",
        output="screen"
    )

    node_d = Node(
        package="balance_pkg",
        executable="balance_node",
        name="node_d",
        output="screen"
    )

    return LaunchDescription([node_a, node_b, node_c, node_d])