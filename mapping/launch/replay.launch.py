import os
import math

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions.node import Node

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    file = LaunchConfiguration("file")
    offset = LaunchConfiguration("offset", default=0.01)
    viz = LaunchConfiguration("viz", default=False)
    config = LaunchConfiguration(
        "config", default=os.path.join(get_package_share_path('mapping'), 'config/octomap.rviz')
    )

    bag = ExecuteProcess(
        cmd=[
            "ros2", "bag", "play", file, "--start-offset", offset 
        ],
        output="screen"
    )

    rviz = Node(
        condition=IfCondition(PythonExpression([
           viz, " == True" 
        ])),
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d", config
        ]
    )

    half_pi = math.pi / 2
    base_sensor_tf =   Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=[
            "--x", "-0.05", "--y", "0", "--z", "-0.9",
            "--roll", str(half_pi), "--pitch", str(math.pi), "--yaw", str(-half_pi),
            "--frame-id", "base_link", "--child-frame-id", "os_sensor"]
    )
    """
    Extracted static transform from lidar metadata. To see the transform:
    1. Replay the output bag from record.launch.py using:
        ros2 launch ouster_ros replay.composite.launch.xml bag_file:=recorded_bag
    2. In new terminal run:
        ros2 run tf2_ros tf2_echo os_sensor os_lidar
    3. Put the observed transform into this static transform publisher
    """
    sensor_lidar_tf =   Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=[
            "--x", "0", "--y", "0", "--z", "0.036",
            "--roll", "0", "--pitch", "0", "--yaw", str(math.pi),
            "--frame-id", "os_sensor", "--child-frame-id", "os_lidar"]
    )

    return LaunchDescription([
        rviz, base_sensor_tf, sensor_lidar_tf, bag
    ])
