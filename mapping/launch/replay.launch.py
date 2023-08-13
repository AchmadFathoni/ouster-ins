import os
import math

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    lidar = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(get_package_share_path('ouster_ros'), 'launch/replay.composite.launch.xml')
        ]),
        launch_arguments={
            'bag_file': LaunchConfiguration('bag_file'),
            'rviz_config': os.path.join(get_package_share_path('ouster_ros'), 'config/my.rviz')
        }.items()
    )

    odom = Node(
        package = 'odom_to_tf_ros2',
        executable = 'odom_to_tf',
        parameters = [os.path.join(get_package_share_path('odom_to_tf_ros2'), 'config/odom_to_tf.yaml')],
    )

    half_pi = math.pi / 2
    tf =   Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen" ,
        arguments=[
            "--x", "-0.05", "--y", "0", "--z", "-0.9",
            "--roll", str(half_pi), "--pitch", "0", "--yaw", str(-half_pi),
            "--frame-id", "base_link", "--child-frame-id", "os_sensor"]
    )
    return LaunchDescription([
        lidar, odom, tf
    ])
