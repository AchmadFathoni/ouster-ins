import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    ins = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(
            get_package_share_path('inertial_sense_ros'), 'launch'
        ), '/lidar.launch.py'])
    )

    lidar = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([os.path.join(
            get_package_share_path('ouster_ros'), 'launch'
        ), '/lidar_record.composite.launch.xml'])
    )

    topics = ['/ouster/lidar_packtes', '/ouster/metadata', '/odom_ins_enu']
    record = ExecuteProcess(
        cmd = ['ros2', 'bag', 'record'] + topics,
        output = 'screen'
    )
    return LaunchDescription([
        ins, lidar, record
    ])
