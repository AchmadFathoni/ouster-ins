import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    lidar = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(get_package_share_path('ouster_ros'), 'launch/replay.composite.launch.xml')
        ]),
        launch_arguments={'bag_file': LaunchConfiguration('bag_file'), 'viz': 'False'}.items()
    )

    record = ExecuteProcess(
        cmd = ['ros2', 'bag', 'record', '/odom_ins_enu', '/ouster/points'],
        output = 'screen'
    )

    return LaunchDescription([
        lidar, record
    ])

