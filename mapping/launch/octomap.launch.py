import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_path

def generate_launch_description():
    replay = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(get_package_share_path('mapping'), 'launch/replay.launch.py')
        ]),
        launch_arguments={
            'bag_file': LaunchConfiguration('bag_file'),
            'bag_offset': LaunchConfiguration('bag_offset'),
            'viz': LaunchConfiguration('viz', default=False),
            'rviz_config': os.path.join(get_package_share_path('mapping'), 'config/octomap.rviz')
        }.items()
    )

    octomap = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap',
        parameters=[
            {'frame_id': 'odom_enu'},
            {'resolution': 0.1},
            {'latch': False},
        ],
        remappings=[
            ('cloud_in','/filtered')
        ]
    )

    pcl_saver = Node(
        package='mapping',
        executable='save_pointcloud',
        name='pcl_saver',
        parameters=[
            {'save_pointcloud.topic': '/octomap_point_cloud_centers'},
        ],
    )

    return LaunchDescription([
        replay, octomap, pcl_saver
    ])
