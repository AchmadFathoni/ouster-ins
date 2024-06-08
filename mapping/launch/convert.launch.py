"""
Convert ouster lidar raw packet to pointcloud & save odometry from INS.
Usage: ros2 launch mapping convert.launch.py bag_file:=my_bag
Script will output my_bag_converted in the same directory as my_bag
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions.node import Node

from ament_index_python.packages import get_package_share_path

launch_args =[
    DeclareLaunchArgument(name='bag_file', description='Input bag file.'),
]

def launch_setup(context):
    config = LaunchConfiguration('bag_file').perform(context)
    lidar = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(get_package_share_path('ouster_ros'), 'launch/replay.composite.launch.xml')
        ]),
        launch_arguments={'bag_file': LaunchConfiguration('bag_file'), 'viz': 'False'}.items()
    )

    record = ExecuteProcess(
        cmd = ['ros2', 'bag', 'record', '/odom_ins_enu', '/filtered', '-o', config + '_converted'],
        output = 'screen'
    )

    pcl_filter =  Node(
        package='mapping',
        executable='pcl_filter',
        name='pcl_filter',
        parameters=[
            {'pcl_filter.min_depth': 0.7},
            {'pcl_filter.in_cloud': '/ouster/points'},
            {'pcl_filter.out_cloud': '/filtered'},
        ],
    )

    return [
        lidar, record, pcl_filter
    ]

def generate_launch_description():
    opfunc = OpaqueFunction(function = launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
