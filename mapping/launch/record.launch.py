import os

import lifecycle_msgs.msg

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import LifecycleNode

from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ins = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            os.path.join(get_package_share_directory('inertial_sense_ros'), 'launch', 'lidar.launch.py')
        ])
    )

    lidar = LifecycleNode(
        package='ouster_ros',
        executable='os_sensor',
        name='os_sensor',
        namespace='ouster',
        parameters=[os.path.join(get_package_share_directory('mapping'), 'config', 'lidar.yaml')],
        output='screen',
    )

    lidar_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(lidar),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    lidar_activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=lidar, goal_state='inactive',
            entities=[
                LogInfo(msg="os_sensor activating..."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(lidar),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
            handle_once=True
        )
    )
    topics = ['/ouster/lidar_packtes', '/ouster/metadata', '/odom_ins_enu']
    record = ExecuteProcess(
        cmd = ['ros2', 'bag', 'record'] + topics,
        output = 'screen'
    )
    return LaunchDescription([
        ins, lidar, record, lidar_configure_event, lidar_activate_event
    ])
