from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os 


def generate_launch_description():

    ld = LaunchDescription()

    namespace = DeclareLaunchArgument('namespace', default_value='jackal') 
    ld.add_action(namespace)
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    ld.add_action(use_sim_time)
    
    goal_threshold = DeclareLaunchArgument('goal_threshold', default_value='1.0')
    ld.add_action(goal_threshold)

    repulsive_threshold_decay = DeclareLaunchArgument('repulsive_threshold_decay', default_value='10.0')
    ld.add_action(repulsive_threshold_decay)    

    safe_unicycle_local_nav_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('dharshan_safe_navigator'), 'launch', 'safe_unicycle_local_nav.launch.py')
                ),
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'goal_pose_topic': 'obj_pose',
            'scan_topic': f'front_laser/scan',
            'cmd_vel_topic': f'ctrl/cmd_vel',
            'goal_threshold': LaunchConfiguration('goal_threshold'),
            'repulsive_threshold_decay': LaunchConfiguration('repulsive_threshold_decay'),
        },
    )
    ld.add_action(safe_unicycle_local_nav_launch)


    return ld