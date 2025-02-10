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
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    ld.add_action(use_sim_time)
    world = DeclareLaunchArgument('world', default_value='empty', description='World name')
    ld.add_action(world)
    
    pose_topic = DeclareLaunchArgument('pose_topic', default_value='pose', description='Pose topic of the robot')
    ld.add_action(pose_topic)

    sim_environment_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('dharshan_safe_navigator'), 'launch', 'sim_gazebo_jackal.launch.py')
                )
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace':LaunchConfiguration('namespace'),
            'world': LaunchConfiguration('world'),
        },
    )
    ld.add_action(sim_environment_launch)

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
            'pose_topic': LaunchConfiguration('pose_topic'),
            'laser_topic': f'front_laser/scan',
            'cmd_vel_topic': f'cmd_vel_ctrl',
        },
    )
    ld.add_action(safe_unicycle_local_nav_launch)


    return ld