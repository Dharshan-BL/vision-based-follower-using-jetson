from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = LaunchDescription()

    namespace = DeclareLaunchArgument('namespace', default_value='jackal') 
    ld.add_action(namespace)
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    ld.add_action(use_sim_time)
    
    goal_pose_topic = DeclareLaunchArgument('goal_pose_topic', default_value='goal_pose', description='Pose topic of the robot')
    ld.add_action(goal_pose_topic)

    scan_topic = DeclareLaunchArgument('scan_topic', default_value='scan', description='Laser topic of the robot')
    ld.add_action(scan_topic)

    cmd_vel_topic = DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel', description='Command velocity topic of the robot')
    ld.add_action(cmd_vel_topic)

    safe_unicycle_local_nav_node = Node(
        package='dharshan_safe_navigator',
        executable='safe_unicycle_local_nav.py',
        name='safe_unicycle_local_nav',
        namespace = LaunchConfiguration('namespace'),
        parameters = [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        remappings=[
            ('goal_pose', LaunchConfiguration('goal_pose_topic')),
            ('odom', 'odom/filtered'),
            ('scan', LaunchConfiguration('scan_topic')),
            ('cmd_vel', LaunchConfiguration('cmd_vel_topic')),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(safe_unicycle_local_nav_node)

    return ld