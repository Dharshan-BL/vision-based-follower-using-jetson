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

    gazebo_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('core_jackal_simulate'), 'launch', 'gazebo_empty.launch.py')
                )
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace':LaunchConfiguration('world'),
        },
    )
    ld.add_action(gazebo_launch)
    
    jackal_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('core_jackal_simulate'), 'launch', 'jackal.launch.py')
                ),
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'world': LaunchConfiguration('world'),
            'front_laser_enable': f'{True}',
            'rear_laser_enable': f'{False}',
            '3d_laser_enable': f'{False}',
            'front_camera_enable': f'{True}',
            'imu_enable': f'{False}'
        },
    )
    ld.add_action(jackal_launch)

    goal_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('dharshan_safe_navigator'), 'launch', 'goal.launch.py')
                ),
            ),
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'x': '5.0',
            'y': '0.0', 
            'world': LaunchConfiguration('world'),    
        },
    )
    ld.add_action(goal_launch)

    jackal_teleop_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('core_jackal_control'), 'launch', 'jackal_teleop_key.launch.py')
                ),
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'cmd_vel': 'cmd_vel_key',
        },
    )
    ld.add_action(jackal_teleop_launch)

    rviz_config = os.path.join(get_package_share_directory('core_jackal_simulate'), 'config', 'jackal_scan.rviz')
    rviz_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('core_robot_simulate'), 'launch', 'rviz.launch.py')
                ),
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'config': rviz_config,
        },
    )
    ld.add_action(rviz_launch)

    return ld