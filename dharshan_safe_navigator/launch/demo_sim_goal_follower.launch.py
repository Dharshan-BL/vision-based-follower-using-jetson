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
    world = DeclareLaunchArgument('world', default_value='empty', description='World name')
    ld.add_action(world)
    
    goal_pose_topic = DeclareLaunchArgument('goal_pose_topic', default_value='goal_pose', description='Pose topic of the robot')
    ld.add_action(goal_pose_topic)

    # plot_tools_launch = GroupAction(
    #     actions=[
    #         IncludeLaunchDescription(
    #             PythonLaunchDescriptionSource(
    #                 os.path.join(get_package_share_directory('dharshan_plot_tools'), 'launch', 'plot_scan.launch.py')
    #             )
    #         )
    #     ],
    #     scoped=True,
    #     forwarding=False,
    #     launch_configurations={
    #         'use_sim_time': LaunchConfiguration('use_sim_time'),
    #         'namespace': LaunchConfiguration('namespace'),
    #         'scan_topic': f'front_laser/scan',
    #     },
    # )

    sim_launch = GroupAction(
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
            'namespace': LaunchConfiguration('namespace'),
            'scan_topic': f'front_laser/scan',
        },
    )
    ld.add_action(sim_launch)

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
            'goal_pose_topic': '/mocap/goal/pose',
            'scan_topic': f'front_laser/scan',
            'cmd_vel_topic': f'cmd_vel_ctrl',
        },
    )
    ld.add_action(safe_unicycle_local_nav_launch)


    return ld