from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    namespace = DeclareLaunchArgument('namespace', default_value='goal')
    world = DeclareLaunchArgument('world', default_value='empty', description='Gazebo world name')
    width = DeclareLaunchArgument('width', default_value='0.3')
    length = DeclareLaunchArgument('length', default_value='0.3')
    color = DeclareLaunchArgument('color', default_value='1 0 0 1')
    x = DeclareLaunchArgument('x', default_value='0.0')
    y = DeclareLaunchArgument('y', default_value='0.0')
    yaw = DeclareLaunchArgument('yaw', default_value='0.0')
    pose_rate = DeclareLaunchArgument('pose_rate', default_value='10.0')

    goal_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('core_robot_simulate'), 'launch', 'goal.launch.py')
                ),            
            )
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'world': LaunchConfiguration('world'),
            'width': LaunchConfiguration('width'),
            'length': LaunchConfiguration('length'),
            'color': LaunchConfiguration('color'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'yaw': LaunchConfiguration('yaw'),
            'pose_rate': LaunchConfiguration('pose_rate'),          
        },
    )      
    
    return LaunchDescription([
        use_sim_time,
        namespace,
        world,
        width,
        length,
        color,
        x, y, yaw,
        pose_rate,
        goal_launch,
    ])