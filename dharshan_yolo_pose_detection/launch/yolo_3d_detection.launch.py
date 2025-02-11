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

    default_yolo_model = os.path.join(
        get_package_share_directory('dharshan_yolo_pose_detection'), 
        'config',
        'yolov11_dharshan.pt'
    )
    yolo_model = DeclareLaunchArgument('yolo_model', default_value=default_yolo_model)
    ld.add_action(yolo_model)

    camera_name = DeclareLaunchArgument('camera_name', default_value='front_camera')
    ld.add_action(camera_name)

    yolo_3d_detection_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('yolo_bringup'),
                        'launch',
                        'yolo.launch.py'
                    )
                )
            ),
        ],
        scoped=True,
        forwarding=False,
        launch_configurations={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'namespace': LaunchConfiguration('namespace'),
            'model': LaunchConfiguration('yolo_model'),
            'camera_name': LaunchConfiguration('camera_name'),
            # 'target_frame': 'jackal/base_link',
        }
    )
    ld.add_action(yolo_3d_detection_launch)

    return ld
                                 