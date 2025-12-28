import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def _resolve_model_file():
    pkg_share = get_package_share_directory('rm_vision_bringup')
    return os.path.abspath(os.path.join(pkg_share, '..', '..', 'model', 'final.bin'))


def generate_launch_description(): 
    from common import node_params
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer
    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration
    from launch.conditions import UnlessCondition
    from launch import LaunchDescription

    def get_camera_node(package, plugin, condition=None):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}],
            condition=condition
        )

    config = yaml.safe_load(open(node_params, 'r'))
    use_image_file_default = str(config.get('/yolo_node', {})
                                    .get('ros__parameters', {})
                                    .get('use_image_file', False)).lower()

    use_image_file = LaunchConfiguration('use_image_file')
    image_file_path = LaunchConfiguration('image_file_path')
    enable_fps_logging = LaunchConfiguration('enable_fps_logging')

    hik_camera_node = get_camera_node(
        'hik_camera',
        'hik_camera::HikCameraNode',
        condition=UnlessCondition(use_image_file)
    )

    # Test container with camera, YOLO detector, and armor detector only
    yolo_test_container = ComposableNodeContainer(
            name='yolo_test_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                hik_camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::YoloNode',
                    name='yolo_node',
                    parameters=[node_params, {
                        'use_image_file': use_image_file,
                        'image_file_path': image_file_path,
                        'enable_fps_logging': enable_fps_logging,
                    }],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
        )

    return LaunchDescription([
        DeclareLaunchArgument('use_image_file', default_value=use_image_file_default,
                              description='Use folder images instead of camera'),
        DeclareLaunchArgument('image_file_path', default_value='/home/sunrise/dateset',
                              description='Folder path for offline images'),
        DeclareLaunchArgument('enable_fps_logging', default_value='false',
                              description='Enable FPS logging when using folder images'),
        yolo_test_container,
    ])
