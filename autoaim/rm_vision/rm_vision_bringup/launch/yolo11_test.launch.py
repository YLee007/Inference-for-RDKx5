import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess  
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def _resolve_model_file():
    pkg_share = get_package_share_directory('rm_vision_bringup')
    return os.path.abspath(os.path.join(pkg_share, '..', '..', 'model', 'final.bin'))


def generate_launch_description():

    from common import node_params, launch_params
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')

    # Container with only hik camera, armor_detector and yolo11 for testing
    cam_detector = ComposableNodeContainer(
            name='camera_detector_test_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                hik_camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                ),
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::YoloNode',
                    name='yolo_node',
                    parameters=[node_params, {'model_file': _resolve_model_file()}],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
        )

    return LaunchDescription([
        cam_detector,
    ])
