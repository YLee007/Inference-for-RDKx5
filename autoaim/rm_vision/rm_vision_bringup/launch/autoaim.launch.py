import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess  
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def _resolve_yolo_config():
    # Resolve a repo-relative path to an absolute path using this package's share dir
    pkg_share = get_package_share_directory('rm_vision_bringup')
    # pkg_share is typically <repo>/autoaim/rm_vision/rm_vision_bringup
    candidate = os.path.abspath(os.path.join(pkg_share, '..', '..', 'model', 'yolov11workconfig.json'))
    return candidate


def generate_launch_description():

    from common import node_params, launch_params, robot_state_publisher, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
                ,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::Yolo11Node',
                    name='yolo11_node',
                    # pass node_params but override config_file with an absolute path
                    parameters=[node_params, {'config_file': _resolve_yolo_config()}],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')


    if (launch_params['camera'] == 'hik'):
        cam_detector = get_camera_detector_container(hik_camera_node)

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )
    
    serial_driver_node = Node(
        package='vision_serial_driver',
        executable='vision_serial_driver_node',
        parameters = [node_params],
    )

    attacker_node = Node(
        package='vision_attacker',
        executable='vision_attacker_node',
        parameters=[node_params],
    )

    return LaunchDescription([
        robot_state_publisher,
        cam_detector,
        delay_tracker_node,
        serial_driver_node,
        attacker_node,
    ])
