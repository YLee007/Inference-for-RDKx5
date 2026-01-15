import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))


def generate_launch_description():

    from common import launch_params, robot_state_publisher, node_params, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer
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
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')

    camera = launch_params.get('camera', 'hik')
    if camera != 'hik':
        raise RuntimeError('launch_params.yaml: no \'hik\' found')
    cam_detector = get_camera_detector_container(hik_camera_node)

    delay_tracker_node = TimerAction(
        period=2.0,
        actions=[tracker_node],
    )

    return LaunchDescription([
        robot_state_publisher,
        cam_detector,
        delay_tracker_node,
    ])
