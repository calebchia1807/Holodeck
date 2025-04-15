from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simulator_ros',
            executable='unity_nav',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='simulator_ros',
            executable='unity_stand_crouch',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='simulator_ros',
            executable='unity_rgb_camera',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='simulator_ros',
            executable='unity_bgr_camera',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='simulator_ros',
            executable='unity_depth_camera',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='simulator_ros',
            executable='unity_segmentation_camera',
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='simulator_ros',
            executable='unity_bounding_box',
            output='screen',
            emulate_tty=True,
        ),
    ])