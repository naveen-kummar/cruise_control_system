
"""Launch a cruise control node and vehicle node."""

from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='cruise_control', executable='cruise_manager', output='screen'),
        launch_ros.actions.Node(
            package='cruise_control', executable='vehicle_stub', output='screen'),
    ])
