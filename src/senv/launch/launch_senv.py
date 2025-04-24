from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='senv',
            executable='lane_con',
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
        Node(
            package='senv',
            executable='intersection_con',
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
        Node(
            package='senv',
            executable='park_con',
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
        Node(
            package='senv',
            executable='driver',
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
    ])