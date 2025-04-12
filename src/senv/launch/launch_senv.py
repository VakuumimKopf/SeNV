from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='senv',
            executable='driver',
            
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],


        ),
        Node(
            package='senv',
            executable='intersection_con.py',

            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],

            parameters=[
                # Add  parameters as needed
            ]
        ),
        Node(
            package='senv',
            executable='lane_con.py',
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
        Node(
            package='senv',
            executable='park_con.py',
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
        Node(
            package='senv',
            executable='driving.py',
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
    ])