from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='senv',
<<<<<<< HEAD
            executable='driver',
            
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],


        ),
        Node(
            package='senv',
            executable='laserscanner',

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
=======
>>>>>>> c16411254d4dbc1762a597ba784d2dfd2d3f7ad0
            executable='lane_con',
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
        Node(
            package='senv',
            executable='camera',
            # activate output
            output='screen',
            emulate_tty=True,
            arguments=[('__log_level:=debug')],
        ),
<<<<<<< HEAD
    ])
=======
        Node(
            package='senv',
            executable='laserscanner',
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
>>>>>>> c16411254d4dbc1762a597ba784d2dfd2d3f7ad0
