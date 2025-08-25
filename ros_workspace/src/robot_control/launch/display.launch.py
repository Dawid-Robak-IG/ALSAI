from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', '/home/nitron/ALSAI/ros_workspace/src/robot_control/urdf/mobile_base.urdf.xacro'
                ]),
                'use_sim_time': True
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/nitron/ALSAI/ros_workspace/src/robot_control/urdf/robot.rviz'],
            parameters=[{'use_sim_time': True}]
        )
    ])
