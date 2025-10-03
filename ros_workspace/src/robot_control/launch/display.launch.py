from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    world_path = '/home/nitron/ALSAI/worlds/maze6_world'
    sdf_path = '/home/nitron/ALSAI/gazebo_backup/inz_rob_2_0/model.sdf'

    gazebo_ros_pkg = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_ros_pkg),
            launch_arguments={'world': world_path}.items()
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_robot',
            arguments=[
                '-file', sdf_path,
                '-entity', 'mobile_base',
                '-x', '0',
                '-y', '0',
                '-z', '0'
            ],
            output='screen'
        ),

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
