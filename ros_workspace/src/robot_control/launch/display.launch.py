from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    declared_world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze1_world',
        description='Gazebo world name in worlds'
    )


    world_name = LaunchConfiguration('world')

    world_dir = os.path.expanduser(f'~/ALSAI/worlds')
    world_path = PathJoinSubstitution([world_dir, world_name])
    sdf_path = os.path.expanduser('~/ALSAI/gazebo_backup/inz_rob_2_0/model.sdf')

    gazebo_ros_pkg = os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    
    rviz_node = TimerAction(
        period=2.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/nitron/ALSAI/ros_workspace/src/robot_control/urdf/robot.rviz'],
            parameters=[{'use_sim_time': True}]
        )]
    )

    return LaunchDescription([
        declared_world_arg,

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
        rviz_node
    ])
