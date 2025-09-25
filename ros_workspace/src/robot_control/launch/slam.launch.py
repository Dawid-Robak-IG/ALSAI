from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'map_update_interval': 1.0,
                'max_laser_range': 5.0,
                'minimum_travel_distance': 0.05,
                'minimum_travel_heading': 0.1,
                'use_scan_matching': True,
                'use_sim_time': True,
                'use_scan_matching': True,
                'use_loop_closure': False
            }]
        )
    ])
