from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    controller_yaml_path = os.path.join(
        get_package_share_directory('robot_control'),
        'config',
        'controller_params.yaml'
    )


    return LaunchDescription([
        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': '/home/nitron/ALSAI/maps/maze2.yaml',
                'global_frame': 'map',
                'robot_base_frame': 'base_link'
            }]
        ),

        # AMCL Localization
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': '/home/nitron/ALSAI/maps/maze2.yaml',
                'base_frame_id': 'base_link',
                'odom_frame': 'odom',
                'scan_topic': 'scan',
            }]
        ),

        # # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[{
                'ros__parameters': {
                    'planner_plugins': ['GridBased'],
                    'planner_plugin_ids': ['GridBased']
                }
            }]
        ),

        # # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml_path]
        ),


        # # BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[{
                'ros__parameters': {
                    'default_bt_xml_filename': '/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_time.xml'
                }
            }]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server', 'amcl', 'planner_server', 'controller_server']

            }]
        )
    ])
