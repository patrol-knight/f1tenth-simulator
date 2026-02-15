
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('f1tenth_simulator'),
        'config',
        'bridge.yaml'
        )
    config_dict = yaml.safe_load(open(config, 'r'))

    simulator_node = Node(
        package='f1tenth_simulator',
        executable='bridge',
        name='bridge',
        parameters=[config]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_simulator'), 'launch', 'bridge.rviz')]
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': config_dict['bridge']['ros__parameters']['map_path'] + '.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': True}]
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': True},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='f1tenth_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_simulator'), 'launch', 'f1tenth.xacro')])}],
    )
    bridge_node = Node(
        package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 9090, 'address': '0.0.0.0'}]
    )

    ld.add_action(rviz_node)
    ld.add_action(simulator_node)
    ld.add_action(nav_lifecycle_node)
    ld.add_action(map_server_node)
    ld.add_action(robot_publisher)
    if config_dict['bridge']['ros__parameters']['rosbridge']:
        ld.add_action(bridge_node)

    return ld