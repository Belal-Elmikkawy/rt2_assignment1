import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # Include the Gazebo Simulation and Robot Spawning Launch File
    pkg_bme_gazebo_sensors = get_package_share_directory('bme_gazebo_sensors')
    
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bme_gazebo_sensors, 'launch', 'spawn_robot.launch.py')
        )
    )

    # Define the C++ Components (Action Server and Client)
    action_server_node = ComposableNode(
        package='nav_action_pkg',
        plugin='nav_action_pkg::ActionServerComponent',
        name='action_server_component',
        parameters=[{'use_sim_time': True}]
    )

    action_client_node = ComposableNode(
        package='nav_action_pkg',
        plugin='nav_action_pkg::ActionClientComponent',
        name='action_client_component',
        parameters=[{'use_sim_time': True}]
    )

    container = ComposableNodeContainer(
        name='nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            action_server_node,
            action_client_node,
        ],
        output='screen',
    )

    # Define Python UI node
    ui_node = Node(
        package='nav_ui_pkg',
        executable='user_interface',
        name='user_interface_node',
        output='screen',
        prefix='xterm -e'
    )

    # Return the Launch Description
    return LaunchDescription([
        simulation_launch,
        container,
        ui_node
    ])