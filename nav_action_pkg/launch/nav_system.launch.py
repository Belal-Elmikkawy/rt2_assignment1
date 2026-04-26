import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    #Define the C++ Components
    action_server_node = ComposableNode(
        package='nav_action_pkg',
        plugin='nav_action_pkg::ActionServerComponent',
        name='action_server_component'
    )


    action_client_node = ComposableNode(
        package='nav_action_pkg',
        plugin='nav_action_pkg::ActionClientComponent',
        name='action_client_component'
    )


    container = ComposableNodeContainer(
        name='nav_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            action_server_node,
            action_client_node,
        ],
        output='screen',
    )

    #Define Python UI node
    ui_node = Node(
        package='nav_ui_pkg',
        executable='user_interface',
        name='user_interface_node',
        output='screen',
        prefix='xterm -e'
    )

    #Return the Launch Description
    return LaunchDescription([
        container,
        ui_node
    ])