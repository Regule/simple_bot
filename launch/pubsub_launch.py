from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    publisher = Node(
        package='simple_bot',
        executable='small_node_pub',
        output='screen'
    )

    subscriber = Node(
        package='simple_bot',
        executable='small_node_sub',
        output='screen'
    )

    launch_description = LaunchDescription()
    launch_description.add_action(publisher)
    launch_description.add_action(subscriber)

    return launch_description
