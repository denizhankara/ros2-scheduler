from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node1 = Node(
        package='my_python_pkg',
        namespace='my_namespace',
        executable='publisher',
        name='my_publisher_node1'
    )

    publisher_node2 = Node(
        package='my_python_pkg',
        namespace='my_namespace',
        executable='publisher',
        name='my_publisher_node2'
    )

    subscriber_node = Node(
        package='my_python_pkg',
        namespace='my_namespace',
        executable='subscriber',
        name='my_subscriber_node'
    )
    

    ld.add_action(publisher_node1)
    ld.add_action(publisher_node2)
    ld.add_action(subscriber_node)

    return ld