from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = list();
    for i in range(4):
        node = Node(package='turtlesim',
                    executable='turtlesim_node',
                    name = 'turtle' + str(i))
        nodes.append(node)
    return LaunchDescription(nodes)