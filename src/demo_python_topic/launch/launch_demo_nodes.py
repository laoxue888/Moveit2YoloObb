from launch import LaunchDescription
from launch_ros.actions import Node
 
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_python_topic',
            namespace='duckietown',
            executable='novel_pub_node',
            name='novel_pub_node1'
        ),
        Node(
            package='demo_python_topic',
            namespace='duckietown',
            executable='novel_sub_node',
            name='novel_sub_node1'
        )
    ])



