from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tracker',             
            executable='controller_node',  
            name='tracker',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])
