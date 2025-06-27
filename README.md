cd ros2_ws

rosdep update

rosdep install --from-paths src --ignore-src -r -y

colcon build

source install/setup.zsh

ros2 launch aws_robomaker_small_warehouse_world no_roof_small_warehouse_launch.py

ros2_ws ros2 run teleop_twist_keyboard teleop_twist_keyboard  


#turtlebot3_warehouse.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
WORLD_MODEL = os.environ['WAREHOUSE_MODEL']

def generate_launch_description():
    model = LaunchConfiguration('model', default='waffle')
    world = LaunchConfiguration('world', default='small_warehouse')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    aws_small_warehouse_dir = os.path.join(
        get_package_share_directory('aws_robomaker_small_warehouse_world'), 'launch'
    )
    if WORLD_MODEL == 'small_warehouse':
        aws_small_warehouse = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([aws_small_warehouse_dir, '/no_roof_small_warehouse_launch.py'])
        )
    elif WORLD_MODEL == 'big_warehouse':
        aws_small_warehouse = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([aws_small_warehouse_dir, '/big_warehouse_launch.py'])
        )

    launch_file_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch'
    )
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # rviz 실행 코드 
    rviz_config_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'rviz',  
        'model.rviz' 
    )
    rviz2 = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(aws_small_warehouse)
    ld.add_action(robot_state_publisher)
    ld.add_action(rviz2)
    return ld
