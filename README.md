cd ros2_ws

rosdep update

rosdep install --from-paths src --ignore-src -r -y

colcon build

source install/setup.zsh

ros2 launch aws_robomaker_small_warehouse_world no_roof_small_warehouse_launch.py

ros2_ws ros2 run teleop_twist_keyboard teleop_twist_keyboard  
