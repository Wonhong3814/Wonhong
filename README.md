cd ros2_ws

rosdep update

rosdep install --from-paths src --ignore-src -r -y

colcon build

source install/setup.zsh

ros2_ws ros2 run teleop_twist_keyboard teleop_twist_keyboard  
