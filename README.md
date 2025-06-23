cd ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build

source install/setup.zsh
