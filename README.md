


1. ~/microswarm_planning# ros2 launch gazebo_sim bookstore.launch.py x_init:=-2.30 y_init:=6.18   
2. ~/microswarm_planning# ros2 launch vins vins_simulation.launch.py robot_name:=turtlebot3_waffle_pi x_init:=-2.30 y_init:=6.18
3. ~/microswarm_planning# ros2 launch thetastar_global_planner planner_launch.py algorithm_name:=thetastar    
4. ~/microswarm_planning# ros2 service call /global_planner/request_path thetastar_global_planner/srv/GetPath "{goal: {x: -3.0, y: -3.0, z: 0.0}}"  
