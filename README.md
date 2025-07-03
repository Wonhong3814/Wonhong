```cd ~/microswarm_planning/src```

```git clone https://github.com/Wonhong3814/Wonhong.git tracker -b tracker```

```cd ..```

```colcon build --packages-select tracker --symlink-install```


1. Start the Gazebo simulation with the robot's initial position:
```ros2 launch gazebo_sim bookstore.launch.py x_init:=-2.30 y_init:=6.18```

   
2. Launch microswarm components
```ros2 launch vins vins_simulation.launch.py robot_name:=turtlebot3_waffle_pi x_init:=-2.30 y_init:=6.18```

3. Run the 3D Theta* global planner
```ros2 launch thetastar_global_planner planner_launch.py algorithm_name:=thetastar```
 
4. Send Goal via Service
```ros2 service call /global_planner/request_path thetastar_global_planner/srv/GetPath "{goal: {x: -3.0, y: -3.0, z: 0.0}}"```

5. Initiate track controller
```ros2 launch tracker controller_launch.py``` 
