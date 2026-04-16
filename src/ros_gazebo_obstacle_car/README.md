# ros_gazebo_obstacle_car

ROS Noetic + Gazebo tutorial package for a small obstacle-avoidance car with EGO-Planner integration.

## Features
- Gazebo visualizes an outdoor football-field-like map
- Random obstacles are generated on each launch (boxes and cylinders)
- EGO-Planner generates avoidance trajectories from `/scan_cloud_world`
- RViz shows obstacles, point clouds, and planner trajectories

## Run
```bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
roslaunch ros_gazebo_obstacle_car demo.launch
```

## Optional launch arguments
```bash
roslaunch ros_gazebo_obstacle_car demo.launch seed:=7 obstacle_count:=30
```
