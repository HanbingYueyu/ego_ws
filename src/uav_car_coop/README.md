# uav_car_coop

Single-UAV + single-car cooperative launch wrapper for this workspace.

## What this package does

- Starts PX4 SITL with the velodyne UAV model.
- Runs the UAV lidar world-cloud pipeline (`/velodyne_points_world`).
- Starts one `top_akm_dl_robot` car in the same Gazebo world.
- Runs car lidar cloud conversion and car controller.
- Reuses one EGO planner stream (`/planning/pos_cmd`) for both controllers.

## Run

```bash
cd ~/ego_ws
source devel/setup.bash
roslaunch uav_car_coop single_uav_single_car.launch
```

## Useful args

- `car_spawn_x`, `car_spawn_y`, `car_spawn_z`: change car initial pose.
- `start_rviz:=true`: open RViz.
