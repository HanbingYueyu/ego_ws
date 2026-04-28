cmake_minimum_required(VERSION 3.0.2)
project(uav_car_coop)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/formation_collision_cloud.py
  scripts/formation_goal_relay.py
  scripts/formation_trajectory_viz.py
  scripts/px4_follower_controller.py
  scripts/start_line_mission.py
  scripts/uav_world_tf_bridge.py
  scripts/world_odom_bridge.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
