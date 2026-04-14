cmake_minimum_required(VERSION 3.0.2)
project(ackermann_ego_gazebo)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ego_car_controller.py
  scripts/laserscan_cloud_world.py
  scripts/random_obstacles.py
  scripts/odom_car_marker.py
  scripts/cmdvel_to_tricycle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf rviz config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
