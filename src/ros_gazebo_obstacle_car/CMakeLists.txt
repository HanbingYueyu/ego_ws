cmake_minimum_required(VERSION 3.0.2)
project(ros_gazebo_obstacle_car)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ego_car_controller.py
  scripts/ackermann_joint_state_bridge.py
  scripts/obstacle_avoidance.py
  scripts/laserscan_cloud_world.py
  scripts/random_obstacles.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch urdf world rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
