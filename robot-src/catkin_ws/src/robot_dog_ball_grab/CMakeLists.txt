cmake_minimum_required(VERSION 3.0.2)
project(robot_dog_ball_grab)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/ball_yolo_grab.py
  scripts/ball_release.py
  scripts/rotate.py
  scripts/ball_grab_release.py
  scripts/yolo_view.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
