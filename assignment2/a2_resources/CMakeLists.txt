cmake_minimum_required(VERSION 2.8.3)
project(a2_resources)

find_package(catkin REQUIRED)

catkin_package()

# Install files
install(DIRECTORY maps robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)