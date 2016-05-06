cmake_minimum_required(VERSION 2.8.3)
project(tilt_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
)

find_package(Eigen REQUIRED) 

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Tilt_controller
#  CATKIN_DEPENDS roscpp
   DEPENDS Eigen
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(tilt_controller src/main.cpp)
target_link_libraries(tilt_controller ${catkin_LIBRARIES} ${Eigen_LIBRARIES})

