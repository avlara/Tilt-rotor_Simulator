cmake_minimum_required(VERSION 2.8.3)
project(aerodinamica)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  #servo_motor
)

catkin_package(
    DEPENDS 
    roscpp 
    gazebo_ros
    ##servo_motor
    CATKIN_DEPENDS
    INCLUDE_DIRS include
    LIBRARIES
)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_library(aerodinamica_plugin src/aerodinamica.cpp)
target_link_libraries(aerodinamica_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  aerodinamica_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
