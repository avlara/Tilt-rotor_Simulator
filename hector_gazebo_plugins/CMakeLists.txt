cmake_minimum_required(VERSION 2.8.3)
project(hector_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS 
		roscpp 
		std_msgs 
		std_srvs 
		geometry_msgs 
		nav_msgs 
		tf 
		dynamic_reconfigure
		tilt_srv
		gazebo_ros)
include_directories(include ${catkin_INCLUDE_DIRS})

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

add_service_files(
  FILES
  SetBias.srv
)

generate_messages(
  DEPENDENCIES geometry_msgs
)

generate_dynamic_reconfigure_options(
  cfg/SensorModel.cfg
  cfg/GNSS.cfg
)

catkin_package(
    DEPENDS gazebo
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf
    INCLUDE_DIRS include
    LIBRARIES
)

add_library(hector_gazebo_reset_plugin src/reset_plugin.cpp)
target_link_libraries(hector_gazebo_reset_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

add_library(hector_gazebo_ros_imu src/gazebo_ros_imu.cpp)
target_link_libraries(hector_gazebo_ros_imu ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_imu ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg)

add_library(hector_gazebo_ros_magnetic src/gazebo_ros_magnetic.cpp)
target_link_libraries(hector_gazebo_ros_magnetic ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_magnetic ${PROJECT_NAME}_gencfg)

add_library(hector_gazebo_ros_gps src/gazebo_ros_gps.cpp)
target_link_libraries(hector_gazebo_ros_gps ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_gps ${PROJECT_NAME}_gencfg)

add_library(hector_gazebo_ros_sonar src/gazebo_ros_sonar.cpp)
target_link_libraries(hector_gazebo_ros_sonar ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(hector_gazebo_ros_sonar ${PROJECT_NAME}_gencfg)

add_library(servo_motor_plugin src/servo_motor_plug.cpp)
target_link_libraries(servo_motor_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(aerodinamica_plugin src/aerodinamica.cpp)
target_link_libraries(aerodinamica_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(tilt_world_plugin src/tilt_world_plugin.cpp)
target_link_libraries(tilt_world_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(AllData_plugin src/AllData.cpp)
target_link_libraries(AllData_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  aerodinamica_plugin
  hector_gazebo_reset_plugin
  hector_gazebo_ros_imu
  hector_gazebo_ros_magnetic
  hector_gazebo_ros_gps
  hector_gazebo_ros_sonar
  servo_motor_plugin
  AllData_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
