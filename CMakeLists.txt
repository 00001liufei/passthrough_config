cmake_minimum_required(VERSION 3.0.2)
project(passthrough_config)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_ros
  roscpp
  sensor_msgs
  visualization_msgs
  std_msgs 
  std_srvs
  velodyne_pcl
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
  FILES
  SendRPY.srv
  SendFieldX.srv
  SendFieldY.srv
  SendFieldZ.srv
  LoadConfig.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
)

catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS message_runtime std_msgs std_srvs
)

###########
## Build ##
###########

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_definitions(
	${PCL_DEFINITIONS}
)

add_library(${PROJECT_NAME}_yaml 
  src/lib/yaml.cpp)
target_link_libraries(${PROJECT_NAME}_yaml ${YAML_CPP_LIBRARIES})

add_executable(${PROJECT_NAME} src/node.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  ${PROJECT_NAME}_yaml 
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
