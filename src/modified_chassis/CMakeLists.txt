cmake_minimum_required(VERSION 2.8.3)
project(modified_chassis)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
rospy
  roborts_msgs
  nav_msgs
  tf2_ros
tf2_geometry_msgs
actionlib_msgs
actionlib
message_generation
)

find_package(Eigen3 REQUIRED)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   MoveMode.srv
 )


add_action_files(
DIRECTORY action 
FILES Recharge.action
FILES MoveMode.action
)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## chassis_server_node
#add_executable(chassis_server_node src/modified_chassis_server_node.cpp)
#target_link_libraries(chassis_server_node
#  ${catkin_LIBRARIES}
#)
#add_dependencies(chassis_server_node
#  ${catkin_EXPORTED_TARGETS})

# chassis_client_node
#add_executable(chassis_client_node src/modified_chassis_client_node.cpp)
#target_link_libraries(chassis_client_node
#  ${catkin_LIBRARIES}
#)
#add_dependencies(chassis_client_node
#  ${catkin_EXPORTED_TARGETS})

# modified_turret_node
add_executable(modified_turret_node src/modified_turret_node.cpp)
target_link_libraries(modified_turret_node
  ${catkin_LIBRARIES}
)
add_dependencies(modified_turret_node
  ${catkin_EXPORTED_TARGETS})

# modified_odom_node
add_executable(modified_odom_node src/modified_odom_node.cpp)
target_link_libraries(modified_odom_node
  ${catkin_LIBRARIES}
)
add_dependencies(modified_odom_node
  ${catkin_EXPORTED_TARGETS})


# teleop_node
add_executable(teleop_node src/teleop.cpp)
target_link_libraries(teleop_node
  ${catkin_LIBRARIES}
)
add_dependencies(teleop_node
  ${catkin_EXPORTED_TARGETS})


# recharge_server_node
add_executable(recharge_server_node src/recharge_server.cpp)
target_link_libraries(recharge_server_node
  ${catkin_LIBRARIES}
)
add_dependencies(recharge_server_node
  ${catkin_EXPORTED_TARGETS})


# recharge_client_node
add_executable(recharge_client_node src/recharge_client.cpp)
target_link_libraries( 
  recharge_client_node
  ${catkin_LIBRARIES}
)
add_dependencies(
  recharge_client_node
  ${catkin_EXPORTED_TARGETS}
)

# mode_change_server_node
add_executable(chassis_mode_server_node src/srv_chassis_server_node.cpp)
target_link_libraries( 
  chassis_mode_server_node
  ${catkin_LIBRARIES}
)
add_dependencies(
  chassis_mode_server_node
  ${catkin_EXPORTED_TARGETS}
)

# set_goal_test
add_executable(set_goal_test src/set_goal_test.cpp)
target_link_libraries( 
  set_goal_test
  ${catkin_LIBRARIES}
)
add_dependencies(
  set_goal_test
  ${catkin_EXPORTED_TARGETS}
)

# find_camera_server_node
add_executable(find_camera_server_node src/recharge_find_camera_server.cpp)
target_link_libraries( 
  find_camera_server_node
  ${catkin_LIBRARIES}
)
add_dependencies(
  find_camera_server_node
  ${catkin_EXPORTED_TARGETS}
)

# find_camera_client_node
add_executable(find_camera_client_node src/recharge_find_camera_client.cpp)
target_link_libraries( 
  find_camera_client_node
  ${catkin_LIBRARIES}
)
add_dependencies(
  find_camera_client_node
  ${catkin_EXPORTED_TARGETS}
)

# lidar_shooter_alternant_node
# add_executable(lidar_shooter_alternant_node src/lidar_shooter_alternant.cpp)
# target_link_libraries( 
#   lidar_shooter_alternant_node
#   ${catkin_LIBRARIES}
# )
# add_dependencies(
#   lidar_shooter_alternant_node
#   ${catkin_EXPORTED_TARGETS}
# )


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_modified_chassis.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
