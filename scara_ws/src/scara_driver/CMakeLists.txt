cmake_minimum_required(VERSION 3.0.2)
project(scara_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES scara_driver
  CATKIN_DEPENDS roscpp rospy std_msgs 
)

include_directories(
include ${catkin_INCLUDE_DIRS}
)

add_executable(driver src/driver.cpp)
add_dependencies(driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(driver ${catkin_LIBRARIES})

add_executable(user src/user.cpp)
add_dependencies(user ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(user ${catkin_LIBRARIES})




