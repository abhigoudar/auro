cmake_minimum_required(VERSION 2.8.3)
project(auro)

## Compile as C++11, supported in ROS Kinetic and newer

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  std_msgs
  image_transport
  image_geometry
  roscpp
  rospy
  tf
)

find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES auro
#  CATKIN_DEPENDS roscpp rospy tf
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(sav_img_pcd_node src/sav_img_pcd.cpp)
target_link_libraries(sav_img_pcd_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

add_executable(solve_pnp src/solve_pnp.cpp)
target_link_libraries(solve_pnp
   ${catkin_LIBRARIES}
)

add_executable(pc_to_rgbpc_node src/pc_to_rgbpc.cpp)
target_link_libraries(pc_to_rgbpc_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

