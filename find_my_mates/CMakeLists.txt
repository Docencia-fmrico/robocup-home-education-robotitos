cmake_minimum_required(VERSION 2.8.3)
project(find_my_mates)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  tf2_geometry_msgs
  tf2
  tf2_ros
  image_transport
  pcl_conversions
  pcl_ros
  behaviortree_cpp_v3
  darknet_ros_msgs
  message_filters
  actionlib
  actionlib_msgs
  move_base_msgs
)

catkin_package(
LIBRARIES geometry_tf
CATKIN_DEPENDS 
  roscpp 
  rospy
  std_msgs 
  kobuki_msgs 
  sensor_msgs 
  cv_bridge
  behaviortree_cpp_v3
  tf2_geometry_msgs
  tf2
  tf2_ros
  image_transport
  pcl_conversions
  pcl_ros
  message_filters
  actionlib_msgs 
  move_base_msgs
)

find_package(OpenCV REQUIRED)
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  src/find_my_mates/GotoReferee.cpp
  src/find_my_mates/GotoArena.cpp
  src/find_my_mates/GotoPerson.cpp
  src/find_my_mates/DetectPerson.cpp
  src/find_my_mates/SayDescriptions.cpp
  src/find_my_mates/GetDescriptions.cpp
  src/find_my_mates/transforms.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_definitions(${PCL_DEFINITIONS})

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

set(LINK_LIBS
  find_my_mates
  ${catkin_LIBRARIES}
)

add_executable(find_my_mates_node src/find_my_mates_node.cpp)
target_link_libraries(find_my_mates_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${LINK_LIBS})

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")

roslint_cpp(
  src/find_my_mates_node.cpp
  include/find_my_mates/BTNavAction.h
  src/find_my_mates/GotoReferee.cpp
  include/find_my_mates/GotoReferee.h
  src/find_my_mates/GotoPerson.cpp
  include/find_my_mates/GotoPerson.h
  src/find_my_mates/DetectPerson.cpp
  include/find_my_mates/DetectPerson.h
  src/find_my_mates/GetDescriptions.cpp
  include/find_my_mates/GetDescriptions.h
  src/find_my_mates/SayDescriptions.cpp
  include/find_my_mates/SayDescriptions.h
  src/find_my_mates/GotoArena.cpp
  include/find_my_mates/GotoArena.h
)