cmake_minimum_required(VERSION 2.8.3)
project(carry_my_luggage)

set(CMAKE_BUILD_TYPE Debug)

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
  src/carry_my_luggage/DetectBag.cpp
  src/carry_my_luggage/DetectObject.cpp
  src/carry_my_luggage/GotoArena.cpp
  src/carry_my_luggage/GotoReferee.cpp
  src/carry_my_luggage/DetectPerson.cpp
  src/carry_my_luggage/GotoPerson.cpp
  src/carry_my_luggage/transforms.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_definitions(${PCL_DEFINITIONS})

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

set(LINK_LIBS
  carry_my_luggage
  ${catkin_LIBRARIES}
)

add_executable(carry_my_luggage_node src/carry_my_luggage_node.cpp)
target_link_libraries(carry_my_luggage_node ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${LINK_LIBS})

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")

roslint_cpp(
  src/carry_my_luggage_node.cpp
  src/carry_my_luggage/DetectBag.cpp
  include/carry_my_luggage/DetectBag.h
  src/carry_my_luggage/DetectObject.cpp
  include/carry_my_luggage/DetectObject.h
  src/carry_my_luggage/GotoArena.cpp
  include/carry_my_luggage/GotoArena.h
  src/carry_my_luggage/GotoReferee.cpp
  include/carry_my_luggage/GotoReferee.h
  src/carry_my_luggage/DetectPerson.cpp
  include/carry_my_luggage/DetectPerson.h
  include/carry_my_luggage/BTNavAction.h
  src/carry_my_luggage/GotoPerson.cpp
  include/carry_my_luggage/GotoPerson.h
  src/carry_my_luggage/transforms.cpp 
  include/carry_my_luggage/transforms.h
)