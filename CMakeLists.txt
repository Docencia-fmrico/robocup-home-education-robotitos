cmake_minimum_required(VERSION 2.8.3)
project(robocup_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslint
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
  tf
  image_transport
  pcl_conversions
  pcl_ros
  behaviortree_cpp_v3
  darknet_ros_msgs
  message_filters
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

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
  tf
  image_transport
  pcl_conversions
  pcl_ros
  message_filters
)

find_package(OpenCV)
find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
	${PCL_LIBRARY_DIRS}
)

# add_library(${PROJECT_NAME}
#   
# )
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_definitions(${PCL_DEFINITIONS})

foreach(bt_plugin ${plugin_libs})
  target_link_libraries(${bt_plugin} ${catkin_LIBRARIES})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

set(LINK_LIBS
  robocup_navigation
  ${catkin_LIBRARIES}
)

# add_executable(example src/example.cpp)
# target_link_libraries(example ${catkin_LIBRARIES} ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${LINK_LIBS})

set(ROSLINT_CPP_OPTS "--filter=-runtime/printf")

roslint_cpp(
    
)