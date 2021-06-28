cmake_minimum_required(VERSION 2.8)
project( cvBridgeTest )
find_package( OpenCV REQUIRED) 
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
include_directories(${OpenCV_INCLUDE_DIRS}) 
include_directories(${catkin_INCLUDE_DIRS})
add_executable( cvBridgeTest ./src/cvBridgeTest.cpp )
target_link_libraries( cvBridgeTest ${OpenCV_LIBRARIES} ${catkin_LIBRARIES} ) 
