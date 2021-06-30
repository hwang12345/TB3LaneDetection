cmake_minimum_required(VERSION 2.8)
project( LaneDetection )
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
include_directories(/home/lorenz/Projects/ROS/TurtlebotFollower/include)
add_executable( LaneDetection ../src/LaneDetection.cpp
        ../include/image_converter.h
        ../src/image_converter.cpp
        )
target_link_libraries( LaneDetection ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
