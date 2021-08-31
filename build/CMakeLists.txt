cmake_minimum_required(VERSION 2.8)
project( ROSImage )
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
add_executable( ROSImage ../src/image_converter.cpp)
target_link_libraries( ROSImage ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
