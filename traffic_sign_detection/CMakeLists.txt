cmake_minimum_required(VERSION 2.8.3)
project(traffic_sign_detection)

add_definitions(-std=c++11)

## Find catkin workspace
find_package(catkin REQUIRED COMPONENTS cv_bridge roscpp std_msgs genmsg geometry_msgs image_transport)
find_package(OpenCV REQUIRED)

## Declare ROS messages and services
add_message_files(FILES sign.msg)

## Generated add message and services
generate_messages(DEPENDENCIES std_msgs)

## Declare catkin_package
catkin_package()

##Excutables
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(detection src/detection_node.cpp src/classify.cpp)
target_link_libraries(detection ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(detection traffic_sign_detection_generate_messages_cpp)



