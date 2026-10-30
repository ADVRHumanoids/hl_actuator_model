cmake_minimum_required(VERSION 3.20)
project(actuator_data_collection)


set(CMAKE_CXX_STANDARD 20)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xbot2 REQUIRED)
find_package(matlogger2 REQUIRED)

add_xbot2_plugin(data_collection_plugin_haotian src/trajectory_plugin.cpp)
target_link_libraries(data_collection_plugin_haotian
    PUBLIC 
    xbot2::xbot2_ros2_support
    std_msgs::std_msgs__rosidl_typesupport_cpp
    matlogger2::matlogger2
)


install(
    TARGETS 
    data_collection_plugin_haotian
    DESTINATION lib
    )

ament_package()
