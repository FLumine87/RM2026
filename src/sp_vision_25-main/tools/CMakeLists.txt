cmake_minimum_required(VERSION 3.16)

add_library(tools OBJECT
    exiter.cpp
    extended_kalman_filter.cpp
    ransac_sine_fitter.cpp
    img_tools.cpp
    math_tools.cpp
    plotter.cpp
    trajectory.cpp
    recorder.cpp
    logger.cpp
    pid.cpp
    crc.cpp
)

find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(cv_bridge QUIET)
find_package(tf2_geometry_msgs QUIET)

if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND geometry_msgs_FOUND AND sensor_msgs_FOUND AND cv_bridge_FOUND AND tf2_geometry_msgs_FOUND)
    message(STATUS "ROS2 environment found, adding ros2_debug to tools library.")
    target_sources(tools PRIVATE ros2_debug.cpp)
    ament_target_dependencies(tools rclcpp std_msgs geometry_msgs sensor_msgs cv_bridge tf2_geometry_msgs)
    target_include_directories(tools PUBLIC ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS} ${geometry_msgs_INCLUDE_DIRS} ${sensor_msgs_INCLUDE_DIRS} ${cv_bridge_INCLUDE_DIRS} ${tf2_geometry_msgs_INCLUDE_DIRS})
    
    # 添加 auto_aim_debug_interfaces 依赖
    find_package(auto_aim_debug_interfaces QUIET)
    if(auto_aim_debug_interfaces_FOUND)
        message(STATUS "auto_aim_debug_interfaces found, adding to tools library.")
        ament_target_dependencies(tools auto_aim_debug_interfaces)
        target_include_directories(tools PUBLIC ${auto_aim_debug_interfaces_INCLUDE_DIRS})
    else()
        message(WARNING "auto_aim_debug_interfaces not found, skipping ROS2 debug functionality.")
    endif()
    
    # 添加对 auto_aim 的依赖，因为 ros2_debug.cpp 使用了 Target 类
    target_compile_features(tools PUBLIC cxx_std_17)
    target_include_directories(tools PUBLIC ${CMAKE_SOURCE_DIR}/tasks/auto_aim)
endif()