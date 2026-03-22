cmake_minimum_required(VERSION 3.16)

find_package(yaml-cpp REQUIRED)

add_subdirectory(serial)

# 创建目标 io
add_library(io STATIC 
    hikrobot/hikrobot.cpp    
    mindvision/mindvision.cpp  
    usbcamera/usbcamera.cpp  
    camera.cpp
    cboard.cpp
    dm_imu/dm_imu.cpp
    gimbal/gimbal.cpp
)

# hikrobot
target_include_directories(io PUBLIC hikrobot/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC hikrobot/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC hikrobot/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# mindvision
target_include_directories(io PUBLIC mindvision/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(io PUBLIC mindvision/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(io PUBLIC mindvision/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(io MvCameraControl MVSDK usb-1.0 yaml-cpp serial)

################## 检查 ROS 环境 ##################
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(sp_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(tf2_msgs QUIET)
find_package(visualization_msgs QUIET)

# 如果找不到 ROS 环境，则跳过 ROS 相关的部分
if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND)
    message(WARNING "ROS2 not found, skipping ROS2 specific code.")
else()
    # 找到 ROS2 环境
    message(STATUS "ROS2 environment found, compiling ROS2-related code.")
    
    # 强制添加 auto_aim_interfaces 的包含路径（参考主 CMakeLists.txt）
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../../install/auto_aim_interfaces/include")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../../install/auto_aim_interfaces/include/auto_aim_interfaces")
    
    # 添加基本 ROS2 依赖
    include_directories(${std_msgs_INCLUDE_DIRS})
    include_directories(${geometry_msgs_INCLUDE_DIRS})
    include_directories(${tf2_msgs_INCLUDE_DIRS})
    include_directories(${visualization_msgs_INCLUDE_DIRS})
    ament_target_dependencies(io rclcpp std_msgs sp_msgs geometry_msgs tf2_msgs visualization_msgs)
endif()
