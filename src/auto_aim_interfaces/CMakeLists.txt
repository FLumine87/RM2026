cmake_minimum_required(VERSION 3.8)
project(auto_aim_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 设置默认安装目录
if(NOT CMAKE_INSTALL_PREFIX)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install")
endif()

# 设置库安装目录
if(NOT CMAKE_INSTALL_LIBDIR)
    set(CMAKE_INSTALL_LIBDIR "lib")
endif()

find_package(ament_cmake QUIET)
find_package(rosidl_default_generators QUIET)
find_package(std_msgs QUIET)
find_package(geometry_msgs QUIET)

if(ament_cmake_FOUND AND rosidl_default_generators_FOUND AND std_msgs_FOUND AND geometry_msgs_FOUND)
    message(STATUS "Building auto_aim_interfaces with ROS2 support")
    message(STATUS "CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")
    message(STATUS "CMAKE_INSTALL_LIBDIR: ${CMAKE_INSTALL_LIBDIR}")
    
    rosidl_generate_interfaces(${PROJECT_NAME}
      "msg/Armor.msg"
      "msg/Armors.msg"
      "msg/Target.msg"
      "msg/Gimbal.msg"
      DEPENDENCIES std_msgs geometry_msgs
    )
    
    # 简化安装配置，避免权限问题
    ament_package()
else()
    message(STATUS "ROS2 dependencies not found, skipping auto_aim_interfaces message generation")
endif()

# 为内嵌式ROS2构建添加导出
if(ament_cmake_FOUND)
    # 导出生成的头文件路径
    set(AUTO_AIM_INTERFACES_INCLUDE_DIRS "${CMAKE_CURRENT_BINARY_DIR}")
    
    # 仅在非colcon构建时使用
    if(NOT COLCON_BUILD)
        include(CMakePackageConfigHelpers)
        configure_package_config_file(
            "${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in"
            "${CMAKE_CURRENT_BINARY_DIR}/auto_aim_interfacesConfig.cmake"
            INSTALL_DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/auto_aim_interfaces"
            PATH_VARS AUTO_AIM_INTERFACES_INCLUDE_DIRS
        )
        install(FILES "${CMAKE_CURRENT_BINARY_DIR}/auto_aim_interfacesConfig.cmake"
            DESTINATION "${CMAKE_INSTALL_LIBDIR}/cmake/auto_aim_interfaces"
        )
    endif()
endif()