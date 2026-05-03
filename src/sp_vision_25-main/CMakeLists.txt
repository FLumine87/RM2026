cmake_minimum_required(VERSION 3.16.3)

project(sp_vision)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
message(STATUS "--------------------CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}--------------------")

find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)
set(OpenVINO_DIR "/opt/intel/openvino_2024.6.0/runtime/cmake/")
find_package(OpenVINO REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR})

# 添加其他子目录
add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(tasks/auto_aim)
add_subdirectory(tasks/auto_buff)
add_subdirectory(tasks/omniperception)
add_subdirectory(tasks/bullet_detector)

# 查找接口包 - 现在通过colcon构建，在install目录中
# 跳过有问题的find_package调用，直接添加头文件路径
set(auto_aim_interfaces_FOUND 1)
message(STATUS "Forcing auto_aim_interfaces_FOUND=1 to avoid config file errors")

# 强制添加接口包的头文件路径
if(ament_cmake_FOUND AND rclcpp_FOUND)
    # 直接添加接口包的头文件路径
    message(STATUS "Adding direct include paths for auto_aim_interfaces")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../install/auto_aim_interfaces/include")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../install/auto_aim_interfaces/include/auto_aim_interfaces")
endif()

####################src###################
add_executable(standard src/standard.cpp)
add_executable(mt_standard src/mt_standard.cpp)
add_executable(standard_mpc src/standard_mpc.cpp)
add_executable(auto_aim_debug_mpc src/auto_aim_debug_mpc.cpp)
add_executable(mt_auto_aim_debug src/mt_auto_aim_debug.cpp)
add_executable(auto_buff_debug src/auto_buff_debug.cpp)
add_executable(auto_buff_debug_mpc src/auto_buff_debug_mpc.cpp)
add_executable(uav src/uav.cpp)
add_executable(uav_debug src/uav_debug.cpp)

target_link_libraries(standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_standard ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(standard_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_aim_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(mt_auto_aim_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(auto_buff_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(auto_buff_debug_mpc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io ${CERES_LIBRARIES})
target_link_libraries(uav ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim auto_buff tools io)
target_link_libraries(uav_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

##################calibration################
add_executable(capture calibration/capture.cpp)
add_executable(calibrate_camera calibration/calibrate_camera.cpp)
add_executable(calibrate_handeye calibration/calibrate_handeye.cpp)
add_executable(calibrate_robotworld_handeye calibration/calibrate_robotworld_handeye.cpp)
add_executable(split_video calibration/split_video.cpp)

target_link_libraries(capture ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(calibrate_camera ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(calibrate_robotworld_handeye ${OpenCV_LIBS} fmt::fmt yaml-cpp tools)
target_link_libraries(split_video ${OpenCV_LIBS} fmt::fmt tools)

##################tests##################
add_executable(auto_aim_test tests/auto_aim_test.cpp)
add_executable(auto_buff_test tests/auto_buff_test.cpp)
add_executable(bullet_detection_test tests/bullet_detection_test.cpp)
add_executable(camera_detect_test tests/camera_detect_test.cpp)
add_executable(camera_test tests/camera_test.cpp)
add_executable(camera_thread_test tests/camera_thread_test.cpp)
add_executable(cboard_test tests/cboard_test.cpp)
add_executable(fire_test tests/fire_test.cpp)
add_executable(detector_video_test tests/detector_video_test.cpp)
add_executable(gimbal_response_test tests/gimbal_response_test.cpp)
add_executable(multi_usbcamera_test tests/multi_usbcamera_test.cpp)
add_executable(usbcamera_detect_test tests/usbcamera_detect_test.cpp)
add_executable(usbcamera_test tests/usbcamera_test.cpp)
add_executable(handeye_test tests/handeye_test.cpp)
add_executable(dm_test tests/dm_test.cpp)
add_executable(minimum_vision_system tests/minimum_vision_system.cpp)

target_link_libraries(auto_aim_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(auto_buff_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_buff tools io)
target_link_libraries(bullet_detection_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim bullet_detector)
target_link_libraries(camera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim io)
target_link_libraries(camera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(camera_thread_test ${OpenCV_LIBS} fmt::fmt auto_aim tools io)
target_link_libraries(cboard_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(fire_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(detector_video_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools auto_aim)
target_link_libraries(gimbal_response_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(multi_usbcamera_test ${OpenCV_LIBS} fmt::fmt tools io)
target_link_libraries(usbcamera_detect_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(usbcamera_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(handeye_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)
target_link_libraries(dm_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io)
target_link_libraries(minimum_vision_system ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

add_executable(gimbal_test tests/gimbal_test.cpp)
target_link_libraries(gimbal_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(gimbal_serial_standalone tests/gimbal_serial_standalone.cpp)
target_link_libraries(gimbal_serial_standalone ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test tests/planner_test.cpp)
target_link_libraries(planner_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_test_offline tests/planner_test_offline.cpp)
target_link_libraries(planner_test_offline ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(planner_visualization_test tests/planner_visualization_test.cpp)
target_link_libraries(planner_visualization_test ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

add_executable(math_tools_test tests/math_tools_test.cpp)
target_link_libraries(math_tools_test ${OpenCV_LIBS} fmt::fmt tools)

add_executable(yolo_door_test tests/yolo_door_test.cpp)
target_link_libraries(yolo_door_test ${OpenCV_LIBS} fmt::fmt yaml-cpp tools io auto_aim)

# 检查 ROS 环境
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(std_msgs QUIET)
find_package(rosidl_typesupport_cpp QUIET)
find_package(rosidl_default_generators QUIET)
find_package(sp_msgs QUIET)
find_package(geometry_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(tf2_msgs QUIET)
find_package(visualization_msgs QUIET)

# 只有在找到 ROS 环境时才会添加 sentry 和 publish_test
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    # 编译需要 sp_msgs 的程序
    add_executable(sentry src/sentry.cpp)
    add_executable(sentry_bp src/sentry_bp.cpp)
    add_executable(sentry_debug src/sentry_debug.cpp)
    add_executable(sentry_multithread src/sentry_multithread.cpp)
    add_executable(publish_test tests/publish_test.cpp)
    add_executable(subscribe_test tests/subscribe_test.cpp)
    add_executable(topic_loop_test tests/topic_loop_test.cpp)

    target_link_libraries(sentry ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_bp ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_debug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(sentry_multithread ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim omniperception tools io)
    target_link_libraries(publish_test ${OpenCV_LIBS} fmt::fmt tools io auto_aim)
    target_link_libraries(subscribe_test ${OpenCV_LIBS} fmt::fmt tools io)
    target_link_libraries(topic_loop_test ${OpenCV_LIBS} fmt::fmt tools io)
endif()

# 添加诊断信息
message(STATUS "=== ROS2 依赖状态诊断 ===")
message(STATUS "ament_cmake_FOUND: ${ament_cmake_FOUND}")
message(STATUS "rclcpp_FOUND: ${rclcpp_FOUND}")
message(STATUS "std_msgs_FOUND: ${std_msgs_FOUND}")
message(STATUS "geometry_msgs_FOUND: ${geometry_msgs_FOUND}")
message(STATUS "sensor_msgs_FOUND: ${sensor_msgs_FOUND}")
message(STATUS "tf2_msgs_FOUND: ${tf2_msgs_FOUND}")
message(STATUS "visualization_msgs_FOUND: ${visualization_msgs_FOUND}")
message(STATUS "rosidl_typesupport_cpp_FOUND: ${rosidl_typesupport_cpp_FOUND}")
message(STATUS "auto_aim_interfaces_FOUND: ${auto_aim_interfaces_FOUND}")
message(STATUS "=========================")

# 修改条件判断，使其更加健壮 - 即使auto_aim_interfaces未找到也尝试构建
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND AND geometry_msgs_FOUND)
    message(STATUS "ROS2 dependencies found, building ROS2-related executables")
    
    # 跳过有问题的find_package调用
    # find_package(auto_aim_interfaces QUIET HINTS ${CMAKE_CURRENT_BINARY_DIR}/auto_aim_interfaces)
    
    # 强制添加包含路径，确保能找到生成的头文件
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../install/auto_aim_interfaces/include")
    include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../../install/auto_aim_interfaces/include/auto_aim_interfaces")
    
    # 添加 auto_aim_debug_mpc_rosdebug 可执行文件
    add_executable(auto_aim_debug_mpc_rosdebug src/auto_aim_debug_mpc_rosdebug.cpp)
    
    # 添加基本依赖
    target_link_libraries(auto_aim_debug_mpc_rosdebug ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
    
    # 添加 ROS 依赖
    ament_target_dependencies(auto_aim_debug_mpc_rosdebug 
        rclcpp 
        std_msgs 
        geometry_msgs 
        sensor_msgs 
        tf2_msgs 
        visualization_msgs
    )
    
    # 直接链接类型支持库（如果存在）
    set(AUTO_AIM_INTERFACES_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install/auto_aim_interfaces/lib")
    if(EXISTS "${AUTO_AIM_INTERFACES_LIB_DIR}")
        # 查找所有类型支持库文件
        file(GLOB AUTO_AIM_INTERFACES_LIBS "${AUTO_AIM_INTERFACES_LIB_DIR}/libauto_aim_interfaces__rosidl_typesupport_*.so")
        if(AUTO_AIM_INTERFACES_LIBS)
            message(STATUS "Linking against auto_aim_interfaces typesupport libraries: ${AUTO_AIM_INTERFACES_LIBS}")
            target_link_libraries(auto_aim_debug_mpc_rosdebug ${AUTO_AIM_INTERFACES_LIBS})
        else()
            message(WARNING "No auto_aim_interfaces typesupport libraries found in ${AUTO_AIM_INTERFACES_LIB_DIR}")
        endif()
    else()
        message(WARNING "auto_aim_interfaces lib directory not found: ${AUTO_AIM_INTERFACES_LIB_DIR}")
    endif()
    
    # 添加 auto_aim_test_ros 可执行文件
    add_executable(auto_aim_test_ros tests/auto_aim_test_ros.cpp)
    
    # 添加基本依赖
    target_link_libraries(auto_aim_test_ros ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)
    
    # 添加 ROS 依赖
    ament_target_dependencies(auto_aim_test_ros 
        rclcpp 
        std_msgs 
        geometry_msgs 
        sensor_msgs 
        tf2_msgs 
        visualization_msgs
    )
    
    # 直接链接类型支持库（如果存在）
    set(AUTO_AIM_INTERFACES_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install/auto_aim_interfaces/lib")
    if(EXISTS "${AUTO_AIM_INTERFACES_LIB_DIR}")
        # 查找所有类型支持库文件
        file(GLOB AUTO_AIM_INTERFACES_LIBS "${AUTO_AIM_INTERFACES_LIB_DIR}/libauto_aim_interfaces__rosidl_typesupport_*.so")
        if(AUTO_AIM_INTERFACES_LIBS)
            message(STATUS "Linking against auto_aim_interfaces typesupport libraries for auto_aim_test_ros: ${AUTO_AIM_INTERFACES_LIBS}")
            target_link_libraries(auto_aim_test_ros ${AUTO_AIM_INTERFACES_LIBS})
        else()
            message(WARNING "No auto_aim_interfaces typesupport libraries found in ${AUTO_AIM_INTERFACES_LIB_DIR}")
        endif()
    else()
        message(WARNING "auto_aim_interfaces lib directory not found: ${AUTO_AIM_INTERFACES_LIB_DIR}")
    endif()
    
# 添加 auto_aim_debug_mpc_rosbc 可执行文件
    add_executable(auto_aim_debug_mpc_rosbc src/auto_aim_debug_mpc_rosbc.cpp)

    # 添加基本依赖
    target_link_libraries(auto_aim_debug_mpc_rosbc ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io bullet_detector)

    # 添加 ROS 依赖
    ament_target_dependencies(auto_aim_debug_mpc_rosbc
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        tf2_msgs
        visualization_msgs
    )

    # 直接链接类型支持库（如果存在）
    set(AUTO_AIM_INTERFACES_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install/auto_aim_interfaces/lib")
    if(EXISTS "${AUTO_AIM_INTERFACES_LIB_DIR}")
        # 查找所有类型支持库文件
        file(GLOB AUTO_AIM_INTERFACES_LIBS "${AUTO_AIM_INTERFACES_LIB_DIR}/libauto_aim_interfaces__rosidl_typesupport_*.so")
        if(AUTO_AIM_INTERFACES_LIBS)
            message(STATUS "Linking against auto_aim_interfaces typesupport libraries for auto_aim_debug_mpc_rosbc: ${AUTO_AIM_INTERFACES_LIBS}")
            target_link_libraries(auto_aim_debug_mpc_rosbc ${AUTO_AIM_INTERFACES_LIBS})
        else()
            message(WARNING "No auto_aim_interfaces typesupport libraries found in ${AUTO_AIM_INTERFACES_LIB_DIR}")
        endif()
    else()
        message(WARNING "auto_aim_interfaces lib directory not found: ${AUTO_AIM_INTERFACES_LIB_DIR}")
    endif()
    
    # 添加 auto_aim_debug_mpcst_ros 可执行文件
    add_executable(auto_aim_debug_mpcst_ros src/auto_aim_debug_mpcst_ros.cpp)

    # 添加基本依赖
    target_link_libraries(auto_aim_debug_mpcst_ros ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

    # 添加 ROS 依赖
    ament_target_dependencies(auto_aim_debug_mpcst_ros
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        tf2_msgs
        visualization_msgs
    )

    # 直接链接类型支持库（如果存在）
    set(AUTO_AIM_INTERFACES_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install/auto_aim_interfaces/lib")
    if(EXISTS "${AUTO_AIM_INTERFACES_LIB_DIR}")
        # 查找所有类型支持库文件
        file(GLOB AUTO_AIM_INTERFACES_LIBS "${AUTO_AIM_INTERFACES_LIB_DIR}/libauto_aim_interfaces__rosidl_typesupport_*.so")
        if(AUTO_AIM_INTERFACES_LIBS)
            message(STATUS "Linking against auto_aim_interfaces typesupport libraries for auto_aim_debug_mpcst_ros: ${AUTO_AIM_INTERFACES_LIBS}")
            target_link_libraries(auto_aim_debug_mpcst_ros ${AUTO_AIM_INTERFACES_LIBS})
        else()
            message(WARNING "No auto_aim_interfaces typesupport libraries found in ${AUTO_AIM_INTERFACES_LIB_DIR}")
        endif()
    else()
        message(WARNING "auto_aim_interfaces lib directory not found: ${AUTO_AIM_INTERFACES_LIB_DIR}")
    endif()
    
    # 添加 sentry_debug_mpc_ros 可执行文件
    add_executable(sentry_debug_mpc_ros src/sentry_debug_mpc_ros.cpp)

    # 添加基本依赖
    target_link_libraries(sentry_debug_mpc_ros ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io)

    # 添加 ROS 依赖
    ament_target_dependencies(sentry_debug_mpc_ros
        rclcpp
        std_msgs
        geometry_msgs
        sensor_msgs
        tf2_msgs
        visualization_msgs
    )

    # 直接链接类型支持库（如果存在）
    set(AUTO_AIM_INTERFACES_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install/auto_aim_interfaces/lib")
    if(EXISTS "${AUTO_AIM_INTERFACES_LIB_DIR}")
        # 查找所有类型支持库文件
        file(GLOB AUTO_AIM_INTERFACES_LIBS "${AUTO_AIM_INTERFACES_LIB_DIR}/libauto_aim_interfaces__rosidl_typesupport_*.so")
        if(AUTO_AIM_INTERFACES_LIBS)
            message(STATUS "Linking against auto_aim_interfaces typesupport libraries for sentry_debug_mpc_ros: ${AUTO_AIM_INTERFACES_LIBS}")
            target_link_libraries(sentry_debug_mpc_ros ${AUTO_AIM_INTERFACES_LIBS})
        else()
            message(WARNING "No auto_aim_interfaces typesupport libraries found in ${AUTO_AIM_INTERFACES_LIB_DIR}")
        endif()
    else()
        message(WARNING "auto_aim_interfaces lib directory not found: ${AUTO_AIM_INTERFACES_LIB_DIR}")
    endif()
    
    # 添加 bullet_dectetor_rostest 可执行文件
    add_executable(bullet_dectetor_rostest tests/bullet_dectetor_rostest.cpp)
    
    # 添加基本依赖
    target_link_libraries(bullet_dectetor_rostest ${OpenCV_LIBS} fmt::fmt yaml-cpp auto_aim tools io bullet_detector)
    
    # 添加 ROS 依赖
    ament_target_dependencies(bullet_dectetor_rostest 
        rclcpp 
        std_msgs 
        geometry_msgs 
        sensor_msgs 
        tf2_msgs 
        visualization_msgs
    )
    
    # 直接链接类型支持库（如果存在）
    set(AUTO_AIM_INTERFACES_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../install/auto_aim_interfaces/lib")
    if(EXISTS "${AUTO_AIM_INTERFACES_LIB_DIR}")
        # 查找所有类型支持库文件
        file(GLOB AUTO_AIM_INTERFACES_LIBS "${AUTO_AIM_INTERFACES_LIB_DIR}/libauto_aim_interfaces__rosidl_typesupport_*.so")
        if(AUTO_AIM_INTERFACES_LIBS)
            message(STATUS "Linking against auto_aim_interfaces typesupport libraries for bullet_dectetor_rostest: ${AUTO_AIM_INTERFACES_LIBS}")
            target_link_libraries(bullet_dectetor_rostest ${AUTO_AIM_INTERFACES_LIBS})
        else()
            message(WARNING "No auto_aim_interfaces typesupport libraries found in ${AUTO_AIM_INTERFACES_LIB_DIR}")
        endif()
    else()
        message(WARNING "auto_aim_interfaces lib directory not found: ${AUTO_AIM_INTERFACES_LIB_DIR}")
    endif()
else()
    message(WARNING "ROS2 dependencies not fully found, skipping ROS2-related executables")
    # 即使条件不满足，也添加目标但不构建，这样至少可以看到目标存在
    add_custom_target(auto_aim_test_ros COMMENT "auto_aim_test_ros requires ROS2 dependencies")
    add_custom_target(auto_aim_debug_mpc_rosdebug COMMENT "auto_aim_debug_mpc_rosdebug requires ROS2 dependencies")
    add_custom_target(auto_aim_debug_mpc_rosbc COMMENT "auto_aim_debug_mpc_rosbc requires ROS2 dependencies")
    add_custom_target(auto_aim_debug_mpcst_ros COMMENT "auto_aim_debug_mpcst_ros requires ROS2 dependencies")
    add_custom_target(sentry_debug_mpc_ros COMMENT "sentry_debug_mpc_ros requires ROS2 dependencies")
    add_custom_target(bullet_dectetor_rostest COMMENT "bullet_dectetor_rostest requires ROS2 dependencies")
endif()

if(NOT ament_cmake_FOUND OR NOT rclcpp_FOUND OR NOT std_msgs_FOUND OR NOT rosidl_typesupport_cpp_FOUND)
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()
