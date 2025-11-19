cmake_minimum_required(VERSION 3.10)
project(fire_control)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN rm_fire_control::FireControlNode
  EXECUTABLE ${PROJECT_NAME}_node
)

# # 测试solver
# add_executable(solver_demo src/solver.cpp) 

# target_link_libraries(solver_demo  
#   ${PROJECT_NAME})


# # 安装可执行文件  
# install(TARGETS solver_demo  
#   DESTINATION lib/${PROJECT_NAME}  
# )  
#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_copyright
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  # find_package(ament_cmake_gtest)
  # set(TEST_NAME test_kalman_filter)
  # ament_add_gtest(${TEST_NAME} test/${TEST_NAME}.cpp)
  # target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
endif()

#############
## Install ##
#############

# test node
install(PROGRAMS
  test/test_fire_control.py
  DESTINATION lib/${PROJECT_NAME}
)

#  launch 
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_auto_package()
