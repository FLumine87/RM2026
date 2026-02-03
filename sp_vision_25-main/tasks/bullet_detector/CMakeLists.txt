cmake_minimum_required(VERSION 3.16)

add_library(bullet_detector OBJECT 
    aim_corrector.cpp
    detect_bullet.cpp
    do_reproj.cpp
)

target_link_libraries(bullet_detector ${OpenCV_LIBS} Eigen3::Eigen)
