#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include "tools/math_tools.hpp"

int main() {
    std::cout << "请输入数据（ypr 或 3x3 旋转矩阵）：" << std::endl;
    std::cout << "ypr 格式：yaw pitch roll" << std::endl;
    std::cout << "旋转矩阵格式：9个数，空格分隔（可在一行输入）" << std::endl;
    
    std::vector<double> input;
    double num;
    
    // 读取用户输入
    while (std::cin >> num) {
        input.push_back(num);
    }
    
    // 自动识别输入类型
    if (input.size() == 3) {
        // 输入是 ypr
        Eigen::Vector3d ypr(input[0], input[1], input[2]);
        std::cout << "输入的 ypr: " << ypr.transpose() << std::endl;
        
        Eigen::Matrix3d R = tools::rotation_matrix(ypr);
        std::cout << "输出旋转矩阵: " << std::endl;
        std::cout << R << std::endl;
    } else if (input.size() == 9) {
        // 输入是 3x3 旋转矩阵
        Eigen::Matrix3d R;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R(i, j) = input[i * 3 + j];
            }
        }
        std::cout << "输入的旋转矩阵: " << std::endl;
        std::cout << R << std::endl;
        
        Eigen::Vector3d ypr_out = tools::eulers(R, 2, 1, 0, false);
        std::cout << "输出 ypr: " << ypr_out.transpose() << std::endl;
    } else {
        std::cout << "输入格式错误，请输入 3 个数（ypr）或 9 个数（旋转矩阵）" << std::endl;
    }
    
    return 0;
}