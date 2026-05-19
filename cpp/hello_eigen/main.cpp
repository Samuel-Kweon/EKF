#include <Eigen/Dense>
#include <iostream>


int main() {
    Eigen::Matrix3d R;
    R << 0, -1, 0,
         1,  0, 0,
         0,  0, 1;

    Eigen::Vector3d v(1,0,0);

    Eigen::Vector3d result = R * v;


    std::cout << "R =\n" << R << "\n\n";
    std::cout << "v = " << v.transpose() << "\n";
    std::cout << "R * v = " << result.transpose() << "\n";
    std::cout << "R^T * R =\n" << R.transpose() * R << "\n";

    return 0;
}