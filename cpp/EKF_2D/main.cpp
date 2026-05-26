#include "ekf.hpp"
#include <iostream>
#include <cmath>
#include <random>

int main(){
    const double dt = 0.1;
    const int n_steps = 1000;
    Eigen::Vector2d true_velocity(0.5, 0.3);
    const double sensor_noise_std = 0.5;

    std::default_random_engine gen(42);
    std::normal_distribution<double> noise(0.0, sensor_noise_std);

    auto f = [](const Eigen::VectorXd& x, const Eigen::VectorXd& u) -> Eigen::VectorXd {
        return x + u;
    };

    auto F_jac = [](const Eigen::MatrixXd& x, const Eigen::VectorXd& u) -> Eigen::MatrixXd {
        return Eigen::MatrixXd::Identity(x.rows(), x.rows());
    };

    auto h = [](const Eigen::VectorXd& x) -> Eigen::VectorXd {
        Eigen::VectorXd z(1);
        z(0) = std::sqrt(x(0)*x(0) + x(1)*x(1));
        return z;
    };

    auto H_jac = [](const Eigen::VectorXd& x) -> Eigen::MatrixXd {
        double r = std::sqrt(x(0) * x(0) + x(1) * x(1));
        Eigen::MatrixXd H(1,2);
        H << x(0)/r, x(1)/r;
        return H;
    };

    Eigen::VectorXd x_init(2);
    x_init << 4.0, 4.0;


    Eigen::MatrixXd P_init = Eigen::MatrixXd::Identity(2,2) * 5.0;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(2,2) * 0.01;
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1,1) * (sensor_noise_std * sensor_noise_std);
    
    EKF ekf(x_init, P_init, Q, R, f, F_jac, h, H_jac);

    Eigen::Vector2d true_pos(5.0, 5.0);
    Eigen::VectorXd u(2);
    u << true_velocity(0) * dt, true_velocity(1) * dt;

    for(int i=0; i<n_steps; i++){
        
        true_pos += true_velocity * dt;
        
        double true_range = true_pos.norm();
        Eigen::VectorXd z(1);
        z(0) = true_range + noise(gen);
        
        ekf.predict(u);
        ekf.update(z);
    }


    // === Print results ===
    Eigen::VectorXd final_state = ekf.getState();
    std::cout << "True final position:      " << true_pos.transpose() << "\n";
    std::cout << "Estimated final position: " << final_state.transpose() << "\n";
    double error = (final_state - true_pos).norm();
    std::cout << "Final position error: " << error << "\n";

    return 0;






}