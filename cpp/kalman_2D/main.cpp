#include "Kalman.hpp"
#include <iostream>
#include <random>


int main(){
    // Setup
    double dt = 1.0;
    int n_steps = 100;

    Eigen::Vector2d true_x(0.0, 0.0);
    Eigen::Vector2d true_v(1.0, 0.5);
    double sensor_noise_std = 1.0;


    // Random number generator
    std::default_random_engine generator(42);  // seed
    std::normal_distribution<double> noise(0.0, sensor_noise_std);

    Eigen::VectorXd x_init(4);
    x_init << 0, 0, 0, 0;
    Eigen::MatrixXd P = Eigen::MatrixXd::Identity(4,4) * 10.0;

    Eigen::Matrix4d F;
    F << 1, 0, dt,  0,
         0, 1,  0, dt,
         0, 0,  1,  0, 
         0, 0,  0,  1;


    Eigen::MatrixXd H(2,4);
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    Eigen::MatrixXd Q = Eigen::Matrix4d::Identity(4,4) * 0.01;

    Eigen::MatrixXd R = Eigen::Matrix2d::Identity(2,2) * (sensor_noise_std * sensor_noise_std);

    KalmanFilter kf(x_init, P, F, H, Q, R);

    for(int i=0; i < n_steps; i++){
        true_x = true_x + true_v * dt;
        Eigen::Vector2d measurement_noise;
        measurement_noise << noise(generator),
                             noise(generator);
        Eigen::VectorXd z = true_x + measurement_noise;

        kf.predict();
        kf.update(z);
    }

       // === Print final estimate ===
    Eigen::VectorXd final_state = kf.getState();
    std::cout << "True final position: " << true_x.transpose() << "\n";
    std::cout << "Estimated final state:\n";
    std::cout << "  position: " << final_state.head<2>().transpose() << "\n";
    std::cout << "  velocity: " << final_state.tail<2>().transpose() << "\n";
    std::cout << "True velocity: " << true_v.transpose() << "\n";

    return 0; 
}