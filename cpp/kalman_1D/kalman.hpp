#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <vector>


class Kalman {
public:

    Kalman(float velocity, float p_noise, float s_noise, int n_steps){
        Q = std::pow(p_noise, 2);
        R = std::pow(s_noise, 2);
        u = velocity;
        n = n_steps;
        process_noise = p_noise;
        sensor_noise = s_noise;
    }

    void setTrue(float true_x);

    void KF(float x, float P);

    



private:
    float process_noise;
    float sensor_noise;
    int n;
    float Q;
    float R;
    float u;
    std::vector<float> true_positions;
    std::vector<float> measurements;
    std::vector<float> estimates;
};