#include "kalman.hpp"
#include <random>


void Kalman::setTrue(float true_x){
    std::mt19937 gen(42);
    std::normal_distribution<double> dist(0.0, 1.0);
    for(int i=0; i < n; i++){

        true_x = true_x + u + dist(gen) * process_noise;
        float z = true_x + dist(gen) * sensor_noise;
        true_positions.push_back(true_x);
        measurements.push_back(z); 
    }
}

void Kalman::KF(float x, float P){
    for(int i = 0; i < n; i++){
        //Predict
        x = x + u;
        P = P + Q;


        //Update
        float z = measurements[i];
        float K = P / (P + R);
        x = x + K * (z - x);
        P = (1 - K) * P;
        
        estimates.push_back(x);
    }
}




