#include "kalman.hpp"
#include <iostream>


int main(){

    float true_velocity = 1.0;
    float process_noise_std = 0.5;
    float sensor_noise_std = 2;
    int true_x = 0;

    Kalman filter (true_velocity, process_noise_std, sensor_noise_std, 100);

    float x = 0.0;
    float P = 1.0;
    filter.setTrue(true_x);
    filter.KF(x, P);



    

}