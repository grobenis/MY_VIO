//
// Created by hyj on 17-6-22.
//

#ifndef IMUSIM_PARAM_H
#define IMUSIM_PARAM_H

#include <eigen3/Eigen/Core>

class Param{

public:

    Param();

    // time
    int imu_frequency = 200;    //IMU频率
    int cam_frequency = 30;     //相机频率
    double imu_timestep = 1./imu_frequency; //IMU一步时长
    double cam_timestep = 1./cam_frequency; //相机一步时长
    double t_start = 0.;    //开始点
    double t_end = 20;  //  持续时间20s

    // noise
    double gyro_bias_sigma = 1.0e-5; //陀螺仪偏差
    double acc_bias_sigma = 0.0001; //加速度计的偏差

    double gyro_noise_sigma = 0.015;    // rad/s 陀螺仪噪声方差
    double acc_noise_sigma = 0.019;      //　m/(s^2) 加速度计噪声方差

    double pixel_noise = 1;              // 1 pixel noise 

    // cam f 相机参数
    double fx = 460;
    double fy = 460;
    double cx = 255;
    double cy = 255;
    double image_w = 640;
    double image_h = 640;

    // 外参数
    Eigen::Matrix3d R_bc;   // cam to body 旋转矩阵
    Eigen::Vector3d t_bc;     // cam to body 平移向量

};


#endif //IMUSIM_PARAM_H
