/**
 * @file imu.hpp
 * @author masato ogura
 * @brief 慣性航法センサクラス
 * @version 0.1
 * @date 2019-03-31
 * 
 * @copyright Copyright (c) 2019 masato ogura
 * 
 */

#include <SparkFun_MPU-9250_Breakout_Arduino_Library/src/MPU9250.h>
#include <SparkFun_MPU-9250_Breakout_Arduino_Library/src/quaternionFilters.h>

#include <defineParams.h>


#ifndef __IMU_HPP__
#define __IMU_HPP__

class Imu
{
public:
    Imu();
    ~Imu();
    void init();
    void update();

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float mx;
    float my;
    float mz;

private:
    MPU9250* imu;
    void getAccelData(MPU9250* imu);
    void getGyroData(MPU9250* imu);
    void getMagData(MPU9250* imu);
    float getTemplature(MPU9250* imu);
};

#endif // __IMU_HPP__
