/**
 * @file imu.cpp
 * @author masato ogura
 * @brief 慣性航法装置クラス
 * @version 0.1
 * @date 2019-04-06
 * 
 * @copyright Copyright (c) 2019 masato ogura
 * 
 */

#include "imu.hpp"

/**
 * @brief Construct a new Imu:: Imu object
 * 
 */
Imu::Imu(){
    imu = new MPU9250(MPU9250_ADDRESS, I2Cport, I2Cclock);
}

/**
 * @brief Destroy the Imu:: Imu object
 * 
 */
Imu::~Imu(){
    // do nothing
    delete imu;
}

/**
 * @brief 温度データ取得
 * 
 * @param imu 
 * @return float 
 */
float Imu::getTemplature(MPU9250* imu)
{
    int templatureCount = imu->readTempData();
    return ((float) templatureCount) / 333.87 + 21.0;
}

/**
 * @brief 加速度データ取得
 * 
 * @param imu 
 * @return void 
 */
void Imu::getAccelData(MPU9250* imu)
{
    imu->readAccelData(imu->accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    imu->ax = (float)imu->accelCount[0] * imu->aRes; // - imu->accelBias[0];
    imu->ay = (float)imu->accelCount[1] * imu->aRes; // - imu->accelBias[1];
    imu->az = (float)imu->accelCount[2] * imu->aRes; // - imu->accelBias[2];
    ax = imu->ax;
    ay = imu->ay;
    az = imu->az;
}

/**
 * @brief 角速度取得
 * 
 * @param imu 
 * @return void 
 */
void Imu::getGyroData(MPU9250* imu)
{
    imu->readGyroData(imu->gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    imu->gx = (float)imu->gyroCount[0] * imu->gRes;
    imu->gy = (float)imu->gyroCount[1] * imu->gRes;
    imu->gz = (float)imu->gyroCount[2] * imu->gRes;

    gx = imu->gx;
    gy = imu->gy;
    gz = imu->gz;
}

/**
 * @brief 地磁気取得
 * 
 * @param imu 
 * @return void 
 */
void Imu::getMagData(MPU9250* imu)
{
    imu->readMagData(imu->magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    imu->mx = (float)imu->magCount[0] * imu->mRes
                * imu->factoryMagCalibration[0] - imu->magBias[0];
    imu->my = (float)imu->magCount[1] * imu->mRes
                * imu->factoryMagCalibration[1] - imu->magBias[1];
    imu->mz = (float)imu->magCount[2] * imu->mRes
                * imu->factoryMagCalibration[2] - imu->magBias[2];
    mx = imu->mx;
    my = imu->my;
    mz = imu->mz;

}

/**
 * @brief 初期化
 * 
 */
void Imu::init()
{
    byte c = imu->readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

    if (c == 0x71) // WHO_AM_I should always be 0x71
    {

        // Start by performing self test and reporting values
        imu->MPU9250SelfTest(imu->selfTest);

        // Calibrate gyro and accelerometers, load biases in bias registers
        imu->calibrateMPU9250(imu->gyroBias, imu->accelBias);
        imu->initMPU9250();
        // Initialize device for active mode read of acclerometer, gyroscope, and
        // temperature

        // Read the WHO_AM_I register of the magnetometer, this is a good test of
        // communication
        byte d = imu->readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

        if (d != 0x48)
        {
            // Communication failed, stop here
            Serial.println(F("Communication failed, abort!"));
            Serial.flush();
            abort();
        }

        // Get magnetometer calibration from AK8963 ROM
        imu->initAK8963(imu->factoryMagCalibration);

        // Get sensor resolutions, only need to do this once
        imu->getAres();
        imu->getGres();
        imu->getMres();
    } // if (c == 0x71)
    else
    {
        Serial.flush();
        abort();
    }
}

/**
 * @brief センサデータ更新
 * 
 */
void Imu::update()
{
    // If intPin goes high, all data registers have new data
    // On interrupt, check if data ready interrupt
    if (imu->readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
    {
        getAccelData(imu);
        getGyroData(imu);
        getMagData(imu);
        getTemplature(imu);

    } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
#if 0
    // Must be called before updating quaternions!
    imu->updateTime();

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    MahonyQuaternionUpdate(imu->ax, imu->ay, imu->az, imu->gx * DEG_TO_RAD,
                            imu->gy * DEG_TO_RAD, imu->gz * DEG_TO_RAD, imu->my,
                            imu->mx, imu->mz, imu->deltat);

    // Serial print and/or display at 0.5 s rate independent of data rates
    imu->delt_t = millis() - imu->count;

    // update LCD once per half-second independent of read rate
    if (imu->delt_t > 500)
    {
        // Define output variables from updated quaternion---these are Tait-Bryan
        // angles, commonly used in aircraft orientation. In this coordinate system,
        // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
        // x-axis and Earth magnetic North (or true North if corrected for local
        // declination, looking down on the sensor positive yaw is counterclockwise.
        // Pitch is angle between sensor x-axis and Earth ground plane, toward the
        // Earth is positive, up toward the sky is negative. Roll is angle between
        // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
        // arise from the definition of the homogeneous rotation matrix constructed
        // from quaternions. Tait-Bryan angles as well as Euler angles are
        // non-commutative; that is, the get the correct orientation the rotations
        // must be applied in the correct order which for this configuration is yaw,
        // pitch, and then roll.
        // For more see
        // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        // which has additional links.
        imu->yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                        * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                        * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                        * *(getQ()+3));
        imu->pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                        * *(getQ()+2)));
        imu->roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                        * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                        * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                        * *(getQ()+3));
        imu->pitch *= RAD_TO_DEG;
        imu->yaw   *= RAD_TO_DEG;

        // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
        //    8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
        // - http://www.ngdc.noaa.gov/geomag-web/#declination
        imu->yaw  -= 8.5;
        imu->roll *= RAD_TO_DEG;
        imu->count = millis();
        imu->sumCount = 0;
        imu->sum = 0;
    } // if (imu->delt_t > 500)
#endif
}
