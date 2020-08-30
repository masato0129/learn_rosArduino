/**
 * @file defineParams.h
 * @author masato ogura 
 * @brief パラメータ定義
 * @version 0.1
 * @date 2019-03-31
 * 
 * @copyright Copyright (c) 2019 masato ogura
 * 
 */

#ifndef __DEFINEPARAMS_H__
#define __DEFINEPARAMS_H__

// 推進機
const uint8_t MOTOR_RIGHT_FRONT_PWM_PIN  = 5;
const uint8_t MOTOR_RIGHT_CENTER_PWM_PIN = 6;
const uint8_t MOTOR_RIGHT_REAR_PWM_PIN   = 9;
const uint8_t MOTOR_LEFT_FRONT_PWM_PIN   = 10;
const uint8_t MOTOR_LEFT_CENTER_PWM_PIN  = 11;
const uint8_t MOTOR_LEFT_REAR_PWM_PIN    = 13;

// pwm信号の上下限
const uint32_t MOTOR_SIGNAL_STOP = 1500; 
const uint32_t MOTOR_SIGNAL_MAX = 1900; 
const uint32_t MOTOR_SIGNAL_MIN = 1100; 

// imu関係
#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1
#define AHRS true         // Set to false for basic data read

// yaw角のオフセット
// Tokyo tama river (35° 35' 21" N 139° 39' 50" E) is
//    7° 39' W  ± 0° 19'()  changing by  0° 4' W per year
// - http://www.ngdc.noaa.gov/geomag-web/#declination
const float YAW_OFFSET = 7.65; 

// シリアル通信
#define BAUNDRATE 115200

// デバイスエラー
#define DVICE_ERORR 0xFF

// デバッグ関係
#define SerialDebug false

#endif // __DEFINEPARAMS_H__
