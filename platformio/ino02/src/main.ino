#define USE_USBCON // for atmega32u4

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Twist.h>
#include <imu.hpp>
#include <thruster.hpp>

/**
 * @brief create device object
 */
Imu imu;

Thruster thruster1(MOTOR_RIGHT_FRONT_PWM_PIN );
Thruster thruster2(MOTOR_RIGHT_CENTER_PWM_PIN);
Thruster thruster3(MOTOR_RIGHT_REAR_PWM_PIN  );
Thruster thruster4(MOTOR_LEFT_FRONT_PWM_PIN  );
Thruster thruster5(MOTOR_LEFT_CENTER_PWM_PIN );
Thruster thruster6(MOTOR_LEFT_REAR_PWM_PIN   );

/**
 * @brief ros setting
 */
ros::NodeHandle  nh;
sensor_msgs::Imu imuMsg;
sensor_msgs::MagneticField magMsg;
geometry_msgs::Twist msg;

ros::Publisher pubimu("imu/data_raw", &imuMsg);
ros::Publisher pubMag("imu/mag", &magMsg);

void callback(const geometry_msgs::Twist& cmd_vel)
{
  thruster1.setRevCmd(cmd_vel.linear.x );
  thruster2.setRevCmd(cmd_vel.linear.y );
  thruster3.setRevCmd(cmd_vel.linear.z );
  thruster4.setRevCmd(cmd_vel.angular.x);
  thruster5.setRevCmd(cmd_vel.angular.y);
  thruster6.setRevCmd(cmd_vel.angular.z);
}

ros::Subscriber <geometry_msgs::Twist> sub("cmd_vel", callback);

/**
 * @brief 定周期変数
 */
unsigned long prev, interval;

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pubimu);
  nh.advertise(pubMag);
  nh.subscribe(sub);

  imu.init();
  thruster1.init();
  thruster2.init();
  thruster3.init();
  thruster4.init();
  thruster5.init();
  thruster6.init();
}

void loop()
{
  imu.update();

  imuMsg.header.frame_id = "imu_link";
  imuMsg.header.stamp = nh.now();
  imuMsg.linear_acceleration.x = imu.ax;
  imuMsg.linear_acceleration.y = imu.ay;
  imuMsg.linear_acceleration.z = imu.az;
  imuMsg.angular_velocity.x = imu.gx;
  imuMsg.angular_velocity.y = imu.gy;
  imuMsg.angular_velocity.z = imu.gz;
  pubimu.publish(&imuMsg);

  magMsg.header.frame_id = "mag_link";
  magMsg.header.stamp = nh.now();
  magMsg.magnetic_field.x = imu.mx;
  magMsg.magnetic_field.y = imu.my;
  magMsg.magnetic_field.z = imu.mz;
  pubMag.publish(&magMsg);

  nh.spinOnce();
  delay(500);
}
