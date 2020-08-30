#define USE_USBCON // for atmega32u4

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <imu.hpp>

Imu imu;

ros::NodeHandle  nh;

sensor_msgs::Imu imuMsg;
sensor_msgs::MagneticField magMsg;

ros::Publisher pubimu("imu/data_raw", &imuMsg);
ros::Publisher pubMag("imu/mag", &magMsg);

void setup()
{
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(pubimu);
  nh.advertise(pubMag);
  imu.init();
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
