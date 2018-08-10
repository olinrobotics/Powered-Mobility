#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include <std_srvs/Trigger.h>
#include "IMUHandle.h"

#define G_TO_M 9.81

void gyro_cal_cb(const std_srvs::Trigger::Request &,  std_srvs::Trigger::Response& res);
void mag_cal_start_cb(const std_srvs::Trigger::Request &,  std_srvs::Trigger::Response& res);
void mag_cal_stop_cb(const std_srvs::Trigger::Request &,  std_srvs::Trigger::Response& res);

IMUHandle imu(18, 0);

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;

ros::Publisher imu_pub("/imu", &imu_msg);
ros::Publisher mag_pub("/imu/mag", &mag_msg);

ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> gyro_cal_srv("imu_gyro_cal", &gyro_cal_cb);
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> mag_cal_start_srv("imu_mag_cal_start", &mag_cal_start_cb);
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> mag_cal_stop_srv("imu_mag_cal_stop", &mag_cal_stop_cb);

void setup()
{
  imu.setup();

  //float dmx=137.1, dmy=265.25, dmz=513.14;
  float dmx=0,dmy=0,dmz=0;
  //  nh.getParam("mag_bias_x", &dmx, 137.101);
  //  nh.getParam("mag_bias_y", &dmy, 265.25);
  //  nh.getParam("mag_bias_z", &dmz, 513.14);

  imu.set_mag_bias(dmx, dmy, dmz);
  imu.set_mag_scale(0.99, 0.58, 3.84); //??

  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(mag_pub);
  nh.advertiseService(gyro_cal_srv);
  nh.advertiseService(mag_cal_start_srv);
  nh.advertiseService(mag_cal_stop_srv);
}

void loop()
{
  nh.spinOnce();
  imu.step();

  imu_msg.header.stamp = nh.now();
  imu_msg.header.seq += 1;
  imu_msg.header.frame_id = "imu";


  imu_msg.orientation.w = imu._q[0];
  imu_msg.orientation.x = imu._q[1];
  imu_msg.orientation.y = imu._q[2];
  imu_msg.orientation.z = imu._q[3];

  // TODO : Estimate + fill covariances
  // TODO : Compute IMU Calibration + Biases

  imu_msg.orientation_covariance[0] = 0.01;
  imu_msg.orientation_covariance[4] = 0.01;
  imu_msg.orientation_covariance[8] = 0.01;

  imu_msg.angular_velocity.x = imu._device.gx * DEG_TO_RAD;
  imu_msg.angular_velocity.y = imu._device.gy * DEG_TO_RAD;
  imu_msg.angular_velocity.z = imu._device.gz * DEG_TO_RAD;

  imu_msg.angular_velocity_covariance[0] = 0.01;
  imu_msg.angular_velocity_covariance[4] = 0.01;
  imu_msg.angular_velocity_covariance[8] = 0.01;

  imu_msg.linear_acceleration.x = imu._device.ax * G_TO_M;
  imu_msg.linear_acceleration.y = imu._device.ay * G_TO_M;
  imu_msg.linear_acceleration.z = imu._device.az * G_TO_M;

  imu_msg.linear_acceleration_covariance[0] = 0.01;
  imu_msg.linear_acceleration_covariance[4] = 0.01;
  imu_msg.linear_acceleration_covariance[8] = 0.01;

  imu_pub.publish(&imu_msg);

  
  mag_msg.header.stamp = nh.now();
  mag_msg.header.seq += 1;
  mag_msg.header.frame_id = "imu";

  mag_msg.magnetic_field.x = imu._device.mx;
  mag_msg.magnetic_field.y = imu._device.my;
  mag_msg.magnetic_field.z = imu._device.mz;
  mag_pub.publish(&mag_msg);
}

void gyro_cal_cb(const std_srvs::Trigger::Request &, std_srvs::Trigger::Response& res) {
  res.success = imu.gyro_cal();
}

void mag_cal_start_cb(const std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
  res.success = imu.mag_cal_start();
}

void mag_cal_stop_cb(const std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
  res.success = imu.mag_cal_stop();
}
