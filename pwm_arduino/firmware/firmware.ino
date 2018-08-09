#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Trigger.h>
#include "IMUHandle.h"

#define G_TO_M 9.81

void gyro_cal_cb(const std_srvs::Trigger::Request& const std_srvs::Trigger::Response& res);
void mag_cal_start_cb(const std_srvs::Trigger::Request& const std_srvs::Trigger::Response& res);
void mag_cal_stop_cb(const std_srvs::Trigger::Request& const std_srvs::Trigger::Response& res);

IMUHandle imu(12);

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu", &imu_msg);
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> server("imu_gyro_cal", &gyro_cal_cb);
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> server("imu_mag_cal_start", &mag_cal_start_cb);
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> server("imu_mag_cal_stop", &mag_cal_stop_cb);

void setup()
{
  imu.setup();

  float dmx, dmy, dmz;
  nh.getParam("mag_bias_x", &dmx, 137.101);
  nh.getParam("mag_bias_y", &dmy, 265.25);
  nh.getParam("mag_bias_z", &dmz, 513.14);
  
  imu.set_mag_bias(dmx, dmy, dmz);
  imu.set_mag_scale(0.99, 0.58, 3.84); //??
  
  nh.initNode();
  nh.advertise(imu_pub);
}

void loop()
{
  nh.spinOnce();
  imu.read();


  imu_msg.header.stamp = nh.now();
  imu_msg.header.seq += 1;
  imu_msg.header.frame_id = "imu";

  imu_msg.orientation.x = imu.qx;
  imu_msg.orientation.y = imu.qy;
  imu_msg.orientation.z = imu.qz;
  imu_msg.orientation.w = imu.qw;

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
}

void gyro_cal_cb(const std_srvs::Trigger::Request& const std_srvs::Trigger::Response& res){
	res.success = imu.gyro_cal();
}

void mag_cal_start_cb(const std_srvs::Trigger::Request&, const std_srvs::Trigger::Response& res){
	res.success = imu.mag_cal_start();
}

void mag_cal_stop_cb(const std_srvs::Trigger::Request&, const std_srvs::Trigger::Response& res){
	res.success = imu.mag_cal_stop();
}
