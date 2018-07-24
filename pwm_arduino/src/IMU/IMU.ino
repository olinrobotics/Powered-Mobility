
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include "IMUHandle.h"

ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("/imu", &imu_msg);
IMUHandle imu(12);

void setup()
{
  imu.setup();
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

  imu_msg.angular_velocity.x = imu.myIMU.gx;
  imu_msg.angular_velocity.y = imu.myIMU.gy;
  imu_msg.angular_velocity.z = imu.myIMU.gz;

  imu_msg.angular_velocity_covariance[0] = 0.01;
  imu_msg.angular_velocity_covariance[4] = 0.01;
  imu_msg.angular_velocity_covariance[8] = 0.01;

  imu_msg.linear_acceleration.x = imu.myIMU.ax;
  imu_msg.linear_acceleration.y = imu.myIMU.ay;
  imu_msg.linear_acceleration.z = imu.myIMU.az;

  imu_msg.linear_acceleration_covariance[0] = 0.01;
  imu_msg.linear_acceleration_covariance[4] = 0.01;
  imu_msg.linear_acceleration_covariance[8] = 0.01;

  imu_pub.publish(&imu_msg);
}
