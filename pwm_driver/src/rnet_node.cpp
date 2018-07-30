/* TEST :
(PC) rosrun	rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
(Pi) rosrun pwm_driver rnet_node.py 
*/ 

#include "roscan.h"
#include <thread>
#include <ctime>
#include <ratio>
#include <chrono>

/* ROS */
#include <ros.h>
#include <sensor_msgs/BatteryState.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>

std::chrono::high_resolution_clock::time_point sysnow();
float sysdt(std::chrono::high_resolution_clock::time_point& t0, std::chrono::high_resolution_clock::time_point& t1);

void cmd_vel_cb(const geometry_msgs::Twist& msg);

ros::NodeHandle nh;
sensor_msgs::BatteryState bat_msg;
ros::Publisher bat_pub("battery", &bat_msg);

geometry_msgs::Twist cmd_vel;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_vel_cb);
std::chrono::high_resolution_clock::time_point last_rcv = sysnow(); // last cmd_vel receive ( from master )
std::chrono::high_resolution_clock::time_point last_cmd = sysnow(); // last cmd_vel send ( to wheelchair )

float rcv_timeout     = 0.5;
float cmd_vel_rate    = 100; // 50 hz
float cmd_vel_period  = (1.0 / cmd_vel_rate);

char can_cmd_buf1[64];
char can_cmd_buf2[64];

//char* rosSrvrIp = "/dev/ttyS0";
//char* rosSrvrIp = "192.168.16.64";
char ros_port[] = "/dev/ttyS0";
char can_port[] = "can0";

float clip(float mn, float x, float mx){
	if(x<mn) return mn;
	if(mx<x) return mx;
	return x;
}

int main(){
	// ros first
	nh.initNode(ros_port);//, "11311");//11311);
	nh.advertise(bat_pub);
	nh.subscribe(cmd_vel_sub);

	ROSCan roscan(can_port);
	roscan.init();
	roscan.set_filter(0x1C0C0000, 0x1FFFF0FF); // battery
	//roscan.set_timeout(0.01);
	//std::thread b_thread(read_battery, &roscan);

	fd_set rdfs;
	timeval tv;
	can_frame cf;

	tv.tv_sec = 0;
	tv.tv_usec = 10000;

	std::chrono::high_resolution_clock::time_point last_send = sysnow();

	//disable joy
	for(int i=0; i<3; ++i){
		roscan.send("0C000000#");
	}

	while(1){//ros::ok()){
		std::chrono::high_resolution_clock::time_point now = sysnow();

		FD_ZERO(&rdfs);
		FD_SET(roscan._s_h, &rdfs);

		int rc = select(roscan._s_h+1, &rdfs, NULL, NULL, &tv);

		if(!rc){
			//float dt = sysdt(last_send, now);
			//if (dt > 1.0){
			//	// beep
			//	//roscan.send("181C0100#0260000000000000");
			//	last_send = now;
			//}else{
			float rcv_dt = sysdt(last_rcv, now);
			if(rcv_dt > rcv_timeout){cmd_vel.linear.x = cmd_vel.angular.z = 0.0;}

			float cmd_dt = sysdt(last_cmd, now);
			if(cmd_dt > cmd_vel_period){
				int cmd_x1 = clip(-100, 1.3 * - 100 * cmd_vel.angular.z, 100);
				int cmd_y1 = clip(-100, 1.4 * + 100 * cmd_vel.linear.x,  100);
				int cmd_x2 = clip(-25, 1.3 * - 25 * cmd_vel.angular.z, 25);
				int cmd_y2 = clip(-25, 1.4 * + 25 * cmd_vel.linear.x,  25);

				sprintf(can_cmd_buf1, "02001100#%02X%02X", (uint8_t)cmd_x1, (uint8_t)cmd_y1);
				sprintf(can_cmd_buf2, "02000200#%02X%02X", (uint8_t)cmd_x2, (uint8_t)cmd_y2);
				roscan.send(can_cmd_buf1);
				roscan.send(can_cmd_buf2);
				last_cmd = now;
			}
			//}
		}

		if(FD_ISSET(roscan._s_h, &rdfs)){
			roscan.read(cf);
			bat_msg.percentage = cf.data[0];
			bat_pub.publish(&bat_msg);
		}
		nh.spinOnce();
		usleep(1000);
	}
	roscan.shutdown();
	return 0;
}

std::chrono::high_resolution_clock::time_point sysnow(){
	return std::chrono::high_resolution_clock::now();
}
float sysdt(std::chrono::high_resolution_clock::time_point& t0, std::chrono::high_resolution_clock::time_point& t1){
	return std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
}
void cmd_vel_cb(const geometry_msgs::Twist& msg){
	cmd_vel = msg;
	last_rcv = sysnow();
}
