
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
std::chrono::high_resolution_clock::time_point last_cmd = sysnow();
float cmd_vel_timeout = 0.5;
char can_cmd_buf[64];

char* rosSrvrIp = "/dev/ttyS0";
//char* rosSrvrIp = "192.168.16.64";

int main(){
	// ros first
	nh.initNode(rosSrvrIp);//, "11311");//11311);
	nh.advertise(bat_pub);
	nh.subscribe(cmd_vel_sub);

	ROSCan roscan("can0");
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
	while(1){//ros::ok()){
		std::chrono::high_resolution_clock::time_point now = sysnow();

		FD_ZERO(&rdfs);
		FD_SET(roscan._s_h, &rdfs);

		int rc = select(roscan._s_h+1, &rdfs, NULL, NULL, &tv);

		if(!rc){
			float dt = sysdt(last_send, now);
			if (dt > 1.0){
				// beep
				//roscan.send("181C0100#0260000000000000");
				last_send = now;
			}else{
				float cmd_dt = sysdt(last_cmd, now);
				if(cmd_dt > cmd_vel_timeout ){cmd_vel.linear.x = cmd_vel.angular.z = 0.0;}
				int cmd_x = 1.3 * + 100 * cmd_vel.angular.z;
				int cmd_y = 1.4 * + 100 * cmd_vel.linear.x;
				sprintf(can_cmd_buf, "02001100#%02X%02X", (uint8_t)cmd_x, (uint8_t)cmd_y);
				roscan.send(can_cmd_buf);
			}
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
	last_cmd = sysnow();
}
