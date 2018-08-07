// adaptation from diff_drive_controller/CanController

#include <cmath>
#include <tf/transform_datatypes.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/urdfdom_compatibility.h>
#include <boost/assign.hpp>
#include "pwm_driver/can_controller.h"

namespace pwm_driver{
	CanController::CanController()
		: name_("can_controller"), cmd_v(0), cmd_w(0), cmd_t(0), cmd_timeout_(0.2),
		v_scale_(1.0), w_scale_(1.0),
		wheel_separation_(1.0), wheel_radius_(1.0){}

	bool CanController::init(hardware_interface::VelocityJointInterface* hw,
			ros::NodeHandle & root_nh,
			ros::NodeHandle & controller_nh)
	{
		controller_nh.param("can_port", can_port_, can_port_);
		controller_nh.param("v_scale", v_scale_, v_scale_);
		controller_nh.param("w_scale", w_scale_, w_scale_);
		controller_nh.param("cmd_timeout", cmd_timeout_, cmd_timeout_);
		controller_nh.param("wheel_separation", wheel_separation_, wheel_separation_);
		controller_nh.param("wheel_radius", wheel_radius_, wheel_radius_);
		controller_nh.param("left_wheel", left_wheel_name_, left_wheel_name_);
		controller_nh.param("right_wheel", right_wheel_name_, right_wheel_name_);

		const std::string complete_ns = controller_nh.getNamespace();
		std::size_t id = complete_ns.find_last_of("/");
		name_ = complete_ns.substr(id + 1);

		// Get the joint object to use in the realtime loop
		left_wheel_joint_ = hw->getHandle(left_wheel_name_);  // throws on failure
		right_wheel_joint_ = hw->getHandle(right_wheel_name_);  // throws on failure
		can_read_thread_ = std::thread (&CanController::read, this);
		return true;
	}

	void CanController::read(){
		can_.reset(new ROSCan(can_port_.c_str()));

		if(!can_->init()){
			std::cout << "CAN Open Failed!" << std::endl;
			return;
		}

		can_->set_filter(0x02001100, 0x1FFFFFFF); // joy -100 ~ 100
		//can_->set_filter(0x02000200, 0x1FFFF0FF); // joy -speed ~ speed

		std::cout << "CAN Open Success!" << std::endl;
		cmd_t = ros::Time::now();

		fd_set rdfs;
		timeval tv;
		can_frame cf;
		tv.tv_sec = 0;
		tv.tv_usec = 10 * 1e3; // 10 ms

		while(true){
			FD_ZERO(&rdfs);
			FD_SET(can_->_s_h, &rdfs);

			int rc = select(can_->_s_h+1, &rdfs, NULL, NULL, &tv);
			if(rc && FD_ISSET(can_->_s_h, &rdfs)){
				can_->read(cf);
				int8_t cmd_x = (int8_t) cf.data[0];
				int8_t cmd_y = (int8_t) cf.data[1];

				cmd_v = cmd_y / 100.0 / v_scale_;
				cmd_w = - cmd_x / 100.0 / w_scale_;
				cmd_t = ros::Time::now();
				//std::cout << cmd_x << std::endl;
			}

		}

	}

	void CanController::update(const ros::Time& time, const ros::Duration& period)
	{
		const double ws  = wheel_separation_;
		const double wr = wheel_radius_;
		const double dt = (time - cmd_t).toSec();

		// Brake if cmd_vel has timeout:
		if (dt > cmd_timeout_)
		{
			cmd_v = cmd_w = 0.0;
		}

		// Compute wheels velocities:
		const double vel_left  = (cmd_v - cmd_w * ws / 2.0)/wr;
		const double vel_right = (cmd_v + cmd_w * ws / 2.0)/wr;

		// Set wheels velocities:
		left_wheel_joint_.setCommand(vel_left);
		right_wheel_joint_.setCommand(vel_right);
	}

	void CanController::starting(const ros::Time& time)
	{
		brake();
		// Register starting time used to keep fixed rate
	}

	void CanController::stopping(const ros::Time& /*time*/)
	{
		brake();
	}

	void CanController::brake()
	{
		const double vel = 0.0;
		left_wheel_joint_.setCommand(vel);
		right_wheel_joint_.setCommand(vel);
	}
}
// namespace diff_drive_controller
