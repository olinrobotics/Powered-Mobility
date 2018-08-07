#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <chrono>
#include <thread>
#include <iostream>

#include "pwm_driver/roscan.h"

namespace gazebo
{
	class CanPlugin : public ModelPlugin{
	  public:

	  private:
		  physics::ModelPtr model;
		  event::ConnectionPtr updateConnection;

		  math::Vector3 lin_vel_;
		  math::Vector3 ang_vel_;

		  boost::shared_ptr<ROSCan> can_;
		  std::string can_port_;
		  double v_scale_;
		  double w_scale_;
		  double cmd_timeout_;
		  std::chrono::system_clock::time_point last_cmd_;
		  std::thread read_thread_;

	  public:
		  void read_cmd(){
			  can_.reset(new ROSCan(can_port_.c_str()));
			  return;

			  if(!can_->init()){
				  std::cout << "CAN Open Failed!" << std::endl;
				  return;
			  }
			  std::cout << "CAN Open Success!" << std::endl;
			  last_cmd_ = std::chrono::system_clock::now();

			  fd_set rdfs;
			  timeval tv;
			  can_frame cf;
			  tv.tv_sec = 0;
			  tv.tv_usec = 10 * 1e3; // 10 ms
			  std::chrono::duration<double> dt;

			  while(true){
				  FD_ZERO(&rdfs);
				  FD_SET(can_->_s_h, &rdfs);

				  int rc = select(can_->_s_h+1, &rdfs, NULL, NULL, &tv);
				  if(rc && FD_ISSET(can_->_s_h, &rdfs)){
					  can_->read(cf);
					  int8_t cmd_x = (int8_t) cf.data[0];
					  int8_t cmd_y = (int8_t) cf.data[1];

					  // TODO : check if local frame
					  lin_vel_.x = cmd_y / v_scale_;
					  ang_vel_.z = - cmd_x / w_scale_;
					  last_cmd_ = std::chrono::system_clock::now();

					  std::cout << cmd_x << std::endl;
				  }

				  // apply timeout
				  dt = std::chrono::system_clock::now() - last_cmd_;
				  if(dt.count() > cmd_timeout_){
					  lin_vel_.x = 0;
					  ang_vel_.z = 0;
				  }

			  }
		  }

		  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){
			  std::cout << "Loading CAN Plugin ... " << std::endl;
			  // initialize can port here
			  if (_sdf->HasElement("port")){
				  can_port_= _sdf->Get<std::string>("port");
			  }
			  if (_sdf->HasElement("v_scale")){
				  v_scale_ = _sdf->Get<double>("v_scale");
			  }
			  if (_sdf->HasElement("w_scale")){
				  w_scale_ = _sdf->Get<double>("w_scale");
			  }
			  this->model = _parent;
			  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
					  boost::bind(&CanPlugin::OnUpdate, this, _1));

			  // start reading CAN
			  read_thread_ = std::thread (&CanPlugin::read_cmd, this);
		  }

		  void OnUpdate(const common::UpdateInfo& _info){
			  this->model->SetLinearVel(lin_vel_);
			  this->model->SetAngularVel(ang_vel_);
		  }
	};
	GZ_REGISTER_MODEL_PLUGIN(CanPlugin)
};
