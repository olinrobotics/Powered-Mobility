#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <thread>
#include "pwm_driver/roscan.h"

//#include <dynamic_reconfigure/server.h>
//#include <geometry_msgs/TwistStamped.h>
//#include <nav_msgs/Odometry.h>
//#include <tf/tfMessage.h>
//#include <realtime_tools/realtime_buffer.h>
//#include <realtime_tools/realtime_publisher.h>
//#include <diff_drive_controller/odometry.h>
//#include <diff_drive_controller/speed_limiter.h>
//#include <diff_drive_controller/DiffDriveControllerConfig.h>


namespace pwm_driver{
	class CanController : public controller_interface::Controller<hardware_interface::VelocityJointInterface>{
	  private:
		  std::string name_;
		  hardware_interface::JointHandle left_wheel_joint_;
		  hardware_interface::JointHandle right_wheel_joint_;

		  // command data
		  float cmd_v, cmd_w;
		  ros::Time cmd_t;

		  // params
		  double v_scale_, w_scale_;
		  std::string can_port_;
		  std::string left_wheel_name_;
		  std::string right_wheel_name_;

		  double cmd_timeout_;
		  double wheel_separation_;
		  double wheel_radius_;

		  // read can handle
		  boost::shared_ptr<ROSCan> can_;
		  std::thread can_read_thread_;

	  public:
		  CanController();
		  bool init(hardware_interface::VelocityJointInterface* hw,
				  ros::NodeHandle& nh,
				  ros::NodeHandle& controller_nh);
		  void update(const ros::Time& time, const ros::Duration& period);
		  void starting(const ros::Time& time);
		  void stopping(const ros::Time&);
		  void brake();
		  void read();

	};
  PLUGINLIB_EXPORT_CLASS(pwm_driver::CanController, controller_interface::ControllerBase);

}
