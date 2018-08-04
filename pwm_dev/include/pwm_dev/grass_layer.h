#ifndef __GRASS_LAYER_H__
#define __GRASS_LAYER_H__

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_ros/message_filter.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <message_filters/subscriber.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/footprint.h>

namespace pwm_dev{
	class GrassLayer : public costmap_2d::CostmapLayer{
	  protected:
		  std::string global_frame_; // should be odom (local)
          std::string topic_;
          std::string sensor_frame_; // camera_color_optical_frame or similar
          double sensor_timeout_; // incoming sensor message timeout
          double sensor_rate_; // expected sensor rate
          bool clearing_;
          bool marking_;

		  bool footprint_clearing_enabled_;
          std::vector<geometry_msgs::Point> transformed_footprint_;


          int combination_method_;
		  bool rolling_window_;

		  // rgbd camera params
		  double fov_;
		  double min_range_;
		  double max_range_;

		  // grass params
		  double min_z_;
		  double max_z_;
          double min_h_;// hue
          double max_h_; 
          double grass_cost_;
          double grass_max_cost_;

          // TODO : buffer??
          
          // ros data
          sensor_msgs::PointCloud2::Ptr pcl_msg_;
          tf::Vector3 pcl_origin_;

          // pcl data
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_rgb_raw_;
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_rgb_;
		  //dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig> *dsrv_;

	  public:
		  GrassLayer(){costmap_ = NULL;}
		  virtual ~GrassLayer();
		  virtual void onInitialize();
		  virtual void updateBounds(double robot_x, double robot_y,
				  double robot_yaw, double* min_x, double* min_y,
				  double* max_x, double* max_y);
          virtual void updateFootprint(double robot_x, double robot_y,
                  double robot_yaw, double* min_x, double* min_y,
                  double* max_x, double* max_y);
          virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
				  int min_i, int min_j, int max_i, int max_j);
		  virtual void activate();
		  virtual void deactivate();
		  virtual void reset();
		  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

	  protected:
		  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);
		  bool getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;
		  bool getClearingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;

		  //void cloudValidInfCallback()

	};
};
#endif
