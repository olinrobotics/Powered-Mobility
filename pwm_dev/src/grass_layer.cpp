#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>


/* include part */

#ifndef __GRASS_LAYER_H__
#define __GRASS_LAYER_H__

namespace pwm_dev{
	class GrassLayer : public costmap_2d::CostmapLayer{
	  protected:
		  std::string global_frame_; // should be odom (local)
		  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  
		  std::vector<boost::shared_ptr<tf::MessageFilterBase> > observation_notifiers_;
		  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > observation_buffers_;
		  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > marking_buffers_;
		  std::vector<boost::shared_ptr<costmap_2d::ObservationBuffer> > clearing_buffers_;

		  // Used only for testing purposes
		  std::vector<costmap_2d::Observation> static_clearing_observations_, static_marking_observations_;
		  bool footprint_clearing_enabled_;
		  bool rolling_window_;

		  // rgbd camera params
		  double fov_;
		  double min_range_;
		  double max_range_;

		  // grass params
		  double min_z_;
		  double max_z_;

		  //dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig> *dsrv_;

	  public:
		  GrassLayer(){costmap_ = NULL;}
		  virtual ~GrassLayer();
		  virtual void onInitialize();
		  virtual void updateBounds(double robot_x, double robot_y,
				  double robot_yaw, double* min_x, double* min_y,
				  double* max_x, double* max_y);
		  virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
				  int min_i, int min_j, int max_i, int max_j);
		  virtual void activate();
		  virtual void deactivate();
		  virtual void reset();
		  void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg,
				  const boost::shared_ptr<costmap_2d::ObservationBuffer>& buffer); //buffer??
	  protected:
		  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);
		  bool getMarkingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;
		  bool getClearingObservations(std::vector<costmap_2d::Observation>& marking_observations) const;

		  //void cloudValidInfCallback()

	};
};

#endif
PLUGINLIB_EXPORT_CLASS(pwm_dev::GrassLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace pwm_dev{
	void GrassLayer::onInitialize()
	{
		ros::NodeHandle nh("~/" + name_), g_nh;
		rolling_window_ = layered_costmap_->isRolling();

		bool track_unknown_space;
		nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
		if (track_unknown_space)
			default_value_ = NO_INFORMATION;
		else
			default_value_ = FREE_SPACE;

		GrassLayer::matchSize();
		current_ = true;

		global_frame_ = layered_costmap_->getGlobalFrameID();
		double transform_tolerance;
		nh.param("transform_tolerance", transform_tolerance, 0.2);

		std::string topics_string;
		// get the topics that we'll subscribe to from the parameter server
		nh.param("observation_sources", topics_string, std::string(""));
		ROS_INFO("    Subscribed to Topics: %s", topics_string.c_str());

		// get our tf prefix
		ros::NodeHandle prefix_nh;
		const std::string tf_prefix = tf::getPrefixParam(prefix_nh);

		// now we need to split the topics based on whitespace which we can use a stringstream for
		std::stringstream ss(topics_string);

		std::string source;
		while (ss >> source) // guess multiple RGBD configuration is possible
		{
			ros::NodeHandle source_node(nh, source);

			// get the parameters for the specific topic
			double observation_keep_time, expected_update_rate, min_obstacle_height, max_obstacle_height;
			std::string topic, sensor_frame, data_type;
			bool inf_is_valid, clearing, marking;

			source_node.param("topic", topic, source);
			source_node.param("sensor_frame", sensor_frame, std::string(""));
			source_node.param("observation_persistence", observation_keep_time, 0.0);
			source_node.param("expected_update_rate", expected_update_rate, 0.0);
			source_node.param("min_z", min_z_, -0.1);
			source_node.param("max_z", max_z_, 0.1);
			source_node.param("min_range", min_range_, 0.16); //D435 default
			source_node.param("max_range", max_range_, 10.0); //D435 default
			source_node.param("clearing", clearing, false);
			source_node.param("marking", marking, true);

			if (!sensor_frame.empty())
			{
				sensor_frame = tf::resolve(tf_prefix, sensor_frame);
			}

			std::string raytrace_range_param_name, obstacle_range_param_name;

			// get the obstacle range for the sensor
			double obstacle_range = max_range_;
			if (source_node.searchParam("obstacle_range", obstacle_range_param_name))
			{
				source_node.getParam(obstacle_range_param_name, obstacle_range);
			}

			// get the raytrace range for the sensor
			double raytrace_range = max_range_;
			if (source_node.searchParam("raytrace_range", raytrace_range_param_name))
			{
				source_node.getParam(raytrace_range_param_name, raytrace_range);
			}

			ROS_DEBUG("Creating an observation buffer for source %s, topic %s, frame %s", source.c_str(), topic.c_str(),
					sensor_frame.c_str());

			// create an observation buffer
			observation_buffers_.push_back(
					boost::shared_ptr < ObservationBuffer
					> (new ObservationBuffer(topic, observation_keep_time, expected_update_rate, min_obstacle_height,
							max_obstacle_height, obstacle_range, raytrace_range, *tf_, global_frame_,
							sensor_frame, transform_tolerance)));

			// check if we'll add this buffer to our marking observation buffers
			if (marking)
				marking_buffers_.push_back(observation_buffers_.back());

			// check if we'll also add this buffer to our clearing observation buffers
			if (clearing)
				clearing_buffers_.push_back(observation_buffers_.back());

			ROS_DEBUG(
					"Created an observation buffer for source %s, topic %s, global frame: %s, "
					"expected update rate: %.2f, observation persistence: %.2f",
					source.c_str(), topic.c_str(), global_frame_.c_str(), expected_update_rate, observation_keep_time);

			boost::shared_ptr < message_filters::Subscriber<sensor_msgs::PointCloud2>
				> sub(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic, 50));

			if (inf_is_valid)
			{
				ROS_WARN("obstacle_layer: inf_is_valid option is not applicable to PointCloud observations.");
			}

			boost::shared_ptr < tf::MessageFilter<sensor_msgs::PointCloud2>
				> filter(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub, *tf_, global_frame_, 50));

			filter->registerCallback(
					boost::bind(&GrassLayer::cloudCallback, this, _1, observation_buffers_.back()));

			observation_subscribers_.push_back(sub);
			observation_notifiers_.push_back(filter);

			if (sensor_frame != "")
			{
				std::vector < std::string > target_frames;
				target_frames.push_back(global_frame_);
				target_frames.push_back(sensor_frame);
				observation_notifiers_.back()->setTargetFrames(target_frames);
			}
		}

		//dsrv_ = NULL;
		//setupDynamicReconfigure(nh);
	}

	void GrassLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
	{
		//dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>(nh);
		//dynamic_reconfigure::Server<costmap_2d::ObstaclePluginConfig>::CallbackType cb = boost::bind(
		//		&GrassLayer::reconfigureCB, this, _1, _2);
		//dsrv_->setCallback(cb);
	}

	GrassLayer::~GrassLayer()
	{
		//if (dsrv_)
		//	delete dsrv_;
	}
	//void GrassLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level)
	//{
		//enabled_ = config.enabled;
		//footprint_clearing_enabled_ = config.footprint_clearing_enabled;
		//max_obstacle_height_ = config.max_obstacle_height;
		//combination_method_ = config.combination_method;
	//}

	void GrassLayer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message,
			const boost::shared_ptr<ObservationBuffer>& buffer)
	{
		// buffer the point cloud
		buffer->lock();
		buffer->bufferCloud(*message);
		buffer->unlock();
	}

	void GrassLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x,
			double* min_y, double* max_x, double* max_y)
	{
		if (rolling_window_)
			updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
		if (!enabled_)
			return;
		useExtraBounds(min_x, min_y, max_x, max_y);

		bool current = true;
		std::vector<Observation> observations, clearing_observations;

		// get the marking observations
		current = current && getMarkingObservations(observations);

		// get the clearing observations
		current = current && getClearingObservations(clearing_observations);

		// update the global current status
		current_ = current;

		// raytrace freespace
		for (unsigned int i = 0; i < clearing_observations.size(); ++i)
		{
			raytraceFreespace(clearing_observations[i], min_x, min_y, max_x, max_y);
		}

		// place the new obstacles into a priority queue... each with a priority of zero to begin with
		for (std::vector<Observation>::const_iterator it = observations.begin(); it != observations.end(); ++it)
		{
			const Observation& obs = *it;

			const pcl::PointCloud<pcl::PointXYZ>& cloud = *(obs.cloud_);

			double sq_obstacle_range = obs.obstacle_range_ * obs.obstacle_range_;

			for (unsigned int i = 0; i < cloud.points.size(); ++i)
			{
				double px = cloud.points[i].x, py = cloud.points[i].y, pz = cloud.points[i].z;

				// if the obstacle is too high or too far away from the robot we won't add it
				if (pz > max_obstacle_height_)
				{
					ROS_DEBUG("The point is too high");
					continue;
				}

				// compute the squared distance from the hitpoint to the pointcloud's origin
				double sq_dist = (px - obs.origin_.x) * (px - obs.origin_.x) + (py - obs.origin_.y) * (py - obs.origin_.y)
					+ (pz - obs.origin_.z) * (pz - obs.origin_.z);

				// if the point is far enough away... we won't consider it
				if (sq_dist >= sq_obstacle_range)
				{
					ROS_DEBUG("The point is too far away");
					continue;
				}

				// now we need to compute the map coordinates for the observation
				unsigned int mx, my;
				if (!worldToMap(px, py, mx, my))
				{
					ROS_DEBUG("Computing map coords failed");
					continue;
				}

				unsigned int index = getIndex(mx, my);
				costmap_[index] = LETHAL_OBSTACLE;
				touch(px, py, min_x, min_y, max_x, max_y);
			}
		}

		updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
	}

	void GrassLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
			double* max_x, double* max_y)
	{
		if (!footprint_clearing_enabled_) return;
		transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

		for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
		{
			touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
		}
	}

	void GrassLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
	{
		if (!enabled_)
			return;

		if (footprint_clearing_enabled_)
		{
			setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
		}

		switch (combination_method_)
		{
			case 0:  // Overwrite
				updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
				break;
			case 1:  // Maximum
				updateWithMax(master_grid, min_i, min_j, max_i, max_j);
				break;
			default:  // Nothing
				break;
		}
	}

	bool GrassLayer::getMarkingObservations(std::vector<Observation>& marking_observations) const
	{
		bool current = true;
		// get the marking observations
		for (unsigned int i = 0; i < marking_buffers_.size(); ++i)
		{
			marking_buffers_[i]->lock();
			marking_buffers_[i]->getObservations(marking_observations);
			current = marking_buffers_[i]->isCurrent() && current;
			marking_buffers_[i]->unlock();
		}
		marking_observations.insert(marking_observations.end(),
				static_marking_observations_.begin(), static_marking_observations_.end());
		return current;
	}

	bool GrassLayer::getClearingObservations(std::vector<Observation>& clearing_observations) const
	{
		bool current = true;
		// get the clearing observations
		for (unsigned int i = 0; i < clearing_buffers_.size(); ++i)
		{
			clearing_buffers_[i]->lock();
			clearing_buffers_[i]->getObservations(clearing_observations);
			current = clearing_buffers_[i]->isCurrent() && current;
			clearing_buffers_[i]->unlock();
		}
		clearing_observations.insert(clearing_observations.end(),
				static_clearing_observations_.begin(), static_clearing_observations_.end());
		return current;
	}

	void GrassLayer::activate()
	{
		// if we're stopped we need to re-subscribe to topics
		for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
		{
			if (observation_subscribers_[i] != NULL)
				observation_subscribers_[i]->subscribe();
		}

		for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
		{
			if (observation_buffers_[i])
				observation_buffers_[i]->resetLastUpdated();
		}
	}

	void GrassLayer::deactivate()
	{
		for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
		{
			if (observation_subscribers_[i] != NULL)
				observation_subscribers_[i]->unsubscribe();
		}
	}

	void GrassLayer::reset()
	{
		deactivate();
		resetMaps();
		current_ = true;
		activate();
	}

}// namespace pwm_dev
