#include <costmap_2d/obstacle_layer.h>
#include <costmap_2d/costmap_math.h>
#include <pluginlib/class_list_macros.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pwm_dev/grass_layer.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>

/* include part */
PLUGINLIB_EXPORT_CLASS(pwm_dev::GrassLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace pwm_dev{

    float rgb2hsv(int r, int g, int b,
            float& h, float& s, float& v
            ){
        float rf=(r/255.), gf=(g/255.), bf=(b/255.);
        float cmax = std::max({rf, gf, bf});
        float delta = cmax - std::min({rf,gf,bf});

        /* hue, 0 - 360*/
        if(delta == 0) h=0;
        if(cmax == rf) h=60.0 * (fmod((gf-bf)/delta, 6.0));
        if(cmax == gf) h=60.0 * ((bf-rf)/delta + 2);
        if(cmax == bf) h=60.0 * ((rf-gf)/delta + 4);

        /* saturation, 0 - 255 */
        s = 255.0 * ((cmax==0)? 0 : delta / cmax);

        /* value, 0 - 255 */
        v = cmax;
    }

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

        nh.param("topic", topic_, std::string(""));
        nh.param("sensor_frame", sensor_frame_, std::string(""));
        nh.param("sensor_timeout", sensor_timeout_, 0.0);
        nh.param("sensor_rate", sensor_rate_, 0.0);
        nh.param("min_z", min_z_, -0.1);
        nh.param("max_z", max_z_, 0.1);
        nh.param("min_h", min_h_, 60.0);
        nh.param("max_h", max_h_, 150.0);
        nh.param("min_s", min_s_, 50.0);
        nh.param("max_s", max_s_, 180.0);
        nh.param("min_v", min_v_, 0.0);
        nh.param("max_v", max_v_, 255.0);
        nh.param("min_range", min_range_, 0.16); //D435 default
        nh.param("max_range", max_range_, 10.0); //D435 default
        nh.param("clearing", clearing_, false);
        nh.param("marking", marking_, true);

        nh.param("grass_cost", grass_cost_, 10.0);
        nh.param("grass_max_cost", grass_max_cost_, LETHAL_OBSTACLE / 2.0);

        // no dynrec yet ...
		enabled_ = true;
		footprint_clearing_enabled_ = true;
		combination_method_ = 1; // updateWithMax

        sub_.reset(new message_filters::Subscriber<sensor_msgs::PointCloud2>(g_nh, topic_, 50));
        filter_.reset(new tf::MessageFilter<sensor_msgs::PointCloud2>(*sub_, *tf_, global_frame_, 50));
        filter_->registerCallback(
                boost::bind(&GrassLayer::cloudCallback, this, _1));
        /*
        {
			if (sensor_frame != "")
			{
				std::vector < std::string > target_frames;
				target_frames.push_back(global_frame_);
				target_frames.push_back(sensor_frame);
				observation_notifiers_.back()->setTargetFrames(target_frames);
			}
		}
        */

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

    /*
	void GrassLayer::reconfigureCB(costmap_2d::ObstaclePluginConfig &config, uint32_t level)
	{
		enabled_ = config.enabled;
		footprint_clearing_enabled_ = config.footprint_clearing_enabled;
		max_obstacle_height_ = config.max_obstacle_height;
		combination_method_ = config.combination_method;
	}
    */

	void GrassLayer::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& message)
    {
        pcl_msg_.reset(new sensor_msgs::PointCloud2);
        pcl_rgb_raw_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl_rgb_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

        //tf_->transformPointCloud
        tf::StampedTransform xform_tf;
        geometry_msgs::TransformStamped xform_msg;
        tf_->lookupTransform(global_frame_, sensor_frame_, message->header.stamp, xform_tf);
        tf::transformStampedTFToMsg(xform_tf, xform_msg);

        pcl_origin_ = xform_tf.getOrigin();
        tf2::doTransform(*message, *pcl_msg_, xform_msg);
        //pcl_ros::transformPointCloud(global_frame_, *message, *pcl_msg_, tfl_);

        // ROS -> PCL
        pcl::fromROSMsg(*pcl_msg_, *pcl_rgb_raw_);
        pcl_msg_->header.frame_id = global_frame_;
        pcl_msg_->header.stamp    = message->header.stamp;

        //Voxel (pcl_rgb_raw -> pcl_rgb)
        pcl::VoxelGrid<pcl::PointXYZRGB> vox;
        vox.setInputCloud(pcl_rgb_raw_);
        vox.setLeafSize(0.05,0.05,0.05);
        vox.filter(*pcl_rgb_);
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
		if(pcl_msg_){
			if( (ros::Time::now() - pcl_msg_->header.stamp).toSec() < sensor_timeout_){
				current = false;
			}
		}

        // update the global current status
        current_ = current;

        // place the new obstacles into a priority queue... each with a priority of zero to begin with
        // filter by
        
        float ph,ps,pv;

		if(pcl_rgb_){	
			const tf::Vector3& o = pcl_origin_;
			float ox=o.x(),oy=o.y(),oz=o.z();

			for (unsigned int i = 0; i < pcl_rgb_->points.size(); ++i)
			{
				const pcl::PointXYZRGB& p = pcl_rgb_->points[i];
				double px = p.x, py = p.y, pz = p.z;
				rgb2hsv(p.r,p.g,p.b, ph,ps,pv);

				if (pz<min_z_ || pz>max_z_) continue;

				double d = sqrt((px-ox)*(px-ox) + (py-oy)*(py-oy));
				if (d<min_range_ || d>max_range_) continue;

				// now we need to compute the map coordinates for the observation
				unsigned int mx, my;
				if (!worldToMap(px, py, mx, my))
				{
					ROS_DEBUG("Computing map coords failed");
					continue;
				}

				unsigned int index = getIndex(mx, my);
				if (ph>min_h_ && ph<max_h_
                        && ps>min_s_ && ps<max_s_
                        && pv>min_v_ && pv<max_v_
                        && marking_){
					costmap_[index] = std::max(costmap_[index] + grass_cost_, grass_max_cost_);
				}else if (clearing_){
					// todo : separate grass_clear_cost_ ?
					costmap_[index] = std::max(costmap_[index] - grass_cost_, (double)FREE_SPACE);
				}
				touch(px, py, min_x, min_y, max_x, max_y);
			}
		}

        updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    }

    void GrassLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
            double* max_x, double* max_y)
    {
        if (!footprint_clearing_enabled_) return;
        costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

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

    void GrassLayer::activate()
    {
        // if we're stopped we need to re-subscribe to topics
        //for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
        //{
        //    if (observation_subscribers_[i] != NULL)
        //        observation_subscribers_[i]->subscribe();
        //}

        //for (unsigned int i = 0; i < observation_buffers_.size(); ++i)
        //{
        //    if (observation_buffers_[i])
        //        observation_buffers_[i]->resetLastUpdated();
        //}
		sub_->subscribe();
    }

    void GrassLayer::deactivate()
    {
		if(sub_)
			sub_->unsubscribe();
        //for (unsigned int i = 0; i < observation_subscribers_.size(); ++i)
        //{
        //    if (observation_subscribers_[i] != NULL)
        //        observation_subscribers_[i]->unsubscribe();
        //}
    }

    void GrassLayer::reset()
    {
        deactivate();
        resetMaps();
        current_ = true;
        activate();
    }

}// namespace pwm_dev
