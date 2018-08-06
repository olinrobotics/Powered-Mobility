#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types_conversion.h>

#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

class GrassDetector
{
    private:
        // ros handles
        ros::NodeHandle _nh;
        ros::Subscriber _pcl_sub;
        ros::Publisher _pcl_pub;
        tf::TransformListener _tfl;

        // ros data
        sensor_msgs::PointCloud2::Ptr _pcl_msg;
        sensor_msgs::PointCloud2::Ptr _pcl_out;

        // pcl data
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pcl_rgb_raw;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pcl_rgb;
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr _pcl_hsv;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr _pcl_grs;

        // parameters
        std::string _base_frame;

    public:
        GrassDetector(ros::NodeHandle nh)
            : _nh(nh)
        {

            // Define Publishers and Subscribers here
            _nh.param<std::string>("base_frame", _base_frame, "base_footprint");
            _pcl_pub = _nh.advertise<sensor_msgs::PointCloud2>("grass_points", 3);
            _pcl_sub = _nh.subscribe("/camera/depth_registered/points", 3, &GrassDetector::pcl_cb, this);
        }

        void pcl_cb(const sensor_msgs::PointCloud2ConstPtr& msg_in){
            // reset all data
            _pcl_msg.reset(new sensor_msgs::PointCloud2);
            _pcl_rgb_raw.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            _pcl_rgb.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            _pcl_hsv.reset(new pcl::PointCloud<pcl::PointXYZHSV>);
            _pcl_grs.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
            _pcl_out.reset(new sensor_msgs::PointCloud2);


            // camera_link -> base_footprint
            _tfl.waitForTransform(_base_frame, msg_in->header.frame_id,
                    msg_in->header.stamp, ros::Duration(0.2));
            pcl_ros::transformPointCloud(_base_frame, *msg_in, *_pcl_msg, _tfl);

            _pcl_msg->header.frame_id = _base_frame;
            _pcl_out->header.frame_id = _base_frame;

            // ROS -> PCL
            pcl::fromROSMsg(*_pcl_msg, *_pcl_rgb_raw);

            //Voxel (pcl_rgb_raw -> pcl_rgb)
            pcl::VoxelGrid<pcl::PointXYZRGB> vox;
            vox.setInputCloud(_pcl_rgb_raw);
            vox.setLeafSize(0.05,0.05,0.05);
            vox.filter(*_pcl_rgb);

            // RGB -> HSV
            pcl::PointCloudXYZRGBtoXYZHSV(*_pcl_rgb, *_pcl_hsv);

            // filter by Z/H
            pcl::PointIndices::Ptr idx (new pcl::PointIndices ());
            const float H_MIN = 60;
            const float H_MAX = 150;
            const float S_MIN = 50 / 255.0;
            const float S_MAX = 180 / 255.0;
            const float V_MIN = 0 / 255.0;
            const float V_MAX = 255 / 255.0;
            const float Z_MIN = -0.1;
            const float Z_MAX = 0.1;

            for(size_t i=0; i<_pcl_hsv->points.size(); ++i){
                if(
                        _pcl_hsv->points[i].h >= H_MIN &&
                        _pcl_hsv->points[i].h < H_MAX &&
                        _pcl_hsv->points[i].s >= S_MIN &&
                        _pcl_hsv->points[i].s < S_MAX &&
                        _pcl_hsv->points[i].v >= V_MIN &&
                        _pcl_hsv->points[i].v < V_MAX &&
                        _pcl_rgb->points[i].z > Z_MIN &&
                        _pcl_rgb->points[i].z < Z_MAX
                  ){
                    idx->indices.push_back(i);
                }
            }

            // extract indices
            pcl::ExtractIndices<pcl::PointXYZRGB> ex;
            ex.setInputCloud(_pcl_rgb);
            ex.setNegative(false);
            ex.setIndices(idx);
            ex.filter(*_pcl_grs); // pcl-grass

            // publish output
            pcl::toROSMsg (*_pcl_grs, *_pcl_out);
            _pcl_out->header.seq += 1;
            _pcl_out->header.stamp = msg_in->header.stamp;
            _pcl_out->header.frame_id = _base_frame;
            _pcl_pub.publish(_pcl_out);
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "grass_detector");
    ros::NodeHandle nh;
    GrassDetector tranform_cloud(nh);

    // Spin until ROS is shutdown
    while (ros::ok())
        ros::spin();

    return 0;
}
