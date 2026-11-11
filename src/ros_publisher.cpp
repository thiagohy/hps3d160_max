#include "ros_publisher.hpp"

#ifdef HPS3D160_MAX_WITH_ROS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#ifdef HPS3D160_MAX_WITH_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#endif

namespace hps {

void RosPointCloudPublisher::init(ros::NodeHandle& nh, const std::string& topic, const std::string& frame_id) {
    frame_id_ = frame_id;
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);
}

void RosPointCloudPublisher::publish(const PointCloudFrame& frame) {
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(frame.points.size());
    msg.is_dense = false;
    msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier mod(msg);
    mod.setPointCloud2FieldsByString(1, "xyz");
    // Add distance as extra field manually is complex; keep xyz for compatibility.

    sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");

    for (const auto& p : frame.points) {
        if (!p.valid) continue;
        *ix = p.x; *iy = p.y; *iz = p.z;
        ++ix; ++iy; ++iz;
    }
    pub_.publish(msg);
}

} // namespace hps
#endif