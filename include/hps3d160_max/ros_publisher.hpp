#pragma once

#include "types.hpp"

#ifdef HPS3D160_MAX_WITH_ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#endif

namespace hps {

class RosPointCloudPublisher {
public:
    RosPointCloudPublisher() = default;
#ifdef HPS3D160_MAX_WITH_ROS
    void init(ros::NodeHandle& nh, const std::string& topic, const std::string& frame_id);
    void publish(const PointCloudFrame& frame);
#endif
private:
#ifdef HPS3D160_MAX_WITH_ROS
    ros::Publisher pub_;
    std::string frame_id_ = "hps3d160";
#endif
};

} // namespace hps