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
    // `organized` keeps the 160x60 pixel grid (invalid pixels become NaN),
    // which allows aligning the cloud with a depth image and with the sensor
    // ROI coordinates. When disabled only the valid points are published.
    void init(ros::NodeHandle& nh, const std::string& topic, const std::string& frame_id,
              bool organized = true);
    void publish(const PointCloudFrame& frame);

    bool organized() const { return organized_; }
#else
    void init() {}
    void publish(const PointCloudFrame&) {}
#endif

private:
#ifdef HPS3D160_MAX_WITH_ROS
    ros::Publisher pub_;
    std::string frame_id_ = "hps3d160";
    bool organized_ = true;
#endif
};

} // namespace hps
