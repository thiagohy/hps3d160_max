#include "ros_publisher.hpp"

#ifdef HPS3D160_MAX_WITH_ROS
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#ifdef HPS3D160_MAX_WITH_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#endif

#include <cmath>
#include <limits>

namespace hps {
namespace {

// Builds the field layout used by the publisher: x, y, z and distance (metres).
// The helper setPointCloud2FieldsByString only understands the special names
// "xyz", "rgb" and "rgba", so the fields are declared explicitly through the
// (name, count, datatype) triplets expected by setPointCloud2Fields.
void build_fields(sensor_msgs::PointCloud2Modifier& mod) {
    mod.setPointCloud2Fields(
        4,
        "x", 1, sensor_msgs::PointField::FLOAT32,
        "y", 1, sensor_msgs::PointField::FLOAT32,
        "z", 1, sensor_msgs::PointField::FLOAT32,
        "distance", 1, sensor_msgs::PointField::FLOAT32);
}

} // namespace

void RosPointCloudPublisher::init(ros::NodeHandle& nh, const std::string& topic, const std::string& frame_id,
                                  bool organized) {
    frame_id_ = frame_id;
    organized_ = organized;
    pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic, 1);
}

void RosPointCloudPublisher::publish(const PointCloudFrame& frame) {
    sensor_msgs::PointCloud2 msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id_;
    msg.is_dense = false;
    msg.is_bigendian = false;

    // PointCloudFrame already carries metres (the SDK reports millimetres and
    // PointCloudBuilder converts them, validated against the per-pixel distance).
    const float nan = std::numeric_limits<float>::quiet_NaN();

    if (organized_ && frame.width > 0 && frame.height > 0 &&
        frame.points.size() >= static_cast<std::size_t>(frame.width) * frame.height) {
        // Organized cloud: keeps the 160x60 pixel grid so it can be aligned with
        // a depth image / with the ROI coordinates of the sensor.
        msg.height = frame.height;
        msg.width = frame.width;

        sensor_msgs::PointCloud2Modifier mod(msg);
        build_fields(mod);
        // resize() resets width/height to (size, 1), so the organised geometry
        // is restored afterwards.
        mod.resize(frame.points.size());
        msg.width = frame.width;
        msg.height = frame.height;
        msg.row_step = msg.point_step * msg.width;

        sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> id(msg, "distance");

        for (std::size_t i = 0; i < frame.points.size(); ++i) {
            const auto& p = frame.points[i];
            if (p.valid) {
                *ix = p.x;
                *iy = p.y;
                *iz = p.z;
                *id = p.distance_mm / 1000.0f;
            } else {
                *ix = nan;
                *iy = nan;
                *iz = nan;
                *id = nan;
            }
            ++ix; ++iy; ++iz; ++id;
        }
    } else {
        // Flat cloud with only the valid points.
        std::size_t valid = 0;
        for (const auto& p : frame.points) if (p.valid) ++valid;

        msg.height = 1;
        msg.width = static_cast<std::uint32_t>(valid);

        sensor_msgs::PointCloud2Modifier mod(msg);
        build_fields(mod);
        mod.resize(valid);

        sensor_msgs::PointCloud2Iterator<float> ix(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iy(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iz(msg, "z");
        sensor_msgs::PointCloud2Iterator<float> id(msg, "distance");

        for (const auto& p : frame.points) {
            if (!p.valid) continue;
            *ix = p.x;
            *iy = p.y;
            *iz = p.z;
            *id = p.distance_mm / 1000.0f;
            ++ix; ++iy; ++iz; ++id;
        }
    }

    pub_.publish(msg);
}

} // namespace hps
#endif
