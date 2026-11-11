// End-to-end validation of the ROS publisher against the REAL sensor_msgs API.
// Runs inside the Noetic container: publishes an organised cloud and checks
// the message received by a subscriber (geometry, fields and values).
#include "ros_publisher.hpp"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <cstdio>
#include <string>

namespace {
bool g_received = false;
bool g_ok = false;
std::string g_error;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    g_received = true;

    if (msg->width != 4 || msg->height != 3) {
        g_error = "unexpected geometry " + std::to_string(msg->width) + "x" + std::to_string(msg->height);
        return;
    }
    if (msg->fields.size() != 4) {
        g_error = "expected 4 fields, got " + std::to_string(msg->fields.size());
        return;
    }
    const std::string names[4] = {"x", "y", "z", "distance"};
    for (std::size_t i = 0; i < 4; ++i) {
        if (msg->fields[i].name != names[i]) {
            g_error = "field " + std::to_string(i) + " is '" + msg->fields[i].name + "'";
            return;
        }
        if (msg->fields[i].datatype != sensor_msgs::PointField::FLOAT32) {
            g_error = "field " + names[i] + " is not FLOAT32";
            return;
        }
    }
    if (msg->data.size() != 12 * msg->point_step) {
        g_error = "data size mismatch";
        return;
    }

    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float> id(*msg, "distance");

    // point 0 is invalid -> NaN
    if (!std::isnan(*ix) || !std::isnan(*iz) || !std::isnan(*id)) {
        g_error = "invalid point is not NaN";
        return;
    }
    ++ix; ++iy; ++iz; ++id;
    // point 1 is valid
    if (std::isnan(*iz) || std::fabs(*iz - 1.01f) > 1e-5) {
        g_error = "valid point z mismatch";
        return;
    }
    if (std::fabs(*id - 1.01f) > 1e-4) {
        g_error = "distance field mismatch: " + std::to_string(*id);
        return;
    }
    g_ok = true;
}
} // namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "hps_publisher_real_test");
    ros::NodeHandle nh;

    hps::PointCloudFrame frame{};
    frame.width = 4;
    frame.height = 3;
    frame.points.resize(12);
    for (std::size_t i = 0; i < frame.points.size(); ++i) {
        auto& p = frame.points[i];
        p.x = 0.1f * static_cast<float>(i);
        p.y = 0.2f * static_cast<float>(i);
        p.z = 1.0f + 0.01f * static_cast<float>(i);
        p.distance_mm = static_cast<std::uint16_t>(1000 + i * 10);
        p.valid = (i != 0);
        p.status = p.valid ? hps::PixelStatus::Ok : hps::PixelStatus::Invalid;
    }

    ros::Subscriber sub = nh.subscribe("/hps3d160/test_cloud", 1, callback);
    hps::RosPointCloudPublisher pub;
    pub.init(nh, "/hps3d160/test_cloud", "hps3d160", true);

    const ros::Time deadline = ros::Time::now() + ros::Duration(10.0);
    while (ros::ok() && !g_received && ros::Time::now() < deadline) {
        pub.publish(frame);
        ros::spinOnce();
        ros::Duration(0.05).sleep();
    }

    if (!g_received) {
        printf("FAILED: no message received\n");
        return 1;
    }
    if (!g_ok) {
        printf("FAILED: %s\n", g_error.c_str());
        return 1;
    }
    printf("PUBLISHER REAL TEST OK (4x3 organised cloud, fields x/y/z/distance, NaN for invalid)\n");
    return 0;
}
