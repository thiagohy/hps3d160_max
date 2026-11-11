// Validates the ROS publisher against a minimal mock of the ROS API, so the
// file can be checked on hosts without a ROS installation.
// The same scenario is covered against the real sensor_msgs API by
// tests/test_ros_publisher_real.cpp (runs inside the Noetic container).
#include "ros_publisher.hpp"

#include <cmath>
#include <cstdio>

int main() {
    hps::PointCloudFrame frame{};
    frame.width = 4;
    frame.height = 3;
    frame.points.resize(4 * 3);
    for (int i = 0; i < 12; ++i) {
        auto& p = frame.points[i];
        p.x = 0.1f * static_cast<float>(i);
        p.y = 0.2f * static_cast<float>(i);
        p.z = 1.0f + 0.01f * static_cast<float>(i);
        p.distance_mm = static_cast<std::uint16_t>(1000 + i * 10);
        p.valid = true;
        p.status = hps::PixelStatus::Ok;
    }
    // the first pixel of the first row is invalid
    frame.points[0].valid = false;
    frame.points[0].status = hps::PixelStatus::Invalid;

    hps::RosPointCloudPublisher pub;
    ros::NodeHandle nh;
    pub.init(nh, "/hps3d160/point_cloud", "hps3d160", true);
    pub.publish(frame);

    const auto& msg = *ros::mock::last_pointcloud();
    printf("width=%u height=%u point_step=%u fields=%zu\n", msg.width, msg.height, msg.point_step, msg.fields.size());
    for (const auto& f : msg.fields) {
        printf("  field %-10s offset=%u datatype=%u count=%u\n", f.name.c_str(), f.offset, f.datatype, f.count);
    }

    auto read = [&](uint32_t idx, uint32_t off) {
        const float* base = reinterpret_cast<const float*>(msg.data.data() + idx * msg.point_step + off);
        return *base;
    };
    uint32_t xoff = 0, yoff = 0, zoff = 0, doff = 0;
    for (const auto& f : msg.fields) {
        if (f.name == "x") xoff = f.offset;
        else if (f.name == "y") yoff = f.offset;
        else if (f.name == "z") zoff = f.offset;
        else if (f.name == "distance") doff = f.offset;
    }

    const float vx = read(0, xoff), vz = read(0, zoff), vd = read(0, doff);
    printf("point0: x=%.4f z=%.4f distance=%.4f (invalid -> nan expected: %s)\n", vx, vz, vd,
           std::isnan(vx) && std::isnan(vz) && std::isnan(vd) ? "yes" : "NO");

    const float gz = read(1, zoff), gd = read(1, doff);
    printf("point1: z=%.4f distance=%.4f\n", gz, gd);

    const bool ok = msg.width == 4 && msg.height == 3 && msg.fields.size() == 4 &&
                    std::isnan(vx) && std::isnan(vz) && std::isnan(vd) &&
                    std::fabs(gz - 1.01f) < 1e-6 && std::fabs(gd - 1.01f) < 1e-4;
    printf("%s\n", ok ? "PUBLISHER TEST OK" : "PUBLISHER TEST FAILED");
    return ok ? 0 : 1;
}
