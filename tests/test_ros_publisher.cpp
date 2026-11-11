#include "ros_publisher.hpp"

#include <cmath>
#include <cstdio>

int main() {
    hps::PointCloudFrame frame{};
    frame.width = 160;
    frame.height = 60;
    frame.points.resize(160 * 60);
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 160; ++x) {
            auto& p = frame.points[y * 160 + x];
            p.x = x * 0.001f; p.y = y * 0.001f; p.z = 1.5f;
            p.distance_mm = 1500; p.valid = true; p.status = hps::PixelStatus::Ok;
        }
    }
    for (int x = 0; x < 160; ++x) {
        frame.points[x].valid = false; frame.points[x].status = hps::PixelStatus::Invalid;
    }

    hps::RosPointCloudPublisher pub;
    ros::NodeHandle nh;
    pub.init(nh, "/hps3d160/point_cloud", "hps3d160", true);
    pub.publish(frame);

    const auto& msg = *ros::mock::last_pointcloud();
    printf("width=%u height=%u point_step=%u fields=%zu\n", msg.width, msg.height, msg.point_step, msg.fields.size());
    for (const auto& f : msg.fields) printf("  field %-10s offset=%u datatype=%u count=%u\n", f.name.c_str(), f.offset, f.datatype, f.count);

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
    const float gx = read(1, xoff), gz = read(1, zoff), gd = read(1, doff);
    printf("point1: x=%.4f z=%.4f distance=%.4f\n", gx, gz, gd);
    const bool ok = std::isnan(vx) && std::isnan(vz) && std::isnan(vd) &&
                    std::fabs(gz - 1.01f) < 1e-6 && std::fabs(gd - 1.01f) < 1e-4;
    printf("%s\n", ok ? "PUBLISHER TEST OK" : "PUBLISHER TEST FAILED");
    return ok ? 0 : 1;
}
