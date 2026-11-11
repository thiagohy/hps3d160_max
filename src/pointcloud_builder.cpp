#include "pointcloud_builder.hpp"

#include <cmath>

namespace hps {
namespace {

// Intrinsics of the HPS-3D160 sensor: 160 x 60 pixels, 76 deg x 32 deg FoV.
constexpr double kWidth = 160.0;
constexpr double kHeight = 60.0;
// The firmware point cloud is a LINEAR (orthographic) approximation:
//   x = d * (80 - col) / 104      y = d * (30 - row) / 104      z = d
// so the stored point is not at the measured distance from the sensor (radial
// error up to 27 % at the FoV border) and the angular mapping is a tangent
// linearisation. The cloud is therefore rebuilt as a proper spherical point:
//   tan(theta_h) = (col - 80) / 104   tan(theta_v) = (30 - row) / 104
//   X = d sin(th) cos(ph)   Y = d sin(ph)   Z = d cos(th) cos(ph)
// which gives |(x,y,z)| == d exactly and the ROS convention (X right, Y up,
// Z forward). 104 px is the manufacturer's own calibration of the optics.
constexpr double kFocalPx = 104.0;
constexpr double kCenterX = 80.0;
constexpr double kCenterY = 30.0;

void build_metric_point(double px, double py, double distance_m,
                        float& out_x, float& out_y, float& out_z) {
    // The payload stores column 0 on the RIGHT side of the field of view
    // (the firmware x = d*(80 - col)/104 is positive there), so the horizontal
    // angle is (center - column).
    const double theta_h = std::atan((kCenterX - px) / kFocalPx);
    const double theta_v = std::atan((kCenterY - py) / kFocalPx);
    const double cos_v = std::cos(theta_v);
    out_x = static_cast<float>(distance_m * std::sin(theta_h) * cos_v);
    out_y = static_cast<float>(distance_m * std::sin(theta_v));
    out_z = static_cast<float>(distance_m * std::cos(theta_h) * cos_v);
}

} // namespace

PointCloudFrame PointCloudBuilder::build(const HPS3D_DepthData_t& depth, std::uint64_t frame_id) {
    PointCloudFrame frame{};
    frame.frame_id = frame_id;
    frame.width = depth.point_cloud_data.width > 0 ? depth.point_cloud_data.width : 160;
    frame.height = depth.point_cloud_data.height > 0 ? depth.point_cloud_data.height : 60;
    frame.distance_average_mm = depth.distance_average;
    frame.distance_min_mm = depth.distance_min;
    frame.saturation_count = depth.saturation_count;
    frame.frame_counter = depth.frame_cnt;

    const std::size_t n = static_cast<std::size_t>(frame.width) * frame.height;
    frame.points.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
        Point3D p{};
        p.distance_mm = (i < HPS3D_MAX_PIXEL_NUMBER) ? depth.distance[i] : 0;

        // Classify the pixel using the SDK sentinel values.
        if (p.distance_mm == 0) {
            p.status = PixelStatus::Zero;
            p.valid = false;
        } else if (p.distance_mm == HPS3D_LOW_AMPLITUDE) {
            p.status = PixelStatus::LowAmplitude;
            p.valid = false;
        } else if (p.distance_mm == HPS3D_SATURATION) {
            p.status = PixelStatus::Saturation;
            p.valid = false;
        } else if (p.distance_mm == HPS3D_ADC_OVERFLOW) {
            p.status = PixelStatus::AdcOverflow;
            p.valid = false;
        } else if (p.distance_mm >= HPS3D_INVALID_DATA) {
            p.status = PixelStatus::Invalid;
            p.valid = false;
        } else {
            p.status = PixelStatus::Ok;
            p.valid = true;
        }

        if (p.valid) {
            const double distance_m = p.distance_mm / 1000.0;
            const double px = static_cast<double>(i % frame.width);
            const double py = static_cast<double>(i / frame.width);
            build_metric_point(px, py, distance_m, p.x, p.y, p.z);
        }
        frame.points.push_back(p);
    }
    frame.point_count = static_cast<std::uint32_t>(frame.points.size());
    return frame;
}

void PointCloudBuilder::filterInvalid(PointCloudFrame& frame) {
    std::vector<Point3D> valid;
    valid.reserve(frame.points.size());
    for (auto& p : frame.points) {
        if (p.valid) valid.push_back(p);
    }
    frame.points.swap(valid);
    frame.point_count = static_cast<std::uint32_t>(frame.points.size());
}

} // namespace hps