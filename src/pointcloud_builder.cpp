#include "pointcloud_builder.hpp"

#include <cmath>

namespace hps {

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
        // HPS3D point cloud is in cm (x/100). Convert to meters.
        if (depth.point_cloud_data.point_data) {
            p.x = depth.point_cloud_data.point_data[i].x / 100.0f;
            p.y = depth.point_cloud_data.point_data[i].y / 100.0f;
            p.z = depth.point_cloud_data.point_data[i].z / 100.0f;
        }
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