#pragma once

#include "sdk_bridge.hpp"
#include "types.hpp"

#include <string>
#include <vector>

namespace hps {

struct ThresholdReport {
    int id = 0;
    bool enable = false;
    int value = 0;
    int hysteresis = 0;
    bool positive = true;
    int pixel_number_threshold = 0;
    std::string reference;   // distance_avg | distance_min | ...
    std::string alarm;       // disable | gpio
};

struct RoiReport {
    int id = 0;
    bool enabled = false;
    int left_top_x = 0;
    int left_top_y = 0;
    int right_bottom_x = 0;
    int right_bottom_y = 0;
    std::vector<ThresholdReport> thresholds;
};

// Collects every piece of hardware information that the SDK can expose:
// public API data plus the low level (HE201_*) device configuration.
struct CapabilityReport {
    // identity
    std::string device_version;
    std::string sdk_version;
    std::string serial_number;

    // link
    std::string transport;      // usb | rs232 | rs485 | can | ethernet
    int transport_code = -1;
    int device_address = -1;
    int multi_camera_code = -1;
    bool keep_alive_enabled = false;
    int keep_alive_time_ms = 0;

    // optics / geometry
    std::string hdr_mode;        // disable | auto | super | simple
    float hdr_overexposed = 0.f;
    float hdr_weak = 0.f;
    int hdr_simple_max_integration_us = 0;
    int hdr_simple_min_integration_us = 0;
    int hdr_super_frame_number = 0;
    int hdr_super_max_integration_us = 0;
    int hdr_disable_integration_us = 0;
    int max_integration_time_us = 0;
    int min_amplitude = 0;
    bool optical_compensation = false;
    int view_angle_horiz = 0;
    int view_angle_vertical = 0;
    int illum_angle_horiz = 0;
    int illum_angle_vertical = 0;
    bool mounting_enabled = false;
    int mounting_angle_deg = 0;
    int mounting_height_mm = 0;

    // filtering
    std::string distance_filter_type;  // disable | simple_kalman
    float distance_filter_k = 0.f;
    int distance_filter_threshold = 0;
    int distance_filter_num_check = 0;
    std::string smooth_filter_type;    // disable | average | gauss
    int smooth_filter_arg = 0;
    int distance_offset_mm = 0;
    bool edge_detection_enabled = false;
    int edge_detection_value = 0;
    int threshold_offset = 0;

    // ROI
    int roi_count = 0;
    int roi_threshold_count = 0;
    int roi_group_id = 0;
    std::vector<RoiReport> rois;

    // GPIO (HPS-3D160-U has 1 optocoupler input and 1 output)
    int gpio_count = 0;
    int gpio_in_count = 0;
    int gpio_out_count = 0;
    std::string gpio_in;   // description of the input configuration
    std::string gpio_out;

    // multi device / interference
    bool interference_detect_enabled = false;
    int interference_integration_time_us = 0;
    int interference_amplitude_threshold = 0;
    int interference_capture_num = 0;
    int interference_number_check = 0;

    // point cloud / obstacle
    bool point_cloud_enabled = false;
    std::string point_cloud_mirror;  // disable | horizontal | vertical | both
    bool obstacle_enabled = false;
    int obstacle_count = 0;
    int obstacle_min_dist_mm = 0;
    int obstacle_max_dist_mm = 0;
    int obstacle_invalid_value_mm = 0;
    int obstacle_pixel_threshold = 0;

    // diagnostics
    float sensor_temperature_c = 0.f;
    std::vector<std::string> notes;

    std::string to_json() const;
};

class CapabilityProbe {
public:
    explicit CapabilityProbe(SdkBridge& bridge) : bridge_(bridge) {}

    CapabilityReport collect() const;

private:
    SdkBridge& bridge_;
};

} // namespace hps
