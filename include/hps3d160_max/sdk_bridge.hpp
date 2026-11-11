#pragma once

#include "hps_internal.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace hps {

// Bridge to the low level API exported by the Hypersen shared library.
//
// The public HPS3DAPI_* API only exposes ~26 functions. The library itself
// exports the full device feature set as HE201_Api* / HE201_* symbols
// (HDR modes, ROI geometry + thresholds, GPIO in/out, obstacle avoidance,
// temperature, point cloud options, keep alive, interference detection,
// multi device splicing, bad pixel calibration, ...).
//
// This class resolves those symbols at runtime (dlsym) and maps the public
// integer handle onto the internal handle structure so they can be used.
// When anything cannot be resolved the advanced calls simply report failure
// instead of crashing.

class SdkBridge {
public:
    SdkBridge();
    ~SdkBridge();

    bool available() const { return available_; }

    // Binds the bridge to a public device handle (as returned by HPS3D_USBConnectDevice).
    bool bind(int public_handle);
    void unbind();
    int public_handle() const { return public_handle_; }
    void* internal_handle() const { return internal_handle_; }

    // --- capability getters -------------------------------------------------
    bool device_version(internal::Version_t& out) const;
    bool sensor_temperature(float& out) const;
    bool device_address(std::uint8_t& out) const;
    bool transport_type(internal::TransportTypeDef& out) const;
    bool number_of_rois(std::uint8_t& roi_count, std::uint8_t& threshold_count) const;
    bool roi_config(std::uint8_t roi_id, internal::ROIConfTypeDef& out) const;
    bool roi_group_id(std::uint8_t& out) const;
    bool hdr_config(internal::HDRConf& out) const;
    bool gpio_in_config(std::uint8_t pin, internal::GPIOInConfTypeDef& out) const;
    bool gpio_out_config(std::uint8_t pin, internal::GPIOOutConfTypeDef& out) const;
    bool gpio_count(std::uint8_t& input_count, std::uint8_t& output_count) const;
    bool optical_config(internal::OpticalParamConfTypeDef& out) const;
    bool mounting_config(internal::MountingAngleParamTypeDef& out) const;
    bool distance_filter_config(internal::DistanceFilterConfTypeDef& out) const;
    bool smooth_filter_config(internal::SmoothFilterConfTypeDef& out) const;
    bool interference_config(internal::InterferenceDetectConfTypeDef& out) const;
    bool obstacle_config(internal::ObstacleConfigTypedef& out) const;
    bool distance_offset(std::int16_t& out) const;
    bool point_cloud_enabled(bool& out) const;
    bool point_cloud_mirror(internal::PointCloudImageMirrorTypeDef& out) const;
    bool edge_detection_enabled(bool& out) const;
    bool edge_detection_value(std::int32_t& out) const;
    bool threshold_offset(std::int32_t& out) const;
    bool obstacle_pixel_threshold(std::uint32_t& out) const;
    bool keep_alive_enabled(bool& out) const;
    bool keep_alive_time(std::uint32_t& out) const;
    bool multi_camera_code(std::uint8_t& out) const;

    // --- capability setters -------------------------------------------------
    bool set_roi_group(std::uint8_t group_id) const;
    bool set_hdr_mode(internal::HDRModeTypeDef mode) const;
    bool set_roi_enable(std::uint32_t roi_id, bool enable) const;
    bool set_gpio_out(const internal::GPIOOutConfTypeDef& conf) const;
    bool set_gpio_in(const internal::GPIOInConfTypeDef& conf) const;
    bool set_keep_alive(bool enable, std::uint32_t time_ms) const;
    bool set_point_cloud_enabled(bool enable) const;
    bool set_point_cloud_mirror(internal::PointCloudImageMirrorTypeDef type) const;

    // Names of the advanced entry points that were resolved successfully.
    const std::vector<std::string>& resolved_symbols() const { return resolved_; }

private:
    template <typename Fn>
    Fn resolve(const char* name) const;

    bool available_ = false;
    bool bound_ = false;
    std::uintptr_t base_ = 0;
    int public_handle_ = -1;
    void* internal_handle_ = nullptr;
    std::vector<std::string> resolved_;
};

} // namespace hps
