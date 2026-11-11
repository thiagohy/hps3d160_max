#include "sdk_bridge.hpp"

#include <dlfcn.h>

#include <cstdio>
#include <cstring>

namespace {

// Layout of the internal handle array inside the shipped SDK library.
// The values are produced by scripts/extract_sdk_layout.sh from the binary
// symbols; the constants below match libHPS3DSDK64_1-8-6.so and are validated
// at runtime (see SdkBridge::bind).
#ifdef HPS3D160_MAX_HAVE_SDK_LAYOUT
#include "generated/sdk_layout.h"
#endif
#ifndef HPS_SDK_BASE_HANDLE_OFFSET
#define HPS_SDK_BASE_HANDLE_OFFSET 0xC33220ULL
#endif
#ifndef HPS_SDK_HANDLE_STRIDE
#define HPS_SDK_HANDLE_STRIDE 104ULL
#endif
constexpr std::uintptr_t kBaseHandleOffset = HPS_SDK_BASE_HANDLE_OFFSET;
constexpr std::uintptr_t kHandleStride = HPS_SDK_HANDLE_STRIDE;
constexpr std::uintptr_t kValidFlagOffset = 0x40;

} // namespace

namespace hps {

SdkBridge::SdkBridge() {
    // The library must already be loaded (the public API pulled it in).
    void* sym = dlsym(RTLD_DEFAULT, "HPS3DAPI_CloseDevice");
    if (!sym) {
        std::fprintf(stderr, "[SdkBridge] SDK library not loaded\n");
        return;
    }
    Dl_info info{};
    if (!dladdr(sym, &info) || !info.dli_fbase) {
        std::fprintf(stderr, "[SdkBridge] unable to resolve SDK load address\n");
        return;
    }
    base_ = reinterpret_cast<std::uintptr_t>(info.dli_fbase);
    available_ = true;
}

SdkBridge::~SdkBridge() = default;

template <typename Fn>
Fn SdkBridge::resolve(const char* name) const {
    void* sym = dlsym(RTLD_DEFAULT, name);
    if (!sym) return nullptr;
    return reinterpret_cast<Fn>(sym);
}

bool SdkBridge::bind(int public_handle) {
    if (!available_) return false;
    if (public_handle < 0) return false;

    auto* arr = reinterpret_cast<std::uint8_t*>(base_ + kBaseHandleOffset);
    auto* h = arr + static_cast<std::uintptr_t>(public_handle) * kHandleStride;

    // Validation: the internal array keeps an "in use" flag at +0x40.
    const bool valid = h[kValidFlagOffset] != 0;
    if (!valid) {
        std::fprintf(stderr,
                     "[SdkBridge] handle %d does not look valid (flag=%d) - advanced API disabled\n",
                     public_handle, static_cast<int>(h[kValidFlagOffset]));
        return false;
    }

    internal_handle_ = h;
    public_handle_ = public_handle;
    bound_ = true;
    return true;
}

void SdkBridge::unbind() {
    internal_handle_ = nullptr;
    public_handle_ = -1;
    bound_ = false;
}

// --- getters ----------------------------------------------------------------

bool SdkBridge::device_version(internal::Version_t& out) const {
    using Fn = int (*)(void*, internal::Version_t*);
    auto fn = resolve<Fn>("HE201_ApiGetDeviceVersion");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::sensor_temperature(float& out) const {
    using Fn = int (*)(void*, float*);
    auto fn = resolve<Fn>("HE201_ApiGetSensorTemperature");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::device_address(std::uint8_t& out) const {
    using Fn = int (*)(void*, std::uint8_t*);
    auto fn = resolve<Fn>("HE201_ApiGetDevAddr");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::transport_type(internal::TransportTypeDef& out) const {
    using Fn = int (*)(void*, internal::TransportTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiGetTransportType");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::number_of_rois(std::uint8_t& roi_count, std::uint8_t& threshold_count) const {
    using Fn = int (*)(void*, std::uint8_t*, std::uint8_t*);
    auto fn = resolve<Fn>("HE201_ApiGetNumberOfROI");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &roi_count, &threshold_count) == 1;
}

bool SdkBridge::roi_config(std::uint8_t roi_id, internal::ROIConfTypeDef& out) const {
    using Fn = int (*)(void*, std::uint8_t, internal::ROIConfTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiGetROIConfById");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, roi_id, &out) == 1;
}

bool SdkBridge::roi_group_id(std::uint8_t& out) const {
    using Fn = int (*)(void*, std::uint8_t*);
    auto fn = resolve<Fn>("HE201_ApiGetROIGroupID");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::hdr_config(internal::HDRConf& out) const {
    using Fn = int (*)(void*, internal::HDRConf*);
    auto fn = resolve<Fn>("HE201_ApiGetHDRConfig");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::gpio_in_config(std::uint8_t pin, internal::GPIOInConfTypeDef& out) const {
    using Fn = int (*)(void*, std::uint8_t, internal::GPIOInConfTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiGetGPIOInConf");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, pin, &out) == 1;
}

bool SdkBridge::gpio_out_config(std::uint8_t pin, internal::GPIOOutConfTypeDef& out) const {
    using Fn = int (*)(void*, std::uint8_t, internal::GPIOOutConfTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiGetGPIOOutConf");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, pin, &out) == 1;
}

bool SdkBridge::gpio_count(std::uint8_t& input_count, std::uint8_t& output_count) const {
    using Fn = int (*)(void*, std::uint8_t*, std::uint8_t*);
    auto fn = resolve<Fn>("HE201_GetNumberOfGPIO");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &input_count, &output_count) == 1;
}

bool SdkBridge::optical_config(internal::OpticalParamConfTypeDef& out) const {
    using Fn = int (*)(void*, internal::OpticalParamConfTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiGetOpticalParamConf");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::mounting_config(internal::MountingAngleParamTypeDef& out) const {
    using Fn = int (*)(void*, internal::MountingAngleParamTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiGetMountingParamConf");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::distance_filter_config(internal::DistanceFilterConfTypeDef& out) const {
    using Fn = int (*)(void*, internal::DistanceFilterConfTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiGetDistanceFilterConf");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::smooth_filter_config(internal::SmoothFilterConfTypeDef& out) const {
    using Fn = int (*)(void*, internal::SmoothFilterConfTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiGetSmoothFilterConf");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::interference_config(internal::InterferenceDetectConfTypeDef& out) const {
    using Fn = int (*)(void*, internal::InterferenceDetectConfTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiGetInterferenceDetectConf");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::obstacle_config(internal::ObstacleConfigTypedef& out) const {
    using Fn = int (*)(internal::ObstacleConfigTypedef*);
    auto fn = resolve<Fn>("HE201_ApiGetObstacleConfig");
    if (!fn) return false;
    const int r = fn(&out);
    return r == 1;
}

bool SdkBridge::distance_offset(std::int16_t& out) const {
    using Fn = int (*)(void*, std::int16_t*);
    auto fn = resolve<Fn>("HE201_ApiGetDistanceOffset");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &out) == 1;
}

bool SdkBridge::point_cloud_enabled(bool& out) const {
    using Fn = int (*)();
    auto fn = resolve<Fn>("HE201_ApiGetPointCloudEn");
    if (!fn) return false;
    out = fn() != 0;
    return true;
}

bool SdkBridge::point_cloud_mirror(internal::PointCloudImageMirrorTypeDef& out) const {
    using Fn = int (*)();
    auto fn = resolve<Fn>("HE201_ApiGetPointCloudMirrorType");
    if (!fn) return false;
    out = static_cast<internal::PointCloudImageMirrorTypeDef>(fn());
    return true;
}

bool SdkBridge::edge_detection_enabled(bool& out) const {
    using Fn = int (*)();
    auto fn = resolve<Fn>("HE201_ApiGetEdgeDetectionEnable");
    if (!fn) return false;
    out = fn() != 0;
    return true;
}

bool SdkBridge::edge_detection_value(std::int32_t& out) const {
    using Fn = int (*)();
    auto fn = resolve<Fn>("HE201_ApiGetEdgeDetectionValue");
    if (!fn) return false;
    out = static_cast<std::int32_t>(fn());
    return true;
}

bool SdkBridge::threshold_offset(std::int32_t& out) const {
    using Fn = int (*)();
    auto fn = resolve<Fn>("HE201_ApiGetThresholdOffset");
    if (!fn) return false;
    out = static_cast<std::int32_t>(fn());
    return true;
}

bool SdkBridge::obstacle_pixel_threshold(std::uint32_t& out) const {
    using Fn = int (*)();
    auto fn = resolve<Fn>("HE201_ApiGetObstaclePixelNumberThreshold");
    if (!fn) return false;
    out = static_cast<std::uint32_t>(fn());
    return true;
}

bool SdkBridge::keep_alive_enabled(bool& out) const {
    using Fn = int (*)();
    auto fn = resolve<Fn>("HE201_ApiGetKeepAliveEnale");
    if (!fn) return false;
    out = fn() != 0;
    return true;
}

bool SdkBridge::keep_alive_time(std::uint32_t& out) const {
    using Fn = int (*)();
    auto fn = resolve<Fn>("HE201_ApiGetKeepAliveTime");
    if (!fn) return false;
    out = static_cast<std::uint32_t>(fn());
    return true;
}

bool SdkBridge::multi_camera_code(std::uint8_t& out) const {
    using Fn = int (*)(void*);
    auto fn = resolve<Fn>("HE201_ApiGetMultiCameraCode");
    if (!fn || !bound_) return false;
    const int v = fn(internal_handle_);
    if (v < 0) return false;
    out = static_cast<std::uint8_t>(v);
    return true;
}

// --- setters ----------------------------------------------------------------

bool SdkBridge::set_roi_group(std::uint8_t group_id) const {
    using Fn = int (*)(void*, std::uint8_t);
    auto fn = resolve<Fn>("HE201_ApiSelectROIGroup");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, group_id) == 1;
}

bool SdkBridge::set_hdr_mode(internal::HDRModeTypeDef mode) const {
    using Fn = int (*)(void*, internal::HDRModeTypeDef);
    auto fn = resolve<Fn>("HE201_ApiSetHDRMode");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, mode) == 1;
}

bool SdkBridge::set_roi_enable(std::uint32_t roi_id, bool enable) const {
    using Fn = int (*)(void*, std::uint32_t, bool);
    auto fn = resolve<Fn>("HE201_ApiSetROIEnable");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, roi_id, enable) == 1;
}

bool SdkBridge::set_gpio_out(const internal::GPIOOutConfTypeDef& conf) const {
    using Fn = int (*)(void*, const internal::GPIOOutConfTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiSetGPIOOut");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &conf) == 1;
}

bool SdkBridge::set_gpio_in(const internal::GPIOInConfTypeDef& conf) const {
    using Fn = int (*)(void*, const internal::GPIOInConfTypeDef*);
    auto fn = resolve<Fn>("HE201_ApiSetGPIOIn");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, &conf) == 1;
}

bool SdkBridge::set_keep_alive(bool enable, std::uint32_t time_ms) const {
    using Fn = int (*)(void*, bool, std::uint32_t);
    auto fn = resolve<Fn>("HE201_ApiSetKeepAliveConfig");
    if (!fn || !bound_) return false;
    return fn(internal_handle_, enable, time_ms) == 1;
}

bool SdkBridge::set_point_cloud_enabled(bool enable) const {
    using Fn = int (*)(bool);
    auto fn = resolve<Fn>("HE201_ApiSetPointCloudEn");
    if (!fn) return false;
    return fn(enable) == 1;
}

bool SdkBridge::set_point_cloud_mirror(internal::PointCloudImageMirrorTypeDef type) const {
    using Fn = int (*)(int);
    auto fn = resolve<Fn>("HE201_ApiSetPointCloudMirrorType");
    if (!fn) return false;
    return fn(static_cast<int>(type)) == 1;
}

} // namespace hps
