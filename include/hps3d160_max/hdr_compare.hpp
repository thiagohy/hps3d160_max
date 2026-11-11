#pragma once

#include "sdk_bridge.hpp"
#include "types.hpp"

#include <chrono>
#include <string>
#include <vector>

namespace hps {

// Aggregated quality metrics for a set of frames.
//
// The HPS-3D160 firmware does not expose the per-pixel signal amplitude in the
// measurement payload, so the SNR is estimated from the spatial and temporal
// behaviour of the depth map:
//
//   * coverage      -> fraction of pixels with a usable distance
//   * weak ratio    -> fraction of pixels flagged as LOW_AMPLITUDE
//   * saturated     -> fraction of pixels flagged SATURATION / ADC_OVERFLOW
//   * spatial noise -> median over 8x8 patches of the std-dev of the distance
//                     (mm). A flat wall gives a direct estimate of the noise.
//   * temporal jitter-> std-dev of the mean distance between consecutive frames
//   * snr proxy     -> mean distance / spatial noise (dB like ratio)
// Metrics for one horizontal sector of the field of view (76 deg / 12 sectors).
struct SectorMetrics {
    double angle_center_deg = 0.0;
    double coverage = 0.0;
    double spatial_noise_mm = 0.0;
    int valid_pixels = 0;
    int total_pixels = 0;
};

struct HdrMetrics {
    std::string mode;
    int frames = 0;
    double fps = 0.0;

    double coverage = 0.0;        // [0..1]
    double low_amplitude_ratio = 0.0;
    double saturated_ratio = 0.0;
    double invalid_ratio = 0.0;

    double mean_distance_mm = 0.0;
    double min_distance_mm = 0.0;
    double spatial_noise_mm = 0.0;
    double temporal_jitter_mm = 0.0;
    double snr_proxy = 0.0;
    std::vector<SectorMetrics> sectors;

    std::string to_json() const;
};

class HdrMetricsAccumulator {
public:
    explicit HdrMetricsAccumulator(std::string mode) : mode_(std::move(mode)) {}

    void add(const PointCloudFrame& frame);
    HdrMetrics finish() const;

private:
    std::string mode_;
    int frames_ = 0;
    std::size_t total_pixels_ = 0;
    std::size_t valid_pixels_ = 0;
    std::size_t low_amplitude_ = 0;
    std::size_t saturated_ = 0;
    std::size_t invalid_ = 0;
    double distance_sum_ = 0.0;
    double distance_min_ = 1e9;
    double patch_noise_sum_ = 0.0;
    int patch_count_ = 0;
    std::vector<double> frame_means_;
    // per horizontal sector (12 sectors over the 76 deg FoV)
    std::vector<int> sector_valid_;
    std::vector<int> sector_total_;
    std::vector<double> sector_noise_sum_;
    std::vector<int> sector_patches_;
    double elapsed_seconds_ = 0.0;
    std::chrono::steady_clock::time_point first_frame_{};
    std::chrono::steady_clock::time_point last_frame_{};
};

// Runs the same measurement sequence for every HDR mode and collects metrics.
class HdrComparator {
public:
    struct ModeSpec {
        internal::HDRModeTypeDef mode;
        const char* name;
    };

    static std::vector<ModeSpec> default_modes();

    // Selects the mode on the device. Returns false when the SDK refuses it.
    static bool apply_mode(SdkBridge& bridge, internal::HDRModeTypeDef mode);
};

} // namespace hps
