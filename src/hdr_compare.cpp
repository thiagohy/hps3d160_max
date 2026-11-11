#include "hdr_compare.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <thread>

namespace hps {
namespace {

constexpr std::uint32_t kPatchSize = 8;

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    std::size_t n = v.size();
    if (n % 2) return v[n / 2];
    return 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

} // namespace

std::string HdrMetrics::to_json() const {
    std::ostringstream o;
    o << "{\n";
    o << "    \"mode\": \"" << mode << "\",\n";
    o << "    \"frames\": " << frames << ",\n";
    o << "    \"fps\": " << fps << ",\n";
    o << "    \"coverage\": " << coverage << ",\n";
    o << "    \"low_amplitude_ratio\": " << low_amplitude_ratio << ",\n";
    o << "    \"saturated_ratio\": " << saturated_ratio << ",\n";
    o << "    \"invalid_ratio\": " << invalid_ratio << ",\n";
    o << "    \"mean_distance_mm\": " << mean_distance_mm << ",\n";
    o << "    \"min_distance_mm\": " << min_distance_mm << ",\n";
    o << "    \"spatial_noise_mm\": " << spatial_noise_mm << ",\n";
    o << "    \"temporal_jitter_mm\": " << temporal_jitter_mm << ",\n";
    o << "    \"snr_proxy\": " << snr_proxy << "\n";
    o << "  }";
    return o.str();
}

void HdrMetricsAccumulator::add(const PointCloudFrame& frame) {
    const auto now = std::chrono::steady_clock::now();
    if (frames_ == 0) {
        first_frame_ = now;
    }
    last_frame_ = now;
    frames_++;

    std::size_t valid = 0, low = 0, sat = 0, inv = 0;
    double sum = 0.0;
    for (const auto& p : frame.points) {
        switch (p.status) {
            case PixelStatus::Ok: valid++; sum += p.distance_mm; break;
            case PixelStatus::LowAmplitude: low++; break;
            case PixelStatus::Saturation:
            case PixelStatus::AdcOverflow: sat++; break;
            default: inv++; break;
        }
    }
    total_pixels_ += frame.points.size();
    valid_pixels_ += valid;
    low_amplitude_ += low;
    saturated_ += sat;
    invalid_ += inv;
    if (valid > 0) {
        double mean = sum / static_cast<double>(valid);
        distance_sum_ += mean;
        frame_means_.push_back(mean);
        distance_min_ = std::min(distance_min_, static_cast<double>(frame.distance_min_mm));
    }

    // Spatial noise: std-dev of the distance inside each 8x8 patch of valid pixels.
    if (frame.width >= kPatchSize && frame.height >= kPatchSize) {
        const std::uint32_t px = frame.width / kPatchSize;
        const std::uint32_t py = frame.height / kPatchSize;
        for (std::uint32_t by = 0; by < py; ++by) {
            for (std::uint32_t bx = 0; bx < px; ++bx) {
                std::vector<double> vals;
                vals.reserve(kPatchSize * kPatchSize);
                for (std::uint32_t y = by * kPatchSize; y < (by + 1) * kPatchSize; ++y) {
                    for (std::uint32_t x = bx * kPatchSize; x < (bx + 1) * kPatchSize; ++x) {
                        std::size_t idx = static_cast<std::size_t>(y) * frame.width + x;
                        if (idx >= frame.points.size()) continue;
                        const auto& p = frame.points[idx];
                        if (p.status == PixelStatus::Ok) vals.push_back(p.distance_mm);
                    }
                }
                if (vals.size() < 8) continue;
                double m = 0.0;
                for (double v : vals) m += v;
                m /= static_cast<double>(vals.size());
                double var = 0.0;
                for (double v : vals) var += (v - m) * (v - m);
                var /= static_cast<double>(vals.size() - 1);
                patch_noise_sum_ += std::sqrt(var);
                patch_count_++;
            }
        }
    }
}

HdrMetrics HdrMetricsAccumulator::finish() const {
    HdrMetrics m;
    m.mode = mode_;
    m.frames = frames_;
    if (frames_ == 0) return m;

    const double pixels = static_cast<double>(total_pixels_);
    m.coverage = pixels > 0 ? static_cast<double>(valid_pixels_) / pixels : 0.0;
    m.low_amplitude_ratio = pixels > 0 ? static_cast<double>(low_amplitude_) / pixels : 0.0;
    m.saturated_ratio = pixels > 0 ? static_cast<double>(saturated_) / pixels : 0.0;
    m.invalid_ratio = pixels > 0 ? static_cast<double>(invalid_) / pixels : 0.0;
    m.mean_distance_mm = static_cast<double>(frame_means_.size())
                             ? distance_sum_ / static_cast<double>(frame_means_.size())
                             : 0.0;
    m.min_distance_mm = distance_min_ < 1e9 ? distance_min_ : 0.0;
    m.spatial_noise_mm = patch_count_ ? patch_noise_sum_ / static_cast<double>(patch_count_) : 0.0;

    if (frame_means_.size() > 1) {
        double mean = 0.0;
        for (double v : frame_means_) mean += v;
        mean /= static_cast<double>(frame_means_.size());
        double var = 0.0;
        for (double v : frame_means_) var += (v - mean) * (v - mean);
        var /= static_cast<double>(frame_means_.size() - 1);
        m.temporal_jitter_mm = std::sqrt(var);
    }

    m.snr_proxy = m.spatial_noise_mm > 1e-6 ? m.mean_distance_mm / m.spatial_noise_mm : 0.0;

    const double elapsed = std::chrono::duration<double>(last_frame_ - first_frame_).count();
    m.fps = elapsed > 0 ? (frames_ - 1) / elapsed : 0.0;
    return m;
}

std::vector<HdrComparator::ModeSpec> HdrComparator::default_modes() {
    return {
        {internal::HDR_DISABLE, "disable"},
        {internal::AUTO_HDR, "auto_hdr"},
        {internal::SUPER_HDR, "super_hdr"},
        {internal::SIMPLE_HDR, "simple_hdr"},
    };
}

bool HdrComparator::apply_mode(SdkBridge& bridge, internal::HDRModeTypeDef mode) {
    if (!bridge.set_hdr_mode(mode)) return false;
    // Let the sensor settle (integration time + HDR frame synthesis).
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    internal::HDRConf cfg{};
    if (!bridge.hdr_config(cfg)) return false;
    return cfg.hdr_mode == mode;
}

} // namespace hps
