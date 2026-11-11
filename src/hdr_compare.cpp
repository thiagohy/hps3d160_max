#include "hdr_compare.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <sstream>
#include <thread>

namespace hps {
namespace {

constexpr std::uint32_t kPatchSize = 8;
constexpr int kSectors = 12;
constexpr double kHalfHFOV = 38.0;   // horizontal half field of view (deg)

// Residual std-dev of a least-squares plane fit over a patch of points.
// Returns -1 when the patch is degenerate.
double local_plane_residual(const std::vector<std::array<double, 3>>& pts) {
    const std::size_t n = pts.size();
    if (n < 8) return -1.0;
    double Sx = 0, Sy = 0, Sz = 0, Sxx = 0, Syy = 0, Sxy = 0, Sxz = 0, Syz = 0;
    for (const auto& p : pts) {
        Sx += p[0]; Sy += p[1]; Sz += p[2];
        Sxx += p[0]*p[0]; Syy += p[1]*p[1]; Sxy += p[0]*p[1];
        Sxz += p[0]*p[2]; Syz += p[1]*p[2];
    }
    double A[3][3] = {{Sxx, Sxy, Sx}, {Sxy, Syy, Sy}, {Sx, Sy, static_cast<double>(n)}};
    double b[3] = {Sxz, Syz, Sz};
    for (int i = 0; i < 3; ++i) {
        int piv = i;
        for (int r = i + 1; r < 3; ++r) if (std::fabs(A[r][i]) > std::fabs(A[piv][i])) piv = r;
        if (std::fabs(A[piv][i]) < 1e-12) return -1.0;
        if (piv != i) {
            for (int c = 0; c < 3; ++c) std::swap(A[i][c], A[piv][c]);
            std::swap(b[i], b[piv]);
        }
        for (int r = i + 1; r < 3; ++r) {
            const double f = A[r][i] / A[i][i];
            for (int c = i; c < 3; ++c) A[r][c] -= f * A[i][c];
            b[r] -= f * b[i];
        }
    }
    double x[3] = {0, 0, 0};
    for (int i = 2; i >= 0; --i) {
        double sum = b[i];
        for (int c = i + 1; c < 3; ++c) sum -= A[i][c] * x[c];
        x[i] = sum / A[i][i];
    }
    double sse = 0.0;
    for (const auto& p : pts) {
        const double r = p[2] - (x[0]*p[0] + x[1]*p[1] + x[2]);
        sse += r * r;
    }
    return std::sqrt(sse / static_cast<double>(n - 3));
}

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
    o << "    \"snr_proxy\": " << snr_proxy;
    if (!sectors.empty()) {
        o << ",\n    \"sectors\": [\n";
        for (std::size_t i = 0; i < sectors.size(); ++i) {
            const auto& s = sectors[i];
            o << "      {\"angle_deg\": " << s.angle_center_deg
              << ", \"coverage\": " << s.coverage
              << ", \"noise_mm\": " << s.spatial_noise_mm
              << ", \"valid_pixels\": " << s.valid_pixels
              << ", \"total_pixels\": " << s.total_pixels << "}";
            if (i + 1 < sectors.size()) o << ",";
            o << "\n";
        }
        o << "    ]\n";
    } else {
        o << "\n";
    }
    o << "  }";
    return o.str();
}

void HdrMetricsAccumulator::add(const PointCloudFrame& frame) {
    if (sector_valid_.empty()) {
        sector_valid_.assign(kSectors, 0);
        sector_total_.assign(kSectors, 0);
        sector_noise_sum_.assign(kSectors, 0.0);
        sector_patches_.assign(kSectors, 0);
    }

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
    // per sector accounting (horizontal angular sector of the pixel)
    const double sector_w = (2.0 * kHalfHFOV) / kSectors;
    for (std::uint32_t y = 0; y < frame.height && y < frame.points.size() / (frame.width ? frame.width : 1); ++y) {
        for (std::uint32_t x = 0; x < frame.width; ++x) {
            const std::size_t idx = static_cast<std::size_t>(y) * frame.width + x;
            if (idx >= frame.points.size()) continue;
            const double angle = ((static_cast<double>(x) + 0.5) / frame.width * 2.0 - 1.0) * kHalfHFOV;
            int s_idx = static_cast<int>((angle + kHalfHFOV) / sector_w);
            if (s_idx < 0) s_idx = 0;
            if (s_idx >= kSectors) s_idx = kSectors - 1;
            sector_total_[s_idx]++;
            if (frame.points[idx].status == PixelStatus::Ok) sector_valid_[s_idx]++;
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
                std::vector<std::array<double, 3>> patch_points;  // x, y, distance
                vals.reserve(kPatchSize * kPatchSize);
                patch_points.reserve(kPatchSize * kPatchSize);
                for (std::uint32_t y = by * kPatchSize; y < (by + 1) * kPatchSize; ++y) {
                    for (std::uint32_t x = bx * kPatchSize; x < (bx + 1) * kPatchSize; ++x) {
                        std::size_t idx = static_cast<std::size_t>(y) * frame.width + x;
                        if (idx >= frame.points.size()) continue;
                        const auto& p = frame.points[idx];
                        if (p.status == PixelStatus::Ok) {
                            vals.push_back(p.distance_mm);
                            patch_points.push_back({p.x, p.y, static_cast<double>(p.distance_mm)});
                        }
                    }
                }
                if (vals.size() < 8) continue;
                // Fit a local plane to the patch and use the residual std-dev:
                // this separates real noise from scene structure (slopes,
                // edges) that would otherwise inflate the raw std-dev.
                double sigma = local_plane_residual(patch_points);
                if (sigma < 0.0) continue;
                patch_noise_sum_ += sigma;
                patch_count_++;
                // attribute the patch to a horizontal sector
                const double centre_x = (bx + 0.5) * kPatchSize;
                const double angle = (centre_x / frame.width * 2.0 - 1.0) * kHalfHFOV;
                int s_idx = static_cast<int>((angle + kHalfHFOV) / sector_w);
                if (s_idx < 0) s_idx = 0;
                if (s_idx >= kSectors) s_idx = kSectors - 1;
                sector_noise_sum_[s_idx] += sigma;
                sector_patches_[s_idx]++;
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

    if (!sector_total_.empty()) {
        const double sector_w = (2.0 * kHalfHFOV) / kSectors;
        for (int i = 0; i < kSectors; ++i) {
            SectorMetrics sm;
            sm.angle_center_deg = -kHalfHFOV + sector_w * (i + 0.5);
            sm.total_pixels = sector_total_[i];
            sm.valid_pixels = sector_valid_[i];
            sm.coverage = sector_total_[i] ? static_cast<double>(sector_valid_[i]) / sector_total_[i] : 0.0;
            sm.spatial_noise_mm = sector_patches_[i] ? sector_noise_sum_[i] / sector_patches_[i] : 0.0;
            m.sectors.push_back(sm);
        }
    }
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
