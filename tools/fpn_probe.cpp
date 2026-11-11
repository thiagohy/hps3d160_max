// Decomposes the HPS-3D160 distance noise into random and fixed-pattern
// components and measures the minimum displacement that can be detected in a
// region of a given size.
//
// Random noise averages down with the number of pixels, fixed pattern noise
// (FPN, speckle, per-pixel bias) does not. For a region of B pixels the
// measured standard deviation of the region mean gives the detection floor:
//
//   displacement_limit(B) = k * sigma_mean(B)      (k = 3 for a 3-sigma call)
//
// Usage: fpn_probe [SECONDS] [PORT] [HDR: disable|auto|super|simple|none]
#include <HPS3DUser_IF.h>

#include "sdk_bridge.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>
#include <mutex>
#include <vector>

namespace {

constexpr int kWidth = 160;
constexpr int kHeight = 60;
constexpr int kPixels = kWidth * kHeight;

std::vector<std::vector<float>> g_series;   // one vector per frame
std::mutex g_mutex;                             // the SDK thread may still call back

void onFrame(int, int eventType, std::uint8_t* data, int, void*) {
    if (eventType != 3) return;
    std::lock_guard<std::mutex> lk(g_mutex);
    std::vector<float> f(kPixels);
    for (int i = 0; i < kPixels; ++i) {
        const std::uint16_t d = static_cast<std::uint16_t>((data[16 + 2 * i] << 8) | data[17 + 2 * i]);
        f[i] = (d > 0 && d < 60000) ? static_cast<float>(d) : NAN;
    }
    g_series.push_back(std::move(f));
}

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const std::size_t n = v.size();
    return (n % 2) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

} // namespace

int main(int argc, char** argv) {
    const int seconds = argc > 1 ? std::atoi(argv[1]) : 20;
    const char* port = argc > 2 ? argv[2] : "/dev/ttyACM0";
    const char* hdr = argc > 3 ? argv[3] : "none";

    int handle = -1;
    if (HPS3D_USBConnectDevice(const_cast<char*>(port), &handle) != HPS3D_RET_OK) {
        std::printf("connect failed\n");
        return 1;
    }
    if (std::strcmp(hdr, "none") != 0) {
        hps::SdkBridge bridge;
        if (bridge.available() && bridge.bind(handle)) {
            hps::internal::HDRModeTypeDef m = hps::internal::HDR_DISABLE;
            if (!std::strcmp(hdr, "disable")) m = hps::internal::HDR_DISABLE;
            else if (!std::strcmp(hdr, "auto")) m = hps::internal::AUTO_HDR;
            else if (!std::strcmp(hdr, "super")) m = hps::internal::SUPER_HDR;
            else if (!std::strcmp(hdr, "simple")) m = hps::internal::SIMPLE_HDR;
            std::printf("HDR mode '%s' -> %s\n", hdr, bridge.set_hdr_mode(m) ? "applied" : "FAILED");
        }
    }
    HPS3D_RegisterEventCallback(&onFrame, nullptr);
    HPS3D_StartCapture(handle);
    std::printf("capturing %d s ...\n", seconds);
    for (int t = 0; t < seconds * 10; ++t) {
        usleep(100000);
        std::fflush(stdout);
    }
    HPS3D_StopCapture(handle);
    HPS3D_CloseDevice(handle);

    std::lock_guard<std::mutex> lk(g_mutex);
    const std::size_t F = g_series.size();
    std::printf("frames: %zu\n", F);
    if (F < 20) return 1;

    // per pixel: temporal mean and temporal std
    std::vector<double> mean(kPixels, 0), sd(kPixels, 0);
    std::vector<int> valid(kPixels, 0);
    for (int p = 0; p < kPixels; ++p) {
        double s = 0;
        int n = 0;
        for (const auto& f : g_series) {
            const float v = f[p];
            if (v == v) { s += v; ++n; }
        }
        valid[p] = n;
        // only pixels that are valid in EVERY frame are used, otherwise the
        // statistics mix the sensor noise with the pixels switching state
        if (n < 10 || n != static_cast<int>(g_series.size())) { mean[p] = NAN; continue; }
        mean[p] = s / n;
        double var = 0;
        for (const auto& f : g_series) {
            const float v = f[p];
            if (v == v) var += (v - mean[p]) * (v - mean[p]);
        }
        sd[p] = std::sqrt(var / (n - 1));
    }

    // ---- random vs fixed pattern ----
    // For every pixel, the difference between consecutive frames removes the
    // static (FPN) part: sigma_diff^2 = 2 * sigma_random^2
    std::vector<double> diff_sd;
    diff_sd.reserve(kPixels);
    for (int p = 0; p < kPixels; ++p) {
        if (valid[p] < F - 1 || std::isnan(mean[p])) continue;
        double s = 0;
        int n = 0;
        for (std::size_t f = 1; f < F; ++f) {
            const float a = g_series[f][p], b = g_series[f - 1][p];
            if (a == a && b == b) { s += (a - b) * (a - b); ++n; }
        }
        if (n > 10) diff_sd.push_back(std::sqrt(s / n));
    }
    const double sd_diff = median(diff_sd);
    const double sd_random = sd_diff / std::sqrt(2.0);

    // FPN: deviation of the per-pixel temporal mean from the local mean,
    // computed only on pixels that sit on a locally planar surface (edges and
    // corners would otherwise dominate the statistic).
    std::vector<double> fpn;
    std::vector<char> planar(kPixels, 0);
    for (int y = 2; y < kHeight - 2; ++y) {
        for (int x = 2; x < kWidth - 2; ++x) {
            const int p = y * kWidth + x;
            if (std::isnan(mean[p])) continue;
            double local = 0;
            int n = 0;
            for (int dy = -2; dy <= 2; ++dy)
                for (int dx = -2; dx <= 2; ++dx) {
                    const int q = (y + dy) * kWidth + (x + dx);
                    if (std::isnan(mean[q])) continue;
                    local += mean[q];
                    ++n;
                }
            if (n < 20) continue;
            local /= n;
            const double dev = mean[p] - local;
            fpn.push_back(dev);
            if (std::fabs(dev) < 60.0) planar[p] = 1;   // planar pixel
        }
    }
    double fpn_var = 0.0;
    {
        double m = 0;
        for (double v : fpn) m += v;
        m /= fpn.size();
        for (double v : fpn) fpn_var += (v - m) * (v - m);
        fpn_var /= fpn.size();
    }
    const double fpn_sd = std::sqrt(std::max(0.0, fpn_var - sd_random * sd_random));

    std::printf("\n=== decomposicao do ruido ===\n");
    std::printf("sigma por pixel (temporal)      : %.2f mm\n", median([&] {
        std::vector<double> a;
        for (int p = 0; p < kPixels; ++p) if (!std::isnan(sd[p])) a.push_back(sd[p]);
        return a; }()));
    std::printf("sigma aleatorio (dif. entre frames): %.2f mm\n", sd_random);
    std::printf("sigma de padrao fixo (FPN)        : %.2f mm\n", fpn_sd);

    // ---- detection floor as a function of the region size ----
    // The scene structure is removed first: the residual of each pixel with
    // respect to its local plane is used, so only the noise remains.
    std::vector<double> resid(kPixels, 0.0);
    std::vector<char> has_resid(kPixels, 0);
    for (int y = 3; y < kHeight - 3; ++y) {
        for (int x = 3; x < kWidth - 3; ++x) {
            const int p = y * kWidth + x;
            if (std::isnan(mean[p])) continue;
            // local plane z = a x + b y + c over a 7x7 window (temporal mean)
            double Sx = 0, Sy = 0, Sz = 0, Sxx = 0, Syy = 0, Sxy = 0, Sxz = 0, Syz = 0;
            int n = 0;
            for (int dy = -3; dy <= 3; ++dy)
                for (int dx = -3; dx <= 3; ++dx) {
                    const int q = (y + dy) * kWidth + (x + dx);
                    if (std::isnan(mean[q])) continue;
                    Sx += dx; Sy += dy; Sz += mean[q];
                    Sxx += dx * dx; Syy += dy * dy; Sxy += dx * dy;
                    Sxz += dx * mean[q]; Syz += dy * mean[q];
                    ++n;
                }
            if (n < 30) continue;
            double A[3][3] = {{Sxx, Sxy, Sx}, {Sxy, Syy, Sy}, {Sx, Sy, (double)n}};
            double b[3] = {Sxz, Syz, Sz};
            for (int i = 0; i < 3; ++i) {
                int piv = i;
                for (int r = i + 1; r < 3; ++r)
                    if (std::fabs(A[r][i]) > std::fabs(A[piv][i])) piv = r;
                if (std::fabs(A[piv][i]) < 1e-9) continue;
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
            double sol[3] = {0, 0, 0};
            bool ok = true;
            for (int i = 2; i >= 0; --i) {
                double s = b[i];
                for (int c = i + 1; c < 3; ++c) s -= A[i][c] * sol[c];
                if (std::fabs(A[i][i]) < 1e-9) { ok = false; break; }
                sol[i] = s / A[i][i];
            }
            if (!ok) continue;
            resid[p] = mean[p] - (sol[0] * 0.0 + sol[1] * 0.0 + sol[2]);   // local plane value at the centre
            has_resid[p] = 1;
        }
    }

    {
        std::vector<double> ar;
        for (int p = 0; p < kPixels; ++p) if (has_resid[p]) ar.push_back(std::fabs(resid[p]));
        std::printf("\nresiduos: n=%zu  mediana |r| = %.2f mm  p90 = %.2f mm\n", ar.size(),
                    ar.empty() ? 0.0 : median(ar), [&] {
                        std::vector<double> b = ar; std::sort(b.begin(), b.end());
                        return b.empty() ? 0.0 : b[(size_t)(b.size() * 0.9)]; }());
        std::printf("pixels planos: %d\n", (int)std::count(planar.begin(), planar.end(), 1));
    }

    std::printf("\n=== limiar de deteccao de deslocamento por tamanho de regiao ===\n");
    std::printf("(residuos em relacao ao plano local - a estrutura da cena e removida)\n");
    std::printf("%-10s %7s %14s %16s %14s\n", "regiao", "pixels", "sigma da media", "deteccao 3s",
                "com FPN cal.");
    for (int bs : {1, 2, 4, 8, 16, 32}) {
        std::vector<double> blocks;
        for (int by = 0; by + bs <= kHeight; by += bs) {
            for (int bx = 0; bx + bs <= kWidth; bx += bs) {
                double s = 0;
                int n = 0;
                for (int y = by; y < by + bs; ++y)
                    for (int x = bx; x < bx + bs; ++x) {
                        const int p = y * kWidth + x;
                        if (!has_resid[p] || !planar[p]) continue;
                        s += resid[p];
                        ++n;
                    }
                if (n >= bs * bs / 2) blocks.push_back(s / n);
            }
        }
        if (blocks.size() < 4) continue;
        const double m = median(blocks);
        std::vector<double> dev;
        for (double v : blocks) dev.push_back(std::fabs(v - m));
        const double sigma_block = median(dev) * 1.4826;
        // after per pixel FPN calibration only the random part remains
        const double sigma_cal = sd_random / std::sqrt((double)(bs * bs));
        std::printf("%-10s %7d %11.2f mm %13.2f mm %11.2f mm\n",
                    (std::to_string(bs) + "x" + std::to_string(bs)).c_str(), bs * bs, sigma_block,
                    3.0 * sigma_block, 3.0 * sigma_cal);
    }

    std::printf("\n(para comparar: batimento 0.3-2 mm, respiracao 5-20 mm)\n");
    return 0;
}
