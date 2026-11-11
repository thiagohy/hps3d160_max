// Measures the temporal noise floor of the HPS-3D160 distance measurement,
// which determines whether sub-millimetre signals (heart beat) can be detected.
#include <HPS3DUser_IF.h>

#include <algorithm>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

static std::atomic<bool> g_stop{false};
static std::vector<std::vector<uint16_t>> g_frames;   // copies of the 9600 distances

static void cb(int, int eventType, uint8_t* data, int dataLen, void*) {
    (void)dataLen;
    if (eventType != 3) return;   // full depth only
    std::vector<uint16_t> d(9600);
    for (int i = 0; i < 9600; ++i) {
        d[i] = (uint16_t)((data[16 + 2 * i] << 8) | data[17 + 2 * i]);
    }
    g_frames.push_back(std::move(d));
}

static double mean(const std::vector<double>& v) {
    double s = 0;
    for (double x : v) s += x;
    return s / static_cast<double>(v.size());
}

static double stdev(const std::vector<double>& v) {
    if (v.size() < 2) return 0;
    const double m = mean(v);
    double s = 0;
    for (double x : v) s += (x - m) * (x - m);
    return std::sqrt(s / static_cast<double>(v.size() - 1));
}

int main(int argc, char** argv) {
    int seconds = argc > 1 ? atoi(argv[1]) : 10;
    int handle = -1;
    if (HPS3D_USBConnectDevice((char*)"/dev/ttyACM0", &handle) != HPS3D_RET_OK) {
        printf("connect failed\n");
        return 1;
    }
    HPS3D_RegisterEventCallback(cb, nullptr);
    HPS3D_StartCapture(handle);
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(seconds)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    HPS3D_StopCapture(handle);
    HPS3D_CloseDevice(handle);

    printf("frames captured: %zu\n", g_frames.size());
    if (g_frames.size() < 10) return 1;

    // ROI sizes: 1x1, 4x4, 8x8, 16x16 pixels around the centre
    const int sizes[] = {1, 4, 8, 16};
    printf("\n%-10s %10s %12s %14s %14s\n", "ROI (px)", "frames", "media (mm)", "sigma temporal", "sigma/px");
    for (int s : sizes) {
        std::vector<double> series;
        for (const auto& f : g_frames) {
            std::vector<double> vals;
            for (int y = 30 - s / 2; y < 30 + s / 2; ++y) {
                for (int x = 80 - s / 2; x < 80 + s / 2; ++x) {
                    const uint16_t d = f[y * 160 + x];
                    if (d > 0 && d < 60000) vals.push_back(d);
                }
            }
            if (!vals.empty()) series.push_back(mean(vals));
        }
        if (series.size() < 5) continue;
        const double sd = stdev(series);
        printf("%-10s %10zu %12.1f %12.2f mm %12.2f mm\n",
               s == 1 ? "1x1" : (std::to_string(s) + "x" + std::to_string(s)).c_str(),
               series.size(), mean(series), sd, sd / std::sqrt(static_cast<double>(s * s)));
    }

    // Per-pixel temporal sigma (single pixel, no averaging)
    std::vector<double> per_pixel;
    for (const auto& f : g_frames) {
        const uint16_t d = f[30 * 160 + 80];
        if (d > 0 && d < 60000) per_pixel.push_back(d);
    }
    printf("\nsingle pixel (80,30) temporal sigma: %.2f mm over %zu frames\n",
           stdev(per_pixel), per_pixel.size());

    // Spectrum of the ROI-mean series (looking for 0.2-0.5 Hz respiration band)
    if (per_pixel.size() > 50) {
        const double fps = static_cast<double>(g_frames.size()) / seconds;
        std::vector<double> centred;
        for (double v : per_pixel) centred.push_back(v - mean(per_pixel));
        // Goertzel over 0.1 .. 2.5 Hz
        printf("\nespectro (Goertzel) do pixel unico:\n");
        for (double f = 0.1; f <= 2.5; f += 0.1) {
            const double w = 2 * M_PI * f / fps;
            const double coeff = 2 * std::cos(w);
            double s1 = 0, s2 = 0;
            for (double x : centred) {
                const double s0 = x + coeff * s1 - s2;
                s2 = s1;
                s1 = s0;
            }
            const double power = s1 * s1 + s2 * s2 - coeff * s1 * s2;
            printf("%4.1f Hz : %8.4f %s\n", f, power, f >= 0.8 && f <= 2.0 ? "  <-- faixa de batimento" : "");
        }
    }
    return 0;
}
