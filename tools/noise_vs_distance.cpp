// Noise floor of the HPS-3D160 as a function of distance.
#include <HPS3DUser_IF.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>
#include <vector>

static std::atomic<bool> g_stop{false};
static std::vector<std::vector<uint16_t>> g_frames;

static void cb(int, int eventType, uint8_t* data, int, void*) {
    if (eventType != 3) return;
    std::vector<uint16_t> d(9600);
    for (int i = 0; i < 9600; ++i) d[i] = (uint16_t)((data[16 + 2 * i] << 8) | data[17 + 2 * i]);
    g_frames.push_back(std::move(d));
}

int main(int argc, char** argv) {
    int seconds = argc > 1 ? atoi(argv[1]) : 12;
    int handle = -1;
    if (HPS3D_USBConnectDevice((char*)"/dev/ttyACM0", &handle) != HPS3D_RET_OK) { printf("connect failed\n"); return 1; }
    HPS3D_RegisterEventCallback(cb, nullptr);
    HPS3D_StartCapture(handle);
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(seconds))
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    HPS3D_StopCapture(handle);
    HPS3D_CloseDevice(handle);

    const int N = (int)g_frames.size();
    printf("frames: %d\n\n", N);
    printf("%-16s %8s %14s %16s\n", "faixa dist (m)", "pixels", "sigma por pixel", "sigma do pixel medio");
    const double edges[] = {0.0, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 12.0};
    for (int b = 0; b + 1 < (int)(sizeof(edges) / sizeof(edges[0])); ++b) {
        const uint16_t lo = (uint16_t)(edges[b] * 1000), hi = (uint16_t)(edges[b + 1] * 1000);
        // collect per-pixel time series for pixels in this distance band
        std::vector<std::vector<double>> series;
        for (int p = 0; p < 9600; ++p) {
            std::vector<double> s;
            for (int f = 0; f < N; ++f) {
                const uint16_t d = g_frames[f][p];
                if (d >= lo && d < hi) s.push_back(d);
            }
            if (s.size() >= N / 2) series.push_back(std::move(s));
            if (series.size() > 400) break;
        }
        if (series.size() < 20) continue;
        double sum_var = 0;
        for (const auto& s : series) {
            double m = 0;
            for (double v : s) m += v;
            m /= s.size();
            double var = 0;
            for (double v : s) var += (v - m) * (v - m);
            sum_var += var / (s.size() - 1);
        }
        const double sigma_px = std::sqrt(sum_var / series.size());
        // sigma of the mean over the whole band (coherent averaging of n pixels)
        const double sigma_mean = sigma_px / std::sqrt((double)series.size());
        printf("%6.1f - %4.1f m %8zu %11.1f mm %13.2f mm\n", edges[b], edges[b + 1], series.size(),
               sigma_px, sigma_mean);
    }
    return 0;
}
