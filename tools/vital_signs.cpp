// Statistical search for a cardiac component in the HPS-3D160 depth stream.
//
// Method (matched filter + consistency test):
//   1. per pixel: temporal series of the measured distance, outliers rejected
//   2. detrend, Hann window, Goertzel spectrum
//   3. cardiac band 0.7-2.5 Hz compared with a control band of the SAME
//      number of bins (4-10 Hz), using mean and std-dev of the control band
//      (comparing with its maximum would be biased: more bins -> larger max)
//   4. consistency: the spectrum is computed on 3 consecutive thirds of the
//      series; a real cardiac component must show up in all three at the same
//      frequency
//
// Usage: vital_signs [SECONDS] [PORT]
#include <HPS3DUser_IF.h>

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
constexpr int kRing = 4096;

std::mutex g_mutex;    // the SDK thread writes while the analysis reads
std::vector<float> g_ring;
std::vector<int> g_count(kPixels, 0);
std::vector<int> g_write(kPixels, 0);
long g_total = 0;

void onFrame(int, int eventType, std::uint8_t* data, int, void*) {
    if (eventType != 3) return;
    std::lock_guard<std::mutex> lk(g_mutex);
    for (int i = 0; i < kPixels; ++i) {
        const std::uint16_t d = static_cast<std::uint16_t>((data[16 + 2 * i] << 8) | data[17 + 2 * i]);
        const float v = (d > 0 && d < 60000) ? static_cast<float>(d) : NAN;
        g_ring[static_cast<std::size_t>(i) * kRing + g_write[i]] = v;
        g_write[i] = (g_write[i] + 1) % kRing;
        if (g_count[i] < kRing) ++g_count[i];
    }
    ++g_total;
}

double goertzel(const std::vector<double>& x, double f, double fs) {
    const double w = 2.0 * M_PI * f / fs;
    const double coeff = 2.0 * std::cos(w);
    double s1 = 0, s2 = 0;
    for (double v : x) {
        const double s0 = v + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    return std::sqrt(s1 * s1 + s2 * s2 - coeff * s1 * s2) / x.size();
}

struct Spectrum {
    double peak_hz = 0;
    double peak = 0;
    double control_mean = 0;
    double control_std = 0;
    double z = 0;
    int bins = 0;
};

Spectrum analyse(const std::vector<double>& raw, double fs) {
    Spectrum sp;
    if (raw.size() < 64) return sp;

    // robust cleaning: reject outliers, then detrend
    std::vector<double> x = raw;
    std::sort(x.begin(), x.end());
    const double med = x[x.size() / 2];
    std::vector<double> dev;
    dev.reserve(x.size());
    for (double v : x) dev.push_back(std::fabs(v - med));
    std::sort(dev.begin(), dev.end());
    const double mad = dev[dev.size() / 2] * 1.4826;
    double mean = 0;
    std::size_t used = 0;
    for (double v : raw) {
        if (std::fabs(v - med) > 5.0 * mad) continue;
        mean += v;
        ++used;
    }
    if (used < 64) return sp;
    mean /= used;
    for (double& v : x) {
        if (std::fabs(v - med) > 5.0 * mad) v = med;   // replace outliers
        v -= mean;
    }

    // Hann window
    std::vector<double> win(x.size());
    for (std::size_t i = 0; i < x.size(); ++i)
        win[i] = x[i] * (0.5 - 0.5 * std::cos(2.0 * M_PI * static_cast<double>(i) /
                                                static_cast<double>(x.size() - 1)));

    const double df = fs / x.size();
    std::vector<double> cardiac;
    for (double f = 0.7; f <= 2.5; f += df) cardiac.push_back(goertzel(win, f, fs));
    // control band with the SAME number of bins, well above the cardiac band
    std::vector<double> control;
    for (std::size_t k = 0; k < cardiac.size(); ++k) {
        const double f = 4.0 + k * df;
        if (f > 12.0) break;
        control.push_back(goertzel(win, f, fs));
    }
    if (cardiac.empty() || control.size() < 10) return sp;

    for (std::size_t k = 0; k < cardiac.size(); ++k) {
        if (cardiac[k] > sp.peak) {
            sp.peak = cardiac[k];
            sp.peak_hz = 0.7 + k * df;
        }
    }
    double cmean = 0;
    for (double v : control) cmean += v;
    cmean /= control.size();
    double cvar = 0;
    for (double v : control) cvar += (v - cmean) * (v - cmean);
    cvar /= control.size();
    sp.control_mean = cmean;
    sp.control_std = std::sqrt(cvar);
    sp.z = sp.control_std > 1e-12 ? (sp.peak - cmean) / sp.control_std : 0.0;
    sp.bins = static_cast<int>(cardiac.size());
    return sp;
}

} // namespace

int main(int argc, char** argv) {
    const int seconds = argc > 1 ? std::atoi(argv[1]) : 60;
    const char* port = argc > 2 ? argv[2] : "/dev/ttyACM0";
    // optional distance gate (metres) so that only the target region is analysed
    const double dmin = argc > 3 ? std::atof(argv[3]) : 0.0;
    const double dmax = argc > 4 ? std::atof(argv[4]) : 12.0;
    std::printf("faixa de analise: %.2f - %.2f m\n", dmin, dmax);

    int handle = -1;
    if (HPS3D_USBConnectDevice(const_cast<char*>(port), &handle) != HPS3D_RET_OK) {
        std::printf("connect failed\n");
        return 1;
    }
    g_ring.assign(static_cast<std::size_t>(kPixels) * kRing, NAN);
    HPS3D_RegisterEventCallback(&onFrame, nullptr);
    HPS3D_StartCapture(handle);

    std::printf("capturing %d s (fique parado na frente do sensor se quiser testar sinais vitais)\n", seconds);
    for (int t = 0; t < seconds * 2; ++t) {
        usleep(500000);
        if (t % 20 == 0) std::printf("  %d s, frames=%ld\n", t / 2, g_total);
        std::fflush(stdout);
    }
    HPS3D_StopCapture(handle);
    HPS3D_CloseDevice(handle);

    std::lock_guard<std::mutex> lk(g_mutex);
    const double fps = g_total / static_cast<double>(seconds);
    const int n = std::min<int>(kRing, g_count[0]);
    std::printf("\nframes=%ld  fps=%.2f  amostras/pixel=%d (%.1f s)\n", g_total, fps, n, n / fps);

    struct Hit {
        double z = -1e9;
        double hz = 0;
        double amp = 0;
        int pixel = 0, x = 0, y = 0;
        unsigned dist = 0;
        int consistent = 0;
        double noise = 0;
    };
    std::vector<Hit> hits;

    for (int p = 0; p < kPixels; ++p) {
        if (g_count[p] < n) continue;
        std::vector<double> series;
        series.reserve(n);
        const int start = (g_write[p] - n + kRing) % kRing;
        bool ok = true;
        for (int k = 0; k < n; ++k) {
            const float v = g_ring[static_cast<std::size_t>(p) * kRing + (start + k) % kRing];
            if (v != v) { ok = false; break; }
            series.push_back(v);
        }
        if (!ok) continue;

        // temporal noise of the pixel
        std::vector<double> sorted = series;
        std::sort(sorted.begin(), sorted.end());
        const double med = sorted[sorted.size() / 2];
        double mad = 0;
        std::vector<double> dev;
        dev.reserve(sorted.size());
        for (double v : sorted) dev.push_back(std::fabs(v - med));
        std::sort(dev.begin(), dev.end());
        mad = dev[dev.size() / 2] * 1.4826;

        const Spectrum sp = analyse(series, fps);
        if (sp.bins == 0) continue;

        // consistency over 3 consecutive thirds
        int consistent = 0;
        const int seg = static_cast<int>(series.size()) / 3;
        for (int s = 0; s < 3; ++s) {
            std::vector<double> part(series.begin() + s * seg, series.begin() + (s + 1) * seg);
            const Spectrum sp2 = analyse(part, fps);
            if (std::fabs(sp2.peak_hz - sp.peak_hz) < 0.15) ++consistent;
        }

        if (med < dmin * 1000.0 || med > dmax * 1000.0) continue;   // distance gate

        Hit h;
        h.z = sp.z;
        h.hz = sp.peak_hz;
        h.amp = sp.peak * 4.0;   // Hann coherent gain
        h.pixel = p;
        h.x = p % kWidth;
        h.y = p / kWidth;
        h.dist = static_cast<unsigned>(med);
        h.consistent = consistent;
        h.noise = mad;
        hits.push_back(h);
    }

    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return a.z > b.z; });

    std::printf("\npixels analisados: %zu\n", hits.size());
    std::printf("\n=== melhores candidatos (banda cardiaca 0.7-2.5 Hz vs controle 4-12 Hz) ===\n");
    std::printf("%-9s %5s %5s %8s %8s %9s %9s %8s %6s\n", "pixel", "dist", "z", "freq", "bpm", "amp(mm)",
                "ruido(mm)", "consist", "pts");
    for (std::size_t i = 0; i < hits.size() && i < 12; ++i) {
        const Hit& h = hits[i];
        std::printf("(%3d,%3d) %5u %5.1f %7.2fHz %7.0f %8.2f %10.2f %6d/3\n", h.x, h.y, h.dist, h.z, h.hz,
                    h.hz * 60.0, h.amp, h.noise, h.consistent);
    }

    // the strongest candidate that is also consistent in the 3 thirds
    const Hit* best = nullptr;
    for (const auto& h : hits) {
        if (h.consistent == 3 && h.amp > 0.05) { best = &h; break; }
    }

    // sanity check: the respiration band (0.1-0.5 Hz) should be detectable
    // much more easily than the cardiac band; if even that is missing, the
    // scene has no moving target in front of the sensor.
    {
        double best_resp_z = -1e9, best_resp_hz = 0;
        for (int p = 0; p < kPixels; ++p) {
            if (g_count[p] < n) continue;
            std::vector<double> series;
            series.reserve(n);
            const int start = (g_write[p] - n + kRing) % kRing;
            bool ok = true;
            for (int k = 0; k < n; ++k) {
                const float v = g_ring[static_cast<std::size_t>(p) * kRing + (start + k) % kRing];
                if (v != v) { ok = false; break; }
                series.push_back(v);
            }
            if (!ok) continue;
            std::vector<double> x = series;
            std::sort(x.begin(), x.end());
            const double med = x[x.size() / 2];
            std::vector<double> dev;
            for (double v : x) dev.push_back(std::fabs(v - med));
            std::sort(dev.begin(), dev.end());
            const double mad = dev[dev.size() / 2] * 1.4826;
            double mean = 0; std::size_t used = 0;
            for (double v : series) { if (std::fabs(v - med) > 5 * mad) continue; mean += v; ++used; }
            if (used < 64) continue;
            mean /= used;
            for (double& v : x) { if (std::fabs(v - med) > 5 * mad) v = med; v -= mean; }
            std::vector<double> win(x.size());
            for (std::size_t i = 0; i < x.size(); ++i)
                win[i] = x[i] * (0.5 - 0.5 * std::cos(2.0 * M_PI * static_cast<double>(i) /
                                                        static_cast<double>(x.size() - 1)));
            const double df = fps / x.size();
            double peak = 0, pf = 0;
            for (double f = 0.1; f <= 0.5; f += df) {
                const double m = goertzel(win, f, fps);
                if (m > peak) { peak = m; pf = f; }
            }
            std::vector<double> ctrl;
            for (double f = 4.0; f <= 12.0; f += df) ctrl.push_back(goertzel(win, f, fps));
            double cm = 0; for (double v : ctrl) cm += v; cm /= ctrl.size();
            double cv = 0; for (double v : ctrl) cv += (v - cm) * (v - cm); cv /= ctrl.size();
            const double z = std::sqrt(cv) > 1e-12 ? (peak - cm) / std::sqrt(cv) : 0;
            if (z > best_resp_z) { best_resp_z = z; best_resp_hz = pf; }
        }
        std::printf("\n=== teste de sanidade: banda de respiracao (0.1-0.5 Hz) ===\n");
        std::printf("melhor z-score: %.1f em %.2f Hz (%.1f rpm)\n", best_resp_z, best_resp_hz,
                    best_resp_hz * 60.0);
    }

    if (best) {
        // detailed spectrum of the best pixel
        const int p = best->pixel;
        std::vector<double> series;
        series.reserve(n);
        const int start = (g_write[p] - n + kRing) % kRing;
        for (int k = 0; k < n; ++k)
            series.push_back(g_ring[static_cast<std::size_t>(p) * kRing + (start + k) % kRing]);
        std::vector<double> x = series;
        std::sort(x.begin(), x.end());
        const double med = x[x.size() / 2];
        std::vector<double> dev;
        for (double v : x) dev.push_back(std::fabs(v - med));
        std::sort(dev.begin(), dev.end());
        const double mad = dev[dev.size() / 2] * 1.4826;
        double mean = 0; std::size_t used = 0;
        for (double v : series) { if (std::fabs(v - med) > 5 * mad) continue; mean += v; ++used; }
        mean /= used;
        for (double& v : x) { if (std::fabs(v - med) > 5 * mad) v = med; v -= mean; }
        std::vector<double> win(x.size());
        for (std::size_t i = 0; i < x.size(); ++i)
            win[i] = x[i] * (0.5 - 0.5 * std::cos(2.0 * M_PI * static_cast<double>(i) /
                                                    static_cast<double>(x.size() - 1)));
        const double df = fps / x.size();
        std::printf("\n=== espectro do pixel (%d,%d) a %.0f mm ===\n", best->x, best->y, med);
        std::printf("ruido temporal (MAD) : %.2f mm\n", mad);
        std::printf("freq    amplitude(mm)   fracao do ruido\n");
        double resp_peak = 0, resp_f = 0;
        for (double f = 0.1; f <= 3.0; f += df) {
            const double amp = goertzel(win, f, fps) * 4.0;
            if (f <= 0.5) { if (amp > resp_peak) { resp_peak = amp; resp_f = f; } }
            if (f > 0.6 && f <= 2.5)
                std::printf("%5.2f Hz  %8.3f        %6.3f\n", f, amp, mad > 0 ? amp / mad : 0.0);
        }
        std::printf("respiracao: %.2f Hz (%.1f rpm), amplitude %.3f mm (%.3f x ruido)\n", resp_f,
                    resp_f * 60.0, resp_peak, mad > 0 ? resp_peak / mad : 0.0);
    }

    std::printf("\n=== VEREDITO ===\n");
    if (!best) {
        std::printf("Nenhum componente cardiaco consistente encontrado.\n");
        std::printf("Nenhum pixel apresentou pico na banda 0.7-2.5 Hz reproduzido nos 3 segmentos.\n");
    } else {
        std::printf("Candidato: pixel (%d,%d) a %u mm\n", best->x, best->y, best->dist);
        std::printf("  frequencia  : %.2f Hz (%.0f bpm)\n", best->hz, best->hz * 60.0);
        std::printf("  amplitude   : %.2f mm\n", best->amp);
        std::printf("  z-score     : %.1f\n", best->z);
        std::printf("  consistencia: %d/3 segmentos\n", best->consistent);
        std::printf("  ruido pixel : %.2f mm\n", best->noise);
        std::printf("  SNR (amp/ruido): %.3f\n", best->noise > 0 ? best->amp / best->noise : 0);
        if (best->z >= 4.0 && best->consistent == 3 && best->amp > 0.2 && best->amp > 3 * best->noise) {
            std::printf("\n  -> possivel deteccao. CONFIRME com um batimento de referencia (ECG/smartwatch).\n");
        } else {
            std::printf("\n  -> abaixo do limiar de deteccao (z>=4, consistente 3/3, amplitude > 3x ruido).\n");
        }
    }
    return 0;
}
