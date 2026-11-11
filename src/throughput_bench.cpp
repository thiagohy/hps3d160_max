#include "throughput_bench.hpp"

#include <HPS3DUser_IF.h>

#include <dlfcn.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <sstream>
#include <thread>
#include <vector>

namespace hps {
namespace {

std::atomic<int> g_frames{0};
std::atomic<std::uint64_t> g_bytes{0};
std::atomic<int> g_event{0};

void benchCallback(int, int eventType, std::uint8_t*, int dataLen, void*) {
    if (eventType >= 1 && eventType <= 4) {
        g_frames++;
        g_bytes += static_cast<std::uint64_t>(dataLen);
        g_event = eventType;
    }
}

ThroughputResult measure(int handle, int seconds, const std::string& label) {
    g_frames = 0;
    g_bytes = 0;
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<std::chrono::steady_clock::time_point> stamps;
    int last = 0;
    while (std::chrono::steady_clock::now() - t0 < std::chrono::seconds(seconds)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        const int f = g_frames.load();
        if (f != last) {
            stamps.push_back(std::chrono::steady_clock::now());
            last = f;
        }
    }
    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    ThroughputResult r;
    r.label = label;
    r.fps = g_frames.load() / elapsed;
    r.payload_mb_s = g_bytes.load() / elapsed / 1e6;
    r.event_type = g_event.load();
    if (stamps.size() > 2) {
        std::vector<double> gaps;
        gaps.reserve(stamps.size());
        for (std::size_t i = 1; i < stamps.size(); ++i)
            gaps.push_back(std::chrono::duration<double, std::milli>(stamps[i] - stamps[i - 1]).count());
        std::sort(gaps.begin(), gaps.end());
        r.interval_ms = gaps[gaps.size() / 2];
    }
    return r;
}

} // namespace

std::vector<ThroughputResult> ThroughputBenchmark::run(SdkBridge& bridge, int seconds, bool with_packet_types) {
    std::vector<ThroughputResult> results;

    int handle = bridge.public_handle();
    HPS3D_RegisterEventCallback(&benchCallback, nullptr);

    const internal::HDRModeTypeDef modes[] = {internal::HDR_DISABLE, internal::AUTO_HDR,
                                               internal::SUPER_HDR, internal::SIMPLE_HDR};
    const char* names[] = {"disable", "auto_hdr", "super_hdr", "simple_hdr"};

    for (int i = 0; i < 4; ++i) {
        bridge.set_hdr_mode(modes[i]);
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
        HPS3D_StartCapture(handle);
        results.push_back(measure(handle, seconds, names[i]));
        HPS3D_StopCapture(handle);
    }

    if (with_packet_types && bridge.internal_handle()) {
        using Fn = int (*)(void*, int);
        auto set_packet = reinterpret_cast<Fn>(dlsym(RTLD_DEFAULT, "HE201_ApiSetPacketType"));
        if (set_packet) {
            for (int t = 0; t <= 1; ++t) {
                set_packet(bridge.internal_handle(), t);
                std::this_thread::sleep_for(std::chrono::milliseconds(300));
                HPS3D_StartCapture(handle);
                results.push_back(measure(handle, seconds, t == 0 ? "packet_depth" : "packet_roi"));
                HPS3D_StopCapture(handle);
            }
            set_packet(bridge.internal_handle(), 0);  // restore
        }
    }
    return results;
}

std::string ThroughputBenchmark::to_json(const std::vector<ThroughputResult>& results) {
    std::ostringstream o;
    o << "{\n  \"measurements\": [\n";
    for (std::size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        o << "    {\"label\": \"" << r.label << "\", \"fps\": " << r.fps
          << ", \"interval_ms\": " << r.interval_ms << ", \"payload_mb_s\": " << r.payload_mb_s
          << ", \"event_type\": " << r.event_type << "}";
        if (i + 1 < results.size()) o << ",";
        o << "\n";
    }
    o << "  ]\n}\n";
    return o.str();
}

} // namespace hps
