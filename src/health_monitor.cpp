#include "health_monitor.hpp"

#include <cstdio>

namespace hps {

HealthMonitor::HealthMonitor(const HealthConfig& cfg) : cfg_(cfg) {}

HealthMonitor::~HealthMonitor() { stop(); }

void HealthMonitor::start(std::function<FrameStats()> stats_provider) {
    if (!cfg_.enabled || running_) return;
    stats_provider_ = std::move(stats_provider);
    running_ = true;
    last_log_ = std::chrono::steady_clock::now();
    thread_ = std::thread(&HealthMonitor::loop, this);
}

void HealthMonitor::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
}

void HealthMonitor::onFrame(const PointCloudFrame& /*frame*/, const FrameStats& stats) {
    if (stats.fps_instant < cfg_.min_fps_warning && stats.frames_total > 10) {
        std::fprintf(stderr, "[Health] low FPS: %.1f (frames=%lu dropped=%lu)\n",
                     stats.fps_instant, (unsigned long)stats.frames_total, (unsigned long)stats.frames_dropped);
    }
}

void HealthMonitor::onDisconnect() {
    std::fprintf(stderr, "[Health] device disconnected\n");
}

void HealthMonitor::loop() {
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.log_interval_ms));
        if (!running_) break;
        if (stats_provider_) {
            FrameStats s = stats_provider_();
            auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log_).count() >= cfg_.log_interval_ms) {
                std::printf("[Health] connected=%d capturing=%d fps=%.1f avg=%.1f total=%lu dropped=%lu\n",
                            s.connected, s.capturing, s.fps_instant, s.fps_average,
                            (unsigned long)s.frames_total, (unsigned long)s.frames_dropped);
                last_log_ = now;
            }
        }
    }
}

} // namespace hps