#pragma once

#include "types.hpp"

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

namespace hps {

class HealthMonitor {
public:
    explicit HealthMonitor(const HealthConfig& cfg);
    ~HealthMonitor();

    void start(std::function<FrameStats()> stats_provider);
    void stop();

    void onFrame(const PointCloudFrame& frame, const FrameStats& stats);
    void onDisconnect();

private:
    void loop();

    HealthConfig cfg_;
    std::function<FrameStats()> stats_provider_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::chrono::steady_clock::time_point last_log_;
};

} // namespace hps