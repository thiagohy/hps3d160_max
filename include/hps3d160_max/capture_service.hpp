#pragma once

#include "device_manager.hpp"
#include "pointcloud_builder.hpp"
#include "types.hpp"

#include <atomic>
#include <functional>
#include <mutex>
#include <thread>

namespace hps {

class CaptureService {
public:
    using FrameHandler = std::function<void(const PointCloudFrame& frame, const FrameStats& stats)>;

    CaptureService(DeviceManager& device, FrameHandler handler);
    ~CaptureService();

    bool start();
    void stop();
    bool isRunning() const;

    bool singleCapture(PointCloudFrame& out);

    FrameStats stats() const;

private:
    void onDeviceFrame(int handle, HPS3D_EventType_t type, HPS3D_MeasureData_t& data);
    void updateStats(std::uint32_t frame_counter);

    DeviceManager& device_;
    FrameHandler handler_;
    std::atomic<bool> running_{false};
    std::atomic<std::uint64_t> frame_seq_{0};

    mutable std::mutex stats_mutex_;
    FrameStats stats_{};
    std::uint32_t last_counter_ = 0;
    bool have_last_ = false;
    std::chrono::steady_clock::time_point last_time_;
    std::chrono::steady_clock::time_point fps_window_start_;
    std::uint64_t fps_window_frames_ = 0;
};

} // namespace hps