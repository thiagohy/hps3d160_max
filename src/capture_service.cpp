#include "capture_service.hpp"

#include <chrono>

namespace hps {

CaptureService::CaptureService(DeviceManager& device, FrameHandler handler)
    : device_(device), handler_(std::move(handler)) {
    device_.setFrameCallback([this](int h, HPS3D_EventType_t t, HPS3D_MeasureData_t& d) {
        onDeviceFrame(h, t, d);
    });
}

CaptureService::~CaptureService() { stop(); }

bool CaptureService::start() {
    if (running_) return true;
    if (!device_.startCapture()) return false;
    running_ = true;
    stats_.connected = device_.isConnected();
    stats_.capturing = true;
    fps_window_start_ = std::chrono::steady_clock::now();
    fps_window_frames_ = 0;
    return true;
}

void CaptureService::stop() {
    if (!running_) return;
    device_.stopCapture();
    running_ = false;
    stats_.capturing = false;
}

bool CaptureService::isRunning() const { return running_.load(); }

bool CaptureService::singleCapture(PointCloudFrame& out) {
    HPS3D_EventType_t type = HPS3D_NULL_EVEN;
    HPS3D_MeasureData_t data{};
    HPS3D_MeasureDataInit(&data);
    bool ok = device_.singleCapture(type, data);
    if (ok && (type == HPS3D_FULL_DEPTH_EVEN || type == HPS3D_SIMPLE_DEPTH_EVEN)) {
        out = PointCloudBuilder::build(data.full_depth_data, ++frame_seq_);
    }
    HPS3D_MeasureDataFree(&data);
    return ok;
}

FrameStats CaptureService::stats() const {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    return stats_;
}

void CaptureService::onDeviceFrame(int /*handle*/, HPS3D_EventType_t type, HPS3D_MeasureData_t& data) {
    if (type != HPS3D_FULL_DEPTH_EVEN && type != HPS3D_SIMPLE_DEPTH_EVEN &&
        type != HPS3D_FULL_ROI_EVEN && type != HPS3D_SIMPLE_ROI_EVEN) {
        return;
    }

    PointCloudFrame frame{};
    if (type == HPS3D_FULL_DEPTH_EVEN || type == HPS3D_SIMPLE_DEPTH_EVEN) {
        frame = PointCloudBuilder::build(data.full_depth_data, ++frame_seq_);
    } else {
        // ROI-only frames: build a minimal frame with stats
        frame.frame_id = ++frame_seq_;
        if (type == HPS3D_FULL_ROI_EVEN && data.full_roi_data) {
            frame.distance_average_mm = data.full_roi_data[0].distance_average;
            frame.distance_min_mm = data.full_roi_data[0].distance_min;
            frame.saturation_count = data.full_roi_data[0].saturation_count;
            frame.frame_counter = data.full_roi_data[0].frame_cnt;
        } else if (data.simple_roi_data) {
            frame.distance_average_mm = data.simple_roi_data[0].distance_average;
            frame.distance_min_mm = data.simple_roi_data[0].distance_min;
            frame.saturation_count = data.simple_roi_data[0].saturation_count;
            frame.frame_counter = data.simple_roi_data[0].frame_cnt;
        }
    }

    std::uint32_t counter = frame.frame_counter;
    updateStats(counter);

    FrameStats s = stats();
    if (handler_) handler_(frame, s);
}

void CaptureService::updateStats(std::uint32_t frame_counter) {
    std::lock_guard<std::mutex> lk(stats_mutex_);
    auto now = std::chrono::steady_clock::now();
    stats_.frames_total++;
    stats_.connected = device_.isConnected();
    stats_.capturing = running_.load();

    if (have_last_) {
        if (frame_counter != last_counter_ + 1 && frame_counter != 0) {
            // dropped frames heuristic
            if (frame_counter > last_counter_) {
                stats_.frames_dropped += (frame_counter - last_counter_ - 1);
            }
        }
    }
    last_counter_ = frame_counter;
    have_last_ = true;

    // Instantaneous FPS from time delta
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time_).count();
    if (dt > 0) stats_.fps_instant = 1000.0 / static_cast<double>(dt);
    last_time_ = now;

    // Windowed average
    if (fps_window_frames_ == 0) fps_window_start_ = now;
    fps_window_frames_++;
    auto wdt = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_window_start_).count();
    if (wdt >= 1000) {
        stats_.fps_average = (fps_window_frames_ * 1000.0) / static_cast<double>(wdt);
        fps_window_frames_ = 0;
        fps_window_start_ = now;
    }
    stats_.last_frame_counter = frame_counter;
}

} // namespace hps