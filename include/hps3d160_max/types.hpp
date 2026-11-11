#pragma once

#include <HPS3DUser_IF.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <vector>

namespace hps {

enum class ConnectionMode {
    USB,
    Ethernet
};

struct ConnectionConfig {
    ConnectionMode mode = ConnectionMode::USB;
    std::string usb_port = "/dev/ttyACM0";
    std::string eth_ip = "192.168.0.10";
    std::uint16_t eth_port = 12345;
    bool reconnect_enabled = true;
    int keepalive_ms = 3000;
};

struct FilterConfig {
    bool distance_filter_enabled = false;
    float distance_filter_k = 0.3f;

    int smooth_filter_type = HPS3D_SMOOTH_FILTER_DISABLE;
    int smooth_filter_args = 2;

    std::int16_t distance_offset = 0;
    bool optical_path_calibration = false;
    bool edge_filter_enabled = false;
};

struct CameraConfig {
    int user_id = 0;
    int roi_group_id = 0;
    int multi_camera_code = 0;
    bool save_settings_to_device = false;
};

struct CaptureConfig {
    bool continuous = true;
    int single_capture_timeout_ms = 1000;
    int target_fps = 30;
};

struct RecordingConfig {
    bool enabled = false;
    std::string output_dir = "./hps_recordings";
    std::string format = "csv"; // csv | pcd (if PCL)
    bool record_metadata = true;
    std::size_t max_frames = 0; // 0 = unlimited
};

struct HealthConfig {
    bool enabled = true;
    int log_interval_ms = 1000;
    int min_fps_warning = 10;
    int reconnect_retry_ms = 10000;
};

struct AppConfig {
    ConnectionConfig connection;
    std::vector<ConnectionConfig> connections; // optional per-device endpoints
    FilterConfig filters;
    CameraConfig camera;
    CaptureConfig capture;
    RecordingConfig recording;
    HealthConfig health;
    int device_count = 1;
};

// A single 3D point with distance (mm) and intensity proxy.
struct Point3D {
    float x = 0.0f; // meters
    float y = 0.0f;
    float z = 0.0f;
    std::uint16_t distance_mm = 0;
    bool valid = true;
};

struct PointCloudFrame {
    std::uint64_t frame_id = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t point_count = 0;
    std::uint16_t distance_average_mm = 0;
    std::uint16_t distance_min_mm = 0;
    std::uint16_t saturation_count = 0;
    std::uint32_t frame_counter = 0;
    std::vector<Point3D> points;
    std::chrono::steady_clock::time_point timestamp = std::chrono::steady_clock::now();
};

struct FrameStats {
    std::uint64_t frames_total = 0;
    std::uint64_t frames_dropped = 0;
    double fps_instant = 0.0;
    double fps_average = 0.0;
    std::uint32_t last_frame_counter = 0;
    bool connected = false;
    bool capturing = false;
};

} // namespace hps