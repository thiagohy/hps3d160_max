#include "types.hpp"
#include "device_manager.hpp"
#include "capture_service.hpp"
#include "multi_camera_manager.hpp"
#include "recorder.hpp"
#include "health_monitor.hpp"
#include "filter_config.hpp"

#ifdef HPS3D160_MAX_WITH_ROS
#include "ros_publisher.hpp"
#include <ros/ros.h>
#endif

#include <atomic>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <string>
#include <thread>

static volatile std::sig_atomic_t g_stop = 0;
static void onSignal(int) { g_stop = 1; }

static void printUsage(const char* prog) {
    std::printf("Usage: %s [options]\n", prog);
    std::printf("  --mode usb|eth          Connection mode (default: usb)\n");
    std::printf("  --port /dev/ttyACM0     USB port\n");
    std::printf("  --ip 192.168.0.10      Ethernet IP\n");
    std::printf("  --eth-port 12345        Ethernet port\n");
    std::printf("  --cameras N             Number of cameras (default: 1)\n");
    std::printf("  --roi-group N           ROI group id (default: 0)\n");
    std::printf("  --record-dir DIR        Enable recording to DIR\n");
    std::printf("  --format csv|pcd        Recording format (default: csv)\n");
    std::printf("  --single                Single capture mode instead of continuous\n");
    std::printf("  --max-frames N          Stop after N frames (0 = unlimited)\n");
}

static bool parseArgs(int argc, char** argv, hps::AppConfig& cfg, bool& single, std::string& record_dir, std::string& format, int& max_frames) {
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--mode" && i + 1 < argc) {
            std::string m = argv[++i];
            cfg.connection.mode = (m == "eth") ? hps::ConnectionMode::Ethernet : hps::ConnectionMode::USB;
        } else if (a == "--port" && i + 1 < argc) {
            cfg.connection.usb_port = argv[++i];
        } else if (a == "--ip" && i + 1 < argc) {
            cfg.connection.eth_ip = argv[++i];
        } else if (a == "--eth-port" && i + 1 < argc) {
            cfg.connection.eth_port = static_cast<std::uint16_t>(std::stoi(argv[++i]));
        } else if (a == "--cameras" && i + 1 < argc) {
            cfg.device_count = std::stoi(argv[++i]);
        } else if (a == "--roi-group" && i + 1 < argc) {
            cfg.camera.roi_group_id = std::stoi(argv[++i]);
        } else if (a == "--record-dir" && i + 1 < argc) {
            record_dir = argv[++i];
            cfg.recording.enabled = true;
        } else if (a == "--format" && i + 1 < argc) {
            format = argv[++i];
        } else if (a == "--single") {
            single = true;
            cfg.capture.continuous = false;
        } else if (a == "--max-frames" && i + 1 < argc) {
            max_frames = std::stoi(argv[++i]);
        } else if (a == "--help" || a == "-h") {
            printUsage(argv[0]);
            return false;
        }
    }
    if (!record_dir.empty()) {
        cfg.recording.output_dir = record_dir;
        cfg.recording.format = format;
        cfg.recording.max_frames = static_cast<std::size_t>(max_frames);
    }
    return true;
}

int main(int argc, char** argv) {
#ifdef HPS3D160_MAX_WITH_ROS
    ros::init(argc, argv, "hps3d160_max_node");
#endif

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    hps::AppConfig cfg{};
    bool single = false;
    std::string record_dir;
    std::string format = "csv";
    int max_frames = 0;

    if (!parseArgs(argc, argv, cfg, single, record_dir, format, max_frames)) {
        return 0;
    }

    // Validate filters
    std::string reason;
    if (!hps::FilterValidator::validate(cfg.filters, reason)) {
        std::fprintf(stderr, "[main] invalid filter config: %s\n", reason.c_str());
        cfg.filters = hps::FilterValidator::clamp(cfg.filters);
    }

    hps::MultiCameraManager mgr(cfg);
    if (!mgr.connectAll()) {
        std::fprintf(stderr, "[main] failed to connect to one or more devices\n");
    }
    if (mgr.size() == 0) {
        std::fprintf(stderr, "[main] no devices connected, exiting\n");
        return 1;
    }

    // Recorder
    hps::Recorder recorder(cfg.recording);
    if (cfg.recording.enabled) {
        if (!recorder.open()) {
            std::fprintf(stderr, "[main] recorder failed to open\n");
        }
    }

    // Health monitor
    hps::HealthMonitor health(cfg.health);

#ifdef HPS3D160_MAX_WITH_ROS
    ros::NodeHandle nh;
    hps::RosPointCloudPublisher ros_pub;
    ros_pub.init(nh, "/hps3d160/point_cloud", "hps3d160");
#endif

    std::atomic<std::uint64_t> total_frames{0};

    auto onFrame = [&](const hps::PointCloudFrame& frame, const hps::FrameStats& stats) {
        total_frames = stats.frames_total;
        if (cfg.recording.enabled && recorder.isOpen()) {
            recorder.record(frame);
        }
#ifdef HPS3D160_MAX_WITH_ROS
        ros_pub.publish(frame);
#endif
        health.onFrame(frame, stats);
        if (cfg.recording.max_frames > 0 && stats.frames_total >= cfg.recording.max_frames) {
            g_stop = 1;
        }
    };

    // Use first device for capture (multi-camera capture can be extended)
    hps::CaptureService capture(mgr.device(0), onFrame);
    health.start([&capture]() { return capture.stats(); });

    if (cfg.capture.continuous) {
        if (!capture.start()) {
            std::fprintf(stderr, "[main] failed to start continuous capture\n");
            mgr.disconnectAll();
            return 1;
        }
        std::printf("[main] continuous capture started. Press Ctrl+C to stop.\n");
        while (!g_stop) {
#ifdef HPS3D160_MAX_WITH_ROS
            ros::spinOnce();
#endif
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        capture.stop();
    } else {
        // Single capture mode
        int n = 0;
        while (!g_stop && (max_frames == 0 || n < max_frames)) {
            hps::PointCloudFrame frame{};
            if (capture.singleCapture(frame)) {
                onFrame(frame, capture.stats());
                n++;
                std::printf("[main] captured frame %d (points=%u)\n", n, frame.point_count);
            } else {
                std::fprintf(stderr, "[main] single capture failed\n");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    health.stop();
    recorder.close();
    mgr.disconnectAll();

    std::printf("[main] done. total frames: %lu\n", (unsigned long)total_frames.load());
    return 0;
}