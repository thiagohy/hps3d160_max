#include "recorder.hpp"

#include <cstdio>
#include <filesystem>
#include <sstream>

namespace fs = std::filesystem;

namespace hps {

Recorder::Recorder(const RecordingConfig& cfg) : cfg_(cfg) {}

Recorder::~Recorder() { close(); }

bool Recorder::open() {
    if (cfg_.output_dir.empty()) return false;
    std::error_code ec;
    fs::create_directories(cfg_.output_dir, ec);
    if (ec) {
        std::fprintf(stderr, "[Recorder] cannot create dir %s: %s\n", cfg_.output_dir.c_str(), ec.message().c_str());
        return false;
    }
    if (cfg_.format == "csv") {
        std::string path = cfg_.output_dir + "/frames.csv";
        csv_.open(path, std::ios::out | std::ios::trunc);
        if (!csv_) {
            std::fprintf(stderr, "[Recorder] cannot open %s\n", path.c_str());
            return false;
        }
        csv_ << "frame_id,timestamp_ms,width,height,point_count,distance_avg_mm,distance_min_mm,saturation_count,frame_counter,valid_points\n";
    }
    opened_ = true;
    std::printf("[Recorder] recording to %s (format=%s)\n", cfg_.output_dir.c_str(), cfg_.format.c_str());
    return true;
}

void Recorder::close() {
    if (csv_.is_open()) csv_.close();
    opened_ = false;
}

bool Recorder::isOpen() const { return opened_; }

bool Recorder::record(const PointCloudFrame& frame) {
    if (!opened_) return false;
    if (cfg_.max_frames > 0 && recorded_ >= cfg_.max_frames) return false;

    if (cfg_.format == "csv") {
        if (!writeCsv(frame)) return false;
    }
#ifdef HPS3D160_MAX_WITH_PCL
    else if (cfg_.format == "pcd") {
        if (!writePcd(frame)) return false;
    }
#endif
    else {
        std::fprintf(stderr, "[Recorder] unknown format %s\n", cfg_.format.c_str());
        return false;
    }
    recorded_++;
    return true;
}

bool Recorder::writeCsv(const PointCloudFrame& frame) {
    std::size_t valid = 0;
    for (auto& p : frame.points) if (p.valid) valid++;
    auto ts = std::chrono::duration_cast<std::chrono::milliseconds>(frame.timestamp.time_since_epoch()).count();
    csv_ << frame.frame_id << "," << ts << "," << frame.width << "," << frame.height << ","
         << frame.point_count << "," << frame.distance_average_mm << "," << frame.distance_min_mm << ","
         << frame.saturation_count << "," << frame.frame_counter << "," << valid << "\n";
    const std::string base = cfg_.output_dir + "/frame_" + std::to_string(frame.frame_id);

    // Per-point XYZ + distance (raw point cloud)
    {
        std::ofstream f(base + ".xyz");
        if (f) {
            for (auto& p : frame.points) {
                if (p.valid) f << p.x << " " << p.y << " " << p.z << " " << p.distance_mm << "\n";
            }
        }
    }

    // Raw distance matrix (width x height), millimeters
    {
        std::ofstream f(base + ".distances.csv");
        if (f) {
            for (std::uint32_t y = 0; y < frame.height; ++y) {
                for (std::uint32_t x = 0; x < frame.width; ++x) {
                    std::size_t idx = static_cast<std::size_t>(y) * frame.width + x;
                    if (idx < frame.points.size()) f << frame.points[idx].distance_mm;
                    if (x + 1 < frame.width) f << ",";
                }
                f << "\n";
            }
        }
    }

    // Metadata JSON
    {
        std::ofstream f(base + ".meta.json");
        if (f) {
            f << "{\n";
            f << "  \"frame_id\": " << frame.frame_id << ",\n";
            f << "  \"timestamp_ms\": " << ts << ",\n";
            f << "  \"width\": " << frame.width << ",\n";
            f << "  \"height\": " << frame.height << ",\n";
            f << "  \"point_count\": " << frame.point_count << ",\n";
            f << "  \"valid_points\": " << valid << ",\n";
            f << "  \"distance_average_mm\": " << frame.distance_average_mm << ",\n";
            f << "  \"distance_min_mm\": " << frame.distance_min_mm << ",\n";
            f << "  \"saturation_count\": " << frame.saturation_count << ",\n";
            f << "  \"frame_counter\": " << frame.frame_counter << "\n";
            f << "}\n";
        }
    }

    csv_.flush();
    return true;
}

#ifdef HPS3D160_MAX_WITH_PCL
bool Recorder::writePcd(const PointCloudFrame& frame) {
    // PCL header (ASCII) minimal
    std::string path = cfg_.output_dir + "/frame_" + std::to_string(frame.frame_id) + ".pcd";
    std::ofstream f(path);
    if (!f) return false;
    std::size_t valid = 0;
    for (auto& p : frame.points) if (p.valid) valid++;
    f << "# .PCD v0.7 - Point Cloud Data file format\n";
    f << "VERSION 0.7\n";
    f << "FIELDS x y z distance\n";
    f << "SIZE 4 4 4 2\n";
    f << "TYPE F F F U\n";
    f << "COUNT 1 1 1 1\n";
    f << "WIDTH " << valid << "\n";
    f << "HEIGHT 1\n";
    f << "VIEWPOINT 0 0 0 1 0 0 0\n";
    f << "POINTS " << valid << "\n";
    f << "DATA ascii\n";
    for (auto& p : frame.points) {
        if (p.valid) f << p.x << " " << p.y << " " << p.z << " " << p.distance_mm << "\n";
    }
    return true;
}
#endif

} // namespace hps