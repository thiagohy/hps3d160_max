// Minimal mock of the ROS API surface used by ros_publisher.cpp, so the file
// can be syntax/type checked on hosts without a ROS installation.
#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace ros {
struct Time {
    static Time now() { return Time{}; }
};
} // namespace ros

namespace sensor_msgs {

struct PointField {
    std::string name;
    uint32_t offset = 0;
    uint8_t datatype = 0;
    uint32_t count = 0;
};

struct PointCloud2 {
    struct Header {
        ros::Time stamp;
        std::string frame_id;
    } header;
    uint32_t height = 0;
    uint32_t width = 0;
    std::vector<PointField> fields;
    bool is_bigendian = false;
    uint32_t point_step = 0;
    uint32_t row_step = 0;
    std::vector<uint8_t> data;
    bool is_dense = false;
};

struct PointCloud2Modifier {
    explicit PointCloud2Modifier(PointCloud2& c) : cloud_(c) {}
    template <typename... Rest>
    void setPointCloud2FieldsByString(int, const std::string& names, Rest... rest) {
        std::string all = names + " " + join(rest...);
        cloud_.fields.clear();
        uint32_t offset = 0;
        std::string cur;
        for (char ch : all + " ") {
            if (ch == ' ' || ch == ',') {
                if (!cur.empty()) {
                    PointField f;
                    f.name = cur;
                    f.offset = offset;
                    f.datatype = 7;  // FLOAT32
                    f.count = 1;
                    cloud_.fields.push_back(f);
                    offset += 4;
                    cur.clear();
                }
            } else {
                cur += ch;
            }
        }
        cloud_.point_step = offset;
        cloud_.row_step = offset * cloud_.width;
        cloud_.data.assign(static_cast<size_t>(cloud_.point_step) * cloud_.width * cloud_.height, 0);
    }

private:
    static std::string join() { return {}; }
    template <typename T, typename... Rest>
    static std::string join(const T& first, Rest... rest) {
        return std::string(first) + " " + join(rest...);
    }
    PointCloud2& cloud_;
};

template <typename T>
class PointCloud2Iterator {
public:
    PointCloud2Iterator(PointCloud2& c, const std::string& field) : cloud_(c) {
        for (const auto& f : c.fields) {
            if (f.name == field) {
                offset_ = f.offset;
                found_ = true;
                break;
            }
        }
    }
    T& operator*() {
        auto* base = reinterpret_cast<float*>(cloud_.data.data() + index_ * cloud_.point_step + offset_);
        return *base;
    }
    PointCloud2Iterator& operator++() {
        ++index_;
        return *this;
    }
    bool found() const { return found_; }

private:
    PointCloud2& cloud_;
    uint32_t offset_ = 0;
    uint32_t index_ = 0;
    bool found_ = false;
};

} // namespace sensor_msgs

namespace ros {

namespace mock {
void store_pointcloud(const void* p);
const sensor_msgs::PointCloud2* last_pointcloud();
} // namespace mock

class Publisher {
public:
    template <typename T>
    void publish(const T& message) {
        mock::store_pointcloud(&message);
    }
};

struct NodeHandle {
    template <typename M>
    Publisher advertise(const std::string&, int) { return {}; }
};

} // namespace ros
