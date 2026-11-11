#pragma once

#include "types.hpp"

#include <string>
#include <vector>

namespace hps {

// Detection of a person-sized object in the depth map.
//
// The HPS-3D160 is a ToF camera: it cannot measure velocity or phase, so
// physiological signals (heart beat) are out of reach - see
// docs/human_sensing.md for the measurements that prove it. Detecting a
// person, however, is well within the sensor's capabilities and is even
// supported natively by the SDK (obstacle mode).
//
// This detector works on the metric point cloud:
//   1. drop invalid points and everything beyond `max_range`
//   2. remove the ground plane with a RANSAC fit
//   3. cluster the remaining points (grid based DBSCAN)
//   4. keep the clusters whose size matches a human body
struct PresenceConfig {
    double max_range_m = 6.0;
    double min_height_m = 0.55;
    double max_height_m = 2.30;
    double min_width_m = 0.22;
    double min_points = 25;
    double cluster_eps_m = 0.12;
    double ground_tolerance_m = 0.05;
};

struct Blob {
    int points = 0;
    float x_min = 0, x_max = 0;
    float y_min = 0, y_max = 0;
    float z_min = 0, z_max = 0;
    float z_mean = 0;
    float height = 0;
    float width = 0;
    float depth = 0;
    bool person_like = false;
};

struct PresenceResult {
    bool present = false;
    int person_count = 0;
    int blob_count = 0;
    float distance_m = 0.0f;      // distance to the closest person-like blob
    float height_m = 0.0f;       // height of the closest person-like blob
    std::vector<Blob> blobs;
    std::string summary;
};

class PresenceDetector {
public:
    explicit PresenceDetector(const PresenceConfig& cfg = {}) : cfg_(cfg) {}

    PresenceResult analyze(const PointCloudFrame& frame) const;

private:
    PresenceConfig cfg_;
};

} // namespace hps
