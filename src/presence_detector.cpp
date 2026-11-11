#include "presence_detector.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <cstdlib>
#include <random>
#include <unordered_map>

namespace hps {
namespace {

struct P3 {
    float x, y, z;
};

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    std::size_t n = v.size();
    return (n % 2) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

// Height of the floor, estimated from the height histogram of the scene.
// The floor is the most extended surface at the bottom of the frame, so the
// peak of the histogram of the lower part of the scene gives its height.
// (A least squares / RANSAC plane fit is not used here: a person standing on
// the floor correlates with depth and biases the fit, while the histogram is
// dominated by the floor itself.)
double estimate_floor_height(const std::vector<P3>& pts, double tol) {
    if (pts.size() < 50) return 0.0;
    std::vector<double> ys;
    ys.reserve(pts.size());
    for (const auto& p : pts) ys.push_back(p.y);
    std::sort(ys.begin(), ys.end());

    // consider the lower 40 % of the heights
    const std::size_t n_low = ys.size() * 40 / 100;
    if (n_low < 20) return 0.0;
    const double lo = ys.front(), hi = ys[n_low - 1];
    const double bin = 0.05;
    const int nbins = std::max(1, static_cast<int>((hi - lo) / bin) + 1);
    std::vector<int> hist(nbins, 0);
    for (std::size_t i = 0; i < n_low; ++i) {
        const int b = std::min(nbins - 1, static_cast<int>((ys[i] - lo) / bin));
        hist[b]++;
    }
    int peak = 0;
    for (int b = 1; b < nbins; ++b)
        if (hist[b] > hist[peak]) peak = b;
    return lo + (peak + 0.5) * bin;
}

} // namespace

PresenceResult PresenceDetector::analyze(const PointCloudFrame& frame) const {
    PresenceResult res;

    // 1. usable points within range
    std::vector<P3> pts;
    pts.reserve(frame.points.size());
    for (const auto& p : frame.points) {
        if (!p.valid) continue;
        if (p.z > cfg_.max_range_m) continue;
        pts.push_back({p.x, p.y, p.z});
    }
    if (pts.size() < 20) {
        res.summary = "not enough valid points";
        return res;
    }

    // 2. ground removal (the floor is the dominant lower surface)
    std::vector<P3> above;
    above.reserve(pts.size());
    const double y_floor = estimate_floor_height(pts, cfg_.ground_tolerance_m);
    const double tol = cfg_.ground_tolerance_m;
    for (const auto& p : pts) {
        if (std::fabs(p.y - y_floor) > tol) above.push_back(p);
    }
    if (above.size() < 20) above = pts;   // no floor found: use everything

    // 3. clustering (grid based DBSCAN)
    const double eps = cfg_.cluster_eps_m;
    std::unordered_map<long long, std::vector<int>> grid;
    auto key = [&](const P3& p) {
        const long long gx = static_cast<long long>(std::floor(p.x / eps));
        const long long gy = static_cast<long long>(std::floor(p.y / eps));
        const long long gz = static_cast<long long>(std::floor(p.z / eps));
        return (gx * 73856093LL) ^ (gy * 19349663LL) ^ (gz * 83492791LL);
    };
    for (std::size_t i = 0; i < above.size(); ++i) grid[key(above[i])].push_back(static_cast<int>(i));

    std::vector<char> seen(above.size(), 0);
    for (std::size_t i = 0; i < above.size(); ++i) {
        if (seen[i]) continue;
        std::vector<int> stack{static_cast<int>(i)};
        seen[i] = 1;
        std::vector<int> members;
        while (!stack.empty()) {
            const int j = stack.back();
            stack.pop_back();
            members.push_back(j);
            const P3& pj = above[j];
            const long long gx = static_cast<long long>(std::floor(pj.x / eps));
            const long long gy = static_cast<long long>(std::floor(pj.y / eps));
            const long long gz = static_cast<long long>(std::floor(pj.z / eps));
            for (long long dx = -1; dx <= 1; ++dx)
                for (long long dy = -1; dy <= 1; ++dy)
                    for (long long dz = -1; dz <= 1; ++dz) {
                        const long long kx = gx + dx, ky = gy + dy, kz = gz + dz;
                        const long long h = (kx * 73856093LL) ^ (ky * 19349663LL) ^ (kz * 83492791LL);
                        auto it = grid.find(h);
                        if (it == grid.end()) continue;
                        for (int m : it->second) {
                            if (seen[m]) continue;
                            const P3& pm = above[m];
                            const float d2 = (pj.x - pm.x) * (pj.x - pm.x) + (pj.y - pm.y) * (pj.y - pm.y) +
                                             (pj.z - pm.z) * (pj.z - pm.z);
                            if (d2 <= eps * eps) {
                                seen[m] = 1;
                                stack.push_back(m);
                            }
                        }
                    }
        }
        if (static_cast<int>(members.size()) < cfg_.min_points) continue;

        Blob blob;
        blob.points = static_cast<int>(members.size());
        blob.x_min = blob.x_max = above[members[0]].x;
        blob.y_min = blob.y_max = above[members[0]].y;
        blob.z_min = blob.z_max = above[members[0]].z;
        double zsum = 0;
        for (int m : members) {
            const P3& p = above[m];
            blob.x_min = std::min(blob.x_min, p.x);
            blob.x_max = std::max(blob.x_max, p.x);
            blob.y_min = std::min(blob.y_min, p.y);
            blob.y_max = std::max(blob.y_max, p.y);
            blob.z_min = std::min(blob.z_min, p.z);
            blob.z_max = std::max(blob.z_max, p.z);
            zsum += p.z;
        }
        blob.z_mean = static_cast<float>(zsum / members.size());
        blob.height = blob.y_max - blob.y_min;
        blob.width = blob.x_max - blob.x_min;
        blob.depth = blob.z_max - blob.z_min;
        blob.person_like = blob.height >= cfg_.min_height_m && blob.height <= cfg_.max_height_m &&
                           blob.width >= cfg_.min_width_m && blob.z_mean <= cfg_.max_range_m;
        res.blobs.push_back(blob);
    }

    std::sort(res.blobs.begin(), res.blobs.end(),
              [](const Blob& a, const Blob& b) { return a.points > b.points; });
    res.blob_count = static_cast<int>(res.blobs.size());
    for (const auto& b : res.blobs) {
        if (!b.person_like) continue;
        res.person_count++;
        if (!res.present) {
            res.present = true;
            res.distance_m = b.z_mean;
            res.height_m = b.height;
        }
    }

    std::ostringstream o;
    o << res.blobs.size() << " blob(s), " << res.person_count << " person-like";
    if (res.present)
        o << "; closest at " << res.distance_m << " m, height " << res.height_m << " m";
    res.summary = o.str();
    return res;
}

} // namespace hps
