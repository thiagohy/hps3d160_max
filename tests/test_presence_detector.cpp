// Unit test for the presence detector: synthetic scenes with a known ground
// plane and a known "person" shaped object.
//
// Camera model: the sensor sits at height h = 1.5 m and looks along +Z.
//   - the floor is the horizontal plane y = -1.5
//   - a person is a vertical cylinder standing on the floor
#include "presence_detector.hpp"

#include <cmath>
#include <cstdio>

using namespace hps;

namespace {

constexpr double kCamHeight = 1.5;
constexpr double kHalfHFOV = 38.0 * M_PI / 180.0;
constexpr double kHalfVFOV = 16.0 * M_PI / 180.0;
constexpr double kFocal = 104.0;

void addPixel(PointCloudFrame& f, int x, int y, float px, float py, float pz) {
    Point3D p;
    p.x = px;
    p.y = py;
    p.z = pz;
    p.distance_mm = static_cast<std::uint16_t>(pz * 1000);
    p.valid = true;
    p.status = PixelStatus::Ok;
    f.points.push_back(p);
}

// Floor: every ray pointing below the horizon hits y = -1.5
void addGround(PointCloudFrame& f) {
    for (int y = 0; y < 60; ++y) {
        for (int x = 0; x < 160; ++x) {
            const double th = std::atan((x - 80) / kFocal);
            const double tv = std::atan((30 - y) / kFocal);
            if (tv >= -0.02) continue;                 // above the horizon
            const double z = kCamHeight / std::tan(-tv);
            if (z > 12.0 || z < 0.3) continue;
            const float px = static_cast<float>(z * std::tan(th));
            addPixel(f, x, y, px, static_cast<float>(-kCamHeight), static_cast<float>(z));
        }
    }
}

// Person: vertical cylinder of radius r and height h standing on the floor
void addPerson(PointCloudFrame& f, float cx, float zdist, float height, float radius) {
    // angular span of the body at that depth, around its lateral position
    const double th_l = std::atan((cx - radius) / zdist);
    const double th_r = std::atan((cx + radius) / zdist);
    const int x0 = 80 + static_cast<int>(th_l * kFocal);
    const int x1 = 80 + static_cast<int>(th_r * kFocal);
    for (int x = std::min(x0, x1); x <= std::max(x0, x1); ++x) {
        const double th = std::atan((x - 80) / kFocal);
        const float px = static_cast<float>(zdist * std::tan(th));
        const float dx = px - cx;
        if (std::fabs(dx) > radius) continue;
        // vertical extent: from the floor up to `height`
        // the body spans from the floor up to `height`, all at the same depth
        const int y_top = 30 - static_cast<int>(height * kFocal / zdist);
        const int y_bottom = 30 + static_cast<int>(kCamHeight * kFocal / zdist);
        for (int y = y_top; y < y_bottom; ++y) {
            const double tv = std::atan((30 - y) / kFocal);
            const double yy = -kCamHeight + std::tan(tv) * zdist;
            if (yy < -kCamHeight) continue;          // the body does not go through the floor
            addPixel(f, x, y, px, static_cast<float>(yy), zdist);
        }
    }
}

int failures = 0;

void check(const char* what, bool cond) {
    std::printf("  [%s] %s\n", cond ? "OK  " : "FAIL", what);
    if (!cond) ++failures;
}

} // namespace

int main() {
    PresenceDetector detector;

    {   // empty scene: only the floor
        PointCloudFrame f;
        addGround(f);
        const auto r = detector.analyze(f);
        std::printf("cena so com chao (%zu pts): %s\n", f.points.size(), r.summary.c_str());
        check("nao detecta pessoa", !r.present);
    }

    {   // person 1.70 m tall at 2.0 m
        PointCloudFrame f;
        addGround(f);
        addPerson(f, 0.0f, 2.0f, 1.70f, 0.25f);
        const auto r = detector.analyze(f);
        std::printf("cena com pessoa 1.70 m a 2.0 m: %s\n", r.summary.c_str());
        check("detecta presenca", r.present);
        check("contagem = 1", r.person_count == 1);
        check("altura ~1.7 m", std::fabs(r.height_m - 1.70f) < 0.25f);
        check("distancia ~2.0 m", std::fabs(r.distance_m - 2.0f) < 0.25f);
    }

    {   // two people at different distances
        PointCloudFrame f;
        addGround(f);
        addPerson(f, -0.4f, 1.5f, 1.75f, 0.22f);
        addPerson(f, 0.5f, 3.0f, 1.60f, 0.22f);
        const auto r = detector.analyze(f);
        std::printf("cena com 2 pessoas: %s\n", r.summary.c_str());
        check("detecta 2 pessoas", r.person_count == 2);
        check("distancia = a mais proxima", r.distance_m < 2.0f);
    }

    {   // small object 0.35 m tall at 1.5 m -> not a person
        PointCloudFrame f;
        addGround(f);
        addPerson(f, 0.0f, 1.5f, 0.35f, 0.12f);
        const auto r = detector.analyze(f);
        std::printf("cena com objeto pequeno 0.35 m: %s\n", r.summary.c_str());
        check("nao classifica como pessoa", !r.present);
    }

    {   // no valid points
        PointCloudFrame f;
        Point3D p;
        p.valid = false;
        p.status = PixelStatus::Invalid;
        f.points.assign(10, p);
        const auto r = detector.analyze(f);
        check("cena sem pontos validos -> ausente", !r.present);
    }

    std::printf("%s\n", failures == 0 ? "ALL PRESENCE TESTS PASSED" : "PRESENCE TESTS FAILED");
    return failures == 0 ? 0 : 1;
}
