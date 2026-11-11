#pragma once

#include "sdk_bridge.hpp"
#include "types.hpp"

#include <string>
#include <vector>

namespace hps {

// Measures the achievable frame rate of the sensor for each HDR mode and for
// the different packet types.
struct ThroughputResult {
    std::string label;
    double fps = 0.0;
    double interval_ms = 0.0;
    double payload_mb_s = 0.0;
    int event_type = 0;
    int valid_pixels = 0;
};

class ThroughputBenchmark {
public:
    // Runs `seconds` of continuous capture for each HDR mode and, when the
    // advanced SDK entry points are available, for each packet type.
    static std::vector<ThroughputResult> run(SdkBridge& bridge, int seconds, bool with_packet_types);

    static std::string to_json(const std::vector<ThroughputResult>& results);
};

} // namespace hps
