#include "filter_config.hpp"

#include <algorithm>

namespace hps {

bool FilterValidator::validate(const FilterConfig& cfg, std::string& reason) {
    if (cfg.distance_filter_k < 0.0f || cfg.distance_filter_k > 1.0f) {
        reason = "distance_filter_k out of range [0,1]";
        return false;
    }
    if (cfg.smooth_filter_type < 0 || cfg.smooth_filter_type > 2) {
        reason = "smooth_filter_type must be 0,1,2";
        return false;
    }
    if (cfg.smooth_filter_args < 0) {
        reason = "smooth_filter_args must be >= 0";
        return false;
    }
    return true;
}

FilterConfig FilterValidator::clamp(const FilterConfig& cfg) {
    FilterConfig out = cfg;
    out.distance_filter_k = std::max(0.0f, std::min(1.0f, out.distance_filter_k));
    out.smooth_filter_type = std::max(0, std::min(2, out.smooth_filter_type));
    out.smooth_filter_args = std::max(0, out.smooth_filter_args);
    return out;
}

} // namespace hps