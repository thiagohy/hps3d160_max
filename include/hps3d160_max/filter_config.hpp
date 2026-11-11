#pragma once

#include "types.hpp"

namespace hps {

// Helpers to validate and clamp filter parameters before applying.
struct FilterValidator {
    static bool validate(const FilterConfig& cfg, std::string& reason);
    static FilterConfig clamp(const FilterConfig& cfg);
};

} // namespace hps