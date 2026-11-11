#pragma once

#include "types.hpp"

#include <HPS3DUser_IF.h>

namespace hps {

class PointCloudBuilder {
public:
    static PointCloudFrame build(const HPS3D_DepthData_t& depth, std::uint64_t frame_id);

    // Utility: remove invalid/saturated points.
    static void filterInvalid(PointCloudFrame& frame);
};

} // namespace hps