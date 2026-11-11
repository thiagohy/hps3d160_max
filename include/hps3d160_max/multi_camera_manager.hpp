#pragma once

#include "device_manager.hpp"
#include "types.hpp"

#include <memory>
#include <vector>

namespace hps {

class MultiCameraManager {
public:
    explicit MultiCameraManager(const AppConfig& cfg);
    ~MultiCameraManager();

    bool connectAll();
    void disconnectAll();

    std::size_t size() const { return devices_.size(); }
    DeviceManager& device(std::size_t i) { return *devices_[i]; }

    // Apply the same filters/config to all devices.
    bool applyFiltersAll(const FilterConfig& cfg);
    bool setRoiGroupAll(std::uint8_t group_id);

private:
    AppConfig cfg_;
    std::vector<std::unique_ptr<DeviceManager>> devices_;
};

} // namespace hps