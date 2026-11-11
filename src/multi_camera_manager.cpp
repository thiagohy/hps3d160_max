#include "multi_camera_manager.hpp"

#include <cstdio>

namespace hps {

MultiCameraManager::MultiCameraManager(const AppConfig& cfg) : cfg_(cfg) {
    std::vector<ConnectionConfig> conns = cfg_.connections;
    if (conns.empty()) {
        int n = cfg_.device_count > 0 ? cfg_.device_count : 1;
        conns.assign(n, cfg_.connection);
    }
    for (std::size_t i = 0; i < conns.size(); ++i) {
        devices_.push_back(std::make_unique<DeviceManager>(conns[i]));
    }
}

MultiCameraManager::~MultiCameraManager() { disconnectAll(); }

bool MultiCameraManager::connectAll() {
    bool ok = true;
    for (std::size_t i = 0; i < devices_.size(); ++i) {
        if (!devices_[i]->connect()) {
            ok = false;
            continue;
        }
        devices_[i]->setMultiCameraCode(static_cast<std::uint8_t>(i & 0x0F));
        devices_[i]->setUserId(static_cast<std::uint8_t>(cfg_.camera.user_id));
        devices_[i]->setRoiGroup(static_cast<std::uint8_t>(cfg_.camera.roi_group_id));
        devices_[i]->applyFilters(cfg_.filters);
        if (cfg_.camera.save_settings_to_device) devices_[i]->saveSettings();
    }
    return ok;
}

void MultiCameraManager::disconnectAll() {
    for (auto& d : devices_) d->disconnect();
}

bool MultiCameraManager::applyFiltersAll(const FilterConfig& cfg) {
    bool ok = true;
    for (auto& d : devices_) ok &= d->applyFilters(cfg);
    return ok;
}

bool MultiCameraManager::setRoiGroupAll(std::uint8_t group_id) {
    bool ok = true;
    for (auto& d : devices_) ok &= d->setRoiGroup(group_id);
    return ok;
}

} // namespace hps