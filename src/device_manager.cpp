#include "device_manager.hpp"

#include <cstdio>
#include <cstring>
#include <chrono>
#include <thread>

namespace hps {

DeviceManager::DeviceManager(const ConnectionConfig& cfg) : cfg_(cfg) {}

DeviceManager::~DeviceManager() {
    disconnect();
}

bool DeviceManager::connect() {
    HPS3D_StatusTypeDef ret = HPS3D_RET_ERROR;
    if (cfg_.mode == ConnectionMode::USB) {
        ret = HPS3D_USBConnectDevice(const_cast<char*>(cfg_.usb_port.c_str()), &handle_);
    } else {
        ret = HPS3D_EthernetConnectDevice(const_cast<char*>(cfg_.eth_ip.c_str()),
                                          cfg_.eth_port, &handle_);
    }
    if (ret != HPS3D_RET_OK) {
        std::fprintf(stderr, "[DeviceManager] connect failed: %d\n", ret);
        handle_ = -1;
        connected_ = false;
        return false;
    }
    connected_ = true;
    std::printf("[DeviceManager] connected (handle=%d)\n", handle_);
    return true;
}

void DeviceManager::disconnect() {
    if (handle_ >= 0) {
        if (capturing_) stopCapture();
        HPS3D_CloseDevice(handle_);
        handle_ = -1;
    }
    connected_ = false;
    capturing_ = false;
}

bool DeviceManager::isConnected() const { return connected_.load(); }
bool DeviceManager::isCapturing() const { return capturing_.load(); }

bool DeviceManager::startCapture() {
    if (!connected_) return false;
    HPS3D_StatusTypeDef ret = HPS3D_StartCapture(handle_);
    if (ret != HPS3D_RET_OK) {
        std::fprintf(stderr, "[DeviceManager] StartCapture failed: %d\n", ret);
        return false;
    }
    capturing_ = true;
    return true;
}

bool DeviceManager::stopCapture() {
    if (!connected_) return false;
    HPS3D_StatusTypeDef ret = HPS3D_StopCapture(handle_);
    if (ret != HPS3D_RET_OK) {
        std::fprintf(stderr, "[DeviceManager] StopCapture failed: %d\n", ret);
        return false;
    }
    capturing_ = false;
    return true;
}

bool DeviceManager::singleCapture(HPS3D_EventType_t& type, HPS3D_MeasureData_t& data) {
    if (!connected_) return false;
    return HPS3D_SingleCapture(handle_, &type, &data) == HPS3D_RET_OK;
}

bool DeviceManager::applyFilters(const FilterConfig& cfg) {
    if (!connected_) return false;
    bool ok = true;
    ok &= HPS3D_SetDistanceFilterConf(handle_, cfg.distance_filter_enabled, cfg.distance_filter_k) == HPS3D_RET_OK;
    ok &= HPS3D_SetSmoothFilterConf(handle_, static_cast<HPS3D_SmoothFilterType_t>(cfg.smooth_filter_type), cfg.smooth_filter_args) == HPS3D_RET_OK;
    ok &= HPS3D_SetDistanceOffset(handle_, cfg.distance_offset) == HPS3D_RET_OK;
    ok &= HPS3D_SetOpticalPathCalibration(handle_, cfg.optical_path_calibration) == HPS3D_RET_OK;
    ok &= HPS3D_SetEdgeFilterEnable(handle_, cfg.edge_filter_enabled) == HPS3D_RET_OK;
    if (!ok) std::fprintf(stderr, "[DeviceManager] applyFilters partially failed\n");
    return ok;
}

bool DeviceManager::setRoiGroup(std::uint8_t group_id) {
    if (!connected_) return false;
    return HPS3D_SetROIGroupID(handle_, group_id) == HPS3D_RET_OK;
}

bool DeviceManager::setMultiCameraCode(std::uint8_t code) {
    if (!connected_) return false;
    return HPS3D_SetMultiCameraCode(handle_, code) == HPS3D_RET_OK;
}

bool DeviceManager::setUserId(std::uint8_t user_id) {
    if (!connected_) return false;
    return HPS3D_SetDeviceUserID(handle_, user_id) == HPS3D_RET_OK;
}

bool DeviceManager::exportSettings(HPS3D_DeviceSettings_t& out) {
    if (!connected_) return false;
    return HPS3D_ExportSettings(handle_, &out) == HPS3D_RET_OK;
}

bool DeviceManager::saveSettings() {
    if (!connected_) return false;
    return HPS3D_SaveSettings(handle_) == HPS3D_RET_OK;
}

void DeviceManager::setFrameCallback(FrameCallback cb) {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    frame_cb_ = std::move(cb);
    if (!callback_registered_) {
        HPS3D_StatusTypeDef ret = HPS3D_RegisterEventCallback(&DeviceManager::staticCallback, this);
        if (ret != HPS3D_RET_OK) {
            std::fprintf(stderr, "[DeviceManager] RegisterEventCallback failed: %d\n", ret);
        } else {
            callback_registered_ = true;
        }
    }
}

bool DeviceManager::reconnect() {
    if (cfg_.mode != ConnectionMode::Ethernet) return false;
    HPS3D_StatusTypeDef ret = HPS3D_EthternetReconnection(handle_);
    if (ret == HPS3D_RET_OK) {
        std::printf("[DeviceManager] reconnected\n");
        connected_ = true;
        return true;
    }
    std::fprintf(stderr, "[DeviceManager] reconnect failed: %d\n", ret);
    return false;
}

void DeviceManager::staticCallback(int handle, int eventType, std::uint8_t* data, int dataLen, void* userPara) {
    auto* self = static_cast<DeviceManager*>(userPara);
    if (self) self->onRawEvent(handle, eventType, data, dataLen);
}

void DeviceManager::onRawEvent(int handle, int eventType, std::uint8_t* data, int dataLen) {
    HPS3D_EventType_t type = static_cast<HPS3D_EventType_t>(eventType);
    if (type == HPS3D_SIMPLE_ROI_EVEN || type == HPS3D_FULL_ROI_EVEN ||
        type == HPS3D_SIMPLE_DEPTH_EVEN || type == HPS3D_FULL_DEPTH_EVEN) {
        HPS3D_MeasureData_t parsed{};
        HPS3D_ConvertToMeasureData(data, &parsed, type);
        std::lock_guard<std::mutex> lk(cb_mutex_);
        if (frame_cb_) frame_cb_(handle, type, parsed);
    } else if (type == HPS3D_DISCONNECT_EVEN) {
        std::fprintf(stderr, "[DeviceManager] device disconnected (handle=%d)\n", handle);
        connected_ = false;
        capturing_ = false;
        if (cfg_.reconnect_enabled) {
            std::thread([this]() {
                std::this_thread::sleep_for(std::chrono::seconds(10));
                if (reconnect()) {
                    // optionally restart capture
                }
            }).detach();
        }
    } else if (type == HPS3D_SYS_EXCEPTION_EVEN) {
        std::fprintf(stderr, "[DeviceManager] system exception: %s\n", data ? reinterpret_cast<char*>(data) : "");
    }
}

} // namespace hps