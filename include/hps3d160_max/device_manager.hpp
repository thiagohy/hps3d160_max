#pragma once

#include "types.hpp"

#include <HPS3DUser_IF.h>

#include <atomic>
#include <functional>
#include <mutex>
#include <string>

namespace hps {

class DeviceManager {
public:
    using FrameCallback = std::function<void(int handle, HPS3D_EventType_t type, HPS3D_MeasureData_t& data)>;

    explicit DeviceManager(const ConnectionConfig& cfg);
    ~DeviceManager();

    DeviceManager(const DeviceManager&) = delete;
    DeviceManager& operator=(const DeviceManager&) = delete;

    bool connect();
    void disconnect();
    bool isConnected() const;
    bool isCapturing() const;

    bool startCapture();
    bool stopCapture();
    bool singleCapture(HPS3D_EventType_t& type, HPS3D_MeasureData_t& data);

    bool applyFilters(const FilterConfig& cfg);
    bool setRoiGroup(std::uint8_t group_id);
    bool setMultiCameraCode(std::uint8_t code);
    bool setUserId(std::uint8_t user_id);
    bool exportSettings(HPS3D_DeviceSettings_t& out);
    bool saveSettings();

    int handle() const { return handle_; }

    void setFrameCallback(FrameCallback cb);

    // Reconnect helper (Ethernet only).
    bool reconnect();

private:
    static void staticCallback(int handle, int eventType, std::uint8_t* data, int dataLen, void* userPara);
    void onRawEvent(int handle, int eventType, std::uint8_t* data, int dataLen);

    ConnectionConfig cfg_;
    int handle_ = -1;
    std::atomic<bool> connected_{false};
    std::atomic<bool> capturing_{false};
    std::atomic<bool> callback_registered_{false};

    FrameCallback frame_cb_;
    std::mutex cb_mutex_;
};

} // namespace hps