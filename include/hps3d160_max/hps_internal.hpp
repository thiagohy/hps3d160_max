#pragma once

// Mirrors of the internal Hypersen SDK types (HPS3D_RM002 / api.h V1.8.5).
// The shipped V1.8.6 shared library exports the low level API as HE201_Api* /
// HE201_* symbols which are not exposed by the public HPS3DAPI_* wrapper.
//
// These declarations are only used to talk to the device through those extra
// entry points. They are resolved dynamically (dlsym) so the program still
// links and runs against SDK builds that do not export them.

#include <cstdint>

namespace hps {
namespace internal {

// --- enumerations -----------------------------------------------------------
enum RunModeTypeDef { RUN_IDLE = 0, RUN_SINGLE_SHOT, RUN_CONTINUOUS };

enum MeasurePacketTypeDef { DEPTH_DATA_PACKET = 0x0, ROI_DATA_PACKET = 1, OBSTACLE_PACKET = 2 };

enum OutPacketTypeDef { PACKET_FULL = 0, PACKET_SIMPLE = 1 };

enum ROIAlarmTypeDef { ROI_ALARM_DISABLE = 0, ROI_ALARM_GPIO = 1 };

enum ROIReferenceTypeDef {
    ROI_REF_DIST_AVR = 1,
    ROI_REF_DIST_MIN,
    ROI_REF_DIST_MAX,
    ROI_REF_SAT_COUNT,
    ROI_REF_AMPLITUDE,
    ROI_REF_VAILD_AMPLITUDE,
    ROI_REF_THRESHOLD_PIX_NUM
};

enum HDRModeTypeDef { HDR_DISABLE = 0, AUTO_HDR = 1, SUPER_HDR = 2, SIMPLE_HDR = 3 };

enum TransportTypeDef {
    TRANSPORT_USB = 0,
    TRANSPORT_CAN,
    TRANSPORT_RS232,
    TRANSPORT_RS485,
    TRANSPORT_ETHERNET
};

enum GPOutFunctionTypeDef {
    GPOUT_FUNC_DISABLE = 0,
    GPOUT_FUNC_ROI_THRESHOLD0_ALARM,
    GPOUT_FUNC_ROI_THRESHOLD1_ALARM,
    GPOUT_FUNC_ROI_THRESHOLD2_ALARM
};

enum GPInFunctionTypeDef { GPIN_FUNC_DISABLE = 0, GPIN_FUNC_CAPTURE = 1 };

enum GPIOPolarityTypeDef { GPIO_POLARITY_LOW = 0, GPIO_POLARITY_HIGH = 1 };

enum GPIOTypeDef { GPIN_1 = 1, GPIN_2 = 2, GPIN_3 = 3, GPOUT_1 = 10, GPOUT_2 = 11, GPOUT_3 = 12 };

enum DistanceFilterTypeDef { DISTANCE_FILTER_DISABLE = 0, DISTANCE_FILTER_SIMPLE_KALMAN };

enum SmoothFilterTypeDef { SMOOTH_FILTER_DISABLE = 0, SMOOTH_FILTER_AVERAGE = 1, SMOOTH_FILTER_GAUSS = 2 };

enum PointCloudImageMirrorTypeDef {
    MIRROR_DISABLE = 0x0,
    MIRROR_HORIZONTAL = 0x1,
    MIRROR_VERTICLA = 0x2,
    MIRROR_HORIZONTAL_VERTICLA = 0x3
};

// --- structures -------------------------------------------------------------
struct Version_t {
    std::uint8_t year;
    std::uint8_t month;
    std::uint8_t day;
    std::uint8_t major;
    std::uint8_t minor;
    std::uint8_t rev;
};

struct HysteresisSingleConfTypeDef {
    std::uint8_t threshold_id;
    std::uint32_t threshold_value;
    std::uint32_t hysteresis;
    bool enable;
    bool positive;
};

struct ROIConfTypeDef {
    bool enable;
    std::uint8_t roi_id;
    std::uint16_t left_top_x;
    std::uint16_t left_top_y;
    std::uint16_t right_bottom_x;
    std::uint16_t right_bottom_y;
    HysteresisSingleConfTypeDef hysteresis_conf[3];
    ROIReferenceTypeDef ref_type[3];
    ROIAlarmTypeDef alarm_type[3];
    std::uint16_t pixel_number_threshold[3];
};

struct HDRConf {
    HDRModeTypeDef hdr_mode;
    float qualtity_overexposed;
    float qualtity_overexposed_serious;
    float qualtity_weak;
    float qualtity_weak_serious;
    std::uint32_t simple_hdr_max_integration;
    std::uint32_t simple_hdr_min_integration;
    std::uint8_t super_hdr_frame_number;
    std::uint32_t super_hdr_max_integration;
    std::uint32_t hdr_disable_integration_time;
};

struct SmoothFilterConfTypeDef {
    SmoothFilterTypeDef type;
    std::uint32_t arg1;
};

struct DistanceFilterConfTypeDef {
    DistanceFilterTypeDef filter_type;
    float kalman_K;
    std::uint32_t kalman_threshold;
    std::uint32_t num_check;
};

struct GPIOOutConfTypeDef {
    GPIOTypeDef gpio;
    GPIOPolarityTypeDef polarity;
    GPOutFunctionTypeDef function;
};

struct GPIOInConfTypeDef {
    GPIOTypeDef gpio;
    GPIOPolarityTypeDef polarity;
    GPInFunctionTypeDef function;
};

struct OpticalParamConfTypeDef {
    bool enable;
    std::uint8_t viewing_angle_horiz;
    std::uint8_t viewing_angle_vertical;
    std::uint8_t illum_angle_horiz;
    std::uint8_t illum_angle_vertical;
};

struct MountingAngleParamTypeDef {
    bool enable;
    std::uint8_t angle_vertical;
    std::uint16_t height;
};

struct InterferenceDetectConfTypeDef {
    bool enable;
    std::uint32_t integ_time;
    std::uint16_t amplitude_threshold;
    std::uint8_t capture_num;
    std::uint8_t number_check;
};

struct ObstacleConfigTypedef {
    bool enable;
    std::uint16_t frame_head;
    std::uint8_t number;
    std::uint16_t vaild_min_dist;
    std::uint16_t vaild_max_dist;
    std::uint16_t invaild_value;
    std::uint16_t frame_size;
};

struct AvoidanceTypeDef {
    bool avoidance_enable;
    std::uint32_t avoidance_width;
    std::uint32_t avoidance_height;
    std::uint32_t avoidance_minDistance;
    std::uint32_t avoidance_mountHeight;
    std::uint32_t avoidance_pixelNumber;
    bool avoidance_alarmStatues;
    GPIOOutConfTypeDef GpioOutConf;
};

} // namespace internal
} // namespace hps
