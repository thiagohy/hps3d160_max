#include "capability_probe.hpp"

#include <HPS3DUser_IF.h>

#include <cstdio>
#include <sstream>

namespace hps {
namespace {

const char* transport_name(internal::TransportTypeDef t) {
    switch (t) {
        case internal::TRANSPORT_USB: return "usb";
        case internal::TRANSPORT_CAN: return "can";
        case internal::TRANSPORT_RS232: return "rs232";
        case internal::TRANSPORT_RS485: return "rs485";
        case internal::TRANSPORT_ETHERNET: return "ethernet";
    }
    return "unknown";
}

const char* hdr_mode_name(internal::HDRModeTypeDef m) {
    switch (m) {
        case internal::HDR_DISABLE: return "disable";
        case internal::AUTO_HDR: return "auto_hdr";
        case internal::SUPER_HDR: return "super_hdr";
        case internal::SIMPLE_HDR: return "simple_hdr";
    }
    return "unknown";
}

const char* reference_name(internal::ROIReferenceTypeDef r) {
    switch (r) {
        case internal::ROI_REF_DIST_AVR: return "distance_average";
        case internal::ROI_REF_DIST_MIN: return "distance_min";
        case internal::ROI_REF_DIST_MAX: return "distance_max";
        case internal::ROI_REF_SAT_COUNT: return "saturation_count";
        case internal::ROI_REF_AMPLITUDE: return "amplitude";
        case internal::ROI_REF_VAILD_AMPLITUDE: return "valid_amplitude";
        case internal::ROI_REF_THRESHOLD_PIX_NUM: return "threshold_pixel_num";
    }
    return "unknown";
}

const char* alarm_name(internal::ROIAlarmTypeDef a) {
    switch (a) {
        case internal::ROI_ALARM_DISABLE: return "disable";
        case internal::ROI_ALARM_GPIO: return "gpio";
    }
    return "unknown";
}

const char* mirror_name(internal::PointCloudImageMirrorTypeDef m) {
    switch (m) {
        case internal::MIRROR_DISABLE: return "disable";
        case internal::MIRROR_HORIZONTAL: return "horizontal";
        case internal::MIRROR_VERTICLA: return "vertical";
        case internal::MIRROR_HORIZONTAL_VERTICLA: return "horizontal_vertical";
    }
    return "unknown";
}

std::string json_escape(const std::string& in) {
    std::string out;
    out.reserve(in.size() + 8);
    for (char c : in) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out += c;
                }
        }
    }
    return out;
}

} // namespace

std::string CapabilityReport::to_json() const {
    std::ostringstream o;
    o << "{\n";
    o << "  \"identity\": {\n";
    o << "    \"device_version\": \"" << json_escape(device_version) << "\",\n";
    o << "    \"sdk_version\": \"" << json_escape(sdk_version) << "\",\n";
    o << "    \"serial_number\": \"" << json_escape(serial_number) << "\",\n";
    o << "    \"sensor_temperature_c\": " << sensor_temperature_c << "\n";
    o << "  },\n";

    o << "  \"link\": {\n";
    o << "    \"transport\": \"" << transport << "\",\n";
    o << "    \"device_address\": " << device_address << ",\n";
    o << "    \"multi_camera_code\": " << multi_camera_code << ",\n";
    o << "    \"keep_alive_enabled\": " << (keep_alive_enabled ? "true" : "false") << ",\n";
    o << "    \"keep_alive_time_ms\": " << keep_alive_time_ms << "\n";
    o << "  },\n";

    o << "  \"optics\": {\n";
    o << "    \"hdr_mode\": \"" << hdr_mode << "\",\n";
    o << "    \"hdr_overexposed_amplitude\": " << hdr_overexposed << ",\n";
    o << "    \"hdr_weak_amplitude\": " << hdr_weak << ",\n";
    o << "    \"hdr_simple_max_integration_us\": " << hdr_simple_max_integration_us << ",\n";
    o << "    \"hdr_simple_min_integration_us\": " << hdr_simple_min_integration_us << ",\n";
    o << "    \"hdr_super_frame_number\": " << hdr_super_frame_number << ",\n";
    o << "    \"hdr_super_max_integration_us\": " << hdr_super_max_integration_us << ",\n";
    o << "    \"hdr_disable_integration_us\": " << hdr_disable_integration_us << ",\n";
    o << "    \"max_integration_time_us\": " << max_integration_time_us << ",\n";
    o << "    \"min_amplitude\": " << min_amplitude << ",\n";
    o << "    \"optical_compensation\": " << (optical_compensation ? "true" : "false") << ",\n";
    o << "    \"view_angle_horiz_deg\": " << view_angle_horiz << ",\n";
    o << "    \"view_angle_vertical_deg\": " << view_angle_vertical << ",\n";
    o << "    \"illum_angle_horiz_deg\": " << illum_angle_horiz << ",\n";
    o << "    \"illum_angle_vertical_deg\": " << illum_angle_vertical << ",\n";
    o << "    \"mounting_enabled\": " << (mounting_enabled ? "true" : "false") << ",\n";
    o << "    \"mounting_angle_deg\": " << mounting_angle_deg << ",\n";
    o << "    \"mounting_height_mm\": " << mounting_height_mm << "\n";
    o << "  },\n";

    o << "  \"filtering\": {\n";
    o << "    \"distance_filter_type\": \"" << distance_filter_type << "\",\n";
    o << "    \"distance_filter_k\": " << distance_filter_k << ",\n";
    o << "    \"distance_filter_threshold\": " << distance_filter_threshold << ",\n";
    o << "    \"distance_filter_num_check\": " << distance_filter_num_check << ",\n";
    o << "    \"smooth_filter_type\": \"" << smooth_filter_type << "\",\n";
    o << "    \"smooth_filter_arg\": " << smooth_filter_arg << ",\n";
    o << "    \"distance_offset_mm\": " << distance_offset_mm << ",\n";
    o << "    \"edge_detection_enabled\": " << (edge_detection_enabled ? "true" : "false") << ",\n";
    o << "    \"edge_detection_value\": " << edge_detection_value << ",\n";
    o << "    \"threshold_offset\": " << threshold_offset << "\n";
    o << "  },\n";

    o << "  \"roi\": {\n";
    o << "    \"count\": " << roi_count << ",\n";
    o << "    \"threshold_count\": " << roi_threshold_count << ",\n";
    o << "    \"current_group_id\": " << roi_group_id << ",\n";
    o << "    \"rois\": [\n";
    for (std::size_t i = 0; i < rois.size(); ++i) {
        const auto& r = rois[i];
        o << "      {\n";
        o << "        \"id\": " << r.id << ",\n";
        o << "        \"enabled\": " << (r.enabled ? "true" : "false") << ",\n";
        o << "        \"region\": [" << r.left_top_x << ", " << r.left_top_y << ", "
          << r.right_bottom_x << ", " << r.right_bottom_y << "],\n";
        o << "        \"thresholds\": [\n";
        for (std::size_t t = 0; t < r.thresholds.size(); ++t) {
            const auto& th = r.thresholds[t];
            o << "          {\"id\": " << th.id << ", \"enabled\": " << (th.enable ? "true" : "false")
              << ", \"value\": " << th.value << ", \"hysteresis\": " << th.hysteresis
              << ", \"positive\": " << (th.positive ? "true" : "false")
              << ", \"pixel_number_threshold\": " << th.pixel_number_threshold
              << ", \"reference\": \"" << th.reference << "\""
              << ", \"alarm\": \"" << th.alarm << "\"}";
            if (t + 1 < r.thresholds.size()) o << ",";
            o << "\n";
        }
        o << "        ]\n";
        o << "      }";
        if (i + 1 < rois.size()) o << ",";
        o << "\n";
    }
    o << "    ]\n";
    o << "  },\n";

    o << "  \"gpio\": {\n";
    o << "    \"count\": " << gpio_count << ",\n";
    o << "    \"input_count\": " << gpio_in_count << ",\n";
    o << "    \"output_count\": " << gpio_out_count << ",\n";
    o << "    \"input\": \"" << json_escape(gpio_in) << "\",\n";
    o << "    \"output\": \"" << json_escape(gpio_out) << "\"\n";
    o << "  },\n";

    o << "  \"interference\": {\n";
    o << "    \"detection_enabled\": " << (interference_detect_enabled ? "true" : "false") << ",\n";
    o << "    \"integration_time_us\": " << interference_integration_time_us << ",\n";
    o << "    \"amplitude_threshold\": " << interference_amplitude_threshold << ",\n";
    o << "    \"capture_num\": " << interference_capture_num << ",\n";
    o << "    \"number_check\": " << interference_number_check << "\n";
    o << "  },\n";

    o << "  \"point_cloud\": {\n";
    o << "    \"enabled\": " << (point_cloud_enabled ? "true" : "false") << ",\n";
    o << "    \"mirror\": \"" << point_cloud_mirror << "\"\n";
    o << "  },\n";

    o << "  \"obstacle\": {\n";
    o << "    \"enabled\": " << (obstacle_enabled ? "true" : "false") << ",\n";
    o << "    \"count\": " << obstacle_count << ",\n";
    o << "    \"min_distance_mm\": " << obstacle_min_dist_mm << ",\n";
    o << "    \"max_distance_mm\": " << obstacle_max_dist_mm << ",\n";
    o << "    \"invalid_value_mm\": " << obstacle_invalid_value_mm << ",\n";
    o << "    \"pixel_number_threshold\": " << obstacle_pixel_threshold << "\n";
    o << "  },\n";

    o << "  \"notes\": [";
    for (std::size_t i = 0; i < notes.size(); ++i) {
        o << "\n    \"" << json_escape(notes[i]) << "\"";
        if (i + 1 < notes.size()) o << ",";
    }
    if (!notes.empty()) o << "\n  ";
    o << "]\n";
    o << "}\n";
    return o.str();
}

CapabilityReport CapabilityProbe::collect() const {
    CapabilityReport r;
    const int h = bridge_.public_handle();

    const std::uint8_t* ver = HPS3DAPI_GetDeviceVersion(h);
    if (ver) r.device_version = reinterpret_cast<const char*>(ver);
    const std::uint8_t* sdk = HPS3DAPI_GetSDKVersion();
    if (sdk) r.sdk_version = reinterpret_cast<const char*>(sdk);
    const std::uint8_t* sn = HPS3DAPI_GetSerialNumber(h);
    if (sn) r.serial_number = reinterpret_cast<const char*>(sn);

    if (!bridge_.available() || !bridge_.internal_handle()) {
        r.notes.push_back("advanced SDK entry points unavailable: only public API data collected");
        return r;
    }

    internal::TransportTypeDef tt{};
    if (bridge_.transport_type(tt)) {
        r.transport_code = static_cast<int>(tt);
        r.transport = transport_name(tt);
    } else {
        r.notes.push_back("transport type unavailable");
    }

    std::uint8_t addr = 0;
    if (bridge_.device_address(addr)) r.device_address = static_cast<int>(addr);

    std::uint8_t code = 0;
    if (bridge_.multi_camera_code(code)) r.multi_camera_code = static_cast<int>(code);

    bool ka = false;
    if (bridge_.keep_alive_enabled(ka)) r.keep_alive_enabled = ka;
    std::uint32_t kat = 0;
    if (bridge_.keep_alive_time(kat)) r.keep_alive_time_ms = static_cast<int>(kat);

    internal::HDRConf hdr{};
    if (bridge_.hdr_config(hdr)) {
        r.hdr_mode = hdr_mode_name(hdr.hdr_mode);
        r.hdr_overexposed = hdr.qualtity_overexposed;
        r.hdr_weak = hdr.qualtity_weak;
        r.hdr_simple_max_integration_us = static_cast<int>(hdr.simple_hdr_max_integration);
        r.hdr_simple_min_integration_us = static_cast<int>(hdr.simple_hdr_min_integration);
        r.hdr_super_frame_number = static_cast<int>(hdr.super_hdr_frame_number);
        r.hdr_super_max_integration_us = static_cast<int>(hdr.super_hdr_max_integration);
        r.hdr_disable_integration_us = static_cast<int>(hdr.hdr_disable_integration_time);
    } else {
        r.notes.push_back("HDR configuration unavailable");
    }

    // NOTE: V1.8.6 does not export the V1.7.x max integration time / min amplitude
    // entry points (HE201_ApiGetMaxIntegTime / GetMinAmplitude are absent), the
    // integration times are reported through the HDR configuration instead.
    r.max_integration_time_us = r.hdr_super_max_integration_us;
    r.min_amplitude = static_cast<int>(r.hdr_weak);

    internal::OpticalParamConfTypeDef opt{};
    if (bridge_.optical_config(opt)) {
        r.optical_compensation = opt.enable;
        r.view_angle_horiz = opt.viewing_angle_horiz;
        r.view_angle_vertical = opt.viewing_angle_vertical;
        r.illum_angle_horiz = opt.illum_angle_horiz;
        r.illum_angle_vertical = opt.illum_angle_vertical;
    }

    internal::MountingAngleParamTypeDef mount{};
    if (bridge_.mounting_config(mount)) {
        r.mounting_enabled = mount.enable;
        r.mounting_angle_deg = mount.angle_vertical;
        r.mounting_height_mm = mount.height;
    }

    internal::DistanceFilterConfTypeDef dfc{};
    if (bridge_.distance_filter_config(dfc)) {
        r.distance_filter_type = dfc.filter_type == internal::DISTANCE_FILTER_SIMPLE_KALMAN ? "simple_kalman" : "disable";
        r.distance_filter_k = dfc.kalman_K;
        r.distance_filter_threshold = static_cast<int>(dfc.kalman_threshold);
        r.distance_filter_num_check = static_cast<int>(dfc.num_check);
    }

    internal::SmoothFilterConfTypeDef sfc{};
    if (bridge_.smooth_filter_config(sfc)) {
        switch (sfc.type) {
            case internal::SMOOTH_FILTER_DISABLE: r.smooth_filter_type = "disable"; break;
            case internal::SMOOTH_FILTER_AVERAGE: r.smooth_filter_type = "average"; break;
            case internal::SMOOTH_FILTER_GAUSS: r.smooth_filter_type = "gauss"; break;
        }
        r.smooth_filter_arg = static_cast<int>(sfc.arg1);
    }

    std::int16_t off = 0;
    if (bridge_.distance_offset(off)) r.distance_offset_mm = off;

    bool ed = false;
    if (bridge_.edge_detection_enabled(ed)) r.edge_detection_enabled = ed;
    std::int32_t edv = 0;
    if (bridge_.edge_detection_value(edv)) r.edge_detection_value = edv;
    std::int32_t tov = 0;
    if (bridge_.threshold_offset(tov)) r.threshold_offset = tov;

    std::uint8_t roi_n = 0, thr_n = 0;
    if (bridge_.number_of_rois(roi_n, thr_n)) {
        r.roi_count = roi_n;
        r.roi_threshold_count = thr_n;
        for (std::uint8_t i = 0; i < roi_n && i < 8; ++i) {
            internal::ROIConfTypeDef rc{};
            if (!bridge_.roi_config(i, rc)) continue;
            RoiReport rr;
            rr.id = rc.roi_id;
            rr.enabled = rc.enable;
            rr.left_top_x = rc.left_top_x;
            rr.left_top_y = rc.left_top_y;
            rr.right_bottom_x = rc.right_bottom_x;
            rr.right_bottom_y = rc.right_bottom_y;
            for (int t = 0; t < 3; ++t) {
                ThresholdReport tr;
                tr.id = t;
                tr.enable = rc.hysteresis_conf[t].enable;
                tr.value = static_cast<int>(rc.hysteresis_conf[t].threshold_value);
                tr.hysteresis = static_cast<int>(rc.hysteresis_conf[t].hysteresis);
                tr.positive = rc.hysteresis_conf[t].positive;
                tr.pixel_number_threshold = rc.pixel_number_threshold[t];
                tr.reference = reference_name(rc.ref_type[t]);
                tr.alarm = alarm_name(rc.alarm_type[t]);
                rr.thresholds.push_back(tr);
            }
            r.rois.push_back(rr);
        }
    } else {
        r.notes.push_back("ROI configuration unavailable");
    }

    std::uint8_t group = 0;
    if (bridge_.roi_group_id(group)) r.roi_group_id = group;

    std::uint8_t gpio_in_n = 0, gpio_out_n = 0;
    if (bridge_.gpio_count(gpio_in_n, gpio_out_n)) {
        r.gpio_count = gpio_in_n + gpio_out_n;
        r.gpio_in_count = gpio_in_n;
        r.gpio_out_count = gpio_out_n;
    }

    // HPS-3D160-U exposes one optocoupler input and one optocoupler output.
    for (std::uint8_t pin = internal::GPIN_1; pin <= internal::GPIN_3 && pin <= gpio_in_n; ++pin) {
        internal::GPIOInConfTypeDef gin{};
        if (!bridge_.gpio_in_config(pin, gin)) continue;
        char buf[128];
        std::snprintf(buf, sizeof(buf), "pin=%d polarity=%s function=%s",
                      static_cast<int>(gin.gpio),
                      gin.polarity == internal::GPIO_POLARITY_HIGH ? "high" : "low",
                      gin.function == internal::GPIN_FUNC_CAPTURE ? "capture" : "disable");
        if (!r.gpio_in.empty()) r.gpio_in += "; ";
        r.gpio_in += buf;
    }

    for (std::uint8_t pin = internal::GPOUT_1; pin <= internal::GPOUT_3 && pin <= (gpio_out_n + internal::GPOUT_1 - 1); ++pin) {
        internal::GPIOOutConfTypeDef gout{};
        if (!bridge_.gpio_out_config(pin, gout)) continue;
        char buf[128];
        std::snprintf(buf, sizeof(buf), "pin=%d polarity=%s function=%s",
                      static_cast<int>(gout.gpio),
                      gout.polarity == internal::GPIO_POLARITY_HIGH ? "high" : "low",
                      gout.function == internal::GPOUT_FUNC_DISABLE
                          ? "disable"
                          : (gout.function == internal::GPOUT_FUNC_ROI_THRESHOLD0_ALARM
                                 ? "roi_threshold0_alarm"
                                 : (gout.function == internal::GPOUT_FUNC_ROI_THRESHOLD1_ALARM
                                        ? "roi_threshold1_alarm"
                                        : "roi_threshold2_alarm")));
        if (!r.gpio_out.empty()) r.gpio_out += "; ";
        r.gpio_out += buf;
    }

    internal::InterferenceDetectConfTypeDef itc{};
    if (bridge_.interference_config(itc)) {
        r.interference_detect_enabled = itc.enable;
        r.interference_integration_time_us = static_cast<int>(itc.integ_time);
        r.interference_amplitude_threshold = itc.amplitude_threshold;
        r.interference_capture_num = itc.capture_num;
        r.interference_number_check = itc.number_check;
    }

    bool pc = false;
    if (bridge_.point_cloud_enabled(pc)) r.point_cloud_enabled = pc;
    internal::PointCloudImageMirrorTypeDef mir{};
    if (bridge_.point_cloud_mirror(mir)) r.point_cloud_mirror = mirror_name(mir);

    internal::ObstacleConfigTypedef obs{};
    if (bridge_.obstacle_config(obs)) {
        r.obstacle_enabled = obs.enable;
        r.obstacle_count = obs.number;
        r.obstacle_min_dist_mm = obs.vaild_min_dist;
        r.obstacle_max_dist_mm = obs.vaild_max_dist;
        r.obstacle_invalid_value_mm = obs.invaild_value;
    }
    std::uint32_t obsthr = 0;
    if (bridge_.obstacle_pixel_threshold(obsthr)) r.obstacle_pixel_threshold = static_cast<int>(obsthr);

    float temp = 0.f;
    if (bridge_.sensor_temperature(temp)) r.sensor_temperature_c = temp;
    else r.notes.push_back("sensor temperature unavailable");

    return r;
}

} // namespace hps
