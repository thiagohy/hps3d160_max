#pragma once

#include "types.hpp"

#include <fstream>
#include <string>

namespace hps {

class Recorder {
public:
    explicit Recorder(const RecordingConfig& cfg);
    ~Recorder();

    bool open();
    void close();
    bool isOpen() const;

    bool record(const PointCloudFrame& frame);

private:
    bool writeCsv(const PointCloudFrame& frame);
#ifdef HPS3D160_MAX_WITH_PCL
    bool writePcd(const PointCloudFrame& frame);
#endif

    RecordingConfig cfg_;
    std::ofstream csv_;
    std::size_t recorded_ = 0;
    bool opened_ = false;
};

} // namespace hps