#!/usr/bin/env bash
# Builds the standalone diagnostic tools (no ROS needed).
#   tools/noise_probe.cpp      temporal noise floor of the distance measurement
#   tools/noise_vs_distance.cpp noise floor as a function of the target distance
#   tools/vital_signs.cpp        search for a cardiac component in the depth stream
set -euo pipefail
cd "$(dirname "$0")/.."
SDK=third_party/hps_sdk
mkdir -p build/tools
for t in noise_probe noise_vs_distance vital_signs; do
    g++ -std=c++17 -O2 "tools/$t.cpp" -o "build/tools/$t" \
        -I"$SDK/include" -L"$SDK/lib" -lHPS3DSDK64_1-8-6 \
        -Wl,-rpath,"$PWD/$SDK/lib"
    echo "built build/tools/$t"
done
