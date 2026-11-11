#!/usr/bin/env bash
# Compiles and runs the ROS publisher test against a minimal mock of the ROS
# API, so the ROS code path can be validated on hosts without ROS installed.
set -euo pipefail
cd "$(dirname "$0")/.."

OUT=${TMPDIR:-/tmp}/hps3d160_test_ros_publisher
g++ -std=c++17 -g -fsanitize=address \
    -DHPS3D160_MAX_WITH_ROS=1 \
    -Itests/ros_mock -Iinclude -Iinclude/hps3d160_max -Ithird_party/hps_sdk/include \
    src/ros_publisher.cpp tests/test_ros_publisher.cpp tests/ros_mock/ros_impl.cpp \
    -o "$OUT"

"$OUT"
echo "ROS publisher test passed"
