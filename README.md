# hps3d160_max — Max-utilization system for HPS-3D160-U (USB + IO)

A ROS-friendly C++ system that exercises the full feature set of the Hypersen HPS-3D160-U
depth camera: continuous and single capture, ROI groups, distance/smooth/edge filters,
optical-path compensation, point-cloud publishing and recording, multi-camera coordination,
reconnect and health monitoring.

## Build (standalone, no ROS)

```bash
mkdir -p build && cd build
cmake .. -DHPS3D160_MAX_WITH_ROS=OFF
make -j$(nproc)
./hps3d160_max_node --mode usb --port /dev/ttyACM0 --record-dir ./out
```

## Build (ROS/catkin)

```bash
# inside your catkin workspace src/
ln -s /path/to/hps3d160_max .
cd .. && catkin_make
source devel/setup.bash
roslaunch hps3d160_max hps3d160.launch
```

## Features mapped to SDK

| Feature | SDK API |
|---|---|
| USB/Ethernet connect | HPS3D_USBConnectDevice / HPS3D_EthernetConnectDevice |
| Reconnect | HPS3D_EthternetReconnection |
| Continuous/single capture | HPS3D_StartCapture / HPS3D_SingleCapture |
| ROI group | HPS3D_SetROIGroupID |
| Distance filter | HPS3D_SetDistanceFilterConf |
| Smooth filter | HPS3D_SetSmoothFilterConf |
| Distance offset | HPS3D_SetDistanceOffset |
| Optical path calibration | HPS3D_SetOpticalPathCalibration |
| Edge filter | HPS3D_SetEdgeFilterEnable |
| Multi-camera code | HPS3D_SetMultiCameraCode |
| Point cloud | HPS3D_DepthData_t.point_cloud_data |
| Export/save settings | HPS3D_ExportSettings / HPS3D_SaveSettings |

## Notes

- The SDK core is proprietary; only headers and x86_64 Linux libraries are bundled.
- For ARM targets, replace `third_party/hps_sdk/lib` with the matching `.so`/`.a`.
- Recording formats: CSV (always) + per-frame XYZ; PCD when built with PCL.