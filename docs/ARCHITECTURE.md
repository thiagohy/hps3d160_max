# Architecture

## Modules

- `DeviceManager` — connection lifecycle, filters, ROI group, multi-camera code, reconnect.
- `CaptureService` — callback registration, frame conversion, stats/FPS/drop tracking.
- `PointCloudBuilder` — converts `HPS3D_DepthData_t` into `PointCloudFrame` (meters), flags invalid pixels.
- `Recorder` — CSV + per-frame XYZ (and PCD when PCL is available).
- `HealthMonitor` — periodic logging, low-FPS warnings, disconnect events.
- `MultiCameraManager` — manages N devices, assigns codes, applies config to all.
- `RosPointCloudPublisher` — publishes `sensor_msgs/PointCloud2` (optional, ROS build).

## Data flow

```
Device -> (callback) -> CaptureService -> PointCloudBuilder -> Recorder / ROS publisher / HealthMonitor
```

## Extending

- Add GPIO sync: use HPS-3D160-U IO lines; trigger capture via external signal and timestamp frames.
- Add HDR mode switching: extend `FilterConfig`/`CameraConfig` and call settings via protocol if needed.
- Add PCL processing: voxel grid, outlier removal, registration between multi-camera frames.