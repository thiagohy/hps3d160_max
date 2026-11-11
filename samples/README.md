# Sample raw capture (HPS-3D160-U)

Real capture from a single frame of an HPS-3D160-U sensor:

- `frame_1.xyz` — point cloud (x y z distance_mm), one point per line
- `frame_1.distances.csv` — raw distance matrix (160 x 60), millimeters
- `frame_1.meta.json` — frame metadata (frame id, timestamp, stats)
- `frames.csv` — per-frame summary

Useful for offline development and testing without hardware.