#include "ros/ros.h"
namespace ros {
namespace mock {
static sensor_msgs::PointCloud2 g_last;
void store_pointcloud(const void* p) {
    g_last = *static_cast<const sensor_msgs::PointCloud2*>(p);
}
const sensor_msgs::PointCloud2* last_pointcloud() { return &g_last; }
} // namespace mock
} // namespace ros
