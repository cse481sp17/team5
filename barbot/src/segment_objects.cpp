#include "perception/crop.h"
#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

namespace barbot {
    SegmentObjects::SegmentObjects(const ros::Service& service_call, const ros::Publisher& marker_pub)
        : service_call_(service_call), marker_pub_(marker_pub){}
    void SegmentObjects::Callback(const sensor_msgs::PointCloud2& msg) {

    }
}  // namespace barbot
