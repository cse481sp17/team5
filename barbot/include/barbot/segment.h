#include "perception/crop.h"
#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"


namespace barbot {
class SegmentObjects {
 public:
  SegmentObjects(const ros::Service& service_call, const ros::Publisher& marker_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Service service_call_;
  ros::Publisher marker_pub_;
};
}  // namespace barbot
