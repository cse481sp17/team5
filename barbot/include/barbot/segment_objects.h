
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "barbot/MoveToPerception.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

typedef barbot::MoveToPerception MpServ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace barbot {
class SegmentObjects {
 public:
  SegmentObjects(const ros::Publisher& marker_pub);
  bool ServiceCallback(MpServ::Request  &req,
      MpServ::Response &res);
  double SegmentObjects::calc_minkowski_distance(geometry_msgs::Position start, geometry_msgs::Position cup);
  void HeadCamCallback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher marker_pub_;
  PointCloudC::Ptr camera_pointCloud_;
};
}  // namespace barbot
