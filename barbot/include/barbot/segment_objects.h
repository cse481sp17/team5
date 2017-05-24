
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
//   double calc_minkowski_distance(geometry_msgs::Point start, geometry_msgs::Point end);
  void HeadCamCallback(const sensor_msgs::PointCloud2& msg);
  bool checkIfCup(geometry_msgs::Vector3 scale);
 private:
  ros::Publisher marker_pub_;
  PointCloudC::Ptr camera_pointCloud_;
};
}  // namespace barbot
