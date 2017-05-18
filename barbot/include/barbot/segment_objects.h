#include "perception/crop.h"
#include "perception/segmentation.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "barbot/MoveToPerception.h"
typedef barbot::MoveToPerception MpServ;


namespace barbot {
class SegmentObjects {
 public:
  SegmentObjects(const ros::Publisher& marker_pub);
  bool ServiceCallback(MpServ::Request  &req,
      MpServ::Response &res);
  void HeadCamCallback(const sensor_msgs::PointCloud2& msg);

 private:
  ros::Publisher marker_pub_;
  sensor_msgs::PointCloud2 camera_pointCloud_;
};
}  // namespace barbot
