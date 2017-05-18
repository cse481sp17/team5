#include "ros/ros.h"
#include "barbot/segment_objects.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle nh;
    ros::Publisher marker_pub =
        nh.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    barbot::SegmentObjects segobjs(marker_pub);
    ros::Subscriber sub =
        nh.subscribe("cloud_in", 1, &barbot::SegmentObjects::HeadCamCallback, &segobjs);
    ros::ServiceServer runPercepService =
        nh.advertiseService("move_to_perception", &barbot::SegmentObjects::ServiceCallback, &segobjs);
    ros::spin();
    return 0;
}
