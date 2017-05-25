#include "perception/segmentation.h"
#include "perception/object.h"
#include "ros/ros.h"
#include "barbot/segment_objects.h"
#include "perception/box_fitter.h"
#include "shape_msgs/SolidPrimitive.h"
#include <string>
#include <math.h>
#include <std_msgs/Float64.h>

#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/filters/crop_box.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/common/angles.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/common/common.h"
#include "pcl/segmentation/extract_clusters.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

typedef barbot::MoveToPerception MpServ;

namespace barbot {
    SegmentObjects::SegmentObjects(const ros::Publisher& marker_pub)
        : marker_pub_(marker_pub), camera_pointCloud_(new PointCloudC()){}

    // double SegmentObjects::calc_minkowski_distance(geometry_msgs::Point start, geometry_msgs::Point end) {
    //     double aggreg = 0.0;
    //     aggreg += pow(abs(start.x-end.x), 2);
    //     aggreg += pow(abs(start.y-end.y), 2);
    //     aggreg += pow(abs(start.z-end.z), 2);
    //     return pow(aggreg, 1.0/2.0);
    // }

    bool SegmentObjects::checkIfCup(geometry_msgs::Vector3 scale) {
        bool x = scale.x > 0.095 && scale.x < 0.12;
        bool y = scale.y > 0.09 && scale.y < 0.12;
        bool z = scale.z > 0.12 && scale.y < 0.13;
        return x && y && z;
    }

    bool SegmentObjects::ServiceCallback(MpServ::Request  &req, MpServ::Response &res) {
        ROS_INFO("Service call received of type %s", req.perception.c_str());
        PointCloudC::Ptr cropped_cloud(new PointCloudC());
        double min_x, min_y, min_z, max_x, max_y, max_z;
        ros::param::param("crop_min_x", min_x, 0.3);
        ros::param::param("crop_min_y", min_y, -1.0);
        ros::param::param("crop_min_z", min_z, 0.5);
        ros::param::param("crop_max_x", max_x, 0.9);
        ros::param::param("crop_max_y", max_y, 1.0);
        ros::param::param("crop_max_z", max_z, 1.5);
        Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
        Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
        pcl::CropBox<PointC> crop;
        crop.setInputCloud(camera_pointCloud_);
        crop.setMin(min_pt);
        crop.setMax(max_pt);
        crop.filter(*cropped_cloud);

        std::vector<int> indices;
        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::removeNaNFromPointCloud(*cropped_cloud, *cloud, indices);

        geometry_msgs::Point finalPosition;
        // // SEND MARKER FOR TABLE
        if(req.perception == req.TABLE) {
            pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
            pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
            perception::SegmentSurface(cloud, table_inliers, coeff);
            PointCloudC::Ptr segmented_cloud(new PointCloudC);
            pcl::ExtractIndices<PointC> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(table_inliers);
            extract.filter(*segmented_cloud);
            visualization_msgs::Marker table_marker;
            table_marker.ns = "table";
            table_marker.header.frame_id = "base_link";
            table_marker.type = visualization_msgs::Marker::CUBE;
            shape_msgs::SolidPrimitive table_shape;
            PointCloudC::Ptr extract_out_table(new PointCloudC());
            geometry_msgs::Pose table_pose;
            perception::FitBox(*segmented_cloud, coeff, *extract_out_table, table_shape, table_pose);
            table_marker.pose = table_pose;
            if (table_shape.type == shape_msgs::SolidPrimitive::BOX) {
                table_marker.scale.x = table_shape.dimensions[0];
                table_marker.scale.y = table_shape.dimensions[1];
                table_marker.scale.z = table_shape.dimensions[2];
            }
            table_marker.pose.position.z -= table_marker.scale.z;
            table_marker.color.r = 1;
            table_marker.color.a = 0.8;
            marker_pub_.publish(table_marker);
            finalPosition.x = table_marker.pose.position.x;
            finalPosition.y = table_marker.pose.position.y;
            finalPosition.z = table_marker.pose.position.z;
        } else if (req.perception == req.CUP) {
            std::vector<perception::Object> objects;
            perception::SegmentTabletopScene(cloud, &objects);

            for (size_t i = 0; i < objects.size(); ++i) {
                const perception::Object& object = objects[i];
                if(checkIfCup(object.dimensions)) {
                    visualization_msgs::Marker object_marker;
                    object_marker.ns = "objects";
                    object_marker.id = i;
                    object_marker.header.frame_id = "base_link";
                    object_marker.type = visualization_msgs::Marker::CUBE;
                    object_marker.pose = object.pose;
                    object_marker.scale = object.dimensions;
                    object_marker.color.b = 1;
                    object_marker.color.a = 0.7;
                    marker_pub_.publish(object_marker);
                    finalPosition.x = object_marker.pose.position.x;
                    finalPosition.y = object_marker.pose.position.y;
                    finalPosition.z = object_marker.pose.position.z;
                    ROS_DEBUG("Object  %ld, x =  %f, y =  %f, z =  %f", i, finalPosition.x, finalPosition.y, finalPosition.z);
                    ROS_DEBUG("Object Scale %ld, x =  %f, y =  %f, z =  %f", i, object_marker.scale.x, object_marker.scale.y, object_marker.scale.z);
                }
            }
            
        }
        res.x = finalPosition.x;
        res.y = finalPosition.y;
        res.z = finalPosition.z;
        res.item = req.perception;
        ROS_INFO("sending back response: %s", res.item.c_str());
        return true;
    }
    void SegmentObjects::HeadCamCallback(const sensor_msgs::PointCloud2& msg) {
        pcl::fromROSMsg(msg, *camera_pointCloud_);
    }
}  // namespace barbot
