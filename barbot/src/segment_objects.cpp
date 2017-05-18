#include "perception/crop.h"
#include "perception/segmentation.h"
#include "ros/ros.h"

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
        : marker_pub_(marker_pub){}
    bool ServiceCallback(MpServ::Request  &req, MpServ::Response &res) {
        ROS_INFO("Service call received of type: %s", req.perception);
        PointCloudC::Ptr cloudIn(new PointCloudC());
        cloudIn->is_dense = false;
        pcl::fromROSMsg(msg, *cloudIn);
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
        crop.setInputCloud(cloudIn);
        crop.setMin(min_pt);
        crop.setMax(max_pt);
        crop.filter(*cropped_cloud);

        std::vector<int> indices;
        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::removeNaNFromPointCloud(*cropped_cloud, *cloud, indices);

        pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
        SegmentSurface(cloud, table_inliers);
        PointCloudC::Ptr segmented_cloud(new PointCloudC);

        // // Extract subset of original_cloud into segmented_cloud:
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(table_inliers);
        extract.filter(*segmented_cloud);

        // // SEND MARKER FOR TABLE
        visualization_msgs::Marker table_marker;
        table_marker.ns = "table";
        table_marker.header.frame_id = "base_link";
        table_marker.type = visualization_msgs::Marker::CUBE;
        perception::segmentation::GetAxisAlignedBoundingBox(segmented_cloud, &table_marker.pose, &table_marker.scale);
        table_marker.color.r = 1;
        table_marker.color.a = 0.8;
        marker_pub_.publish(table_marker);

        std::vector<pcl::PointIndices> object_indices;
        std::vector<visualization_msgs::Marker> marker_vector;
        perception::segmentation::SegmentSurfaceObjects(cloud, table_inliers, &object_indices);

        for (size_t i = 0; i < object_indices.size(); ++i) {
        // Reify indices into a point cloud of the object.
            pcl::PointIndices::Ptr indice(new pcl::PointIndices);
            *indice = object_indices[i];
            PointCloudC::Ptr object_cloud(new PointCloudC());

            pcl::ExtractIndices<PointC> extract3;
            extract3.setInputCloud(cloud);
            extract3.setIndices(indice);
            extract3.setNegative(false);
            extract3.filter(*object_cloud);
            // Publish a bounding box around it.
            visualization_msgs::Marker object_marker;
            object_marker.ns = "objects";
            object_marker.id = i;
            object_marker.header.frame_id = "base_link";
            object_marker.type = visualization_msgs::Marker::CUBE;
            perception::segmentation::GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                                        &object_marker.scale);
            object_marker.color.b = 1;
            object_marker.color.g = 0.7;
            object_marker.color.a = 0.5;
            marker_vector.push_back(object_marker);
            marker_pub_.publish(object_marker);
        }

    }
    void SegmentObjects::HeadCamCallback(const sensor_msgs::PointCloud2& msg) {
        camera_pointCloud_ = *msg;
    }
}  // namespace barbot
