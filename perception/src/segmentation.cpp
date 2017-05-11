#include "perception/segmentation.h"

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
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

namespace perception {
    void SegmentSurface(PointCloudC::Ptr cloud, pcl::PointIndices::Ptr indices) {
        pcl::PointIndices indices_internal;
        pcl::SACSegmentation<PointC> seg;
        seg.setOptimizeCoefficients(true);
        // Search for a plane perpendicular to some axis (specified below).
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // Set the distance to the plane for a point to be an inlier.
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(cloud);
        // Make sure that the plane is perpendicular to Z-axis, 10 degree tolerance.
        Eigen::Vector3f axis;
        axis << 0, 0, 1;
        seg.setAxis(axis);
        seg.setEpsAngle(pcl::deg2rad(10.0));

        // coeff contains the coefficients of the plane:
        // ax + by + cz + d = 0
        pcl::ModelCoefficients coeff;
        seg.segment(indices_internal, coeff);

        double distance_above_plane;
        ros::param::param("distance_above_plane", distance_above_plane, 0.005);

        // Build custom indices that ignores points above the plane.
        for (size_t i = 0; i < cloud->size(); ++i) {
            const PointC& pt = cloud->points[i];
            float val = coeff.values[0] * pt.x + coeff.values[1] * pt.y +
                        coeff.values[2] * pt.z + coeff.values[3];
            if (val <= distance_above_plane) {
                indices->indices.push_back(i);
            }
        }
        // or
        // *indices = indices_internal;

        if (indices->indices.size() == 0) {
            ROS_ERROR("Unable to find surface.");
            return;
        }
        ROS_INFO("Segmented Cloud");
    }

    void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, geometry_msgs::Pose* pose, geometry_msgs::Vector3* dimensions) {
        PointC min_pcl;
        PointC max_pcl;
        pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
        pose->orientation.w = 1;
        pose->position.x = (max_pcl.x + min_pcl.x) / 2;
        pose->position.y = (max_pcl.y + min_pcl.y) / 2;
        pose->position.z =  (max_pcl.z + min_pcl.z) / 2;
        dimensions->x = max_pcl.x - min_pcl.x;
        dimensions->y = max_pcl.y - min_pcl.y;
        dimensions->z = max_pcl.z - min_pcl.z;
    }

    void SegmentSurfaceObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr surface_indices, std::vector<pcl::PointIndices>* object_indices) {
        pcl::ExtractIndices<PointC> extract;
        pcl::PointIndices::Ptr above_surface_indices(new pcl::PointIndices());
        extract.setInputCloud(cloud);
        extract.setIndices(surface_indices);
        extract.setNegative(true);
        extract.filter(above_surface_indices->indices);

        double cluster_tolerance;
        int min_cluster_size, max_cluster_size;
        ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
        ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
        ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);
        // Extracts clustered objects
        pcl::EuclideanClusterExtraction<PointC> euclid;
        euclid.setInputCloud(cloud);
        euclid.setIndices(above_surface_indices);
        euclid.setClusterTolerance(cluster_tolerance);
        euclid.setMinClusterSize(min_cluster_size);
        euclid.setMaxClusterSize(max_cluster_size);
        euclid.extract(*object_indices);
        // Find the size of the smallest and the largest object,
        // where size = number of points in the cluster
        // size_t min_size = std::numeric_limits<size_t>::max();
        // size_t max_size = std::numeric_limits<size_t>::min();
        // for (size_t i = 0; i < object_indices->size(); ++i) {
        //     pcl::PointIndices index = (*object_indices)[i];
        //     size_t curClusterSize = index.indices.size();
        //     if(curClusterSize > max_size) {
        //         max_size = curClusterSize;
        //     }
        //     if(curClusterSize < min_size) {
        //         min_size = curClusterSize;
        //     }
        // }
        // ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
        //         object_indices->size(), min_size, max_size);
    }

    Segmenter::Segmenter(const ros::Publisher& surface_points_pub, const ros::Publisher& marker_pub)
        : surface_points_pub_(surface_points_pub), marker_pub_(marker_pub){}

    void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud);
        ROS_INFO("CLOUD REC");
        pcl::PointIndices::Ptr table_inliers(new pcl::PointIndices());
        SegmentSurface(cloud, table_inliers);
        PointCloudC::Ptr segmented_cloud(new PointCloudC);

        // Extract subset of original_cloud into segmented_cloud:
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(table_inliers);
        extract.filter(*segmented_cloud);

        // SEND MARKER FOR TABLE
        visualization_msgs::Marker table_marker;
        table_marker.ns = "table";
        table_marker.header.frame_id = "base_link";
        table_marker.type = visualization_msgs::Marker::CUBE;
        GetAxisAlignedBoundingBox(segmented_cloud, &table_marker.pose, &table_marker.scale);
        table_marker.color.r = 1;
        table_marker.color.a = 0.8;
        marker_pub_.publish(table_marker);

        // SEGMENT TOP OBJECTS
        std::vector<pcl::PointIndices> object_indices;
        SegmentSurfaceObjects(segmented_cloud, table_inliers, &object_indices);
        for (size_t i = 0; i < object_indices.size(); ++i) {
            // Reify indices into a point cloud of the object.
            pcl::PointIndices::Ptr index(new pcl::PointIndices);
            *index = object_indices[i];
            PointCloudC::Ptr object_cloud(new PointCloudC());
            // TODO: fill in object_cloud using indices
            for(size_t j = 0; j < index->indices.size(); ++j) {
                object_cloud->points.push_back(segmented_cloud->points[index->indices[j]]);
            }
            // Publish a bounding box around it.
            visualization_msgs::Marker object_marker;
            object_marker.ns = "objects";
            object_marker.id = i;
            object_marker.header.frame_id = "base_link";
            object_marker.type = visualization_msgs::Marker::CUBE;
            GetAxisAlignedBoundingBox(object_cloud, &object_marker.pose,
                                        &object_marker.scale);
            object_marker.color.g = 1;
            object_marker.color.a = 0.9;
            marker_pub_.publish(object_marker);
        }
        // PUBLISH TOP CLOUD
        extract.setNegative(true);
        sensor_msgs::PointCloud2 msg_out;
        extract.filter(*segmented_cloud);
        pcl::toROSMsg(*segmented_cloud, msg_out);
        surface_points_pub_.publish(msg_out);
        ROS_INFO("Published Point and Marker");        
    }
}  // namespace perception