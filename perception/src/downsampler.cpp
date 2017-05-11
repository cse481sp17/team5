#include "perception/downsampler.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"


typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
    Downsampler::Downsampler(const ros::Publisher& pub) : pub_(pub) {}

    void Downsampler::Callback(const sensor_msgs::PointCloud2& msg) {
        PointCloudC::Ptr cloud(new PointCloudC());
        pcl::fromROSMsg(msg, *cloud);
        PointCloudC::Ptr downsampled_cloud(new PointCloudC());
        pcl::VoxelGrid<PointC> vox;
        vox.setInputCloud(cloud);
        double voxel_size;
        ros::param::param("voxel_size", voxel_size, 0.01);
        vox.setLeafSize(voxel_size, voxel_size, voxel_size);
        vox.filter(*downsampled_cloud);
        sensor_msgs::PointCloud2 msg_out;
        pcl::toROSMsg(*downsampled_cloud, msg_out);
        pub_.publish(msg_out);
        ROS_INFO("Published Point to downsampled_cloud");
    }
}

