#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <cloud_common.h>

#define NODE_NAME "cloud_downsampled"
#define PUB_NODE_NAME "cloud_filtered"
class CloudDownsamper : public CloudOperator
{
public:
    CloudDownsamper(ros::NodeHandle &nh) :
        cloud_pub_(nh.advertise<sensor_msgs::PointCloud2>(NODE_NAME, 1))
    {}

    void operate()
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_input_pcl;
        pcl::PointCloud<pcl::PointXYZ> cloud_downsampled_pcl;

        pcl::fromROSMsg(cloud_input_ros_, cloud_input_pcl);

        pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
        voxelSampler.setInputCloud(cloud_input_pcl.makeShared());
        voxelSampler.setLeafSize(0.01f, 0.01f, 0.01f);
        voxelSampler.filter(cloud_downsampled_pcl);
   
        pcl::toROSMsg(cloud_downsampled_pcl, cloud_filterd_ros_);
    }

    void publish()
    {
        cloud_pub_.publish(cloud_filterd_ros_);
    }

protected:
    ros::Publisher cloud_pub_;
    sensor_msgs::PointCloud2 cloud_filterd_ros_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;

    CloudOperationHandler handler(nh, new CloudDownsamper(nh), PUB_NODE_NAME);
    ros::spin();

    return 0;
}
