#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cloud_common.h>
#define NODE_NAME "cloud_loader"

class CloudLoadingHandler
{
public:
    CloudLoadingHandler(ros::NodeHandle &nh, const std::string &pcd_file_name):
        loader_(CloudLoader(nh, "cloud_raw", pcd_file_name))
    {}

    void operate()
    {
        loader_.load();
        loader_.convertPCLtoROS();
    }

    void publish()
    {
        loader_.publish();
    }

private:
    CloudLoader loader_;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    std::string pcd_file_name = "data/table_scene_lms400.pcd";
    pnh.getParam("pcd", pcd_file_name);

    CloudLoadingHandler handler(nh, pcd_file_name);
    handler.operate();

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        handler.publish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
