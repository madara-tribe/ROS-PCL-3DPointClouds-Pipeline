#include <iostream>
#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/io/pcd_io.h>
#include <sensor_msgs/image_encodings.h>

#define RGB_TOPIC_NAME "/camera/color/image_raw"
#define POINT_TOPIC_NAME "/camera/depth/color/points"
#define RGB_DIR "./rgb/"
#define PCD_DIR "./pcd/"


class SavePCD {
private:
  ros::NodeHandle _nh;
  ros::Subscriber _sub1, _sub2;
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  int save_count;
public:
  SavePCD() : save_count(0) {
    //* subscribe ROS topics
    _sub1 = _nh.subscribe (RGB_TOPIC_NAME, 1,  &SavePCD::image_cb, this);
    system("mkdir rgb");
    //ROS_INFO ("Listening for incoming data on" + rgb_topic_name + "topic..." );
    _sub2 = _nh.subscribe (POINT_TOPIC_NAME, 1,  &SavePCD::points_cb, this);
    system("mkdir pcd");
    //ROS_INFO ("Listening for incoming data on topic..." );
  }
  ~SavePCD() {}

  //* get points
  void points_cb( const sensor_msgs::PointCloud2ConstPtr& cloud ){
    if ((cloud->width * cloud->height) == 0)
      return;
    pcl::fromROSMsg (*cloud, input_cloud);
  }

  //* show color img and save color img + point cloud
  void image_cb( const sensor_msgs::ImageConstPtr& msg ){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg);

    //* show color img
    cv::Mat color_img = cv_ptr->image;
    cv::imshow( "color image", color_img );
    cv::waitKey(10);

    if ((input_cloud.width * input_cloud.height) == 0)
      return;

    //* save
    std::stringstream filename1;
    filename1 << save_count << ".png";
    cv::imwrite(RGB_DIR+filename1.str(), color_img );
    std::cout << RGB_DIR+ filename1.str() << " saved." << std::endl;
    std::stringstream filename2;
    filename2 << save_count << ".pcd";
    pcl::io::savePCDFileBinary(PCD_DIR+filename2.str(), input_cloud );
    std::cout << PCD_DIR+filename2.str() << " saved." << std::endl;
    save_count++;
    usleep( 300000 );
  }
};

int main( int argc, char** argv ){
  ros::init(argc,argv,"save_pcd");
  SavePCD spcd;
  ros::spin();

  return 1;
}
