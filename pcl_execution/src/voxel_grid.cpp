#include <ros/ros.h>
#include <iostream>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>

#define INPUT_PATH "data/table_scene_lms400.pcd"
#define OUTPUT_PCD_NAME "data/downsampled_table_scene_lms400.pcd"
#define OUTPUT_PLY_NAME "data/downsampled_table_scene_lms400.ply"

int main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read(INPUT_PATH, *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ")." << std::endl; 
  std::cout << "Loaded width "<< cloud_filtered->width << " height " << cloud_filtered->height << std::endl;
  pcl::PCDWriter writer;
  writer.write(OUTPUT_PCD_NAME, *cloud_filtered, 
         Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
  // https://docs.ros.org/en/hydro/api/pcl/html/classpcl_1_1PLYWriter.html#a9ab45e2e58bb9e564a1e8a98da9f95e0
  pcl::PLYWriter ply_writer;
  ply_writer.write(OUTPUT_PLY_NAME, *cloud_filtered, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false, false);
  
  return 0;
}
