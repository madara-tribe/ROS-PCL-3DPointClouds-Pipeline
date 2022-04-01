#include <stdio.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

class Saver
{
 public:
  void save2ply(std::string path, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
      pcl::io::savePLYFileBinary(path, *cloud);
  }
  void save2pcd(std::string path2, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud){
      pcl::io::savePCDFile(path2, *cloud, true); // Binary format
        //pcl::io::savePCDFile( "cloud.pcd", *cloud, false ); // ASCII format
  }

 private:
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
};
