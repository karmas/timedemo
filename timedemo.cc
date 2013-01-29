#include <iostream>

#include "fileUtils.h"
#include "Demo.h"



int main(int argc, char* argv[])
{
  // check whether a path containing the point cloud files is given
  if (!checkSourceDir(argc, argv)) {
    std::cout << "Please provide the source directory "
      	      << "containing the point cloud files" << std::endl;
    return 1;
  }

  // directory exists so read the files into memory
  std::string sourceDir(argv[1]);
  sourceDir += "/";
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> laserClouds;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr robotCloud;
  readCloudFiles(sourceDir, laserClouds, robotCloud);

  // now display the demo
  Demo demo("TIME STAMP DEMO", laserClouds, robotCloud);

  return 0;
}
