#ifndef DEMO_H
#define DEMO_H

#include "pcl/visualization/cloud_viewer.h"


// The time stamp demo program class
class Demo {
public:
  Demo(const std::string &title,
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &laserClouds,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &robotCloud);
  void incrementIndex();
  void decrementIndex();
  void showCurrIndex();

private:
  pcl::visualization::PCLVisualizer myViewer;
  const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &myLaserClouds;
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &myRobotCloud;
  int currIndex;
};


void viewerKeyHandler(const pcl::visualization::KeyboardEvent &ke,
    		      void *cookie = NULL);


#endif
