#ifndef DEMO_H
#define DEMO_H

#include "pcl/visualization/cloud_viewer.h"

// Demo class is responsible for displaying point clouds on a viewer.
// Pressing the LEFT and RIGHT arrow keys will cycle backward or forward
// through the time stamped point clouds. It maintains an index into the
// vector which stores the laser clouds. This index identifies the
// currently shown laser cloud. This same index can be used to get the
// robot location point which is displayed as a sphere.
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
