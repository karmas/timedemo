#ifndef DEMO_H
#define DEMO_H

#include "pcl/visualization/cloud_viewer.h"

#include "fileUtils.h"

// Demo class is responsible for displaying point clouds on a viewer.
// Pressing the LEFT and RIGHT arrow keys will cycle backward or forward
// through the time stamped point clouds. It maintains an index into the
// vector which stores the laser clouds. This index identifies the
// currently shown laser cloud. This same index can be used to get the
// robot location point which is displayed as a sphere.
class Demo {
public:
  Demo(const std::string &title,
    const std::list<RobotInfo *> &robotInfos,
    const std::list<TSCloud *> &laserClouds);
  void incrementIndex();
  void decrementIndex();
  void showCurrIndex();
  void switchAggregateMode();
  void displayControls();
  void setCurrIndex(size_t n);

private:
  pcl::visualization::PCLVisualizer myViewer;
  const std::list<RobotInfo *> &myRobotInfos;
  const std::list<TSCloud *> &myLaserClouds;
  int myCurrIndex;
  bool myAggregateMode;
};


void viewerKeyHandler(const pcl::visualization::KeyboardEvent &ke,
    		      void *cookie = NULL);


#endif
