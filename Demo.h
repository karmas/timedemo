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
    std::vector<RobotType *> &robotTypes,
    std::vector<RobotInfo *> &robotInfos,
    std::vector<TSCloud *> &laserClouds);
  void incrementIndex();
  void decrementIndex();
  void showCurrIndex();
  void switchAggregateMode();
  void displayControls();
  void printColorInfo();
  void resetIndex();
  void printCurrIndexInfo();

  static const int INVALID;

private:
  pcl::visualization::PCLVisualizer myViewer;
  std::vector<RobotType *> &myRobotTypes;
  std::vector<RobotInfo *> &myRobotInfos;
  std::vector<TSCloud *> &myLaserClouds;
  int myCurrIndex;
  int myPrevRobotIndex;	// previous robot in time stamp order
  bool myAggregateMode;
  int myRobotRadius; // in mm

  void findPrevRobotIndex();
  void markOtherRobot();
  void printTitle(const std::string &title);
  int longestLen(std::string array[], int n);
};


void viewerKeyHandler(const pcl::visualization::KeyboardEvent &ke,
    		      void *cookie = NULL);


#endif
