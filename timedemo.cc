#include <iostream>

#include "fileUtils.h"
#include "Demo.h"


int main(int argc, char* argv[])
{
  // check whether a path containing the point cloud files is given
  checkSourceDir(argc, argv);

  // check for subdirectories and remember path names
  std::string sourceDir(argv[1]);
  appendSlash(sourceDir);
  std::vector<std::string> subDirs;
  getSubDirs(sourceDir, subDirs);
  
  // get laser point clouds from subdirectories
  // and points from robot point clouds
  std::list<TSCloud *> laserClouds;
  std::list<RobotInfo *> robotInfos;
  readTimeStampClouds(subDirs, robotInfos, laserClouds);

  // now display the demo
  Demo demo("TIME STAMP DEMO", robotInfos, laserClouds);

  return 0;
}
