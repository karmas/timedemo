#include <iostream>

#include "fileUtils.h"
#include "Demo.h"

template<typename T>
void list2vector(std::list<T> &src, std::vector<T> &dest)
{
  typename std::list<T>::iterator it;
  for (it = src.begin(); it != src.end(); it++)
    dest.push_back(*it);
}

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

  // convert the lists to vectors for faster access operations
  std::vector<TSCloud *> laserCloudsVector;
  std::vector<RobotInfo *> robotInfosVector;
  list2vector<TSCloud *>(laserClouds, laserCloudsVector);
  list2vector<RobotInfo *>(robotInfos, robotInfosVector);

  // now display the demo
  Demo demo("TIME STAMP DEMO", robotInfosVector, laserCloudsVector);

  return 0;
}
