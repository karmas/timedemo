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
  /*
  for (int i = 0; i < subDirs.size(); i++)
    std::cout << subDirs[i] << std::endl;
    */
  
  // get laser point clouds and store in time stamp order
  std::list<TSCloud *> tsClouds;
  readTimeStampClouds(subDirs, tsClouds);

  /*

  // now display the demo
  Demo demo("TIME STAMP DEMO", laserClouds, robotCloud);
  */

  return 0;
}
