#include <iostream>
#include <cassert>

#include <opencv/cv.h>

#include "fileUtils.h"
#include "Demo.h"


template<typename T>
void list2vector(std::list<T> &src, std::vector<T> &dest)
{
  typename std::list<T>::iterator it;
  for (it = src.begin(); it != src.end(); it++)
    dest.push_back(*it);
}

void nearestNeighbors(std::vector<TSCloud *> &laserCloudsVector);


int main(int argc, char* argv[])
{
  // check whether a path containing the point cloud files is given
  checkSourceDir(argc, argv);

  // check for subdirectories and remember path names
  std::string sourceDir(argv[1]);
  appendSlash(sourceDir);
  std::vector<std::string> subDirs;
  getSubDirs(sourceDir, subDirs);

  std::vector<RobotType *> robotTypes;
  createRobotTypes(subDirs, robotTypes);
  
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

  nearestNeighbors(laserCloudsVector);

  // now display the demo
  Demo demo("TIME STAMP DEMO", robotTypes,
            robotInfosVector, laserCloudsVector);

  return 0;
}

// Use Flann library's nearest neighbor search
void nearestNeighbors(std::vector<TSCloud *> &laserCloudsVector)
{
  // first create a single point cloud with all the laser data
  MyCloud cloud;
  for (size_t i = 0; i < laserCloudsVector.size(); i++)
    cloud += *(laserCloudsVector[i]->getCloud());

  // Below is an example of initializing using raw data
//  cv::Mat features = (cv::Mat_<float>(2, 3) << 1, 1, 1,  5, 5, 5);

  // create a matrix data set for use with opencv
  cv::Mat features;  //(cloud.size(), 3, CV_32FC1);

  // fill the data set with points from the point cloud
  cv::Mat aPoint;  //(1, 3, CV_32FC1);
  for (size_t i = 0; i < cloud.size(); i++) {
    aPoint = (cv::Mat_<float>(1, 3) << cloud[i].x, cloud[i].y, cloud[i].z);
    features.push_back(aPoint);
  }

  assert(cloud.size() == features.rows);

  // LinearIndexParams iterates through all neighbors one by one
  // remembering the ones with low distances
  cv::flann::LinearIndexParams params;

  // create a search index for a dataset
  cv::flann::Index index(features, // dataset
      		         params);  // specifies type of index

  // Below are iteration techniques to access each element
  //
//  for (cv::MatConstIterator_<float> it = features.begin<float>();
//      it != features.end<float>(); it++)
//    std::cout << *it << endl;

//  for (int i = 0; i < features.cols; i++)
//    std::cout << features.at<float>(i) << std::endl;


  std::vector<float> query;  // find neighbors for this point
  std::vector<int> indices;  // indices of neighbors
  std::vector<float> dists;  // distance to neighbors
  const int knn = 3;
  // number of times to check, higher value gives better
  // precision at the cost of running time
  const int nChecks = 32;

  query.push_back(1);
  query.push_back(2);
  query.push_back(3);
  index.knnSearch(query, indices, dists, knn,
      		  cv::flann::SearchParams(nChecks));

  std::cout << "Nearest neighbors are: " << std::endl;
  for (size_t i = 0; i < indices.size(); i++)
    std::cout << i+1 << ") " << features.row(indices[i])
      << std::endl;
}
