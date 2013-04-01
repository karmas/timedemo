#include <cassert>

#include <opencv/cv.h>

#include "cloudUtils.h"


typedef cv::flann::L2<float> Distance;// distance functor


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

  const int nClusters = 3;
  cv::Mat centers(nClusters, 3, CV_32FC1);	// cluster centers
  cvflann::KMeansIndexParams kMeansParams;

  Distance d;
  cv::flann::hierarchicalClustering<Distance>(features, centers, 
      kMeansParams, d);

  std::cout << "Centers are: " << std::endl;
  for (int i = 0; i < centers.rows; i++) {
    for (int j = 0; j < centers. cols; j++)
      std::cout << centers.at<float>(i,j) << ",";
    std::cout << std::endl;
  }
}

// Performs hierarchical clustering on the full cloud using opencv.
// The matrix holds 3 columns for the 3 co-ordinates.
// Color information is not stored in the matrix.
MyCloud::Ptr cluster(std::vector<TSCloud *> &clouds)
{
  // create a copy of the clouds
  MyCloud fullCloud;
  for (size_t i = 0; i < clouds.size(); i++)
    fullCloud += *(clouds[i]->getCloud());

  // create opencv matrix which is a copy of the point cloud
  // note: color info is not saved only co-ordinate info
  cv::Mat features(fullCloud.width, 3, CV_32FC1);

  // fill matrix with points from the cloud
  for (int i = 0; i < fullCloud.size(); i++) {
    features.at<float>(i, 0) = fullCloud.points[i].x;
    features.at<float>(i, 1) = fullCloud.points[i].y;
    features.at<float>(i, 2) = fullCloud.points[i].z;
  }

#ifdef CHECK_MATRIX
  for (int i = 0; i < 3; i++) {
    std::cout << fullCloud.points[i] << std::endl;
    std::cout << features.at<float>(i, 0) << ","
      << features.at<float>(i, 1) << ","
      << features.at<float>(i, 2)
      << std::endl;
  }
#endif

  // perform clustering
  const int clustersWanted = 5;
  cv::Mat centers(clustersWanted, 3, CV_32FC1);
  cvflann::KMeansIndexParams params;
  const int clustersFound = 
    cv::flann::hierarchicalClustering<Distance>(features, centers, params);

  // create a point cloud of clusters to return
  MyCloud::Ptr clustersCloud(new MyCloud);
  MyPoint point;
  for (int i = 0; i < clustersFound; i++) {
#ifdef SHOW_CLUSTERS 
    std::cout << i+1 << ") "
      << centers.at<float>(i, 0) << ", "
      << centers.at<float>(i, 1) << ", "
      << centers.at<float>(i, 2) << std::endl;
#endif
    point.x = centers.at<float>(i, 0);
    point.y = centers.at<float>(i, 1);
    point.z = centers.at<float>(i, 2);
    clustersCloud->push_back(point);
  }

  return clustersCloud;
}
