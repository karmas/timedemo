#ifndef FILEUTILS_H
#define FILEUTILS_H

#include <list>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

typedef pcl::PointXYZRGB MyPoint;
typedef pcl::PointCloud<MyPoint> MyCloud;

class TSCloud {
public:
  TSCloud(MyCloud::Ptr cloud, int timeStamp);
  MyCloud::Ptr getCloud() { return myCloud; }
  int getTimeStamp() { return myTimeStamp; }
private:
  MyCloud::Ptr myCloud;
  int myTimeStamp;

  // disable copying
  TSCloud(const TSCloud &);
  TSCloud operator=(const TSCloud &);
};

void errorExit(const std::string &msg, bool showExtraInfo = false);
void checkSourceDir(int c, char* v[]);
int pcdFileFilter(const struct dirent* entry);
int numCompare(const void *v1, const void *v2);
void appendSlash(std::string &name);
void getSubDirs(const std::string &sourceDir,
    		std::vector<std::string> &subDirs);
void getSortedFileNames(struct dirent **unsortedList, int n,
    			std::vector<std::string> &sortedFileNames);
void readCloudFiles(const std::string &sourceDir,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &laserClouds,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &robotCloud);
void readTimeStampClouds(const std::vector<std::string> &subDirs,
    			 std::list<TSCloud *> &tsClouds);

#endif
