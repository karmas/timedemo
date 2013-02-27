#ifndef FILEUTILS_H
#define FILEUTILS_H

#include <list>

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

typedef pcl::PointXYZRGB MyPoint;
typedef pcl::PointCloud<MyPoint> MyCloud;

// class for time stamped point cloud
class TSCloud {
public:
  TSCloud(MyCloud::Ptr cloud, int timeStamp,
      int index);
  MyCloud::Ptr getCloud() { return myCloud; }
  int getTimeStamp() { return myTimeStamp; }
private:
  MyCloud::Ptr myCloud;
  int myTimeStamp;
  int myRobotTypeIndex;

  // disable copying
  TSCloud(const TSCloud &);
  TSCloud operator=(const TSCloud &);
};

// group robot position, heading and timestamp
struct RobotInfo {
  RobotInfo(const MyPoint pt, int ts, double h, int index)
    : point(pt), timeStamp(ts), th(h), robotTypeIndex(index) { }
  MyPoint point;
  int timeStamp;
  double th;	// heading in degrees
  int robotTypeIndex;
};

// group type information of robot
struct RobotType {
  RobotType(const std::string &name, const MyPoint &color) 
    : robotName(name), regionColor(color) {}
  std::string robotName;
  MyPoint regionColor;
};

void errorExit(const std::string &msg, bool showExtraInfo = false);
void checkSourceDir(int c, char* v[]);
int pcdFileFilter(const struct dirent* entry);
int numCompare(const void *v1, const void *v2);
void appendSlash(std::string &name);
void getSubDirs(const std::string &sourceDir,
    		std::vector<std::string> &subDirs);
void readTimeStampClouds(const std::vector<std::string> &subDirs,
    		   	 std::list<RobotInfo *> &robotInfos,
    			 std::list<TSCloud *> &laserClouds);
std::string pathToRobotName(const std::string &dir);
void createRobotTypes(std::vector<std::string> &subDirs, 
    		      std::vector<RobotType *> &robotTypes);

#endif
