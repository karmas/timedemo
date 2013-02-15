#include <dirent.h>
#include <fnmatch.h>
#include <stdlib.h>

#include "pcl/io/pcd_io.h"

#include "fileUtils.h"

#define FIX

void error(const std::string &msg, bool showExtraInfo = false)
{
  std::cout << msg << std::endl;
  if (showExtraInfo)
    std::cout << strerror(errno) << std::endl;
}

// exit with error message
void errorExit(const std::string &msg, bool showExtraInfo)
{
  error(msg, showExtraInfo);
  std::exit(1);
}

// free up the allocation by scandir()
void freeNameList(struct dirent **namelist, int n)
{
  while (n--) free(namelist[n]);
  free(namelist);
}

// return 0 if it is not a directory or one of the dots
int dirFilter(const struct dirent *dir)
{
  if (strcmp(dir->d_name, ".") == 0 ||
      strcmp(dir->d_name, "..") == 0) return 0;
}

// Returns 1 if name of the given entry matches the glob pattern.
// For now this pattern is any filename with .pcd extension.
int pcdFileFilter(const struct dirent* entry)
{
  const char *pcdFilePattern = "*.pcd";
  return (fnmatch(pcdFilePattern, entry->d_name, FNM_CASEFOLD) == 0) 
    ? 1 : 0;
}

// Returns 1 if it is a laser point cloud file
int laserFileFilter(const struct dirent* entry)
{
  return (pcdFileFilter(entry) &&
          strncmp(entry->d_name, "path", 4) != 0) 
    	  ? 1 : 0;
}

// needed for qsort function to compare 2 numbers
int numCompare(const void *v1, const void *v2)
{
  int i1 = *(int *)v1;
  int i2 = *(int *)v2;
  return i1 - i2;
}




// useful for appending '/' to strings to make directory name if it
// is missing
void appendSlash(std::string &name)
{
  // already has slash so it is fine
  if (name[name.size()-1] == '/') return;
  else name += "/";
}

// Check whether a directory name is given on the command line and whether
// it exists.
// Returns true if the directory exists.
void checkSourceDir(int c, char* v[])
{
  // directory name not provided
  if (c < 2)
    errorExit("Please provide the point clouds source directory!!!");

  // check given directory for existence
  DIR *dir = opendir(v[1]);
  if (!dir)
    errorExit("Error with: " + std::string(v[1]), true);

  // close the directory
  closedir(dir);
}

// Goes in the given directory.
// Remembers paths to all the subdirectories in it.
void getSubDirs(const std::string &sourceDir,
    		std::vector<std::string> &subDirs)
{
  struct dirent **namelist(NULL);
  int n(0);

  // error occurred while scanning into source directory
  if ((n = scandir(sourceDir.c_str(), &namelist, dirFilter, versionsort)) 
      == -1)
    errorExit("Error scanning: " + sourceDir, true);
  else if (n == 0)
    errorExit("Empty: " + sourceDir);

  // remember the subdirectory paths
  for (int i = 0; i < n; i++) {
    subDirs.push_back(sourceDir + namelist[i]->d_name + "/");
  }

  // free the namelist
  freeNameList(namelist, n);
}

static MyPoint downRobotFix(const MyPoint &pt)
{
  MyPoint newPt = pt;

  float xFix = 265;
  float zFix = 109;

  newPt.x += xFix;
  // y axis is the same for laser and robot so no need to fix y values
  newPt.z += zFix;
  return newPt;
}

static MyPoint upRobotFix(const MyPoint &pt)
{
  MyPoint newPt = pt;

  float xFix = 252;
  float zFix = 134;

  newPt.x += xFix;
  // y axis is the same for laser and robot so no need to fix y values
  newPt.z += zFix;
  return newPt;
}

// load laser point cloud files from given directory
void readClouds(const std::string &dir,
		     std::list<RobotInfo *> &robotInfos,
    		     std::list<TSCloud *> &laserClouds,
		     MyPoint (*robotPositionFix) (const MyPoint &pt))
{
  struct dirent **namelist(NULL);
  int n(0);

  // load the robot cloud file
  MyCloud robotCloud;
  pcl::io::loadPCDFile(dir + "path.pcd", robotCloud);

  // error scanning directory for laser cloud files
  if ((n = scandir(dir.c_str(), &namelist, laserFileFilter, versionsort)) 
      == -1) {
    error("Error scanning: " + dir, true);
    return;
  }
  // no laser cloud files
  else if (n == 0) {
    error("No laser point clouds in: " + dir);
    return;
  }

  // load and add point clouds to list
  for (int i = 0; i < n; i++) {
    MyCloud cloud;
    int timeStamp = atoi(namelist[i]->d_name);
    pcl::io::loadPCDFile(dir + namelist[i]->d_name, cloud);
    laserClouds.push_back(new TSCloud(cloud.makeShared(), timeStamp));
    // also use the time stamp value to create a robot position info
    // and add to robot path list
#ifdef FIX
    robotInfos.push_back(new RobotInfo(
	  robotPositionFix(robotCloud[i]), 
	  timeStamp, 0.0));
#else
    robotInfos.push_back(new RobotInfo(robotCloud[i], timeStamp, 0.0));
#endif
  }

  freeNameList(namelist, n);
}

// comparision function for the list of time stamped clouds
bool compareRobotInfo(RobotInfo *t1, RobotInfo *t2)
{
  return t1->timeStamp <= t2->timeStamp;
}

// comparision function for the list of time stamped clouds
bool compareTSCloud(TSCloud *t1, TSCloud *t2)
{
  return t1->getTimeStamp() <= t2->getTimeStamp();
}

// Go through each subdirectory and read all the laser point clouds
// into a single list.
void readTimeStampClouds(const std::vector<std::string> &subDirs,
    		   	 std::list<RobotInfo *> &robotInfos,
    			 std::list<TSCloud *> &laserClouds)
{
  for (int i = 0; i < subDirs.size(); i++) {
    if (subDirs[i].find("bot110") != std::string::npos)
      readClouds(subDirs[i], robotInfos, laserClouds, downRobotFix);
    else if (subDirs[i].find("bot111") != std::string::npos)
      readClouds(subDirs[i], robotInfos, laserClouds, upRobotFix);
  }

#ifdef DEBUG
  std::list<TSCloud *>::const_iterator it;
  for (it = laserClouds.begin(); it != laserClouds.end(); it++)
    std::cout << (*it)->getTimeStamp() << std::endl;

#endif

  // now sort the list according to time stamp value
  robotInfos.sort(compareRobotInfo);
  laserClouds.sort(compareTSCloud);

#ifdef DEBUG
  error("sorted version");

  for (it = laserClouds.begin(); it != laserClouds.end(); it++)
    std::cout << (*it)->getTimeStamp() << std::endl;
#endif
}




TSCloud::TSCloud(MyCloud::Ptr cloud, int timeStamp)
  : myCloud(cloud), myTimeStamp(timeStamp)
{
}
