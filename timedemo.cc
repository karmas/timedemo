#include <dirent.h>
#include <fnmatch.h>
#include <stdlib.h>
#include <iostream>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/visualization/cloud_viewer.h"


// Check whether a directory name is given on the command line and whether
// it exists.
// Returns true if the directory exists.
bool checkSourceDir(int c, char* v[])
{
  // directory name not provided
  if (c < 2)
    return false;

  DIR *dir = opendir(v[1]);
  // directory does not exist
  if (!dir) {
    std::cout << v[1] << " directory does not exist!!!" << std:: endl;
    return false;
  }
  closedir(dir);
  
  return true;
}

// Returns 1 if name of the given entry matches the glob pattern.
// For now this pattern is any filename with .pcd extension.
int pcdFileFilter(const struct dirent* entry)
{
  const char *pcdFilePattern = "*.pcd";
  return (fnmatch(pcdFilePattern, entry->d_name, FNM_CASEFOLD) == 0) 
    ? 1 : 0;
}

// needed for qsort function to compare 2 numbers
int numCompare(const void *v1, const void *v2)
{
  int i1 = *(int *)v1;
  int i2 = *(int *)v2;
  return i1 - i2;
}

// The parameter unsortedList points to an array of files which are present
// the directory. This function fills sortedFileNames vector so that the
// file names are sorted by time stamp values. The first entry of this list
// is the name for the robot location cloud file.
void getSortedFileNames(struct dirent **unsortedList, int n,
    			std::vector<std::string> &sortedFileNames)
{
  const char SEPARATOR = '_';
  std::string timeStamp("");
  std::string ipAddr("");
  std::string suffix(".pcd");

  int t = 0;
  // will hold integer values of the time stamps
  int timeStampValues[n-1];

  // first get all the time stamp values from the file names
  for (int i = 0; i < n; i++) {
    timeStamp = unsortedList[i]->d_name;

    int timeStampIndex;
    // time stamp is there so a laser point cloud file
    if ((timeStampIndex = timeStamp.find(SEPARATOR)) != std::string::npos) {
      timeStamp = timeStamp.substr(timeStampIndex+1,
	  timeStamp.find('.', timeStampIndex) - timeStampIndex - 1);
      timeStampValues[t++] = atoi(timeStamp.c_str());
    }
    // no time stamp means robot location point cloud
    else {
      // use this to get the ip address
      ipAddr = timeStamp.substr(0, timeStamp.rfind('.'));
    }
  }

  // now sort the array of time stamp values
  qsort(timeStampValues, n-1, sizeof(int), numCompare);

  // add the robot cloud file as the first one
  sortedFileNames.push_back(ipAddr + suffix);
  
  // fill up the sorted file names array
  for (int j = 0; j < n-1; j++) {
    std::ostringstream os;
    os << timeStampValues[j];
    sortedFileNames.push_back(ipAddr + SEPARATOR + os.str() + suffix);
  }
}

// First looks through the directory for pcd files.
// Then creates an array of file names sorted by time stamp values.
// Stores this cloud files in vector.
void readCloudFiles(const std::string &sourceDir,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &laserClouds,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &robotCloud)
{
  std::cout << "Reading from source directory: " << sourceDir << std::endl;

  struct dirent **namelist;
  int n = 			// number of valid files
    scandir(sourceDir.c_str(),	// source directory to scan
            &namelist,		// pointers to valid files in source
	    pcdFileFilter,	// filter applied to files
	    alphasort);		// valid files are sorted using this

  // no error occurred scanning the directory
  if (n != -1) {
    // get an sorted order of file names
    std::vector<std::string> sortedFileNames;
    getSortedFileNames(namelist, n, sortedFileNames);

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    std::string fullName(sourceDir + sortedFileNames[0]);

    // read the robot locations cloud file
    // file could not be loaded
    if (pcl::io::loadPCDFile(fullName, cloud) == -1)
      std::cout << "Could not load: " << sortedFileNames[0] << std::endl;
    else {
      robotCloud = cloud.makeShared();
      std::cout << "Loaded: " << sortedFileNames[0] << std::endl;
    }

    // read the laser clouds according to time stamp order
    for (int i = 1; i < sortedFileNames.size(); i++) {
      fullName = sourceDir + sortedFileNames[i];

      // file could not be loaded
      if (pcl::io::loadPCDFile(fullName, cloud) == -1)
	std::cout << "Could not load: " << sortedFileNames[i] << std::endl;
      else {
	// store a copy of the cloud in the list
	laserClouds.push_back(cloud.makeShared());

	std::cout << "Loaded: " << sortedFileNames[i] << std::endl;
      }
    }
  }
  else {
    std::cout << "Error while scanning " << sourceDir << std::endl;
    return;
  }

  // free memory allocated by various routines
  for (int j = 0; j < n; j++) {
    free(namelist[j]);
  }
  free(namelist);
}


// The time stamp demo program class
class Demo {
public:
  Demo(const std::string &title,
    const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &laserClouds,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &robotCloud);
  void incrementIndex();
  void decrementIndex();
  void showCurrIndex();

private:
  pcl::visualization::PCLVisualizer myViewer;
  const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &myLaserClouds;
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &myRobotCloud;
  int currIndex;
};


// handles keyboard events captured by the demo viewer
void viewerKeyHandler(const pcl::visualization::KeyboardEvent &ke,
    		      void *cookie = NULL)
{
  Demo *demo = static_cast<Demo *>(cookie);

  if (ke.getKeySym() == "Right" && ke.keyDown()) {
    demo->incrementIndex();
    demo->showCurrIndex();
  }
  else if (ke.getKeySym() == "Left" && ke.keyDown()) {
    demo->decrementIndex();
    demo->showCurrIndex();
  }
}

// initializes the viewer and shows the first time stamped cloud
Demo::Demo(const std::string &title,
  const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &laserClouds,
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &robotCloud)
  : myViewer(title), 
    myLaserClouds(laserClouds),
    myRobotCloud(robotCloud),
    currIndex(0)
{
  // initialize viewer
  myViewer.setBackgroundColor(0,0,0);
  myViewer.addCoordinateSystem(170.0);
  myViewer.initCameraParameters();

  myViewer.registerKeyboardCallback(viewerKeyHandler, (void *)this);
  showCurrIndex();
  myViewer.spin();
}

void Demo::incrementIndex()
{
  currIndex++;
  if (currIndex >= myLaserClouds.size()) currIndex = 0;
}

void Demo::decrementIndex()
{
  currIndex--;
  if (currIndex < 0) currIndex = myLaserClouds.size() - 1;
}

void Demo::showCurrIndex()
{
  myViewer.removeAllPointClouds();

  std::cout << "curr time index: " << currIndex << std::endl;

  myViewer.addPointCloud(myLaserClouds[currIndex], "laser");
}


int main(int argc, char* argv[])
{
  // check whether a path containing the point cloud files is given
  if (!checkSourceDir(argc, argv)) {
    std::cout << "Please provide the source directory "
      	      << "containing the point cloud files" << std::endl;
    return 1;
  }

  // directory exists so read the files into memory
  std::string sourceDir(argv[1]);
  sourceDir += "/";
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> laserClouds;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr robotCloud;
  readCloudFiles(sourceDir, laserClouds, robotCloud);

  // now display the demo
  Demo demo("TIME STAMP DEMO", laserClouds, robotCloud);

  return 0;
}
