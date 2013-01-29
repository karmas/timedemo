#include <dirent.h>
#include <fnmatch.h>
#include <stdlib.h>

#include "pcl/io/pcd_io.h"

#include "fileUtils.h"

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

