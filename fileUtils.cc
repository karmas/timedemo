#include <dirent.h>
#include <fnmatch.h>
#include <stdlib.h>

#include "pcl/io/pcd_io.h"

#include "fileUtils.h"

void error(const std::string &msg, bool showExtraInfo = false)
{
  std::cout << msg << std::endl;
  if (showExtraInfo)
    std::cout << strerror(errno) << std::endl;
}

// exit with error message
void errorExit(const std::string &msg, bool showExtraInfo)
{
  std::cout << msg << std::endl;
  if (showExtraInfo)
    std::cout << strerror(errno) << std::endl;
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

// Returns 1 if it is a robot location point cloud file
int robotFileFilter(const struct dirent* entry)
{
  return (strncmp(entry->d_name, "path", 4) == 0 && pcdFileFilter(entry)) 
    	  ? 1 : 0;
}

// Returns 1 if it is a laser point cloud file
int laserFileFilter(const struct dirent* entry)
{
  return (pcdFileFilter(entry) && !robotFileFilter(entry)) 
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

// load laser point cloud files from given directory
void readLaserClouds(const std::string &dir, std::list<TSCloud *> &tsClouds)
{
  struct dirent **namelist(NULL);
  int n(0);

  // error scanning directory
  if ((n = scandir(dir.c_str(), &namelist, laserFileFilter, NULL)) == -1) {
    error("Error scanning: " + dir, true);
    return;
  }
  // empty directory
  else if (n == 0) {
    error("No laser point clouds in: " + dir);
    return;
  }

  // load and add point clouds to list
  for (int i = 0; i < n; i++) {
    MyCloud cloud;
    pcl::io::loadPCDFile(dir + namelist[i]->d_name, cloud);
    tsClouds.push_back(new TSCloud(cloud.makeShared(),
	               atoi(namelist[i]->d_name)));
  }

  freeNameList(namelist, n);
}

// comparision function for the list of time stamped clouds
bool compareTSCloud(TSCloud *t1, TSCloud *t2)
{
  return t1->getTimeStamp() <= t2->getTimeStamp();
}

// Go through each subdirectory and read all the laser point clouds
// into a single list.
void readTimeStampClouds(const std::vector<std::string> &subDirs,
    			 std::list<TSCloud *> &tsClouds)
{
  for (int i = 0; i < subDirs.size(); i++) {
    readLaserClouds(subDirs[i], tsClouds);
  }

  std::list<TSCloud *>::const_iterator it;
  for (it = tsClouds.begin(); it != tsClouds.end(); it++)
    std::cout << (*it)->getTimeStamp() << std::endl;

  // now sort the list according to time stamp value
  tsClouds.sort(compareTSCloud);

  error("sorted version");

  for (it = tsClouds.begin(); it != tsClouds.end(); it++)
    std::cout << (*it)->getTimeStamp() << std::endl;
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
    // get rid of the extension
    timeStamp = timeStamp.substr(0, timeStamp.rfind('.'));

    // the laser point clouds
    if (timeStamp != "path") {
      timeStampValues[t++] = atoi(timeStamp.c_str());
    }
  }

  // now sort the array of time stamp values
  qsort(timeStampValues, n-1, sizeof(int), numCompare);

  // add the robot cloud file as the first one
  sortedFileNames.push_back("path" + suffix);
  
  // fill up the sorted file names array
  for (int j = 0; j < n-1; j++) {
    std::ostringstream os;
    os << timeStampValues[j];
    sortedFileNames.push_back(os.str() + suffix);
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
    // get a sorted order of file names
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
  freeNameList(namelist, n);
}


TSCloud::TSCloud(MyCloud::Ptr cloud, int timeStamp)
  : myCloud(cloud), myTimeStamp(timeStamp)
{
}
