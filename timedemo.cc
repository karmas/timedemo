#include <dirent.h>
#include <fnmatch.h>
#include <stdlib.h>
#include <iostream>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"

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

// Store the point cloud files in memory.
// First looks through the directory for pcd files.
void readCloudFiles(const std::string &sourceDir)
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
    // load each file that matched the filter
    for (int i = 0; i < n; i++) {
      std::string fileName(namelist[i]->d_name);
      std::string fullName = sourceDir + "/" + fileName;
      pcl::PointCloud<pcl::PointXYZRGBA> cloud;

      // file could not be loaded
      if (pcl::io::loadPCDFile(fullName, cloud) == -1)
	std::cout << "Could not load: " << fileName << std::endl;
      else
	std::cout << "Loaded: " << fileName << std::endl;
    }
  }
  else {
    std::cout << "Error while scanning " << sourceDir << std::endl;
  }

  // free up the namelist found out by scanning the directory
  for (int j = 0; j < n; j++) {
    free(namelist[j]);
  }
  free(namelist);
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
  readCloudFiles(sourceDir);


  return 0;
}
