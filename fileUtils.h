#ifndef FILEUTILS_H
#define FILEUTILS_H

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"


bool checkSourceDir(int c, char* v[]);
int pcdFileFilter(const struct dirent* entry);
int numCompare(const void *v1, const void *v2);
void getSortedFileNames(struct dirent **unsortedList, int n,
    			std::vector<std::string> &sortedFileNames);
void readCloudFiles(const std::string &sourceDir,
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &laserClouds,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &robotCloud);


#endif
