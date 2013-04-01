#ifndef CLOUD_UTILS_H
#define CLOUD_UTILS_H

#include "fileUtils.h"


void nearestNeighbors(std::vector<TSCloud *> &laserCloudsVector);
MyCloud::Ptr cluster(std::vector<TSCloud *> &clouds);

template<typename T>
void list2vector(std::list<T> &src, std::vector<T> &dest)
{
  typename std::list<T>::iterator it;
  for (it = src.begin(); it != src.end(); it++)
    dest.push_back(*it);
}


#endif
