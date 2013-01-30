#include "pcl/io/pcd_io.h"

#include "Demo.h"


// initializes the viewer and shows the first time stamped cloud
Demo::Demo(const std::string &title,
  const std::list<TSCloud *> &laserClouds,
  const MyCloud::Ptr &robotCloud)
  : myViewer(title), 
    myLaserClouds(laserClouds),
    myRobotCloud(robotCloud),
    currIndex(0)
{
  // initialize viewer
  myViewer.setBackgroundColor(0,0,0);
  myViewer.addCoordinateSystem(170.0);
  myViewer.initCameraParameters();

  std::cout << "Press LEFT and RIGHT arrow keys to cycle through clouds"
    << std::endl;

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

  std::list<TSCloud *>::const_iterator it = myLaserClouds.begin();
  for (int i = 0; i < currIndex; i++) it++;
  myViewer.addPointCloud((*it)->getCloud(), "laser");
  std::cout << currIndex << ") " << (*it)->getTimeStamp() << std::endl;

  myViewer.removeShape("sphere");
  //myViewer.addSphere((*myRobotCloud)[currIndex], 10);
}


// handles keyboard events captured by the demo viewer
void viewerKeyHandler(const pcl::visualization::KeyboardEvent &ke,
    		      void *cookie)
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

