#include "pcl/io/pcd_io.h"

#include "Demo.h"


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

  std::cout << "showing time index: " << currIndex << std::endl;

  myViewer.addPointCloud(myLaserClouds[currIndex], "laser");

  myViewer.removeShape("sphere");
  myViewer.addSphere((*myRobotCloud)[currIndex], 10);
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

