#include "pcl/io/pcd_io.h"

#include "Demo.h"


// initializes the viewer and shows the first time stamped cloud
Demo::Demo(const std::string &title,
  std::list<RobotInfo *> &robotInfos,
  std::list<TSCloud *> &laserClouds)
  : myViewer(title), 
    myRobotInfos(robotInfos),
    myLaserClouds(laserClouds),
    myCurrIndex(0),
    myAggregateMode(false)
{
  // initialize viewer
  myViewer.setBackgroundColor(0,0,0);
  myViewer.addCoordinateSystem(170.0);
  myViewer.initCameraParameters();

  displayControls();

  myViewer.registerKeyboardCallback(viewerKeyHandler, (void *)this);
  showCurrIndex();
  myViewer.spin();
}

void Demo::incrementIndex()
{
  myCurrIndex++;
  if (myCurrIndex >= myLaserClouds.size()) myCurrIndex = 0;
}

void Demo::decrementIndex()
{
  myCurrIndex--;
  if (myCurrIndex < 0) myCurrIndex = myLaserClouds.size() - 1;
}

void Demo::showCurrIndex()
{
  // remove previous robot position
  myViewer.removeAllShapes();
  // remove previous cloud
  myViewer.removeAllPointClouds();

  std::ostringstream os;

  // find iterators to current robot position and laser point cloud
  std::list<RobotInfo *>::iterator rit = myRobotInfos.begin();
  std::list<RobotInfo *>::iterator prev_rit = myRobotInfos.begin();
  std::list<TSCloud *>::iterator lit = myLaserClouds.begin();
  for (int i = 0; i < myCurrIndex; i++) {
    if (myAggregateMode) {
      os.str("");
      os << i;
      myViewer.addSphere((*rit)->point, 10.0,
	  (*rit)->point.r, (*rit)->point.g, (*rit)->point.b,
	  "robot" + os.str());
      myViewer.addPointCloud((*lit)->getCloud(), "laser" + os.str());
    }
    rit++; lit++;
    if (i != 0) prev_rit++;
  }

  // deghost laser positions near other robots by using the positions of
  // those other robots from the previous time step
  if (rit != myRobotInfos.begin())
    markRegion((*prev_rit)->point, 200, (*lit)->getCloud());

  os.str("");
  os << myCurrIndex;
  // display the robot location as a sphere of same color
  myViewer.addSphere((*rit)->point, 10.0,
		     (*rit)->point.r, (*rit)->point.g, (*rit)->point.b,
		     "robot" + os.str());
  myViewer.addPointCloud((*lit)->getCloud(), "laser" + os.str());

  std::cout << myCurrIndex << ") " << (*rit)->timeStamp << std::endl;
  std::cout << myCurrIndex << ") " << (*rit)->robotName << std::endl;
  //std::cout << myCurrIndex << ") " << (*lit)->getTimeStamp() << std::endl;
}

// switch aggregate mode on and off
void Demo::switchAggregateMode()
{
  if (!myAggregateMode)
    std::cout << "Aggregate mode displays all clouds from first time stamp"
      << std::endl;
  myAggregateMode = !myAggregateMode;
}

// show the keyboard buttons and what they do
void Demo::displayControls()
{
  const size_t keyColSpace = 20;
  std::cout << std::setw(keyColSpace) << std::left << "KEY" 
    << "FUNCTION" << std::endl;
  std::cout << std::setw(keyColSpace) << "Left arrow key" 
    << "Show older cloud" << std::endl;
  std::cout << std::setw(keyColSpace) << "Rigth arrow key" 
    << "Show newer cloud" << std::endl;
  std::cout << std::setw(keyColSpace) << "'A' key" 
    << "Alternate between aggregate and single cloud mode" << std::endl;
  std::cout << std::setw(keyColSpace) << "'S' key" 
    << "Start over" << std::endl;
  std::cout << std::endl;
}

// set the index
void Demo::setCurrIndex(size_t n) { myCurrIndex = n; }

// check if the given pt is withing a sphere of center and radius by
// calculating the distance between the two points
static bool inRegion(const MyPoint &center, int radius,
    		    const MyPoint &pt)
{
  return sqrt(pow(center.x - pt.x, 2) + 
              pow(center.y - pt.y, 2) +
	      pow(center.z - pt.z, 2)) < 350 ? true : false;
}

// mark all points in given cloud which is in the spherical region
// with given center and radius
void Demo::markRegion(const MyPoint &center, int radius,
    		      MyCloud::Ptr cloud)
{
  for (size_t i = 0; i < cloud->size(); i++) {
    if (inRegion(center, radius, (*cloud)[i])) {
      (*cloud)[i].r = 0;
      (*cloud)[i].g = 255;
      (*cloud)[i].b = 0;
    }
  }
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
  else if (ke.getKeySym() == "a" && ke.keyDown()) {
    demo->switchAggregateMode();
  }
  else if (ke.getKeySym() == "s" && ke.keyDown()) {
    demo->setCurrIndex(0);
    demo->showCurrIndex();
  }
}

