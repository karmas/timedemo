#include "pcl/io/pcd_io.h"

#include "Demo.h"


const int Demo::INVALID = -99;

// initializes the viewer and shows the first time stamped cloud
Demo::Demo(const std::string &title,
  std::vector<RobotInfo *> &robotInfos,
  std::vector<TSCloud *> &laserClouds)
  : myViewer(title), 
    myRobotInfos(robotInfos),
    myLaserClouds(laserClouds),
    myCurrIndex(0),
    myPrevRobotIndex(INVALID),
    myAggregateMode(false),
    myRobotRadius(200)
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

// find the index which represents the other robot that occurs
// right before the current robot
void Demo::findPrevRobotIndex()
{
  int i = myCurrIndex - 1;
  while (i >= 0 && myRobotInfos[i]->robotName == 
      		  myRobotInfos[myCurrIndex]->robotName) {
    i--;
  }

  if (i < 0) myPrevRobotIndex = INVALID;
  else myPrevRobotIndex = i;
}

void Demo::incrementIndex()
{
  myCurrIndex++;
  if (myCurrIndex >= myLaserClouds.size()) myCurrIndex = 0;
  findPrevRobotIndex();
}

void Demo::decrementIndex()
{
  myCurrIndex--;
  if (myCurrIndex < 0) myCurrIndex = myLaserClouds.size() - 1;
  findPrevRobotIndex();
}

void Demo::showCurrIndex()
{
  RobotInfo *currRobotInfo = myRobotInfos[myCurrIndex];
  TSCloud *currLaserCloud = myLaserClouds[myCurrIndex];

  // remove previous robot position
  myViewer.removeAllShapes();
  // remove previous cloud
  myViewer.removeAllPointClouds();

  std::ostringstream os;
  os.str("");
  os << myCurrIndex;

  markOtherRobot();
  
  // display the robot location as a sphere of same color
  myViewer.addSphere(currRobotInfo->point, 10.0,
		     currRobotInfo->point.r, 
		     currRobotInfo->point.g, 
		     currRobotInfo->point.b,
		     "robot" + os.str());
  myViewer.addPointCloud(currLaserCloud->getCloud(),
      			 "laser" + os.str());
  printCurrIndexInfo();
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
void Demo::resetIndex() 
{ 
  myCurrIndex = 0; 
  findPrevRobotIndex();
}

// check if the given pt is withing a sphere of center and radius by
// calculating the distance between the two points
static bool inRegion(const MyPoint &center, int radius,
    		    const MyPoint &pt)
{
  return sqrt(pow(center.x - pt.x, 2) + 
              pow(center.y - pt.y, 2) +
	      pow(center.z - pt.z, 2)) < 350 ? true : false;
}


// show current index information on the command line
void Demo::printCurrIndexInfo()
{
  std::cout << "index = " << myCurrIndex << " | "
    << "timestamp = " << myRobotInfos[myCurrIndex]->timeStamp << " ms | "
    << "name = " << myRobotInfos[myCurrIndex]->robotName << " | "
    << "prev = " << myPrevRobotIndex
    << std::endl;
}

// color the points that are in the vicinity of other robots
void Demo::markOtherRobot()
{
  if (myPrevRobotIndex == INVALID) return;

  MyCloud::Ptr currLaserCloud = myLaserClouds[myCurrIndex]->getCloud();
  RobotInfo *prevRobotInfo = myRobotInfos[myPrevRobotIndex];

  for (size_t i = 0; i < currLaserCloud->size(); i++) {
    if (inRegion(prevRobotInfo->point, myRobotRadius, 
	  (*currLaserCloud)[i])) {
      (*currLaserCloud)[i].r = 0;
      (*currLaserCloud)[i].g = 100;
      (*currLaserCloud)[i].b = 0;
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
    demo->resetIndex();
    demo->showCurrIndex();
  }
}

