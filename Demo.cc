#include "pcl/io/pcd_io.h"

#include "Demo.h"


const int Demo::INVALID = -99;

// initializes the viewer and shows the first time stamped cloud
Demo::Demo(const std::string &title,
  std::vector<RobotType *> &robotTypes,
  std::vector<RobotInfo *> &robotInfos,
  std::vector<TSCloud *> &laserClouds)
  : myViewer(title), 
    myRobotTypes(robotTypes),
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

  printColorInfo();
  displayControls();

  myViewer.registerKeyboardCallback(viewerKeyHandler, (void *)this);
  printTitle("Current Display information");
  showCurrIndex();
  myViewer.spin();
}

// find the index which represents the other robot that occurs
// right before the current robot
void Demo::findPrevRobotIndex()
{
  int i = myCurrIndex - 1;
  while (i >= 0 && myRobotInfos[i]->robotTypeIndex == 
      		  myRobotInfos[myCurrIndex]->robotTypeIndex) {
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
  printTitle("Keyboard Controls");
  std::string keys[] = {
    "LEFT arrow",
    "RIGHT arrow",
    "A",
    "S"
  };
  std::string keysDesc[] = {
    "Show older cloud",
    "Show newer cloud",
    "Alternate aggregate and single mode",
    "Start over"
  };

  const int rows = sizeof(keys)/sizeof(keys[0]);
  const int padding = 2;
  const int keysColWidth = longestLen(keys, rows);
  const int keysDescColWidth = longestLen(keysDesc, rows);
  const char colSeparator = '|';

  for (int i = 0; i < rows; i++) {
    std::cout << std::setw(padding) << ""
      << std::setw(keysColWidth) << keys[i]
      << std::setw(padding) << "" << colSeparator
      << std::setw(padding) << ""
      << std::setw(keysDescColWidth) << keysDesc[i]
      << std::endl;
  }
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
    << "name = " 
    << myRobotTypes[myRobotInfos[myCurrIndex]->robotTypeIndex]->robotName
    << " | "
    << "prev = " << myPrevRobotIndex
    << std::endl;
}

// color the points that are in the vicinity of other robots
void Demo::markOtherRobot()
{
  if (myPrevRobotIndex == INVALID) return;

  MyCloud::Ptr currLaserCloud = myLaserClouds[myCurrIndex]->getCloud();
  RobotInfo *prevRobotInfo = myRobotInfos[myPrevRobotIndex];
  RobotType *prevRobotType = myRobotTypes[prevRobotInfo->robotTypeIndex];

  for (size_t i = 0; i < currLaserCloud->size(); i++) {
    if (inRegion(prevRobotInfo->point, myRobotRadius, 
	  (*currLaserCloud)[i])) {
      (*currLaserCloud)[i].r = prevRobotType->regionColor.r;
      (*currLaserCloud)[i].g = prevRobotType->regionColor.g;
      (*currLaserCloud)[i].b = prevRobotType->regionColor.b;
    }
  }

}

// print out some information on the various colors in the point cloud
void Demo::printColorInfo()
{
  printTitle("Meaning of Point colors (maybe invalid)");
  const char colSeparator = '|';
  const int padding = 2;
  const int nInfo = 3;

  // create information arrays
  std::string infoType[nInfo] = {
    " laser",
    " center",
    " region"
  };
  const int rows = myRobotTypes.size() * nInfo;
  std::string colorFor[rows];
  // this should eventually be made dynamic
  std::string colorName[6] = {
    " brown",
    " red",
    " green",
    " lightblue",
    " darkblue",
    " yellow"
  };
  int k;

  for (size_t i = 0; i < myRobotTypes.size(); i++) {
    for (size_t j = 0; j < nInfo; j++) {
      k = i*nInfo + j; 
      colorFor[k] = myRobotTypes[i]->robotName + infoType[j];
    }
  }

  const int descColWidth = longestLen(colorFor, rows);
  const int colorColWidth = longestLen(colorName, rows);

  for (size_t i = 0; i < myRobotTypes.size() * nInfo; i++) {
    std::cout << std::setw(padding) << ""
      << std::setw(descColWidth) << colorFor[i]
      << std::setw(padding) << ""
      << colSeparator
      << std::setw(padding) << ""
      << std::setw(colorColWidth) << colorName[i] << std::endl;
  }
}

// print a section title
void Demo::printTitle(const std::string &title)
{
  static const int leftMargin = 8;
  const std::string horizontalBar = std::string(title.length(), '-');

  std::cout << std::setw(leftMargin) << ""
    << horizontalBar << std::endl
    << std::setw(leftMargin) << ""
    << title << std::endl 
    << std::setw(leftMargin) << ""
    << horizontalBar << std::endl;
}

// return the length of the longest string
int Demo::longestLen(std::string array[], int n)
{
  int maxVal = array[0].length();
  for (int i = 1; i < n; i++) {
    if (array[i].length() > maxVal)
      maxVal = array[i].length();
  }
  return maxVal;
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

