cmake_minimum_required(VERSION 2.6)

# source files
set (SRC fileUtils.cc cloudUtils.cc Demo.cc timedemo.cc)

# program name
set (PROG timedemo)

# find PCL library
find_package (PCL 1.3 REQUIRED
  COMPONENTS common io visualization)

# PCL headers location
include_directories (${PCL_INCLUDE_DIRS})
# OpenCV headers location
include_directories(/usr/include/opencv)

# PCL libraries location
link_directories (${PCL_LIBRARY_DIRS})

add_definitions (-Wno-deprecated-declarations -g)
# make the executable
add_executable (${PROG} ${SRC})

# link the libraries
# Required OpenCV libraries
set(OpenCV_LIBS opencv_core opencv_flann)
target_link_libraries (${PROG}
  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES}
  ${PCL_VISUALIZATION_LIBRARIES}
  ${OpenCV_LIBS}
)
