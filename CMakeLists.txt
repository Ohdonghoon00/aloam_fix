cmake_minimum_required(VERSION 2.8.3)
project(fix_aloam)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  # std_msgs
  # visualization_msgs
  roscpp
  # rospy
  # rosbag
  # image_transport
  # cv_bridge
  tf
)
# find_package( Boost REQUIRED COMPONENTS 
# timer )
 

find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS} 
  # ${Boost_INCLUDE_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs sensor_msgs roscpp
  DEPENDS EIGEN3  
  INCLUDE_DIRS include
)

add_library(aloam_library
src/PublishLidarData.cpp 
src/ScanRegistration.cpp
src/LaserMapping.cpp
src/utils.cpp)

target_link_libraries(aloam_library glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES})

# set(OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/build)

add_executable(main_test src/main_test.cpp)
target_link_libraries(main_test glog ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${CERES_LIBRARIES} aloam_library)

# set_target_properties(main_test PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_DIR})
