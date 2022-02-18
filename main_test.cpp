#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <glog/logging.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "utils.h"
#include "common.h"
#include "ScanRegistration.h"
#include "PublishLidarData.h"
#include "LaserMapping.h"

using namespace std;



LidarData lidar_data;
DataBase DB;
ScanRegistration ScanRegistration;
LaserMapping LaserMapping;


int main(int argc, char** argv)
{
    std::string data_dir = argv[1];
    
    ///////////// VIO pose data /////////////
    std::cout << " Load VIO Data ... " << std::endl;
    std::string VIOPoses_lidarframes = data_dir + "VIOLidarPoses_lidarframes.txt";

    ReadVIOdata(VIOPoses_lidarframes, &DB);

    //////////// Undistortion Points ////////////////
    std::cout << " Load Lidar Data ... " << std::endl;
    std::string LidarcsvPath = data_dir + "lidar_timestamp.csv";
    std::string Lidar_binary_path = data_dir + "lidar2/";
    ReadLidardata(LidarcsvPath, Lidar_binary_path, &DB);

    std::cout << std::endl;
    std::cout << " Load Finish !! " << std::endl;
    std::cout << DB.LidarPoints.size() << std::endl;


    size_t LidarFrameNum = 0;
    while(LidarFrameNum < DB.LidarPoints.size()){

        std::cout << " LidarFrame Num : " << LidarFrameNum << " @@@@@@@@@@ " << std::endl;
        //////////////////// Scan Registration ///////////////////////
        
        std::vector<Eigen::Vector3d> TotalPointCloud = Mat3XdToVec3d(DB.LidarPoints[LidarFrameNum]);

        // Remove useless points
        ScanRegistration.RemoveClosedPointCloud(&TotalPointCloud);
        ScanRegistration.RemoveNaNFromPointCloud(&TotalPointCloud);

        ScanRegistration.cloudSize = TotalPointCloud.size();

        // Sort Points by Scan Line
        
        ScanRegistration.DividePointsByChannel(TotalPointCloud);

        // Total lasercloud and point distance
        std::vector<Eigen::Vector3d> laserCloud;
        std::vector<double> PointRange;
        ScanRegistration.SetPointCloudAndDistance(&laserCloud, &PointRange);

        // Caculate Curvature and Mark Occluded and Parallel Points    
        ScanRegistration.CalculateCurvature(PointRange);
        ScanRegistration.MarkOccludedPoints(laserCloud, PointRange);

        // sort Lidar points to edge and plane
        ScanRegistration.DividePointsByEdgeAndPlanePoints(laserCloud);

        // print corner and flat num
        std::cout << "Total laserCloud num : " << laserCloud.size() << std::endl;
        std::cout << "cornerPointsSharp num : " << ScanRegistration.cornerPointsSharp.size() << std::endl;
        std::cout << "cornerPointsLessSharp num : " << ScanRegistration.cornerPointsLessSharp.size() << std::endl;
        std::cout << "surfPointsFlat num : " << ScanRegistration.surfPointsFlat.size() << std::endl;
        std::cout << "surfPointsLessFlat num : " << ScanRegistration.surfPointsLessFlat.size() << std::endl;        

        ///////////////////// Laser Mapping ////////////////////////////






        LidarFrameNum++;
    }

    





    // parameter.h -> 공통으로 쓰이는 파라미터들 정리
    // ScanRegistration.h -> 여기서 쓰이는 함수들?
    // LaserMapping.h
    // Common.h
    // lidarFactor.hpp

    return 0;
}