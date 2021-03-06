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
#include <ros/ros.h>

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

    ros::init(argc, argv, "main_test");
    ros::NodeHandle nh;
    // std::string data_dir = "/home/multipleye/Dataset/201014_skt_lobby_day_lidar/";
    std::string data_dir, result_dir;
    nh.getParam("data_dir", data_dir);
    nh.getParam("result_dir", result_dir);

    // for visualize
    ros::Publisher pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    ros::Publisher pubMappingPose = nh.advertise<nav_msgs::Odometry>("/mapping_Odom", 100);
    ros::Publisher pubMappingPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 100);
    nav_msgs::Path mappingPath;

    ros::Publisher pubVIOPose = nh.advertise<nav_msgs::Odometry>("/VIO_Odom", 100);
    ros::Publisher pubVIOPath = nh.advertise<nav_msgs::Path>("/VIO_path", 100);
    nav_msgs::Path VIOPath;    

    // Save Mapping Pose
    std::ofstream MappingPoseFile;
    MappingPoseFile.open(result_dir);

    // std::string data_dir = argv[1];
    
    ///////////// VIO pose data /////////////
    std::cout << " Load VIO Data ... " << std::endl;
    std::string VIOPoses_lidarframes = data_dir + "VIOLidarPoses_lidarframes.txt";
    std::cout <<VIOPoses_lidarframes << std::endl;
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
    while(LidarFrameNum < DB.LidarPoints.size() && ros::ok()){

        std::cout << " LidarFrame Num : " << LidarFrameNum << " @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ " << std::endl;
        //////////////////// Scan Registration ///////////////////////
        
        std::vector<Eigen::Vector3d> TotalPointCloud = Mat3XdToVec3d(DB.LidarPoints[LidarFrameNum]);

        // Remove useless points
        ScanRegistration.RemoveClosedPointCloud(&TotalPointCloud);
        ScanRegistration.RemoveNaNFromPointCloud(&TotalPointCloud);

        ScanRegistration.cloudSize = TotalPointCloud.size();

        // Sort Points by Scan Line
        
        ScanRegistration.DividePointsByChannel(TotalPointCloud);

        // Total lasercloud and point distance
        
        std::vector<double> PointRange;
        ScanRegistration.SetPointCloudAndDistance(&PointRange);

        // Caculate Curvature and Mark Occluded and Parallel Points    
        ScanRegistration.CalculateCurvature(PointRange);
        ScanRegistration.MarkOccludedPoints(PointRange);

        // sort Lidar points to edge and plane
        ScanRegistration.DividePointsByEdgeAndPlanePoints();

        // print corner and flat num
        std::cout << "Total laserCloud num : " << ScanRegistration.laserCloud.size() << std::endl;
        std::cout << "cornerPointsSharp num : " << ScanRegistration.cornerPointsSharp.size() << std::endl;
        std::cout << "cornerPointsLessSharp num : " << ScanRegistration.cornerPointsLessSharp.size() << std::endl;
        std::cout << "surfPointsFlat num : " << ScanRegistration.surfPointsFlat.size() << std::endl;
        std::cout << "surfPointsLessFlat num : " << ScanRegistration.surfPointsLessFlat.size() << std::endl;        

        ///////////////////// Laser Mapping ////////////////////////////
        
        LaserMapping.topcl(ScanRegistration);
        LaserMapping.loadOdomData(DB.VIOLidarPoses[LidarFrameNum]); // get ROVINS VIO pose
        LaserMapping.transformAssociateToMap(); 

            /////////// original code ///////////
            
            LaserMapping.getSurroundPointsfromCube();
            LaserMapping.surroundPoints2Map();
            LaserMapping.downSizeCurrentScan();

            LaserMapping.optimizePose();
            LaserMapping.transformUpdate();

            LaserMapping.currentScanToCube();
            LaserMapping.surroundMapDownSize();

            // Visualize
            ros::Time timestampNow = ros::Time::now();
            if (LidarFrameNum % 20 == 0)
                LaserMapping.visualizePointCloud(pubLaserCloud, timestampNow);
        
        
            ///////////////////// Test /////////////////////////////////
            // LaserMapping.downSizeCurrentScan();
            // LaserMapping.recentScan2Map();
            
            // LaserMapping.optimizePose();
            // LaserMapping.transformUpdate();

            // LaserMapping.setRecentlyMap();
            // LaserMapping.recentlyMapDownSize();
    
            // // Visualize
            // LaserMapping.visuzlize(pubLaserCloud, timestampNow);	

            /////////////////////////////////////////////////////////////

        
        LaserMapping.visualizePose(pubMappingPose, pubMappingPath, mappingPath, timestampNow, MappingPoseFile);
        LaserMapping.visualizePose(pubVIOPose, pubVIOPath, VIOPath, timestampNow, DB.VIOLidarPoses[LidarFrameNum]);


        // tf
        LaserMapping.transform(timestampNow);

        LidarFrameNum++;
    }

    return 0;
}