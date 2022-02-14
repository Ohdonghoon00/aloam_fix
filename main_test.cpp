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

#include "common.h"

using namespace std;

// Extrinsic parameter rig - lidar
const Eigen::Matrix4d LidarToRig = To44RT(lidar2rig_pose);

LidarData lidar_data;
DataBase DB;

int ReadVIOdata(const std::string Path, DataBase *db)
{
    std::ifstream Rovins2PoseFile(Path, std::ifstream::in);

    if(!Rovins2PoseFile.is_open()){
        std::cout << " Rovins2Pose file failed to open " << std::endl;
        return EXIT_FAILURE;
    }

    std::string line;
    int line_num = 0;

    while(std::getline(Rovins2PoseFile, line) && ros::ok()){

        std::string value;
        std::vector<std::string> values;        

        // values[0]        -> camera fidx
        // values[1] ~ [6]  -> lidar pose (vio result)
        
        std::stringstream ss(line);
        while(std::getline(ss, value, ' '))
            values.push_back(value);

        Vector6d pos;
        pos << std::stod(values[1]), std::stod(values[2]), std::stod(values[3]), std::stod(values[4]), std::stod(values[5]), std::stod(values[6]);


        db->VIOLidarPoses.push_back(pos);

        line_num++;
    }

    Rovins2PoseFile.close();
    return EXIT_SUCCESS;
}

int ReadLidardata(const std::string Path, const std::string LidarBinaryPath, DataBase* db)
{
    std::ifstream LidarcsvFile(Path, std::ifstream::in);

    if(!LidarcsvFile.is_open()){
        std::cout << " Lidar csv file failed to open " << std::endl;
        return EXIT_FAILURE;
    }
    
    // Read Lidar timestamp.csv
    Eigen::Matrix4d LidarRotation;
    std::string Lidarcsvline;
    int Lidarline_num(1);

    while(std::getline(LidarcsvFile, Lidarcsvline) && ros::ok())
    {
        if(Lidarline_num == 1){
            Lidarline_num++;
            continue;
        }
        std::string value;
        std::vector<std::string> values;
        
        // values[0] -> First Seq Timestamp (ns)
        // values[1] -> Last Seq Timestamp (ns)
        // values[2] -> Fidx
        // values[3] -> Num pts
        // values[4] -> Date
        
        std::stringstream ss(Lidarcsvline);
        while(std::getline(ss, value, ','))
            values.push_back(value);
        int fidx = std::stoi(values[2]);
        
        // Binary Data Path
        std::stringstream Lidar_binary_path;
        Lidar_binary_path <<    LidarBinaryPath << std::setfill('0') << 
                                std::setw(5) << fidx << ".bin";
        
        std::ifstream ifs(Lidar_binary_path.str(), std::ifstream::in);
        
        if (!ifs.is_open()){
            std::cout << "bin file failed to open: " << std::endl;
            return EXIT_FAILURE;
        }        
                
        // Read Binary data file
        int PointNum = 0;
        ifs.read((char*) &PointNum, sizeof(int));
        
        float Points[PointNum * 3];
        ifs.read((char*) &Points, sizeof(float) * PointNum * 3);
        
        Eigen::Matrix3Xd Points_(3, PointNum);
        for(int i = 0; i < PointNum; i++){
            Points_(0, i) = static_cast<double>(Points[3 * i]);
            Points_(1, i) = static_cast<double>(Points[3 * i + 1]);
            Points_(2, i) = static_cast<double>(Points[3 * i + 2]);
        }
        
        db->LidarPoints.push_back(Points_);

        ifs.close();
        Lidarline_num++;
    }    
    LidarcsvFile.close();
    return EXIT_SUCCESS;
}

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
    std::cout << " Finish !! " << std::endl;
    std::cout << " Start Publish !!! " << std::endl;

    // Scan Registration node




    // Laser Mapping node

    // parameter.h -> 공통으로 쓰이는 파라미터들 정리
    // ScanRegistration.h -> 여기서 쓰이는 함수들?
    // LaserMapping.h
    // Common.h
    // lidarFactor.hpp

    return 0;
}