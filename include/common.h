#pragma once

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


#include <pcl/point_types.h>


typedef pcl::PointXYZI PointType;
typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<double, 6, 1> Vector6d;






struct LidarData 
{
    std::vector<char> binary_data;
    int64_t timestamp_ns;
    int num_points, num_blocks;
    uint8_t num_channels;

    
    LidarData()
        : num_points(0), num_blocks(0), num_channels(0) {}
    virtual ~LidarData() {}
    
    
    float* points_ptr() const { return (float*) binary_data.data(); }
    uint8_t* intensities_ptr() const { return (uint8_t*) &binary_data[3 * num_points * sizeof(float)]; } // reflectivity
    uint8_t* azimuth_idxs_ptr() const { return intensities_ptr() + num_points; }
    float* azimuth_degs_ptr() const { return (float*) (azimuth_idxs_ptr() + num_points); }
};

struct DataBase
{
    std::vector<Vector6d> VIOLidarPoses;

    // std::vector<double> SlamKFtimestamps;
    // std::vector<Vector6d> SlamKFPoses;
    // std::vector<int> Slamcamidxs;

    std::vector<Eigen::Matrix3Xd> LidarPoints;

};

