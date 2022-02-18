#pragma once

#include "common.h"
#include "utils.h"


#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>


class ScanRegistration{

public:

    ScanRegistration( )
    {}

    // Sort
    bool comp(int i, int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
    bool SameLeaf(int i, int j) { return (LeafIds[i] < LeafIds[j]); }

    // DownsizeFiltering
    void getMinMax(std::vector< Eigen::Vector3d > &inCloud, Eigen::Vector3d &minp, Eigen::Vector3d &maxp);
    void DownSizeFiltering(Eigen::Vector3d &LeafSize, std::vector< Eigen::Vector3d > &InCloud, std::vector< Eigen::Vector3d > &OutCloud);

    // Remove Closed Points and Nan
    void RemoveClosedPointCloud(std::vector<Eigen::Vector3d> *pointcloud);
    void RemoveNaNFromPointCloud(std::vector<Eigen::Vector3d> *pointcloud);

    // Decide Channel of Points
    void DividePointsByChannel( const std::vector<Eigen::Vector3d>& laserPoints);


    void SetPointCloudAndDistance( 
                                    std::vector<Eigen::Vector3d> *laserCloud, 
                                    std::vector<double> *PointRange);
        
    void CalculateCurvature(const std::vector<double>& PointRange);


    void MarkOccludedPoints(const std::vector<Eigen::Vector3d>& laserCloud, 
                            const std::vector<double>& PointRange);

    // Sort Lidar Points by Corner and Flat Points
    void DividePointsByEdgeAndPlanePoints(const std::vector<Eigen::Vector3d>& laserCloud);

public:
    
    int N_SCANS = 16;
    int cloudSize = 0;
    std::vector<int> scanStartInd = std::vector<int>(N_SCANS, 0);
    std::vector<int> scanEndInd = std::vector<int>(N_SCANS, 0);

    float cloudCurvature[400000];
    int cloudSortInd[400000];
    int cloudNeighborPicked[400000];
    int cloudLabel[400000];

    std::vector<Eigen::Vector3d> cornerPointsSharp;
    std::vector<Eigen::Vector3d> cornerPointsLessSharp;
    std::vector<Eigen::Vector3d> surfPointsFlat;
    std::vector<Eigen::Vector3d> surfPointsLessFlat;

    // downsize leaf
    std::vector<int> LeafIds;

    float VerticalAngelRatio = 0;
    const size_t kMaxNumberOfPoints = 1e5;
    double LidarToPointsThres = 1.0;
    Eigen::Vector3d DownSizeLeafSize = Eigen::Vector3d(0.2, 0.2, 0.2);

    std::vector<Eigen::Matrix3Xd> laserCloudScans;

};




