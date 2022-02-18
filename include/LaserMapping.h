#pragma once

#include "common.h"
#include "utils.h"
#include "lidarFactor.hpp"
#include "ScanRegistration.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <mutex>
#include <queue>


class LaserMapping{

public:

	LaserMapping()
	{}

	// set initial guess
	void transformAssociateToMap();
	void transformUpdate();

	void pointAssociateToMap(PointType const *const pi, PointType *const po);
	void pointAssociateTobeMapped(PointType const *const pi, PointType *const po);







public:
	
	double timeLaserCloudCornerLast = 0;
	double timeLaserCloudSurfLast = 0;
	double timeLaserCloudFullRes = 0;
	double timeLaserOdometry = 0;


	int laserCloudCenWidth = 10;
	int laserCloudCenHeight = 10;
	int laserCloudCenDepth = 5;
	const int laserCloudWidth = 21;
	const int laserCloudHeight = 21;
	const int laserCloudDepth = 11;

	const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851

	int laserCloudValidInd[125];
	int laserCloudSurroundInd[125];

	double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
	Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
	Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);

	// wmap_T_odom * odom_T_curr = wmap_T_curr;
	// transformation between odom's world and map's world frame
	Eigen::Quaterniond q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
	Eigen::Vector3d t_wmap_wodom = Eigen::Vector3d(0, 0, 0);

	Eigen::Quaterniond q_wodom_curr = Eigen::Quaterniond(1, 0, 0, 0);
	Eigen::Vector3d t_wodom_curr = Eigen::Vector3d(0, 0, 0);

	

};