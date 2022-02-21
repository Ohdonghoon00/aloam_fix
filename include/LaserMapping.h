#pragma once

#include "common.h"
#include "utils.h"
#include "../src/lidarFactor.hpp"
#include "ScanRegistration.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <mutex>
#include <queue>


class LaserMapping{

public:

	LaserMapping()
	{
		laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());
		laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());
		laserCloudFullRes.reset(new pcl::PointCloud<PointType>());

		laserCloudSurround.reset(new pcl::PointCloud<PointType>());

		laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
		laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());

		kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
		kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

		downSizeFilterCorner.setLeafSize(0.2, 0.2,0.2);
		downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

		laserCloudCornerStack.reset(new pcl::PointCloud<PointType>());
		laserCloudSurfStack.reset(new pcl::PointCloud<PointType>());

		for (int i = 0; i < laserCloudNum; i++)
		{
			laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
			laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
		}
		
	}

	void Topcl(ScanRegistration S);
	void LoadOdomData(Vector6d pose);

	// set initial guess
	void transformAssociateToMap();
	void transformUpdate();

	void pointAssociateToMap(PointType const *const pi, PointType *const po);
	void pointAssociateTobeMapped(PointType const *const pi, PointType *const po);

	void CenterCube();
	void SaveLastMap();
	void DownSizeFiltering();

	// optimize ////
	void OptimizePose();
	///////////////

	void Cube();
	void DownSize();

	// Visualize //
	void VisualizePointCloud(const ros::Publisher &publisher, const ros::Time &timestamp);
	void VisualizePose(	const ros::Publisher &pubMappingOdom, 
						const ros::Publisher &pubMappingPath, 
						nav_msgs::Path &MappingPath, 
						const ros::Time &timestamp,
						std::ofstream &MappingPoseFile);
	//////////////

	void transform(const ros::Time &timestamp);





public:
	
	double timeLaserCloudCornerLast = 0;
	double timeLaserCloudSurfLast = 0;
	double timeLaserCloudFullRes = 0;
	double timeLaserOdometry = 0;


	int laserCloudCenWidth = 10;
	int laserCloudCenHeight = 10;
	int laserCloudCenDepth = 5;
	const static int laserCloudWidth = 21;
	const static int laserCloudHeight = 21;
	const static int laserCloudDepth = 11;

	const static int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851

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

	// input: from odom
	pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;
	
	//input & output: points in one frame. local --> global
	pcl::PointCloud<PointType>::Ptr laserCloudFullRes;

	// ouput: all visualble cube points
	pcl::PointCloud<PointType>::Ptr laserCloudSurround;

	// surround points in map to build tree
	pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;


	// points in every cube
	pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
	pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];

	//kd-tree
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
	pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

	pcl::VoxelGrid<PointType> downSizeFilterCorner;
	pcl::VoxelGrid<PointType> downSizeFilterSurf;

	std::vector<int> pointSearchInd;
	std::vector<float> pointSearchSqDis;

	PointType pointOri, pointSel;


	int laserCloudValidNum = 0;
	int laserCloudSurroundNum = 0;

	int laserCloudCornerFromMapNum = 0;
	int laserCloudSurfFromMapNum = 0;

	int laserCloudCornerStackNum = 0;
	int laserCloudSurfStackNum = 0;

	pcl::PointCloud<PointType>::Ptr laserCloudCornerStack;
	pcl::PointCloud<PointType>::Ptr laserCloudSurfStack;


};