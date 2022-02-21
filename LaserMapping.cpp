#include "LaserMapping.h"
#include "ScanRegistration.h"
#include "common.h"


void LaserMapping::Topcl(ScanRegistration S)
{
	pcl::PointCloud<PointType> surfPoints;
	for(auto i : S.surfPointsFlat){
		PointType point;
		point.x = i.x();
		point.y = i.y();
		point.z = i.z();
		surfPoints.push_back(point);
	}
	*laserCloudSurfLast += surfPoints;
	
	pcl::PointCloud<PointType> cornerPoints;
	for(auto i : S.cornerPointsSharp){
		PointType point;
		point.x = i.x();
		point.y = i.y();
		point.z = i.z();
		cornerPoints.push_back(point);
	}
	*laserCloudCornerLast += cornerPoints;

	pcl::PointCloud<PointType> fullPoints;
	for(auto i : S.laserCloud){
		PointType point;
		point.x = i.x();
		point.y = i.y();
		point.z = i.z();
		fullPoints.push_back(point);
	}
	*laserCloudFullRes += fullPoints;
}

void LaserMapping::LoadOdomData(Vector6d pose)
{
	Eigen::Quaterniond q = ToQuaternion(pose);
	q_wodom_curr.x() = q.x();
	q_wodom_curr.y() = q.y();
	q_wodom_curr.z() = q.z();
	q_wodom_curr.w() = q.w();
	t_wodom_curr.x() = pose[3];
	t_wodom_curr.y() = pose[4];
	t_wodom_curr.z() = pose[5];	
}

// set initial guess
void LaserMapping::transformAssociateToMap()
{
	q_w_curr = q_wmap_wodom * q_wodom_curr;
	t_w_curr = q_wmap_wodom * t_wodom_curr + t_wmap_wodom;
}

void LaserMapping::transformUpdate()
{
	q_wmap_wodom = q_w_curr * q_wodom_curr.inverse();
	t_wmap_wodom = t_w_curr - q_wmap_wodom * t_wodom_curr;
}

void LaserMapping::pointAssociateToMap(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;		
    //po->intensity = 1.0;
}

void LaserMapping::pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
	po->x = point_curr.x();
	po->y = point_curr.y();
	po->z = point_curr.z();
	po->intensity = pi->intensity;
}


void LaserMapping::hoho()
{
			int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
			int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
			int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

			if (t_w_curr.x() + 25.0 < 0)
				centerCubeI--;
			if (t_w_curr.y() + 25.0 < 0)
				centerCubeJ--;
			if (t_w_curr.z() + 25.0 < 0)
				centerCubeK--;

			while (centerCubeI < 3)
			{
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{ 
						int i = laserCloudWidth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; 
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i >= 1; i--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI++;
				laserCloudCenWidth++;
			}

			while (centerCubeI >= laserCloudWidth - 3)
			{ 
				for (int j = 0; j < laserCloudHeight; j++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int i = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; i < laserCloudWidth - 1; i++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeI--;
				laserCloudCenWidth--;
			}

			while (centerCubeJ < 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int j = laserCloudHeight - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j >= 1; j--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ++;
				laserCloudCenHeight++;
			}

			while (centerCubeJ >= laserCloudHeight - 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int k = 0; k < laserCloudDepth; k++)
					{
						int j = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; j < laserCloudHeight - 1; j++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeJ--;
				laserCloudCenHeight--;
			}

			while (centerCubeK < 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						int k = laserCloudDepth - 1;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k >= 1; k--)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK++;
				laserCloudCenDepth++;
			}

			while (centerCubeK >= laserCloudDepth - 3)
			{
				for (int i = 0; i < laserCloudWidth; i++)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						int k = 0;
						pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
						for (; k < laserCloudDepth - 1; k++)
						{
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
						}
						laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeCornerPointer;
						laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
							laserCloudCubeSurfPointer;
						laserCloudCubeCornerPointer->clear();
						laserCloudCubeSurfPointer->clear();
					}
				}

				centerCubeK--;
				laserCloudCenDepth--;
			}

			laserCloudValidNum = 0;
			laserCloudSurroundNum = 0;

			for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
			{
				for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
				{
					for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++)
					{
						if (i >= 0 && i < laserCloudWidth &&
							j >= 0 && j < laserCloudHeight &&
							k >= 0 && k < laserCloudDepth)
						{ 
							laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudValidNum++;
							laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
							laserCloudSurroundNum++;
						}
					}
				}
			}	
}

void LaserMapping::SaveLastMap()
{
	laserCloudCornerFromMap->clear();
	laserCloudSurfFromMap->clear();
	for (int i = 0; i < laserCloudValidNum; i++)
	{
		*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
		*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
	}
	laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
	laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();	
}

void LaserMapping::DownSizeFiltering()
{
	// pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
	// laserCloudCornerStack  clear()?
	downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
	downSizeFilterCorner.filter(*laserCloudCornerStack);
	laserCloudCornerStackNum = laserCloudCornerStack->points.size();

	// pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
	downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
	downSizeFilterSurf.filter(*laserCloudSurfStack);
	laserCloudSurfStackNum = laserCloudSurfStack->points.size();	
}


////optimize/////
void LaserMapping::Optimize()
{
	
}



/////////////////

void LaserMapping::pupu()
{
	for (int i = 0; i < laserCloudCornerStackNum; i++)
	{
		pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

		int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
		int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
		int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

		if (pointSel.x + 25.0 < 0)
			cubeI--;
		if (pointSel.y + 25.0 < 0)
			cubeJ--;
		if (pointSel.z + 25.0 < 0)
			cubeK--;

		if (cubeI >= 0 && cubeI < laserCloudWidth &&
			cubeJ >= 0 && cubeJ < laserCloudHeight &&
			cubeK >= 0 && cubeK < laserCloudDepth)
		{
			int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
			laserCloudCornerArray[cubeInd]->push_back(pointSel);
		}
	}

	for (int i = 0; i < laserCloudSurfStackNum; i++)
	{
		pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

		int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
		int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
		int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

		if (pointSel.x + 25.0 < 0)
			cubeI--;
		if (pointSel.y + 25.0 < 0)
			cubeJ--;
		if (pointSel.z + 25.0 < 0)
			cubeK--;

		if (cubeI >= 0 && cubeI < laserCloudWidth &&
			cubeJ >= 0 && cubeJ < laserCloudHeight &&
			cubeK >= 0 && cubeK < laserCloudDepth)
		{
			int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
			laserCloudSurfArray[cubeInd]->push_back(pointSel);
		}
	}
}

void LaserMapping::DownSize()
{
	for (int i = 0; i < laserCloudValidNum; i++)
	{
		int ind = laserCloudValidInd[i];

		pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
		downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
		downSizeFilterCorner.filter(*tmpCorner);
		laserCloudCornerArray[ind] = tmpCorner;

		pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
		downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
		downSizeFilterSurf.filter(*tmpSurf);
		laserCloudSurfArray[ind] = tmpSurf;
	}
}