#include "LaserMapping.h"
#include "ScanRegistration.h"
#include "common.h"
#include "utils.h"




void LaserMapping::topcl(ScanRegistration S)
{
	laserCloudSurfLast->clear();
	pcl::PointCloud<PointType> surfPoints;
	for(auto i : S.surfPointsFlat){
		PointType point;
		point.x = i.x();
		point.y = i.y();
		point.z = i.z();
		surfPoints.push_back(point);
	}
	*laserCloudSurfLast += surfPoints;
	
	laserCloudCornerLast->clear();
	pcl::PointCloud<PointType> cornerPoints;
	for(auto i : S.cornerPointsSharp){
		PointType point;
		point.x = i.x();
		point.y = i.y();
		point.z = i.z();
		cornerPoints.push_back(point);
	}
	*laserCloudCornerLast += cornerPoints;
	
	laserCloudFullRes->clear();
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

void LaserMapping::loadOdomData(Vector6d pose)
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


void LaserMapping::getSurroundPointsfromCube()
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

void LaserMapping::surroundPoints2Map()
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

void LaserMapping::downSizeCurrentScan()
{

	downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
	downSizeFilterCorner.filter(*laserCloudCornerStack);
	laserCloudCornerStackNum = laserCloudCornerStack->points.size();

	downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
	downSizeFilterSurf.filter(*laserCloudSurfStack);
	laserCloudSurfStackNum = laserCloudSurfStack->points.size();	
}


////optimize/////
void LaserMapping::optimizePose()
{
	if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
	{

			// original
			// kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
			// kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
		
			// test
			kdtreeCornerFromMap->setInputCloud(nearCornerMap);
			kdtreeSurfFromMap->setInputCloud(nearSurfMap);
		
		//printf("build tree time %f ms \n", t_tree.toc());

		for (int iterCount = 0; iterCount < 2; iterCount++)
		{
			//ceres::LossFunction *loss_function = NULL;
			ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
			ceres::LocalParameterization *q_parameterization =
				new ceres::EigenQuaternionParameterization();
			ceres::Problem::Options problem_options;

			ceres::Problem problem(problem_options);
			problem.AddParameterBlock(parameters, 4, q_parameterization);
			problem.AddParameterBlock(parameters + 4, 3);

			// TicToc t_data;
			int corner_num = 0;

			for (int i = 0; i < laserCloudCornerStackNum; i++)
			{
				pointOri = laserCloudCornerStack->points[i];
				//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
				pointAssociateToMap(&pointOri, &pointSel);
				kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); 

				if (pointSearchSqDis[4] < 1.0)
				{ 
					std::vector<Eigen::Vector3d> nearCorners;
					Eigen::Vector3d center(0, 0, 0);
					for (int j = 0; j < 5; j++)
					{
						///// original //////
						// Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
						// 					laserCloudCornerFromMap->points[pointSearchInd[j]].y,
						// 					laserCloudCornerFromMap->points[pointSearchInd[j]].z);
						
						////// test ////////
						Eigen::Vector3d tmp(nearCornerMap->points[pointSearchInd[j]].x,
											nearCornerMap->points[pointSearchInd[j]].y,
											nearCornerMap->points[pointSearchInd[j]].z);
						center = center + tmp;
						nearCorners.push_back(tmp);
					}
					center = center / 5.0;
						
						
						

						

						

					Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
					for (int j = 0; j < 5; j++)
					{
						Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
						covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
					}

					Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

					// if is indeed line feature
					// note Eigen library sort eigenvalues in increasing order
					Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
					Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
					if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
					{ 
						Eigen::Vector3d point_on_line = center;
						Eigen::Vector3d point_a, point_b;
						point_a = 0.1 * unit_direction + point_on_line;
						point_b = -0.1 * unit_direction + point_on_line;

						ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
						problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						corner_num++;	
					}							
				}
				/*
				else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
				{
					Eigen::Vector3d center(0, 0, 0);
					for (int j = 0; j < 5; j++)
					{
						Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
											laserCloudCornerFromMap->points[pointSearchInd[j]].y,
											laserCloudCornerFromMap->points[pointSearchInd[j]].z);
						center = center + tmp;
					}
					center = center / 5.0;	
					Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
					ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
					problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
				}
				*/
			}

			int surf_num = 0;
			for (int i = 0; i < laserCloudSurfStackNum; i++)
			{
				pointOri = laserCloudSurfStack->points[i];
				//double sqrtDis = pointOri.x * pointOri.x + pointOri.y * pointOri.y + pointOri.z * pointOri.z;
				pointAssociateToMap(&pointOri, &pointSel);
				kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

				Eigen::Matrix<double, 5, 3> matA0;
				Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
				if (pointSearchSqDis[4] < 1.0)
				{
							
					for (int j = 0; j < 5; j++)
					{
						///// original ////
						// matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
						// matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
						// matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
						
						/////// test //////
						matA0(j, 0) = nearSurfMap->points[pointSearchInd[j]].x;
						matA0(j, 1) = nearSurfMap->points[pointSearchInd[j]].y;
						matA0(j, 2) = nearSurfMap->points[pointSearchInd[j]].z;
						////printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
					}
					// find the norm of plane
					Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
					double negative_OA_dot_norm = 1 / norm.norm();
					norm.normalize();

					// Here n(pa, pb, pc) is unit norm of plane
					bool planeValid = true;
					for (int j = 0; j < 5; j++)
					{
						// if OX * n > 0.2, then plane is not fit well
						// if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
						// 		norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
						// 		norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
						
						//////// test ////////
						if (fabs(norm(0) * nearSurfMap->points[pointSearchInd[j]].x +
								norm(1) * nearSurfMap->points[pointSearchInd[j]].y +
								norm(2) * nearSurfMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
						{
							planeValid = false;
							break;
						}
					}
					Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
					if (planeValid)
					{
						ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
					problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
						surf_num++;
					}
				}
				/*
				else if(pointSearchSqDis[4] < 0.01 * sqrtDis)
				{
					Eigen::Vector3d center(0, 0, 0);
					for (int j = 0; j < 5; j++)
					{
						Eigen::Vector3d tmp(laserCloudSurfFromMap->points[pointSearchInd[j]].x,
											laserCloudSurfFromMap->points[pointSearchInd[j]].y,
											laserCloudSurfFromMap->points[pointSearchInd[j]].z);
						center = center + tmp;
					}
					center = center / 5.0;	
					Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
					ceres::CostFunction *cost_function = LidarDistanceFactor::Create(curr_point, center);
					problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
				}
				*/
			}

			////printf("corner num %d used corner num %d \n", laserCloudCornerStackNum, corner_num);
			////printf("surf num %d used surf num %d \n", laserCloudSurfStackNum, surf_num);

			//printf("mapping data assosiation time %f ms \n", t_data.toc());

			// TicToc t_solver;
			ceres::Solver::Options options;
			options.linear_solver_type = ceres::DENSE_QR;
			options.max_num_iterations = 4;
			options.minimizer_progress_to_stdout = false;
			options.check_gradients = false;
			options.gradient_check_relative_precision = 1e-4;
			ceres::Solver::Summary summary;
			ceres::Solve(options, &problem, &summary);
			//printf("mapping solver time %f ms \n", t_solver.toc());

			////printf("time %f \n", timeLaserOdometry);
			////printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
			////printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
			//	   parameters[4], parameters[5], parameters[6]);
		}
				//printf("mapping optimization time %f \n", t_opt.toc());
	}
	else
	{
		ROS_WARN("time Map corner and surf num are not enough");
	}	
}



///////////////// 

void LaserMapping::currentScanToCube()
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

void LaserMapping::surroundMapDownSize()
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

//////////////// Test /////////////////////////

void LaserMapping::recentScan2Map()
{
	nearCornerMap->clear();
	for(int i = 0; i < mapScanNum; i ++)
		*nearCornerMap += *scanCornerMapArray[i];	
	
	nearSurfMap->clear();
	for(int i = 0; i < mapScanNum; i ++)
		*nearSurfMap += *scanSurfMapArray[i];

	laserCloudCornerFromMapNum = nearCornerMap->points.size();
	laserCloudSurfFromMapNum = nearSurfMap->points.size();	
}

void LaserMapping::setRecentlyMap()
{
	// corner
	for(int i = 0; i < mapScanNum - 1; i++){
		
		scanCornerMapArray[i]->clear();
		*scanCornerMapArray[i] = *scanCornerMapArray[i + 1];
		
		scanSurfMapArray[i]->clear();
		*scanSurfMapArray[i] = *scanSurfMapArray[i + 1];
	}

	scanCornerMapArray[mapScanNum - 1]->clear();
	for(int i = 0; i < laserCloudCornerStackNum; i++){
		pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);
		scanCornerMapArray[mapScanNum - 1]->push_back(pointSel);
	}

	scanSurfMapArray[mapScanNum - 1]->clear();
	for(int i = 0; i < laserCloudSurfStackNum; i++){
		pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);
		scanSurfMapArray[mapScanNum - 1]->push_back(pointSel);
	}
}
	
void LaserMapping::recentlyMapDownSize()
{
	for (int i = 0; i < mapScanNum; i++)
	{
		pcl::PointCloud<PointType>::Ptr tmpCorner(new pcl::PointCloud<PointType>());
		downSizeFilterCorner.setInputCloud(scanCornerMapArray[i]);
		downSizeFilterCorner.filter(*tmpCorner);
		laserCloudCornerArray[i] = tmpCorner;

		pcl::PointCloud<PointType>::Ptr tmpSurf(new pcl::PointCloud<PointType>());
		downSizeFilterSurf.setInputCloud(scanSurfMapArray[i]);
		downSizeFilterSurf.filter(*tmpSurf);
		laserCloudSurfArray[i] = tmpSurf;
	}	
}	

void LaserMapping::visuzlize(const ros::Publisher &publisher, const ros::Time &timestamp)
{
	pcl::PointCloud<PointType> laserCloudMap;
	for (int i = 0; i < mapScanNum; i++)
	{
		laserCloudMap += *scanCornerMapArray[i];
		laserCloudMap += *scanSurfMapArray[i];
	}
	sensor_msgs::PointCloud2 laserCloudMsg;
	pcl::toROSMsg(laserCloudMap, laserCloudMsg);
	laserCloudMsg.header.stamp = timestamp;
	laserCloudMsg.header.frame_id = "/camera_init";
	publisher.publish(laserCloudMsg);		
}

//////////////////////////////////////////	
	






///////// Visualize //////////////////////////////

void LaserMapping::visualizePointCloud(const ros::Publisher &publisher, const ros::Time &timestamp)
{
	pcl::PointCloud<PointType> laserCloudMap;
	for (int i = 0; i < 4851; i++)
	{
		laserCloudMap += *laserCloudCornerArray[i];
		laserCloudMap += *laserCloudSurfArray[i];
	}

	sensor_msgs::PointCloud2 laserCloudMsg;
	pcl::toROSMsg(laserCloudMap, laserCloudMsg);
	laserCloudMsg.header.stamp = timestamp;
	laserCloudMsg.header.frame_id = "/camera_init";
	publisher.publish(laserCloudMsg);	
}

void LaserMapping::visualizePose(	const ros::Publisher &pubMappingOdom, 
									const ros::Publisher &pubMappingPath, 
									nav_msgs::Path &MappingPath, 
									const ros::Time &timestamp,
									std::ofstream &MappingPoseFile)
{
        // publish odometry
        nav_msgs::Odometry VIOodometry;
        VIOodometry.header.frame_id = "/camera_init";
        VIOodometry.child_frame_id = "/laser_odom";
        VIOodometry.header.stamp = timestamp;
        // VIOodometry.pose.pose.orientation.x = q.x();
        // VIOodometry.pose.pose.orientation.y = q.y();
        // VIOodometry.pose.pose.orientation.z = q.z();
        // VIOodometry.pose.pose.orientation.w = q.w();
        // VIOodometry.pose.pose.position.x = p.x();
        // VIOodometry.pose.pose.position.y = p.y();
        // VIOodometry.pose.pose.position.z = p.z();
        VIOodometry.pose.pose.orientation.x = q_w_curr.x();
        VIOodometry.pose.pose.orientation.y = q_w_curr.y();
        VIOodometry.pose.pose.orientation.z = q_w_curr.z();
        VIOodometry.pose.pose.orientation.w = q_w_curr.w();
        VIOodometry.pose.pose.position.x = t_w_curr.x();
        VIOodometry.pose.pose.position.y = t_w_curr.y();
        VIOodometry.pose.pose.position.z = t_w_curr.z();
        pubMappingOdom.publish(VIOodometry);

        geometry_msgs::PoseStamped VIOPose;
        VIOPose.header = VIOodometry.header;
        VIOPose.pose = VIOodometry.pose.pose;
        MappingPath.header.stamp = timestamp;
        MappingPath.poses.push_back(VIOPose);
        MappingPath.header.frame_id = "/camera_init";
        pubMappingPath.publish(MappingPath);


		Vector6d CurrPose = To6DOF(q_w_curr, t_w_curr);
		MappingPoseFile << CurrPose[0] << " " << CurrPose[1] << " " << CurrPose[2] << " " << CurrPose[3] << " " << CurrPose[4] << " " << CurrPose[5] << std::endl;
		std::cout << " Mapping Pose !!!! " << std::endl;
		std::cout << CurrPose[0] << " " << CurrPose[1] << " " << CurrPose[2] << " " << CurrPose[3] << " " << CurrPose[4] << " " << CurrPose[5] << std::endl;
}

void LaserMapping::visualizePose(	const ros::Publisher &pubVIOodom, 
						const ros::Publisher &pubVIOPath, 
						nav_msgs::Path &VIOPath, 
						const ros::Time &timestamp,
						Vector6d pose)
{
        // publish odometry
        nav_msgs::Odometry VIOodometry;
        VIOodometry.header.frame_id = "/camera_init";
        VIOodometry.child_frame_id = "/laser_odom";
        VIOodometry.header.stamp = timestamp;
		Eigen::Quaterniond q = ToQuaternion(pose);
        VIOodometry.pose.pose.orientation.x = q.x();
        VIOodometry.pose.pose.orientation.y = q.y();
        VIOodometry.pose.pose.orientation.z = q.z();
        VIOodometry.pose.pose.orientation.w = q.w();
        VIOodometry.pose.pose.position.x = pose[3];
        VIOodometry.pose.pose.position.y = pose[4];
        VIOodometry.pose.pose.position.z = pose[5];
        pubVIOodom.publish(VIOodometry);

        geometry_msgs::PoseStamped VIOPose;
        VIOPose.header = VIOodometry.header;
        VIOPose.pose = VIOodometry.pose.pose;
        VIOPath.header.stamp = timestamp;
        VIOPath.poses.push_back(VIOPose);
        VIOPath.header.frame_id = "/camera_init";
        pubVIOPath.publish(VIOPath);	
}
////////////////////////////////////////////////////////////////


void LaserMapping::transform(const ros::Time &timestamp)
{
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	tf::Quaternion q;
	transform.setOrigin(tf::Vector3(t_w_curr(0),
									t_w_curr(1),
									t_w_curr(2)));
	q.setW(q_w_curr.w());
	q.setX(q_w_curr.x());
	q.setY(q_w_curr.y());
	q.setZ(q_w_curr.z());
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, timestamp, "/camera_init", "/aft_mapped"));	
}