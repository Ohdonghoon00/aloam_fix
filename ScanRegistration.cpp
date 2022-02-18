#include "ScanRegistration.h"
#include "utils.h"

// bool ScanRegistration::comp(int i, int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
// bool ScanRegistration::SameLeaf(int i, int j) { return (LeafIds[i] < LeafIds[j]); }
// bool ScanRegistration::comp(int i, int j) { return (cloudCurvature[i]<cloudCurvature[j]); }
// bool ScanRegistration::SameLeaf(int i, int j) { return (LeafIds[i] < LeafIds[j]); }

// DownsizeFiltering
void ScanRegistration::getMinMax(std::vector< Eigen::Vector3d > &inCloud, Eigen::Vector3d &minp, Eigen::Vector3d &maxp)
{
    for(size_t i = 0; i < inCloud.size(); i++){
        minp.x() = std::min(minp.x(), inCloud[i].x());
        minp.y() = std::min(minp.y(), inCloud[i].y());
        minp.z() = std::min(minp.z(), inCloud[i].z());
	
        maxp.x() = std::max(maxp.x(), inCloud[i].x());
        maxp.y() = std::max(maxp.y(), inCloud[i].y());
        maxp.z() = std::max(maxp.z(), inCloud[i].z());
    }
}

void ScanRegistration::DownSizeFiltering(Eigen::Vector3d &LeafSize, std::vector< Eigen::Vector3d > &InCloud, std::vector< Eigen::Vector3d > &OutCloud)
{
	
    // Compute minimum and maximum point values
    Eigen::Vector3d minp(DBL_MAX, DBL_MAX, DBL_MAX), maxp(-DBL_MAX, -DBL_MAX, -DBL_MAX);
    getMinMax(InCloud, minp, maxp);

    // Compute Leaf Count
    Eigen::Vector3i MaxLeafCount(ceil((maxp.x() - minp.x())/LeafSize.x()), ceil((maxp.y() - minp.y())/LeafSize.y()), ceil((maxp.z() - minp.z())/LeafSize.z()));
    std::vector<int> LeafInd;


    // Leaf Idx
    LeafIds.clear();
    for(size_t i = 0; i < InCloud.size(); i++){

        int LeafCount_x = ceil((InCloud[i].x() - minp.x()) / LeafSize.x());
        int LeafCount_y = ceil((InCloud[i].y() - minp.y()) / LeafSize.y());
        int LeafCount_z = ceil((InCloud[i].z() - minp.z()) / LeafSize.z());

        int LeafId = (LeafCount_x - 1) + (LeafCount_y - 1) * MaxLeafCount.x() + (LeafCount_z - 1) * MaxLeafCount.x() * MaxLeafCount.y();
        
        LeafInd.push_back(i);
        LeafIds.push_back(LeafId);

    }
    
    // std::sort(LeafInd.begin(), LeafInd.end(), SameLeaf);
    
    for(size_t cp = 0; cp < InCloud.size();){
        Eigen::Vector3d Centroid(InCloud[LeafInd[cp]].x(), InCloud[LeafInd[cp]].y(), InCloud[LeafInd[cp]].z());
        size_t i = cp + 1;
        while(i < InCloud.size() && LeafIds[LeafInd[cp]] == LeafIds[LeafInd[i]]){
            Centroid += InCloud[LeafInd[i]];
            ++i;
        }
        
        Centroid.x() /= (double)(i - cp);
        Centroid.y() /= (double)(i - cp);
        Centroid.z() /= (double)(i - cp);

        OutCloud.push_back(Centroid);

        cp = i;
    }


}

void ScanRegistration::RemoveClosedPointCloud(std::vector<Eigen::Vector3d> *pointcloud)
{
    std::vector<Eigen::Vector3d> CloudOut;
    CloudOut.resize(pointcloud->size());

    size_t j = 0;
    for (size_t i = 0; i < pointcloud->size(); ++i)
    {
        double distance = PointDistance((*pointcloud)[i]);
        if (distance * distance < LidarToPointsThres * LidarToPointsThres)
            continue;
        CloudOut[j] = (*pointcloud)[i];
        j++;
    }
    
    pointcloud->clear();
    pointcloud->assign(CloudOut.begin(), CloudOut.begin() + j);
}
    
void ScanRegistration::RemoveNaNFromPointCloud(std::vector<Eigen::Vector3d> *pointcloud)
{
    std::vector<Eigen::Vector3d> CloudOut;
    CloudOut.resize(pointcloud->size());    
    
    size_t j = 0;
    // Remove Nan, Inf
    for(size_t i = 0; i < pointcloud->size(); i++){
        
        bool is_nan_x = isnan((*pointcloud)[i].x());
        bool is_nan_y = isnan((*pointcloud)[i].y());
        bool is_nan_z = isnan((*pointcloud)[i].z());
        if(is_nan_x || is_nan_y || is_nan_z)
            continue;

        bool is_inf_x = isinf((*pointcloud)[i].x());
        bool is_inf_y = isinf((*pointcloud)[i].y());
        bool is_inf_z = isinf((*pointcloud)[i].z());
        if(is_inf_x || is_inf_y || is_inf_z)
            continue;
        
        CloudOut[j] = (*pointcloud)[i];
        j++;
    }
        
    pointcloud->clear();
    pointcloud->assign(CloudOut.begin(), CloudOut.begin() + j);
}

void ScanRegistration::DividePointsByChannel( const std::vector<Eigen::Vector3d>& laserPoints)
{   
    laserCloudScans.clear();
    laserCloudScans.resize(N_SCANS, Eigen::Matrix3Xd(3, kMaxNumberOfPoints ));

    // Calculate Max and Min Vertical Angle
    std::vector<float> Verticalangles;
    Eigen::Vector3d point;
    std::vector<int> PointNumByScanID(N_SCANS, 0);
    
    for(int i = 0; i < cloudSize; i++){
        
        point.x() = laserPoints[i].x();
        point.y() = laserPoints[i].y();
        point.z() = laserPoints[i].z();        
        float angle = VerticalAngle(point);
        Verticalangles.push_back(angle);
    }

    float MaxAngle = *max_element(Verticalangles.begin(), Verticalangles.end());
    float MinAngle = *min_element(Verticalangles.begin(), Verticalangles.end());
    VerticalAngelRatio = (MaxAngle - MinAngle) / N_SCANS;
    
    // Sort Points by channel
    for (int i = 0; i < cloudSize; i++){
        point.x() = laserPoints[i].x();
        point.y() = laserPoints[i].y();
        point.z() = laserPoints[i].z();
        
        float angle = VerticalAngle(point);
        int scanID = 0;
        for(int j = 0; j < N_SCANS; j++){
            if(angle < MinAngle + VerticalAngelRatio * (j + 1)){
                scanID = j;
                break;
            }
        }

        laserCloudScans[scanID](0, PointNumByScanID[scanID]) = point.x();
        laserCloudScans[scanID](1, PointNumByScanID[scanID]) = point.y();
        laserCloudScans[scanID](2, PointNumByScanID[scanID]) = point.z();

        PointNumByScanID[scanID]++;

    }
    
    // resize             
    for(int i = 0; i < N_SCANS; i++)
        laserCloudScans[i].conservativeResize(3, PointNumByScanID[i]);
    
    
    // PointIndexByChannel.resize(N_SCANS);
    // FeatureNumByScan.clear();
    // for(size_t i = 0; i < laserCloudScans->size(); i++){
    //     FeatureNumByScan.push_back((*laserCloudScans)[i].cols());
    //     PointIndexByChannel[i].assign((*laserCloudScans)[i].cols(), -1);
    //     // std::cout << PointIndexByChannel[i] << "  ";

    // }
}

void ScanRegistration::SetPointCloudAndDistance(std::vector<double> *PointRange)
{
    scanStartInd.resize(N_SCANS);
    scanEndInd.resize(N_SCANS);
    
    int idx = 0;
    for (int i = 0; i < N_SCANS; i++)
    { 
        scanStartInd[i] = laserCloud.size() + 5;
        for(int j = 0; j < laserCloudScans[i].cols(); j++){
            laserCloud.resize(idx + 1);
            Eigen::Vector3d p;
            p << laserCloudScans[i](0, j), laserCloudScans[i](1, j), laserCloudScans[i](2, j);
            laserCloud[idx] = p;
            PointRange->push_back(PointDistance(laserCloud[idx]));
            idx++;
        //     // std::cout << PointDistance(laserCloud[idx]) << std::endl;
        }

        scanEndInd[i] = laserCloud.size() - 6;
        // std::cout << " start index : " << scanStartInd[i] << " end index : " << scanEndInd[i] << std::endl;
    }
}

void ScanRegistration::CalculateCurvature(const std::vector<double>& PointRange)
{
    for (int i = 5; i < cloudSize - 5; i++){         
        double cloudDiff = PointRange[i - 5] + PointRange[i - 4] +
                            PointRange[i - 3] + PointRange[i - 2] +
                            PointRange[i - 1] + PointRange[i + 1] +
                            PointRange[i + 2] + PointRange[i + 3] +
                            PointRange[i + 4] + PointRange[i + 5] +
                            - 10 * PointRange[i];
        cloudCurvature[i] = cloudDiff * cloudDiff;
        cloudSortInd[i] = i;
        cloudNeighborPicked[i] = 0;
        cloudLabel[i] = 0;
    }
}

void ScanRegistration::MarkOccludedPoints(const std::vector<double>& PointRange)
{
    // Occluded and Parallel beam
    int occluded_cnt = 0;
    int parallel_cnt = 0;
    for (int i = 5; i < cloudSize - 6; i++){
        
        // occluded points
        double depth1 = PointRange[i];
        double depth2 = PointRange[i + 1];
        // double Diff = PointDistance(laserCloud[i + 1], laserCloud[i]);
        double Diff = std::abs(laserCloud[i + 1].x() - laserCloud[i].x());

        if (Diff * Diff< 0.05){
            if (depth1 - depth2 > 0.3){
                occluded_cnt++;
                cloudNeighborPicked[i - 5] = 1;
                cloudNeighborPicked[i - 4] = 1;
                cloudNeighborPicked[i - 3] = 1;
                cloudNeighborPicked[i - 2] = 1;
                cloudNeighborPicked[i - 1] = 1;
                cloudNeighborPicked[i] = 1;
            }else if (depth2 - depth1 > 0.3){
                occluded_cnt++;
                cloudNeighborPicked[i + 1] = 1;
                cloudNeighborPicked[i + 2] = 1;
                cloudNeighborPicked[i + 3] = 1;
                cloudNeighborPicked[i + 4] = 1;
                cloudNeighborPicked[i + 5] = 1;
                cloudNeighborPicked[i + 6] = 1;
            }
        } 
            
        // parallel beam
        double diff1 = PointDistance(laserCloud[i - 1], laserCloud[i]);
        double diff2 = PointDistance(laserCloud[i + 1], laserCloud[i]);

        if (diff1 > 0.02 * PointRange[i] && diff2 > 0.02 * PointRange[i]){
            parallel_cnt++;
            cloudNeighborPicked[i] = 1;
        }
      
    }
    
    std::cout << "Occluded points : " << occluded_cnt << std::endl;
    std::cout << "Parallel points : " << parallel_cnt << std::endl;

}

void ScanRegistration::DividePointsByEdgeAndPlanePoints()
{

    cornerPointsSharp.clear();
    cornerPointsLessSharp.clear();
    surfPointsFlat.clear();
    surfPointsLessFlat.clear();
    
    // Edge Points and Plane Points
    for (int i = 0; i < N_SCANS; i++)
    {
        if( scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        std::vector<Eigen::Vector3d> surfPointsLessFlatScan;
        for (int j = 0; j < 6; j++){
            
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6; 
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;

            // std::sort (cloudSortInd + sp, cloudSortInd + ep + 1, comp);

            // Edge Points
            int largestPickedNum = 0;
            for (int k = ep; k >= sp; k--){
                int ind = cloudSortInd[k]; 
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.3)
                {

                    largestPickedNum++;
                    if (largestPickedNum <= 2){                        
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(laserCloud[ind]);
                        cornerPointsLessSharp.push_back(laserCloud[ind]);
                                   
                    }
                    else if (largestPickedNum <= 20){                        
                        cloudLabel[ind] = 1; 
                        cornerPointsLessSharp.push_back(laserCloud[ind]);

                    }
                    else
                        break;
                    
                    

                    cloudNeighborPicked[ind] = 1; 

                    for (int l = 1; l <= 5; l++){
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l - 1]);
                        if(diff * diff > 0.05)
                            break;
                        
                        cloudNeighborPicked[ind + l] = 1;
                    }
                        

                    for (int l = -1; l >= -5; l--){
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l + 1]);
                        if(diff * diff > 0.05)
                            break;
                        
                        cloudNeighborPicked[ind + l] = 1;
                    }
                        

                }
            }
            
            // Plane Points
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++){
                int ind = cloudSortInd[k];

                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {
                    cloudLabel[ind] = -1; 
                    surfPointsFlat.push_back(laserCloud[ind]);

                    smallestPickedNum++;
                    if (smallestPickedNum >= 10)
                        break;                   

                    cloudNeighborPicked[ind] = 1;
                    for (int l = 1; l <= 5; l++){ 
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l - 1]);
                        if(diff * diff > 0.05)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--){
                        double diff = PointDistance(laserCloud[ind + l], laserCloud[ind + l - 1]);
                        if(diff * diff > 0.05)
                            break;

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }

            for (int k = sp; k <= ep; k++){
                if (cloudLabel[k] <= 0){

                    //downsize test
                    surfPointsLessFlatScan.push_back(laserCloud[k]);

                }
            }
        }
        
        DownSizeFiltering(DownSizeLeafSize, surfPointsLessFlatScan, surfPointsLessFlat);
        
    } 
        // std::cout << " flat points num After downsize filtering : " << surfPointsLessFlat.size() << std::endl;
}