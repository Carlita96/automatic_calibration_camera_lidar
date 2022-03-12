/*
This class can be used to get a plane from a cloud. 
    * Parameters needed:
        - normalDistanceWeigth: relative weight (0-1) to give to the angular distance (0-pi/2) between point normals and the plane normal. 
        - distanceThreshold: how close a point must be to the model in order to be considered an inlier.
        - maxIterations: maximum number of iterations trying to fit the plane.
This script cannot be run directly and needs to be used as an import.
*/

#ifndef CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_PLANESEGMENTATIONTOOLS_H_
#define CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_PLANESEGMENTATIONTOOLS_H_

// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/console.h> 
// PCL libraries
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/extract_indices.h>

class PlaneSegmentationTool
{
    public:
        PlaneSegmentationTool(const double normalDistanceWeight, const double distanceThreshold, const int maxIterations);

        ~PlaneSegmentationTool();

        int segmentCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud, const pcl::PointCloud<pcl::Normal>::Ptr pInputCloudNormals);

        pcl::PointCloud<pcl::PointXYZI> getSegmentedCloud(const bool getPlane = true);

        pcl::PointCloud<pcl::Normal> getSegmentedCloudNormals(const bool getPlane = true);

        pcl::ModelCoefficients getPlaneCoefficients();

        double getRateInliersOutliers();
    private:
        // Initialization of parameters
        double mNormalDistanceWeight; 
        double mDistanceThreshold; 
        int mMaxIterations; 

        // Initialization of tools
        pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> mSegmentation; 
        pcl::ExtractIndices<pcl::PointXYZI> mExtract;
        pcl::ExtractIndices<pcl::Normal> mExtractNormals;

        // Initialization of clouds
        pcl::PointCloud<pcl::PointXYZI>::Ptr mInputCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr mCloudPlane;
        pcl::PointCloud<pcl::PointXYZI>::Ptr mCloudWithoutPlane;
        pcl::PointCloud<pcl::PointXYZI>::Ptr mEmptyCloud;
        pcl::PointCloud<pcl::Normal>::Ptr mInputCloudNormals;
        pcl::PointCloud<pcl::Normal>::Ptr mCloudPlaneNormals;
        pcl::PointCloud<pcl::Normal>::Ptr mCloudWithoutPlaneNormals;
        pcl::PointCloud<pcl::Normal>::Ptr mEmptyCloudNormals;
        // Initialization of coefficients and inliers
        pcl::ModelCoefficients::Ptr mCoefficientsPlane;
        pcl::ModelCoefficients::Ptr mEmptyCoefficients;
        pcl::PointIndices::Ptr mInliersPlane;

        // Process control
        bool mSegmentationDone = false;
}; 

#endif // CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_PLANESEGMENTATIONTOOLS_H_ 