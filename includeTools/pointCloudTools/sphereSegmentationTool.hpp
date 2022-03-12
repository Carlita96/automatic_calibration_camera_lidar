/*
This class can be used to get a sphere from a cloud. 
    * Parameters needed:
        - normalDistanceWeigth: relative weight (0-1) to give to the angular distance (0-pi/2) between point normals and the sphere normal. 
        - distanceThreshold: how close a point must be to the model in order to be considered an inlier.
        - maxIterations: maximum number of iterations trying to fit the sphere.
        - minRadius and maxRadius: set the threshold for the radius of the sphere to segment.
This script cannot be run directly and needs to be used as an import.
*/

#ifndef CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_SPHERESEGMENTATIONTOOLS_H_
#define CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_SPHERESEGMENTATIONTOOLS_H_

#include <iostream>

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


class SphereSegmentationTool
{
    public:
        SphereSegmentationTool(const double normalDistanceWeight, const double distanceThreshold, const int maxIterations,
    const double minRadius, const double maxRadius);

        ~SphereSegmentationTool();

        int segmentCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud, const pcl::PointCloud<pcl::Normal>::Ptr pCloudNormals);

        pcl::PointCloud<pcl::PointXYZI> getSegmentedCloud(const bool getSphere = true);

        pcl::PointCloud<pcl::Normal> getSegmentedCloudNormals(const bool getSphere = true);

        pcl::ModelCoefficients getSphereCoefficients();

        double getRateInliersOutliers();
    private:
        // Initialization of parameters
        double mNormalDistanceWeight; 
        double mDistanceThreshold; 
        int mMaxIterations; 
        double mMinRadius; 
        double mMaxRadius; 

        // Initialization of tools
        pcl::SACSegmentationFromNormals<pcl::PointXYZI, pcl::Normal> mSegmentation; 
        pcl::ExtractIndices<pcl::PointXYZI> mExtract;
        pcl::ExtractIndices<pcl::Normal> mExtractNormals;

        // Initialization of clouds
        pcl::PointCloud<pcl::PointXYZI>::Ptr mInputCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr mCloudSphere;
        pcl::PointCloud<pcl::PointXYZI>::Ptr mCloudWithoutSphere;
        pcl::PointCloud<pcl::PointXYZI>::Ptr mEmptyCloud;
        pcl::PointCloud<pcl::Normal>::Ptr mInputCloudNormals;
        pcl::PointCloud<pcl::Normal>::Ptr mCloudSphereNormals;
        pcl::PointCloud<pcl::Normal>::Ptr mCloudWithoutSphereNormals;
        pcl::PointCloud<pcl::Normal>::Ptr mEmptyCloudNormals;
        // Initialization of coefficients and inliers
        pcl::ModelCoefficients::Ptr mCoefficientsSphere;
        pcl::ModelCoefficients::Ptr mEmptyCoefficients;
        pcl::PointIndices::Ptr mInliersSphere;

        // Process control
        bool mSegmentationDone = false;
}; 

#endif // CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_SPHERESEGMENTATIONTOOLS_H_ 