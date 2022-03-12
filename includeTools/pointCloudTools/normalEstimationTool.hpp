/*
This class can be used to estimate the normals of a cloud.
    * Parameters needed:
        - kSearch: K number of closest points that will be used to calculate the normal of a point.
This script cannot be run directly and needs to be used as an import.
*/

#ifndef CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_NORMALESTIMATIONTOOLS_H_
#define CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_NORMALESTIMATIONTOOLS_H_

// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/console.h> 
// PCL libraries
#include <pcl/features/normal_3d.h>

class NormalEstimationTool
{
    public:
        NormalEstimationTool(const int kSearch);

        ~NormalEstimationTool();

        int getNormalsCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud, pcl::PointCloud<pcl::Normal>::Ptr rpCloudNormals);
    private:
        // Initialization of parameters
        int mKSearch; 

        // Initialization of tools
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> mNormalEstimation;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr mpTree;
}; 

#endif // CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_NORMALESTIMATIONTOOLS_H_ 