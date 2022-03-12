/*
This class can be used to segment an organized cloud into different parts with similar depth. Then it can
cluster these segments together if they are close to each other.

This script cannot be run directly and needs to be used as an import.
*/

#ifndef CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_CLUSTERTOOLS_H_
#define CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_CLUSTERTOOLS_H_

// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/console.h> 
// PCL libraries
#include <pcl/common/transforms.h>

// Import Eigen tools
#include "eigenTools/generalTools.hpp"


class ClusterTool
{
    public:
        ClusterTool();

        ~ClusterTool();

        int segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud, 
            const double maxDepthDiffSegment, const double maxLengthSegment);

        int cluster(const double maxDistanceBetweenCentersSegmentForCluster);

        int getEdgesSegmentsCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr rOutputCloud);

        std::vector<pcl::PointCloud<pcl::PointXYZI>> getListSegmentsClouds();
        std::vector<pcl::PointCloud<pcl::PointXYZI>> getListClustersClouds();

    private:
        std::vector<pcl::PointCloud<pcl::PointXYZI>> mListSegmentClouds;
        std::vector<pcl::PointCloud<pcl::PointXYZI>> mListClusterClouds;
}; 

#endif // CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_CLUSTERTOOLS_H_ 