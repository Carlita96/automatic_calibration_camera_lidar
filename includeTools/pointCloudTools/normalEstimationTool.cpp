/*
This class can be used to estimate the normals of a cloud. 
    * Parameters needed:
        - kSearch: K number of closest points that will be used to calculate the normal of a point.
This script cannot be run directly and needs to be used as an import.
*/

#include "normalEstimationTool.hpp"

NormalEstimationTool::NormalEstimationTool(const int kSearch)
{
    // Initialize params
    mKSearch = kSearch;

    // Initialize tools
    mpTree = pcl::search::KdTree<pcl::PointXYZI>::Ptr (new pcl::search::KdTree<pcl::PointXYZI> ());
}

NormalEstimationTool::~NormalEstimationTool()
{
}

int NormalEstimationTool::getNormalsCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud, pcl::PointCloud<pcl::Normal>::Ptr rpCloudNormals) 
{
    /*
    Method that .
    * Input parameters: 
        - pInputCloud: cloud which normals will be calculated.
        - rpCloudNormals: cloud of normals where the results will be retrieved. CLOUD WILL BE OVERWRITTEN.
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();
    ROS_DEBUG_STREAM("NORMAL ESTIMATION TOOL: Estimating normals with K search: " << mKSearch << ".");

    // Check input cloud
    if (pInputCloud->points.size() == 0) {
        ROS_ERROR_STREAM("NORMAL ESTIMATION TOOL: Input cloud has size " << pInputCloud->points.size() << ".");
        return 1;
    }

    // Estimate point normals
    mNormalEstimation.setSearchMethod (mpTree);
    mNormalEstimation.setInputCloud (pInputCloud);
    mNormalEstimation.setKSearch (mKSearch);
    mNormalEstimation.compute (*rpCloudNormals);

    // Check output cloud
    if (rpCloudNormals->points.size() == 0) {
        ROS_WARN_STREAM("NORMAL ESTIMATION TOOL: Output cloud has size " << rpCloudNormals->points.size() << ".");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("NORMAL ESTIMATION TOOL: Normals calculated in " << duration.count() << " seconds. Input cloud has size " 
        << pInputCloud->points.size() << " and output cloud has size " << rpCloudNormals->points.size() << ".");
    return 0;
}