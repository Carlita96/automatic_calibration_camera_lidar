/*
This class can be used to get a plane from a cloud. 
    * Parameters needed:
        - normalDistanceWeigth: relative weight (0-1) to give to the angular distance (0-pi/2) between point normals and the plane normal. 
        - distanceThreshold: how close a point must be to the model in order to be considered an inlier.
        - maxIterations: maximum number of iterations trying to fit the plane.
This script cannot be run directly and needs to be used as an import.
*/

#include "planeSegmentationTool.hpp"

PlaneSegmentationTool::PlaneSegmentationTool(const double normalDistanceWeight, const double distanceThreshold, const int maxIterations)
{
    // Initialize params
    mNormalDistanceWeight = normalDistanceWeight;
    mDistanceThreshold = distanceThreshold;
    mMaxIterations = maxIterations;

    // Initialize segmentation coefficients, inliers and cloud
    mInputCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() ); 
    mCloudPlane = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() ); 
    mCloudWithoutPlane = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() ); 
    mEmptyCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() ); 
    mInputCloudNormals =  pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    mCloudPlaneNormals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    mCloudWithoutPlaneNormals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    mEmptyCloudNormals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    // Initialize coefficients and inliers
    mCoefficientsPlane = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
    mEmptyCoefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
    mInliersPlane = pcl::PointIndices::Ptr (new pcl::PointIndices);

}

PlaneSegmentationTool::~PlaneSegmentationTool()
{
}

int PlaneSegmentationTool::segmentCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud, const pcl::PointCloud<pcl::Normal>::Ptr pInputCloudNormals) 
{
    /*
    Method that segments a plane of the input cloud.
    * Input parameters: 
        - pInputCloud: cloud where the plane will be fitted.
        - pInputCloudNormals: cloud of normals corresponding to the input cloud.
    */
    mSegmentationDone = false;
    ROS_DEBUG_STREAM("PLANE SEGMENTATION TOOL: Doing plane segmentation with normal distance weigth: " << mNormalDistanceWeight 
        << ", distance threshold: " << mDistanceThreshold << " and max iterations: " << mMaxIterations << ".");
    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Check input cloud
    if (pInputCloud->points.size() != pInputCloudNormals->points.size()) {
        ROS_ERROR_STREAM("PLANE SEGMENTATION TOOL: Input cloud has size " << pInputCloud->points.size() 
            << " and cloud of normals has size " << pInputCloudNormals->points.size() << ". Both sizes need to be the same.");
        return 1;
    }
    // Save input cloud
    *mInputCloud = *pInputCloud;
    *mInputCloudNormals = *pInputCloudNormals;

    // Set the segmentation parameters 
    mSegmentation.setOptimizeCoefficients (true);
    mSegmentation.setModelType (pcl::SACMODEL_NORMAL_PLANE);
    mSegmentation.setNormalDistanceWeight (mNormalDistanceWeight);
    mSegmentation.setMethodType (pcl::SAC_RANSAC);
    mSegmentation.setDistanceThreshold (mDistanceThreshold);
    mSegmentation.setMaxIterations (mMaxIterations);
    mSegmentation.setInputCloud (mInputCloud);
    mSegmentation.setInputNormals (mInputCloudNormals);
    // Obtain the plane inliers and coefficients
    mSegmentation.segment (*mInliersPlane, *mCoefficientsPlane);

    // Check if a plane has been segmented
    if (mInliersPlane->indices.size() == 0) {
        ROS_WARN_STREAM("PLANE SEGMENTATION TOOL: Plane segmented has size: " << mInliersPlane->indices.size() << ".");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;
    ROS_DEBUG_STREAM("PLANE SEGMENTATION TOOL: Plane segmentation done in " << duration.count() 
        << " seconds. Plane coefficients: (" << mCoefficientsPlane->values[0] << ", " << mCoefficientsPlane->values[1] << ", "
        << mCoefficientsPlane->values[2] << ", " << mCoefficientsPlane->values[3] << "). Input cloud size: " << mInputCloud->points.size()
        << " and plane cloud size: " << mInliersPlane->indices.size() << ".");
    mSegmentationDone = true;
    return 0;
}

pcl::PointCloud<pcl::PointXYZI> PlaneSegmentationTool::getSegmentedCloud(const bool getPlane) 
{
    /*
    Method that retrieves the cloud of the plane or the cloud without the plane segmented.
    * Input parameters: 
        - getPlane: if true, only the plane segmented will be retrieved. 
            If false, the cloud without the segmented plane will be retrieved.
    */
    // Check segmentation done
    if (!mSegmentationDone) {
        ROS_ERROR_STREAM("PLANE SEGMENTATION TOOL: Cloud cannot be retrieved as function segmentCloud has not been called or has failed.");
        return *mEmptyCloud;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;

    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Extract the planar inliers from the input cloud
    mExtract.setInputCloud (mInputCloud);
    mExtract.setIndices (mInliersPlane);
    mExtract.setNegative (!getPlane);
    // Write the planar inliers to disk
    mExtract.filter (cloud);

    if (getPlane) {
        *mCloudPlane = cloud;
    } else  {
        *mCloudWithoutPlane = cloud;
    }

    // Check output cloud
    if (cloud.points.size() == 0) {
        ROS_WARN_STREAM("PLANE SEGMENTATION TOOL: Output cloud has size " << cloud.points.size()<< ".");
        return *mEmptyCloud;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;
    ROS_DEBUG_STREAM("PLANE SEGMENTATION TOOL: Plane segmentation extraction done in " << duration.count() << " seconds. Input cloud has size " 
        << mInputCloud->points.size() << " and output cloud has size " << cloud.points.size()<< ".");
    return cloud;
}

pcl::PointCloud<pcl::Normal> PlaneSegmentationTool::getSegmentedCloudNormals(const bool getPlane) 
{
    /*
    Method that retrieves the cloud of normals of the plane or the cloud of normals without the plane segmented.
    * Input parameters: 
        - getPlane: if true, only the plane segmented will be retrieved. 
            If false, the cloud of normals without the segmented plane will be retrieved.
    */
    // Check segmentation done
    if (!mSegmentationDone) {
        ROS_ERROR_STREAM("PLANE SEGMENTATION TOOL: Cloud cannot be retrieved as function segmentCloud has not been called or has failed.");
        return *mEmptyCloudNormals;
    }

    pcl::PointCloud<pcl::Normal> cloudNormals;

    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Extract the planar inliers from the input cloud
    mExtractNormals.setInputCloud (mInputCloudNormals);
    mExtractNormals.setIndices (mInliersPlane);
    mExtractNormals.setNegative (!getPlane);
    mExtractNormals.filter (cloudNormals);

    if (getPlane) {
        *mCloudPlaneNormals = cloudNormals;
    } else  {
        *mCloudWithoutPlaneNormals = cloudNormals;
    }

    // Check output cloud
    if (cloudNormals.points.size() == 0) {
        ROS_WARN_STREAM("PLANE SEGMENTATION TOOL: Output cloud of normals has size " << cloudNormals.points.size()<< ".");
        return *mEmptyCloudNormals;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;
    ROS_DEBUG_STREAM("PLANE SEGMENTATION TOOL: Plane segmentation extraction in normals done in " << duration.count() << " seconds. Input cloud has size " 
        << mInputCloudNormals->points.size() << " and output cloud has size " << cloudNormals.points.size()<< ".");
    return cloudNormals;
}

pcl::ModelCoefficients PlaneSegmentationTool::getPlaneCoefficients() 
{
    /*
    Method that retrieves the coefficients of the plane segmented.
    */
    // Check segmentation done
    if (!mSegmentationDone) {
        ROS_ERROR_STREAM("PLANE SEGMENTATION TOOL: Coefficients cannot be retrieved as function segmentCloud has not been called or has failed.");
        return *mEmptyCoefficients;
    }
    return *mCoefficientsPlane;
}

double PlaneSegmentationTool::getRateInliersOutliers()
{
    /*
    Method that returns the rate of inliers in the sphere with respect of the inputed cloud
    */
    if (!mSegmentationDone) {
        ROS_ERROR_STREAM("PLANE SEGMENTATION TOOL: Inlier rate cannot be retrieved as function segmentCloud has not been called or has failed.");
        return -1;
    }
    return mInliersPlane->indices.size() / mInputCloud->points.size();
}
