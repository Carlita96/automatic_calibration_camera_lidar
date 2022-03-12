/*
This class can be used to get a sphere from a cloud. 
    * Parameters needed:
        - normalDistanceWeigth: relative weight (0-1) to give to the angular distance (0-pi/2) between point normals and the sphere normal. 
        - distanceThreshold: how close a point must be to the model in order to be considered an inlier.
        - maxIterations: maximum number of iterations trying to fit the sphere.
        - minRadius and maxRadius: set the threshold for the radius of the sphere to segment.
This script cannot be run directly and needs to be used as an import.
*/

#include "sphereSegmentationTool.hpp"

SphereSegmentationTool::SphereSegmentationTool(const double normalDistanceWeight, const double distanceThreshold, const int maxIterations,
    const double minRadius, const double maxRadius)
{
    // Initialize params
    mNormalDistanceWeight = normalDistanceWeight;
    mDistanceThreshold = distanceThreshold;
    mMaxIterations = maxIterations;
    mMinRadius = minRadius;
    mMaxRadius = maxRadius;

    // Initialize segmentation coefficients, inliers and cloud
    mInputCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() ); 
    mCloudSphere = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() ); 
    mCloudWithoutSphere = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() ); 
    mEmptyCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() ); 
    mInputCloudNormals =  pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    mCloudSphereNormals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    mCloudWithoutSphereNormals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    mEmptyCloudNormals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    // Initialize coefficients and inliers
    mCoefficientsSphere = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
    mEmptyCoefficients = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
    mInliersSphere = pcl::PointIndices::Ptr (new pcl::PointIndices);

}

SphereSegmentationTool::~SphereSegmentationTool()
{
}

int SphereSegmentationTool::segmentCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud, 
        const pcl::PointCloud<pcl::Normal>::Ptr pInputCloudNormals) 
{
    /*
    Method that segments a sphere of the input cloud.
    * Input parameters: 
        - pInputCloud: cloud where the sphere will be fitted.
        - pInputCloudNormals: cloud of normals corresponding to the input cloud.
    */
    mSegmentationDone = false;
    ROS_DEBUG_STREAM("SPHERE SEGMENTATION TOOL: Doing sphere segmentation with normal distance weigth: " << mNormalDistanceWeight 
        << ", distance threshold: " << mDistanceThreshold << ", max iterations: " << mMaxIterations << ", min radius: "
        << mMinRadius << " and max radius: " << mMaxRadius << ".");
    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Check input cloud
    if (pInputCloud->points.size() != pInputCloudNormals->points.size()) {
        ROS_ERROR_STREAM("SPHERE SEGMENTATION TOOL: Input cloud has size " << pInputCloud->points.size() 
            << " and cloud of normals has size " << pInputCloudNormals->points.size() << ". Both sizes need to be the same.");
        return 1;
    }
    if (pInputCloud->points.size() < 4) {
        ROS_ERROR_STREAM("SPHERE SEGMENTATION TOOL: Input cloud has size " << pInputCloud->points.size() 
            << " and at least 4 are needed.");
        return 1;
    }
    // Save input cloud
    *mInputCloud = *pInputCloud;
    *mInputCloudNormals = *pInputCloudNormals;

    // Set the segmentation parameters
    mSegmentation.setOptimizeCoefficients (true);
    mSegmentation.setModelType (pcl::SACMODEL_SPHERE);
    mSegmentation.setMethodType (pcl::SAC_RANSAC);
    mSegmentation.setNormalDistanceWeight (mNormalDistanceWeight);
    mSegmentation.setMaxIterations (mMaxIterations);
    mSegmentation.setDistanceThreshold (mDistanceThreshold);
    mSegmentation.setRadiusLimits (mMinRadius, mMaxRadius);
    mSegmentation.setInputCloud (pInputCloud);
    mSegmentation.setInputNormals (pInputCloudNormals);
    // Obtain the sphere inliers and coefficients
    mSegmentation.segment (*mInliersSphere, *mCoefficientsSphere);

    // Check if a sphere has been segmented
    if (mInliersSphere->indices.size() <= 4) {
        ROS_DEBUG_STREAM("SPHERE SEGMENTATION TOOL: No sphere could be fit in the cloud.");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;
    ROS_DEBUG_STREAM("SPHERE SEGMENTATION TOOL: Sphere segmentation done in " << duration.count() 
        << " seconds. Sphere coefficients: (" << mCoefficientsSphere->values[0] << ", " << mCoefficientsSphere->values[1] << ", "
        << mCoefficientsSphere->values[2] << ", " << mCoefficientsSphere->values[3] << "). Input cloud size: " << mInputCloud->points.size()
        << " and sphere cloud size: " << mInliersSphere->indices.size() << ".");
    mSegmentationDone = true;
    return 0;
}

pcl::PointCloud<pcl::PointXYZI> SphereSegmentationTool::getSegmentedCloud(const bool getSphere) 
{
    /*
    Method that retrieves the cloud of the sphere or the cloud without the sphere segmented.
    * Input parameters: 
        - getSphere: if true, only the sphere segmented will be retrieved. 
            If false, the cloud without the segmented sphere will be retrieved.
    */
    // Check segmentation done
    if (!mSegmentationDone) {
        ROS_ERROR_STREAM("SPHERE SEGMENTATION TOOL: Cloud cannot be retrieved as function segmentCloud has not been called or has failed.");
        return *mEmptyCloud;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;

    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Extract the planar inliers from the input cloud
    mExtract.setInputCloud (mInputCloud);
    mExtract.setIndices (mInliersSphere);
    mExtract.setNegative (!getSphere);
    // Write the planar inliers to disk
    mExtract.filter (cloud);

    if (getSphere) {
        *mCloudSphere = cloud;
    } else  {
        *mCloudWithoutSphere = cloud;
    }

    // Check output cloud
    if (cloud.points.size() == 0) {
        ROS_WARN_STREAM("SPHERE SEGMENTATION TOOL: Output cloud has size " << cloud.points.size()<< ".");
        return *mEmptyCloud;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;
    ROS_DEBUG_STREAM("SPHERE SEGMENTATION TOOL: Sphere segmentation extraction done in " << duration.count() << " seconds. Input cloud has size " 
        << mInputCloud->points.size() << " and output cloud has size " << cloud.points.size()<< ".");
    return cloud;
}

pcl::PointCloud<pcl::Normal> SphereSegmentationTool::getSegmentedCloudNormals(const bool getSphere) 
{
    /*
    Method that retrieves the cloud of normals of the sphere or the cloud of normals without the sphere segmented.
    * Input parameters: 
        - getSphere: if true, only the sphere segmented will be retrieved. 
            If false, the cloud of normals without the segmented sphere will be retrieved.
    */
    // Check segmentation done
    if (!mSegmentationDone) {
        ROS_ERROR_STREAM("SPHERE SEGMENTATION TOOL: Cloud cannot be retrieved as function segmentCloud has not been called or has failed.");
        return *mEmptyCloudNormals;
    }

    pcl::PointCloud<pcl::Normal> cloudNormals;

    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Extract the planar inliers from the input cloud
    mExtractNormals.setInputCloud (mInputCloudNormals);
    mExtractNormals.setIndices (mInliersSphere);
    mExtractNormals.setNegative (!getSphere);
    mExtractNormals.filter (cloudNormals);

    if (getSphere) {
        *mCloudSphereNormals = cloudNormals;
    } else  {
        *mCloudWithoutSphereNormals = cloudNormals;
    }

    // Check output cloud
    if (cloudNormals.points.size() == 0) {
        ROS_WARN_STREAM("SPHERE SEGMENTATION TOOL: Output cloud of normals has size " << cloudNormals.points.size()<< ".");
        return *mEmptyCloudNormals;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;
    ROS_DEBUG_STREAM("SPHERE SEGMENTATION TOOL: Sphere segmentation extraction in normals done in " << duration.count() << " seconds. Input cloud has size " 
        << mInputCloudNormals->points.size() << " and output cloud has size " << cloudNormals.points.size()<< ".");
    return cloudNormals;
}

pcl::ModelCoefficients SphereSegmentationTool::getSphereCoefficients() 
{
    /*
    Method that retrieves the coefficients of the sphere segmented.
    */
    // Check segmentation done
    if (!mSegmentationDone) {
        ROS_ERROR_STREAM("SPHERE SEGMENTATION TOOL: Coefficients cannot be retrieved as function segmentCloud has not been called or has failed.");
        return *mEmptyCoefficients;
    }
    return *mCoefficientsSphere;
}

double SphereSegmentationTool::getRateInliersOutliers()
{
    /*
    Method that returns the rate of inliers in the sphere with respect of the inputed cloud
    */
    if (!mSegmentationDone) {
        ROS_ERROR_STREAM("SPHERE SEGMENTATION TOOL: Inlier rate cannot be retrieved as function segmentCloud has not been called or has failed.");
        return -1;
    }
    return (double) mInliersSphere->indices.size() / mInputCloud->points.size();
}
