/*
This class can be used to create a box in a point cloud and crop it. 
It is possible to get or delete the data in that box.

This script cannot be run directly and needs to be used as an import.
*/

#include "cropboxTool.hpp"

CropboxTool::CropboxTool()
{
    // Initialize cloud
    mpCroppedCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );
}

CropboxTool::~CropboxTool(){
}

int CropboxTool::cropCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud, 
    const double minX, const double maxX, const double minY, const double maxY, const double minZ, 
    const double maxZ, const bool getPointsInCropbox) 
{
    /*
    Method that does a cropbox to the inputed cloud with the inputed margins and retrieves the desired points.
    * Input parameters: 
        - pInputCloud: cloud that wants to be treated. It won't be changed in the process.
        - minX and maxX: margins of the cropbox in the X axis.
        - minY and maxY: margins of the cropbox in the Y axis.
        - minZ and maxZ: margins of the cropbox in the Z axis.
        - getPointsInCropbox: if true, it retrieves a cloud with the points in the margins. 
            If false it retrieves a cloud with the points outside the margins
    */
    ROS_DEBUG_STREAM("CROPBOX TOOL: Doing cropbox between minX: " << minX << ", maxX: " << maxX << ", minY: " 
        << minY << ", maxY: " << maxY << ", minZ: " << minZ << " and maxZ: " << maxZ << ". Translation of box: ("
        << mTranslation[0] << ", " << mTranslation[1] << ", " << mTranslation[2] << ") and rotation: (" << mRotation[0]
        << ", " << mRotation[1] << ", " << mRotation[2] << ").");
    // Check input cloud
    if (pInputCloud->points.size() == 0) {
        ROS_ERROR_STREAM("CROPBOX TOOL: Input cloud has size " << pInputCloud->points.size() << ".");
        return 1;
    }
    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Define margins
	Eigen::Vector4f minPoint = Eigen::Vector4f(minX, minY, minZ, 0);
	Eigen::Vector4f maxPoint = Eigen::Vector4f(maxX, maxY, maxZ, 0);

    // Do cropbox
    mCropBoxFilter.setInputCloud(pInputCloud);
    mCropBoxFilter.setMin(minPoint);
    mCropBoxFilter.setMax(maxPoint);

    // Translate and rotate the box
    mCropBoxFilter.setTranslation(mTranslation);
    mCropBoxFilter.setRotation(mRotation);

    // Get the filtered cloud
    mCropBoxFilter.setNegative (!getPointsInCropbox);
    mCropBoxFilter.filter(*mpCroppedCloud);

    // Check output cloud
    if (mpCroppedCloud->points.size() == 0) {
        ROS_WARN_STREAM("CROPBOX TOOL: Output cloud has size " << mpCroppedCloud->points.size() << ".");
        return 1;
    }
    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("CROPBOX TOOL: Cropbox done in " << duration.count() << " seconds. Input cloud has size " << pInputCloud->points.size() 
        << " and output cloud has size " << mpCroppedCloud->points.size() << ".");
    return 0;
}

pcl::PointCloud<pcl::PointXYZI> CropboxTool::getCroppedCloud() 
{
    /*
    Method that retrieves the outputCloud calculated in cropbox
    */
    // Check cropbox has been done
    if (mpCroppedCloud->points.size() == 0) {
        ROS_ERROR_STREAM("CROPBOX TOOL: Output cloud has size " << mpCroppedCloud->points.size() 
                << ". Call the function cropbox to create this pointcloud.");
        return *mpCroppedCloud;
    }
    return *mpCroppedCloud;
}