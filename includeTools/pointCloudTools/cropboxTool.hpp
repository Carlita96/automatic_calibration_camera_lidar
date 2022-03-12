/*
This class can be used to crop a box in a point cloud and get or delete the data in that box.
    * Parameters needed:
        - minX and maxX: margins in the X axis.
        - minY and maxY: margins in the Y axis.
        - minZ and maxZ: margins in the Z axis.
        - translation: translation parameters for positioning the box of the margins specified.
        - rotation: rotation parameters for positioning the box of the margins specified.
        - getPointsInCropbox: if true, it retrieves a cloud with the points in the margins. 
            If false it retrieves a cloud with the points outside the margins.
This script cannot be run directly and needs to be used as an import.
*/

#ifndef CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_CROPBOXTOOLS_H_
#define CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_CROPBOXTOOLS_H_

// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/console.h> 
// PCL libraries
#include <pcl/filters/crop_box.h>


class CropboxTool
{
    public:
        CropboxTool();

        ~CropboxTool();

        int cropCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud,  
            const double minX, const double maxX, const double minY, const double maxY, const double minZ, 
            const double maxZ, const bool getPointsInCropbox); 

        pcl::PointCloud<pcl::PointXYZI> getCroppedCloud(); 

    private:
        // Initialization of tools
        pcl::CropBox<pcl::PointXYZI> mCropBoxFilter;

        // Initialization of default parameters
        Eigen::Vector3f mTranslation = Eigen::Vector3f(0,0,0);
	    Eigen::Vector3f mRotation = Eigen::Vector3f(0,0,0);

        // Initialization of clouds
        pcl::PointCloud<pcl::PointXYZI>::Ptr mpCroppedCloud;
}; 

#endif // CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_CROPBOXTOOLS_H_ 