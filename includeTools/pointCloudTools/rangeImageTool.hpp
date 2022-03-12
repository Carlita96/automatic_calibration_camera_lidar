/*
This class can be used to .

This script cannot be run directly and needs to be used as an import.
*/

#ifndef CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_RANGEIMAGETOOLS_H_
#define CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_RANGEIMAGETOOLS_H_

// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/console.h> 
// PCL libraries
#include <pcl/common/transforms.h>
#include <pcl/range_image/range_image.h>
#include <pcl/visualization/range_image_visualizer.h>


class RagenImageTool
{
    public:
        RagenImageTool();

        ~RagenImageTool();

        int getRangeImage(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud);

    private:
}; 

#endif // CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_RANGEIMAGETOOLS_H_ 