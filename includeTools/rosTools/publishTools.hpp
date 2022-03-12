/*
In this file, functions to publish an image, pointcloud or marker are defined.
They can be imported and used as wanted.
*/

#ifndef CALIBRATION_INCLUDETOOLS_ROSTOOLS_PUBLISHTOOLS_H_
#define CALIBRATION_INCLUDETOOLS_ROSTOOLS_PUBLISHTOOLS_H_

// ROS libraries
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <tf2_ros/static_transform_broadcaster.h>
// Image processing libraries
#include <cv_bridge/cv_bridge.h>
// PCL libraries
#include <pcl_conversions/pcl_conversions.h> 

void publishPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, const ros::Publisher publisher, 
        const std::string frame);

void publishColorPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud, const ros::Publisher publisher, 
        const std::string frame);

void publishImage(const cv::Mat image, const image_transport::Publisher publisher);

void publishGrayScaleImage(const cv::Mat image, const image_transport::Publisher publisher);

void publishSphereMarker(const double positionX, const double positionY, const double positionZ, const double size, 
    const ros::Publisher publisher, const std::string frame, const float red, const float green, const float blue);

void publishStaticTf(const double tx, const double ty, const double tz, const double qx, const double qy, 
    const double qz, const double qw, const std::string sourceFrame, const std::string targetFrame,
    tf2_ros::StaticTransformBroadcaster broadcaster);

#endif // CALIBRATION_INCLUDETOOLS_ROSTOOLS_PUBLISHTOOLS_H_