/*
In this file, functions related to projecting point cloud to image.
They can be imported and used as wanted.
*/

#ifndef CALIBRATION_INCLUDETOOLS_IMAGETOOLS_PROJECTIONTOOLs_H_
#define CALIBRATION_INCLUDETOOLS_IMAGETOOLS_PROJECTIONTOOLs_H_


// Import necessary libraries
// ROS libraries
#include <image_transport/image_transport.h>
// PCL libraries
#include <pcl/common/transforms.h>
// Eigen libraries
#include <Eigen/Eigen>
// OpenCV libraries
#include <opencv2/imgproc.hpp>
// Calibration optimization Tools
#include "rosTools/publishTools.hpp"
// Eigen Tool
#include "eigenTools/rotationMatrixTools.hpp"

void projectPointCloudToImage(pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, 
        const cv::Mat inputImage, const image_transport::Publisher depthProjectedImagePublisher, 
        const image_transport::Publisher intensityProjectedImagePublisher, const ros::Publisher cloudPublisher,
        const Eigen::Matrix3d intrinsicMatrixKCameraEigen,
        const Eigen::Vector3d translationVector, const Eigen::Vector4d quaternionsVector);

void projectPointCloudToImageInColor(pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, const cv::Mat inputImage,
        const int b, const int g, const int r, const image_transport::Publisher imagePublisher, 
	const Eigen::Matrix3d intrinsicMatrixKCameraEigen,
        const Eigen::Vector3d translationVector, const Eigen::Vector4d quaternionsVector);

int getPointsInCloudWithinImageMargin(const pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, 
        const Eigen::Vector3d translationVector, const Eigen::Vector4d quaternionsVector, 
		const Eigen::Matrix3d intrinsicMatrixKCameraEigen,
        pcl::PointCloud<pcl::PointXYZI>::Ptr rpOutputCloud);

#endif // CALIBRATION_INCLUDETOOLS_IMAGETOOLS_PROJECTIONTOOLs_H_