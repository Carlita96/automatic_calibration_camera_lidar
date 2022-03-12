/*
In this file, functions related to projecting point cloud to image.
They can be imported and used as wanted.
*/

#include "projectionTool.hpp"

void projectPointCloudToImage(pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, 
        const cv::Mat inputImage, const image_transport::Publisher depthProjectedImagePublisher, 
        const image_transport::Publisher intensityProjectedImagePublisher, const ros::Publisher cloudPublisher,
        const Eigen::Matrix3d intrinsicMatrixKCameraEigen,
        const Eigen::Vector3d translationVector, const Eigen::Vector4d quaternionsVector)
{
    /*
    Method that projects the point cloud received to the image received by the RPY vector and translation vector.
    * Input parameters:
        - pCloud: cloud that will be projected onto the image.
        - inputImage: image where the cloud will be projected.
        - depthProjectedImagePublisher: publisher for point cloud projected by depth.
        - intensityProjectedImagePublisher: publisher for point cloud projected by intensity.
        - cloudPublisher: publisher of point clouds that are in image margin.
		- intrinsicMatrixKCameraEigen: intrinsic K matrix of camera.
        - translationVector: vector of 3 doubles with translation in X, Y and Z.
                The point cloud will be translated by this vector. 
        - quaternionsVector: vector of 4 doubles with quaternions.
    */

    // Define parameters
    Eigen::Vector3d lidarPointVector, cameraPointVector;
    Eigen::Matrix3d rotationMatrix;
    double u, v, depth, intensity;
    std::vector<double> listUPixels;
    std::vector<double> listVPixels;
    std::vector<double> listDepth;
    std::vector<double> listIntensity;
    double minDepth = 1000.0;
    double maxDepth = -1000.0;
    double minIntensity = 1000.0;
    double maxIntensity = -1000.0;

    // Save cloud of points that are in image projection
    pcl::PointCloud<pcl::PointXYZI>::Ptr imageProjectedCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );  

    // Read image and save
    cv::Mat image;
    inputImage.copyTo(image);
    cv::Mat imageWithPointsByDepth;
    cv::Mat imageWithPointsByIntensity;
    image.copyTo(imageWithPointsByDepth);
    image.copyTo(imageWithPointsByIntensity);

    // Get points in image and save depth values
    for(std::size_t i=0; i<pCloud->width; i++){
        for(std::size_t j=0; j<pCloud->height; j++){
            // Not consider NaN values of points
            if (isnan(pCloud->at(i,j).x) || isnan(pCloud->at(i,j).y) || isnan(pCloud->at(i,j).z) ) {
                continue;
            }

            // Go through all points in cloud
            lidarPointVector = Eigen::Vector3d(pCloud->at(i,j).x, pCloud->at(i,j).y, pCloud->at(i,j).z);

            // Find the point from camera frame
            rotationMatrix = getRotationMatrixFromQuaternions(quaternionsVector);    
            cameraPointVector = rotationMatrix * lidarPointVector + translationVector;

            // Project point into image plane
            u = intrinsicMatrixKCameraEigen(0, 0) * cameraPointVector[0] / cameraPointVector[2] + intrinsicMatrixKCameraEigen(0, 2);
            v = intrinsicMatrixKCameraEigen(1, 1) * cameraPointVector[1] / cameraPointVector[2] + intrinsicMatrixKCameraEigen(1, 2);

            if ((u > 0) && (u < intrinsicMatrixKCameraEigen(0, 2)*2) && (v > 0) && (v < intrinsicMatrixKCameraEigen(1, 2)*2)
                        && (cameraPointVector[2] > 0)) {
                if (cameraPointVector[2] < minDepth) {
                    minDepth = cameraPointVector[2];
                }
                if (cameraPointVector[2] > maxDepth) {
                    maxDepth = cameraPointVector[2];
                }
                if (pCloud->at(i,j).intensity < minIntensity) {
                    minIntensity = pCloud->at(i,j).intensity;
                }
                if (pCloud->at(i,j).intensity > maxIntensity) {
                    maxIntensity = pCloud->at(i,j).intensity;
                }
                listUPixels.push_back(u);
                listVPixels.push_back(v);
                listDepth.push_back(cameraPointVector[2]);
                listIntensity.push_back(pCloud->at(i,j).intensity);
                imageProjectedCloud->push_back(pCloud->at(i, j));
            }
        }
    }

    // Put color in pixels in image by depth range and intensity range
    int i;
    float colorDepth, colorIntensity;
    for (auto it = std::begin(listUPixels); it != std::end(listUPixels); ++it) { 
        i = it - listUPixels.begin();
		u = listUPixels[i];
		v = listVPixels[i];
        depth = listDepth[i];
        intensity = listIntensity[i];

        // Calculate colour by depth range
        colorDepth = abs(255 - ((depth - minDepth) / (maxDepth - minDepth)) * (255 - 0));
        colorIntensity = abs(((intensity - minIntensity) / (maxIntensity - minIntensity)) * (255 - 0));
        // Draw
        cv::Point center = cv::Point(u, v);
        // Draw circle center
        circle(imageWithPointsByDepth, center, 1, cv::Scalar(colorDepth, colorDepth, colorDepth), 2, cv::LINE_AA);
        circle(imageWithPointsByIntensity, center, 1, cv::Scalar(colorIntensity, colorIntensity, colorIntensity), 2, cv::LINE_AA);
    }
    
    // Publish information
    publishImage(imageWithPointsByDepth, depthProjectedImagePublisher);
    publishImage(imageWithPointsByIntensity, intensityProjectedImagePublisher);
    publishPointCloud(imageProjectedCloud, cloudPublisher, "velodyne");
}

void projectPointCloudToImageInColor(pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, const cv::Mat inputImage,
        const int b, const int g, const int r, const image_transport::Publisher imagePublisher, 
		const Eigen::Matrix3d intrinsicMatrixKCameraEigen,
        const Eigen::Vector3d translationVector, const Eigen::Vector4d quaternionsVector)
{
    /*
    Method that projects the point cloud received to the image received by the RPY vector and translation vector.
    * Input parameters:
        - pCloud: cloud that will be projected onto the image.
        - pImage: image where the cloud will be projected.
        - translationVector: vector of 3 doubles with translation in X, Y and Z.
                The point cloud will be translated by this vector. 
		- intrinsicMatrixKCameraEigen: intrinsic K matrix of camera.
        - quaternionsVector: vector of 4 doubles with quaternions.
        - r, g, b: color of point cloud projected to image.
    */

    // Define parameters
    Eigen::Vector3d lidarPointVector, cameraPointVector;
    Eigen::Matrix3d rotationMatrix;
    double u, v;

    // Save cloud of points that are in image projection
    pcl::PointCloud<pcl::PointXYZI>::Ptr imageProjectedCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );  

    // Read image and save
    cv::Mat image;
    inputImage.copyTo(image);
    cv::Mat imageWithPoints;
    image.copyTo(imageWithPoints);

    // Get points in image and save depth values
    for(std::size_t i=0; i<pCloud->points.size(); i++){
        // Not consider NaN values of points
        if (isnan(pCloud->points[i].x) || isnan(pCloud->points[i].y) || isnan(pCloud->points[i].z) ) {
            continue;
        }

        // Go through all points in cloud
        lidarPointVector = Eigen::Vector3d(pCloud->points[i].x, pCloud->points[i].y, pCloud->points[i].z);

        // Find the point from camera frame
        rotationMatrix = getRotationMatrixFromQuaternions(quaternionsVector);    
        cameraPointVector = rotationMatrix * lidarPointVector + translationVector;

        // Project point into image plane
        u = intrinsicMatrixKCameraEigen(0, 0) * cameraPointVector[0] / cameraPointVector[2] + intrinsicMatrixKCameraEigen(0, 2);
        v = intrinsicMatrixKCameraEigen(1, 1) * cameraPointVector[1] / cameraPointVector[2] + intrinsicMatrixKCameraEigen(1, 2);

        if ((u > 0) && (u < intrinsicMatrixKCameraEigen(0, 2)*2) && (v > 0) && (v < intrinsicMatrixKCameraEigen(1, 2)*2)
                    && (cameraPointVector[2] > 0)) {
            cv::Point center = cv::Point(u, v);
            circle(imageWithPoints, center, 1, cv::Scalar(r, g, b), 5, cv::LINE_AA);
        }
    }

    // Publish information
    publishImage(imageWithPoints, imagePublisher);
}

int getPointsInCloudWithinImageMargin(const pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, 
        const Eigen::Vector3d translationVector, const Eigen::Vector4d quaternionsVector, 
		const Eigen::Matrix3d intrinsicMatrixKCameraEigen,
        pcl::PointCloud<pcl::PointXYZI>::Ptr rpOutputCloud)
{
    /*
    Method that gets the points inside a point cloud that are within the margins of the image.
    * Input parameters:
        - pCloud: cloud which points will go throw the filter.
        - translationVector and quaternionsVector: vectors defining the calibration parameters between sensors.
    * Output parameters:
        - rpOutputCloud: cloud where the points that are within image margin will be retreived.
    */
    // Define parameters
    Eigen::Vector3d lidarPointVector, cameraPointVector;
    Eigen::Matrix3d rotationMatrix;
    double u, v; 

    // Get points in image and save depth values
    for(std::size_t i=0; i<pCloud->points.size(); i++){
        // Not consider NaN values of points
        if (isnan(pCloud->points[i].x) || isnan(pCloud->points[i].y) || isnan(pCloud->points[i].z) ) {
            continue;
        }

        // Go through all points in cloud
        lidarPointVector = Eigen::Vector3d(pCloud->points[i].x, pCloud->points[i].y, pCloud->points[i].z);

        // Find the point from camera frame
        rotationMatrix = getRotationMatrixFromQuaternions(quaternionsVector);    
        cameraPointVector = rotationMatrix * lidarPointVector + translationVector;

        // Project point into image plane
        u = intrinsicMatrixKCameraEigen(0, 0) * cameraPointVector[0] / cameraPointVector[2] + intrinsicMatrixKCameraEigen(0, 2);
        v = intrinsicMatrixKCameraEigen(1, 1) * cameraPointVector[1] / cameraPointVector[2] + intrinsicMatrixKCameraEigen(1, 2);

        if ((u > 0) && (u < intrinsicMatrixKCameraEigen(0, 2)*2) && (v > 0) && (v < intrinsicMatrixKCameraEigen(1, 2)*2)
                    && (cameraPointVector[2] > 0)) {
            rpOutputCloud->points.push_back(pCloud->points[i]);
        }
    }

    // Check that segments have been found
    if (rpOutputCloud->points.size() == 0) {
        ROS_ERROR_STREAM("POINT CLOUD PROCESS: Getting edges points that are within image margins. " << rpOutputCloud
                << " number of edge points where in image margins.");
        return 1;
    }
    return 0;
}