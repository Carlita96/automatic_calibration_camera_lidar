/*
In this file, functions related to Eigen and calculating absolute distance between points.
They can be imported and used as wanted.
*/

#include "generalTools.hpp"


double getAbsoluteDistanceBetweenPoints(const double point1X, const double point1Y, const double point1Z, 
        const double point2X, const double point2Y, const double point2Z) 
{
    /*
    Method that calculates the absolute distance between two 3D points.
    */
    double difX = point1X - point2X;
    double difY = point1Y - point2Y;
    double difZ = point1Z - point2Z;
    double distance = sqrt(pow(difX,2) + pow(difY,2) + pow(difZ,2));
    return distance;
}

double getAbsoluteDistanceBetweenPointsVectors(const Eigen::Vector3d point1, const Eigen::Vector3d point2)
{
    /*
    Method that calculates the absolute distance between two 3D points.
    */
    double difX = point1[0] - point2[0];
    double difY = point1[1] - point2[1];
    double difZ = point1[2] - point2[2];
    double distance = sqrt(pow(difX,2) + pow(difY,2) + pow(difZ,2));
    return distance;
}

double getAbsoluteDistanceErrorTransformation3Dto3D(const Eigen::Vector3d pointPointCloud, const Eigen::Vector3d pointImage, 
		const Eigen::Matrix3d rotationMatrix, const Eigen::Vector3d translationMatrix)
{
	/*
	Method that gets the distance error of an observation with certain calibration parameters.
	* Input parameters:
		- pointPointCloud: point of pointcloud to transform to image.
		- pointImage: point of image.
		- rotationMatrix: rotation matrix of calibration.
		- translationMatrix: translation matrix of calibration.
	*/
	// Get camera point with calibration params
	Eigen::Vector3d lidarPointInCamera = rotationMatrix * pointPointCloud + translationMatrix;
	double distance = sqrt(pow(lidarPointInCamera[0] - pointImage[0], 2) + pow(lidarPointInCamera[1] - pointImage[1], 2)
			 + pow(lidarPointInCamera[2] - pointImage[2], 2));
	return distance;
}

double getAbsoluteDistanceErrorTransformation3Dto2D(const Eigen::Vector3d pointPointCloud, const Eigen::Vector2d pointImage, 
		const Eigen::Matrix3d intrinsicCameraMatrixK,
        const Eigen::Matrix3d rotationMatrix, const Eigen::Vector3d translationMatrix)
{
	/*
	Method that gets the distance error of a 3D point projected to a image plane with rotation and translation matrix.
	* Input parameters:
		- pointPointCloud: point of pointcloud to transform to image.
		- pointImage: point of image.
        - intrinsicCameraMatrixK: K matrix of camera intrinsics.
		- rotationMatrix: rotation matrix of calibration.
		- translationMatrix: translation matrix of calibration.
	*/
	// Get camera point with calibration params
	Eigen::Vector3d point3dInCamera = rotationMatrix * pointPointCloud + translationMatrix;

    // Assuming the depth to be the one of the 3D transformed point of the point cloud
    // calculate the point in the image plane in the 3D world
	Eigen::Vector3d pointImage3d;
	pointImage3d[0] = (pointImage[0] - intrinsicCameraMatrixK(0, 2)) * point3dInCamera[2] / intrinsicCameraMatrixK(0,0);
	pointImage3d[1] = (pointImage[1] - intrinsicCameraMatrixK(1, 2)) * point3dInCamera[2] / intrinsicCameraMatrixK(1,1);
    pointImage3d[2] = point3dInCamera[2];


	// Get absolute distance
	double distance = sqrt(pow(pointImage3d[0] - point3dInCamera[0], 2) + pow(pointImage3d[1] - point3dInCamera[1], 2)
                    + pow(pointImage3d[2] - point3dInCamera[2], 2));

	return distance;
}