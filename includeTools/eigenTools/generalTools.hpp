/*
In this file, functions related to Eigen and calculating absolute distance between points.
They can be imported and used as wanted.
*/

#ifndef CALIBRATION_INCLUDETOOLS_EIGENTOOLS_GENERALTOOLS_H_
#define CALIBRATION_INCLUDETOOLS_EIGENTOOLS_GENERALTOOLS_H_

// Eigen libraries
#include <Eigen/Eigen>

double getAbsoluteDistanceBetweenPoints(const double point1X, const double point1Y, const double point1Z, 
        const double point2X, const double point2Y, const double point2Z);

double getAbsoluteDistanceBetweenPointsVectors(const Eigen::Vector3d point1, const Eigen::Vector3d point2);

double getAbsoluteDistanceErrorTransformation3Dto3D(const Eigen::Vector3d pointPointCloud, const Eigen::Vector3d pointImage, 
		const Eigen::Matrix3d rotationMatrix, const Eigen::Vector3d translationMatrix);

double getAbsoluteDistanceErrorTransformation3Dto2D(const Eigen::Vector3d pointPointCloud, const Eigen::Vector2d pointImage, 
	const Eigen::Matrix3d intrinsicCameraMatrixK,
        const Eigen::Matrix3d rotationMatrix, const Eigen::Vector3d translationMatrix);

#endif // CALIBRATION_INCLUDETOOLS_EIGENTOOLS_GENERALTOOLS_H_