/*
This class can be used to solve an optimization problem of a 3D-2D calibration.
The problem is a minimization problem. In this minimization problem, the rotation matrix R and translation vector T
are wanted.
The distance error from a 3D lidar point projected in the image plane
with the R and a the T to the 2D point of the image has to be minimized.
*/

#ifndef CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_CERESSOLVER3DTO2DTOOL_H_
#define CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_CERESSOLVER3DTO2DTOOL_H_


// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
// Eigen libraries
#include <Eigen/Eigen>
// Optimization libraries
#include "ceres/ceres.h"
// Functor algorithm
#include "functor3Dto2D.cpp"

// Rotation Matrix tool
#include "eigenTools/rotationMatrixTools.hpp"


class CeresSolver3Dto2DTool
{
    public:
        CeresSolver3Dto2DTool();

        ~CeresSolver3Dto2DTool();

        int computeCalibrationParams(const std::vector<Eigen::Vector3d> listPositionCenterSphericalTargetPointCloud,
            const std::vector<Eigen::Vector2d> listPixelCenterSphericalImage, const int numberCalibrationParameters, 
			const Eigen::Vector3d startingEulerAngleVector, const Eigen::Vector3d startingTranslationVector, 
            const Eigen::Matrix3d intrinsicCameraMatrix, Eigen::VectorXd &rOutputCalibrationParams,
            Eigen::Quaterniond &rOutputQuaternions); 

    private:

        void buildOptimizationProblem(const std::vector<Eigen::Vector2d> &rPointsImage,
                              const Eigen::Matrix3d intrinsicCameraMatrix,
                              std::vector<Eigen::Vector3d> &rPointsPointCloud,
                              Eigen::Quaterniond &rQuaternionsRotation,
                              Eigen::Vector3d &rTranslationVector,
                              ceres::Problem* pProblem);
}; 

#endif // CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_CERESSOLVER3DTO2DTOOL_H_ 