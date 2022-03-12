/*
This class can be used to solve an optimization problem of a 3D-3D calibration.
The problem is a minimization problem. In this minimization problem, the rotation matrix R and translation vector T
are wanted.
The distance error from a 3D lidar point transformed by R and T 
to the 3D point of the image data has to be minimized.
*/

#ifndef CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_EIGENSOLVER3DTO3DTOOL_H_
#define CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_EIGENSOLVER3DTO3DTOOL_H_


// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
// Eigen-LM libraries
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>
// Functor for LM algorithm
#include "functor3Dto3D.cpp"


class EigenSolver3Dto3DTool
{
    public:
        EigenSolver3Dto3DTool();

        ~EigenSolver3Dto3DTool();

        int computeCalibrationParams(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud,
            const std::vector<Eigen::Vector3d> listPointCenterSphericalImage, const int numberCalibrationParameters,
                Eigen::VectorXd &rOutputCalibrationParams); 

    private:
}; 

#endif // CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_EIGENSOLVER3DTO3DTOOL_H_ 