/*
This class can be used to solve an optimization problem of a 3D-2D calibration.
Process is:
    1. From a list of N numbers of observations, M numbers of optimization problems are done.
        This problem is run on L<N number of random observations from the N observations. 
    2. The distance error sum of all observations in each M optimization problem is calculated.
    3. The problem m from M with lowest distance error sum is considerd the best calibration fit.
    4. All points that are within a threshold right in this best calibration are considered inliers.
    5. Run a last optimization problem with the inliers of the best calibration.
*/

#ifndef CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_SEVERAL3DTO2DCALIBRATIONTOOL_H_
#define CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_SEVERAL3DTO2DCALIBRATIONTOOL_H_


// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
// Eigen libraries
#include <Eigen/Eigen>

// Ceres Solver Tool
#include "optimizationProblemTools/ceresSolver3Dto2DTool.hpp"
// Rotation Matrix tool
#include "eigenTools/rotationMatrixTools.hpp"
#include "eigenTools/generalTools.hpp"

class Several3Dto2DCalibrationTool
{
    public:
        Several3Dto2DCalibrationTool(const int numberOfCalibration, const int observationsPerCalibration, 
                const double maxDistanceErrorCalibrationForInliers, const Eigen::Matrix3d intrinsicCameraMatrixK);

        ~Several3Dto2DCalibrationTool();
        
        int getFinalCalibration(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud, 
            const std::vector<Eigen::Vector2d> listPixelCenterSphericalTargetImage,
			Eigen::Vector3d &rEulerAngleVector, Eigen::Vector3d &rTranslationVector, Eigen::Quaterniond &rQuaternions);

    private:
        int implementSeveralCalibration(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud,
            const std::vector<Eigen::Vector2d> listPixelCenterSphericalTargetImage, const Eigen::Vector3d startingEulerAngleVector, 
            const Eigen::Vector3d startingTranslationVector, std::vector<Eigen::VectorXd> &listCalibrationParameters); 

        int getInliersBestCalibration(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud,
            const std::vector<Eigen::Vector2d> listPixelCenterSphericalTargetImage, const std::vector<Eigen::VectorXd> listCalibrationParameters,
			std::vector<int> &rListIndexFinalCalibration);

        std::vector<int> getRandomObservationsIndex(int numberOfObservationsPerCalibration, int maxNumber, int minNumber=0);

        // Ceres Solver
        CeresSolver3Dto2DTool *mCeresSolver3Dto2DTool;

        // Parameters for the tool
        int mNumberOfCalibration;
        int mObservationsPerCalibration;
        double mMaxDistanceErrorCalibrationForInliers;
        Eigen::Matrix3d mIntrinsicCameraMatrixK;

        // After calibration
	    int mNumberOfCalibrationParameters;
}; 

#endif // CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_SEVERAL3DTO2DCALIBRATIONTOOL_H_ 