/*
This class can be used to solve an optimization problem of a 3D-3D calibration.
Process is:
    1. From a list of N numbers of observations, M numbers of optimization problems are done.
        This problem is run on L<N number of random observations from the N observations. 
    2. The distance error sum of all observations in each M optimization problem is calculated.
    3. The problem m from M with lowest distance error sum is considerd the best calibration fit.
    4. All points that are within a threshold right in this best calibration are considered inliers.
    5. Run a last optimization problem with the inliers of the best calibration.
*/

#ifndef CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_SEVERAL3DTO3DCALIBRATIONTOOL_H_
#define CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_SEVERAL3DTO3DCALIBRATIONTOOL_H_


// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
// Eigen libraries
#include <Eigen/Eigen>

// LM Tool
#include "optimizationProblemTools/eigenSolver3Dto3DTool.hpp"
// Rotation Matrix tool
#include "eigenTools/rotationMatrixTools.hpp"
#include "eigenTools/generalTools.hpp"

class Several3Dto3DCalibrationTool
{
    public:
        Several3Dto3DCalibrationTool(const int numberOfCalibration, const int observationsPerCalibration, 
                const double maxDistanceErrorCalibrationForInliers);

        ~Several3Dto3DCalibrationTool();
        
        int getFinalCalibration(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud,
            const std::vector<Eigen::Vector3d> listPointCenterSphericalImage, Eigen::VectorXd &rOutputCalibrationParameters);

    private:
        int implementSeveralCalibration(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud,
            const std::vector<Eigen::Vector3d> listPointCenterSphericalImage, std::vector<Eigen::VectorXd> &rListCalibrationParameters); 
        
        int getInliersBestCalibration(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud,
            const std::vector<Eigen::Vector3d> listPointCenterSphericalImage, const std::vector<Eigen::VectorXd> listCalibrationParameters,
			std::vector<int> &rListIndexFinalCalibration);

        std::vector<int> getRandomObservationsIndex(int numberOfObservationsPerCalibration, int maxNumber, int minNumber=0);

        // Eigen Solver tool
        EigenSolver3Dto3DTool *mEigenSolver3Dto3DTool;

        // Parameters for the tool
        int mNumberOfCalibration;
        int mObservationsPerCalibration;
        double mMaxDistanceErrorCalibrationForInliers;

        // After calibration
	    int mNumberOfCalibrationParameters = 6;
}; 

#endif // CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_SEVERAL3DTO3DCALIBRATIONTOOL_H_ 