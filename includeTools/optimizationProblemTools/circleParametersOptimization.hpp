/*
*/

#ifndef CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_CIRCLEPARAMETERSOPTIMIZERTOOL_H_
#define CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_CIRCLEPARAMETERSOPTIMIZERTOOL_H_


// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
// Eigen libraries
#include <Eigen/Eigen>


class CircleParametersOptimization
{
    public:
        CircleParametersOptimization();

        ~CircleParametersOptimization();

        int getCircleParametersFromPoints(const  std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > &circlePoints, 
                double &rXCenterCircle, double &rYCenterCircle, double &rRadiusCircle); 
    private:
}; 

#endif // CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_CIRCLEPARAMETERSOPTIMIZERTOOL_H_ 