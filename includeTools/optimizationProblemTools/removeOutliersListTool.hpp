/*
This class can be used to remove outliers from lists of observations in 3D and 2D.
*/

#ifndef CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_REMOVEOUTLIERSLISTTOOL_H_
#define CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_REMOVEOUTLIERSLISTTOOL_H_


// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
// Eigen libraries
#include <Eigen/Eigen>


class RemoveOutliersListTool
{
    public:
        RemoveOutliersListTool();

        ~RemoveOutliersListTool();

        int removeOutliers3d3d(const std::vector<Eigen::Vector3d> list1,
            const std::vector<Eigen::Vector3d> list2, const double kTimesStdDev, std::vector<Eigen::Vector3d> &rOutputList1,
            std::vector<Eigen::Vector3d> &rOutputList2); 

        int removeOutliers3d2d(const std::vector<Eigen::Vector3d> list1,
            const std::vector<Eigen::Vector2d> list2, const double kTimesStdDev, std::vector<Eigen::Vector3d> &rOutputList1,
            std::vector<Eigen::Vector2d> &rOutputList2); 

    private:
        void getMeanAndStdDev3d(const std::vector<Eigen::Vector3d> list,
		        double &rMeanParam1, double &rMeanParam2, double &rMeanParam3, 
                double &rStdDevParam1, double &rStdDevParam2, double &rStdDevParam3);
        
        void getMeanAndStdDev2d(const std::vector<Eigen::Vector2d> list,
		            double &rMeanParam1, double &rMeanParam2, double &rStdDevParam1, double &rStdDevParam2);
}; 

#endif // CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_REMOVEOUTLIERSLISTTOOL_H_ 