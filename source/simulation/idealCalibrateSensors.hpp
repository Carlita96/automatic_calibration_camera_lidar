/*
This class can be used to calibrate two sensors, a LiDAR and a camera.
    * ROS Parameters that can be used:
        * sphericalTarget/radius: radius in meters of the spherical target used to calibrate the sensors.
        * pointcloudTopicName: topic to which subscribe to get point clouds.
        * pointcloudNumberOfMessages: number of messages that will be concatenated from the pointcloud topic.
        * imageTopicName: topic to which subscribe to get point clouds.
        * calibration:
            - outlierRemovalList: an outlier removal filter is done on the list of positions of the spheres.
                - kTimeStdDev: mean and std deviation of X, Y, Z or U, V is calculated. Points with values
                    smaller than (mean - kTimeStdDev * stdDev) or bigger than (mean - kTimeStdDev * stdDev) are deleted.
            - severalCalibration3Dto3D: several calibration are done on random samples from the position of spheres.
                - numberOfCalibration: number of calibrations to implement.
                - observationsPerCalibration: amount of random observations per calibration.
                - maxDistanceErrorCalibrationForInliers: maximum distance in meters for a observation to be considered an
                    inlier of the best calibration.
            - severalCalibration3Dto2D: several calibration are done on random samples from the position of spheres.
                - numberOfCalibration: number of calibrations to implement.
                - observationsPerCalibration: amount of random observations per calibration.
                - maxDistanceErrorCalibrationForInliers: maximum distance in pixels for a observation to be considered an
                    inlier of the best calibration.
*/

#ifndef CALIBRATION_SOURCE_IDEALCALIBRATESENSORS_H_
#define CALIBRATION_SOURCE_IDEALCALIBRATESENSORS_H_


// Import necessary libraries
// General libraries
#include <experimental/filesystem>
#include <chrono>
#include <thread>
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>   
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <visualization_msgs/Marker.h>
#include "gazebo_msgs/GetModelState.h"
#include <tf/transform_listener.h>
// Eigen libraries
#include <Eigen/Eigen>

// Tools for finding the spherical target
#include "../findSphericalTargetImage.hpp" 
#include "../findSphericalTargetPointCloud.hpp"

// Calibration optimization Tools
#include "optimizationProblemTools/removeOutliersListTool.hpp"
#include "optimizationProblemTools/ceresSolver3Dto2DTool.cpp"
#include "optimizationProblemTools/several3Dto3DCalibrationTool.hpp"
#include "optimizationProblemTools/several3Dto2DCalibrationTool.hpp"


#define CV_OVERRIDE   override

class CalibrateSensors
{
    public:
        CalibrateSensors();

        ~CalibrateSensors();

        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& pCameraInfoMsg);

        void dataCallback(const sensor_msgs::ImageConstPtr& pImageMsg, const sensor_msgs::PointCloud2::ConstPtr& pPointcloudMsg);

        bool mRequestShutdown = false;

    private:
        // Functions
        void initParameters(ros::NodeHandle &nodeHangle);

        void initTools(); 

        int calibrateSensors();

        void datetime();

        // Attributes
        // Create ros objects necessary for the class
        ros::NodeHandle nodeHandle;

        // Subscribers and sync policies definition
        ros::Subscriber mCameraInfoSubscriber;
        message_filters::Subscriber<sensor_msgs::Image> mImageSubscriber;
        message_filters::Subscriber<sensor_msgs::PointCloud2> mPointCloudSubscriber;
        // There are different sync_policies like ExactTime or ApproximateTime
        typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> mpSync;
        // Service of spheric target pose
        ros::ServiceClient mSphericalTargetModelStateServiceClient;
        tf::TransformListener mTfListener;

        // File writer
        std::ofstream mOutfile;
        std::string mStartingDatetime;

        // Intirnsic matrix K
        Eigen::Matrix3d mIntrinsicMatrixKCameraEigen; 

        // List with spherical target position
        std::vector<Eigen::Vector3d> mListPointsCenterSphericalTargetGroundTruth;
        std::vector<Eigen::Vector3d> mListPointsCenterSphericalTargetPointCloud;
        std::vector<Eigen::Vector3d> mListPointsCenterSphericalTargetImage;
        std::vector<Eigen::Vector2d> mListPixelsCenterSphericalTargetImage;

        // Create attributes necessary for saving ROS parameters
        // Setup
        std::string mSphericalTargetGazeboModelName;
        // Analysis of data
        bool mAnalysisDataCalibrationParams;
        int mNumberOfCalibrationsDone = 0;
        int mNumberOfCalibrationsToDo;
        double mTimeSyncError;
        // Spherical target
        double mSphericalTargetRadius;
        // Output files directory
        std::string mOutputFilesDirectory;
        // Calibration parameters
        int mNumberPositions;
        // To listen to the topic
        std::string mPointcloudTopicName;
        std::string mImageTopicName;
        std::string mCameraInfoTopicName;
        // Calibration
        // Outlier removal
        int mOutlierRemovalKTimesStdDev;
        // Several Calibration 3D-3D
        int mSeveralCalibrationNumberOfCalibration3D;
        int mSeveralCalibrationObservationsPerCalibration3D;
        double mSeveralCalibrationMaxDistanceErrorCalibrationForInliers3D;
        // Several Calibration 3D-2D
        int mSeveralCalibrationNumberOfCalibration2D;
        int mSeveralCalibrationObservationsPerCalibration2D;
        double mSeveralCalibrationMaxDistanceErrorCalibrationForInliers2D;

        // Count fails of each process
        bool mParametersDefinedWrongly = true;

        // Tools used
        RemoveOutliersListTool *mRemoveOutliersListTool;
        Several3Dto3DCalibrationTool *mSeveral3Dto3DCalibrationTool;
        Several3Dto2DCalibrationTool *mSeveral3Dto2DCalibrationTool;
}; 

#endif // CALIBRATION_SOURCE_IDEALCALIBRATESENSORS_H_ 