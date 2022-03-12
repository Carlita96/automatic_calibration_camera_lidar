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

#ifndef CALIBRATION_SOURCE_SIMULATECALIBRATESENSORS_H_
#define CALIBRATION_SOURCE_SIMULATECALIBRATESENSORS_H_


// Import necessary libraries
// General libraries
#include <experimental/filesystem>
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>   
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <visualization_msgs/Marker.h>
#include "gazebo_msgs/GetModelState.h"
#include <tf/transform_listener.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// Eigen libraries
#include <Eigen/Eigen>

// Tools for finding the spherical target
#include "../findSphericalTargetImage.hpp" 
#include "../findSphericalTargetPointCloud.hpp"

// Calibration optimization Tools
#include "eigenTools/rotationMatrixTools.hpp"
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

        void projectPointCloudToImage(pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, cv_bridge::CvImageConstPtr pimage,
                Eigen::Vector3d rollPithYawVector=Eigen::Vector3d(0, 0, 0), Eigen::Vector3d tranlationVector=Eigen::Vector3d(0, 0, 0));

        int calibrateSensors();

        int calculatePerformanceSensorFindingTarget();

        void datetime();

        // Attributes
        // Create ros objects necessary for the class
        ros::NodeHandle nodeHandle;

        // Subscribers and sync policies definition
        ros::Subscriber mCameraInfoSubscriber;
        message_filters::Subscriber<sensor_msgs::Image> mImageSubscriber;
        message_filters::Subscriber<sensor_msgs::PointCloud2> mPointCloudSubscriber;
        // There are different sync_policies like ExactTime or ApproximateTime
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
        typedef message_filters::Synchronizer<MySyncPolicy> Sync;
        boost::shared_ptr<Sync> mpSync;
        // Service of spheric target pose
        ros::ServiceClient mSphericalTargetModelStateServiceClient;
        tf::TransformListener mTfListener;

        // Publishers
        image_transport::Publisher mCloudProjectedImagePublisher;

        // File writer
        std::ofstream mOutfile;
        std::string mStartingDatetime;

        // List with spherical target position
        std::vector<Eigen::Vector3d> mListPointsCenterSphericalTargetGroundTruth;
        std::vector<Eigen::Vector3d> mListPointsCenterSphericalTargetPointCloud;
        std::vector<Eigen::Vector3d> mListPointsCenterSphericalTargetImage;
        std::vector<Eigen::Vector2d> mListPixelsCenterSphericalTargetImage;

        // Create attributes necessary for saving ROS parameters
        // Setup
        std::string mSphericalTargetGazeboModelName;
        // Analysis of data
        bool mTesting;
        bool mSeeProjectPointCloudToImage;
        bool mAnalysisDataCalibrationParams;
        int mNumberOfCalibrationsDone = 0;
        int mNumberOfCalibrationsToDo;
        bool mAnalysisDataAccuracySensors;
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
        double mOutlierRemovalKTimesStdDev;
        // Several Calibration 3D-3D
        int mSeveralCalibrationNumberOfCalibration3D;
        int mSeveralCalibrationObservationsPerCalibration3D;
        double mSeveralCalibrationMaxDistanceErrorCalibrationForInliers3D;
        // Several Calibration 3D-2D
        int mSeveralCalibrationNumberOfCalibration2D;
        int mSeveralCalibrationObservationsPerCalibration2D;
        double mSeveralCalibrationMaxDistanceErrorCalibrationForInliers2D;

        // Count fails of each process
        int mFailPointCloud = 0;
        int mFailImage = 0;
        bool mParametersDefinedWrongly = true;
        Eigen::Matrix3d mIntrinsicMatrixKCameraEigen;

        // Calibration params
        Eigen::Vector3d eulerAnglesCalibrationVector;
        Eigen::Vector3d translationCalibrationVector;
        bool mIsCalibrationDone = false;

        // Tools used
        FindSphericalTargetImage *mFindSphericalTargetImage;
        FindSphericalTargetPointCloud *mFindSphericalTargetPointCloud;
        RemoveOutliersListTool *mRemoveOutliersListTool;
        Several3Dto3DCalibrationTool *mSeveral3Dto3DCalibrationTool;
        Several3Dto2DCalibrationTool *mSeveral3Dto2DCalibrationTool;
}; 

#endif // CALIBRATION_SOURCE_SIMULATECALIBRATESENSORS_H_ 