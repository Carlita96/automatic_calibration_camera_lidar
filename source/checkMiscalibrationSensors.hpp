/*
*/

#ifndef CALIBRATION_SOURCE_CHECKMISCALIBRATIONSENSORS_H_
#define CALIBRATION_SOURCE_CHECKMISCALIBRATIONSENSORS_H_


// Import necessary libraries
// General libraries
#include <experimental/filesystem>
#include <cstdio>
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
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
// Eigen libraries
#include <Eigen/Eigen>
// PCL libraries
#include <pcl/common/transforms.h>  
#include <pcl_conversions/pcl_conversions.h> 

// Calibration optimization Tools
#include "rosTools/publishTools.hpp"
#include "eigenTools/rotationMatrixTools.hpp"
// Point cloud Tools
#include "pointCloudTools/clusterTool.hpp"
#include "pointCloudTools/rangeImageTool.hpp"
// Image Tools
#include "imageTools/generalProcessingTool.hpp"
#include "imageTools/projectionTool.hpp"
#include "imageTools/smoothEdgeTool.hpp"


class CheckMiscalibrationSensors
{
    public:
        CheckMiscalibrationSensors();

        ~CheckMiscalibrationSensors();

        void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& pCameraInfoMsg);

        void dataCallback(const sensor_msgs::ImageConstPtr& pImageMsg, const sensor_msgs::PointCloud2::ConstPtr& pPointcloudMsg);

        bool mRequestShutdown = false;

    private:
        // Functions
        void initParameters(ros::NodeHandle &nodeHandle);

        void initTools(); 

        int getCalibrationRate(const pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, const cv::Mat smoothEdgeImage,
                const Eigen::Vector3d translationVector, const Eigen::Vector4d quaternionsVector);

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

        // TF broadcaster
        tf2_ros::StaticTransformBroadcaster mStaticBroadcaster;

        // Publishers
        image_transport::Publisher mEdgesCloudProjectedImagePublisher;
        image_transport::Publisher mCloudProjectedImageDepthPublisher;
        image_transport::Publisher mCloudProjectedImageIntensityPublisher;
        image_transport::Publisher mEdgeImagePublisher;
        image_transport::Publisher mEdgeSmoothImagePublisher;
        image_transport::Publisher mBothEdgesImagePublisher;
        ros::Publisher mImageProjectedCloudPublisher;
        ros::Publisher mEdgesCloudPublisher;

        // Create attributes necessary for saving ROS parameters
        // To listen to the topic
        std::string mPointcloudTopicName;
        std::string mImageTopicName;
        std::string mCameraInfoTopicName;
        // Calibration parameters
        double mTx, mTy, mTz;
        double mQx, mQy, mQz, mQw;

        // Create attributes necessary for saving ROS parameters
        // Miscalibrations
        int mKSizeCloudPointCalibrationRate; 
        double mMinCalibrationRate;
        int mConsecutiveMiscalibrationsAllowed;
        int mMiscalibrationsConsecutive = 0;
        // Process Point cloud
        // Segment
        double mSegmentMaxDepthDiffSegment;
        // Edges
        int mMinNumberEdgesInCloud;
        // Process Image
        // Median Blur
        int mMedianBlurKSize;
        // Canny edges
        int mCannyEdgesMinThreshold;
        int mCannyEdgesMaxThreshold;
        // Sobel
        int mCannyEdgesKSizeSobel;
        // Smooth edge
        int mSmoothEdgeKSizeEdgeCalculation;
        int mSmoothEdgeKSizeEdgeSmooth;
        double mSmoothEdgeAlpha;
        double mSmoothEdgeGamma;

        // Intrinsic parameters
        Eigen::Matrix3d mIntrinsicMatrixKCameraEigen;

        // File writer
        // std::ofstream mOutfile;
        std::string mStartingDatetime;

        // Tools used
        GeneralProcessingTool *mGeneralProcessingTool;
        ClusterTool *mClusterTool;
        SmoothEdgeTool *mSmoothEdgeTool;
        RagenImageTool *mRangeImageTool;
}; 

#endif // CALIBRATION_SOURCE_CHECKMISCALIBRATIONSENSORS_H_ 