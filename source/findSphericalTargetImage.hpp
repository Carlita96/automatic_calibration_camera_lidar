/*
This class can be used to find the center of the spherical target in an image.
    * ROS Parameters that can be used:
        * showImageViewer: image viewer window independent of Rviz used for debugging.
        * sphericalTarget/radius: radius in meters of the spherical target used to calibrate the sensors.
        * image: parameters used for finding the spherical target in the image data.
            - sphericalWarper: tool used to warp the image in a spherical screen. This makes spheres be seen as circles.
                - sphereRadius: radius of the sphere where the image will be projected.
            - medianBlur: tool used to blur an image.
                - kSize: median blur will be done with matrix of size kSize x kSize.
            - houghCircles:
                - accumulatorResolution: the inverse ratio of resolution.
                - minimumDistanceBetweenCircles: minimum distance between detected centers.
                - cannyHighThreshold: upper threshold for the internal Canny edge detector (how strong the edges of the circles need to be).
                - minimumVotes: threshold for center detection (how many edge points it needs to find to declare that it's found a circle).
                - minDepth: minimum depth from the camera sensor where the spherical target could be found.
                - maxDepth: maximum depth from the camera sensor where the spherical target could be found.
            - bestCircles:
                - minIntensityPixel: minimum intensity of a pixel for it to be considered a super pixel.
                - minMSP: MSP = super pixels/total pixels * 100. Minimum MSP for a circle to be considered good.
*/

#ifndef CALIBRATION_SOURCE_FINDSPHERICALTARGETIMAGE_H_
#define CALIBRATION_SOURCE_FINDSPHERICALTARGETIMAGE_H_

// Import necessary libraries
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
// Image processing libraries
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// Eigen libraries
#include <Eigen/Eigen>
// Tools
#include "rosTools/publishTools.hpp"
#include "imageTools/generalProcessingTool.hpp"
#include "imageTools/sphericalWarperTool.hpp"

class FindSphericalTargetImage
{
    public:
        FindSphericalTargetImage(const ros::NodeHandle &rNodeHandle);

        ~FindSphericalTargetImage();

        void setCameraIntrinsicParameters(const cv::Mat intrinsicMatrixKCamera, const cv::Mat intrinsicMatrixRCamera);

        int getCenterSphericalTarget(const cv_bridge::CvImageConstPtr pImage, Eigen::Vector2d &rPixelsCenterSphereImage,
                Eigen::Vector3d &rPointCenterSphereImage) ;

    private:
        // Functions
        void initParameters(ros::NodeHandle &nodeHandle);

        void initTools();

        // Attributes
        // Create ros objects necessary for the class
        ros::NodeHandle mNodeHandle;

        // Publishers
        ros::Publisher mSphericalTargetMarkerPublisher;
        image_transport::Publisher mWarpedImage;
        image_transport::Publisher mWarpedImageWithCircle;
        image_transport::Publisher mUnwarpedImageWithCircle;
        image_transport::Publisher mImageHoughCirclesPublisher;
        image_transport::Publisher mImageHoughAndBestCirclePublisher;
        image_transport::Publisher mImageCannyEdgesPublisher;

        // Boolean that defines if class has been initialized properly
        bool mIsToolInitialized = false;

        // Create attributes necessary for saving ROS parameters
        // Spherical target
        double mSphericalTargetRadius;
        // Process Image
        // Spherical warper
        int mSphericalWarperSphereRadius;
        // Median Blur
        int mMedianBlurKSize;
        // Canny edges
        int mCannyEdgesMinThreshold;
        int mCannyEdgesMaxThreshold;
        int mCannyEdgesKSizeSobel;
        // Hough Circles
        int mHoughCirclesAccumulatorResolution;
        int mHoughCirclesMinimumDistanceBetweenCircles;
        int mHoughCirclesCannyHighThreshold;
        int mHoughCirclesMinimumVotes;
        int mHoughCirclesMinDepth;
        int mHoughCirclesMaxDepth;
        // Get best circles
        int mOptimizeCirclesPixelThresholdHorizontalEdge;
        int mOptimizecirclesMinIntensityPixelsEdgesOptimization;
        int mOptimizecirclesNumberPixelAroundRadiusEdgesOptimization;
        int mOptimizecirclesMinIntensityPixelInliers;
        double mOptimizeCirclesMinMSP;
        int mOptimizecirclesNumberPixelAroundRadiusEdgesBest;
        double mOptimizeCirclesMinRateSuperPixelInCircleByAround;

        // Camera info
        cv::Mat mIntrinsicMatrixKCamera;
        cv::Mat mIntrinsicMatrixRCamera;

        // Tools used
        GeneralProcessingTool *mGeneralProcessingTool;
        SphericalWarperTool *mSphericalWarperTool;
}; 

#endif // CALIBRATION_SOURCE_FINDSPHERICALTARGETIMAGE_H_ 