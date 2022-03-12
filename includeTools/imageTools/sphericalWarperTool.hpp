/*
This class can be used to project an image to an sphere of an specific radius.
    * Parameters needed:
        - radiusSphericalScreen: radius of the sphere where the image will be projected.
        - cameraInfoK: intrinsic parameters matrix K. This is a matrix that depends on the camera.
        - cameraInfoR: rotation matrix R. This is a matrix that depends on the camera.
This script cannot be run directly and needs to be used as an import.
*/

#ifndef CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_SPHERICALWARPERTOOLS_H_
#define CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_SPHERICALWARPERTOOLS_H_

// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/console.h>
// OpenCV libraries
#include <opencv2/stitching/warpers.hpp>

class SphericalWarperTool : private cv::detail::SphericalWarper
{
    public:
        using SphericalWarper::warp;
        using SphericalWarper::buildMaps;
        using SphericalWarper::RotationWarperBase::warpBackward;

        SphericalWarperTool(const int radiusSphericalScreen, const cv::Mat cameraInfoK
            , const cv::Mat cameraInfoR);

        ~SphericalWarperTool();
    
        void warpImage(const cv::Mat inputImage, cv::Mat &rOutputImage);

        int unwarpImage(const cv::Mat inputWarpedImage, cv::Mat &rOutputUnwarpedImage);

        double getDepthSphericalTargetByRadius(const double radiusOfSphereInRealLife, const double radiusOfSphereFoundInWarped);

        double getRadiusInImageSphericalTargetByDepth(const double radiusOfSphereInRealLife, const double depthInRealLife);

        double getRadiusSphericalTargetByDepthAndWarpedRadius(const double radiusOfSphereFoundInWarped, const double depthInRealLife);

        int unwarpPoint(const double u, const double v, double &x, double &y);
    
    private:
        // Initialize parameters
        double mRadiusSphericalScreen;
        cv::Mat mCameraInfoK;
        cv::Mat mCameraInfoR;
        cv::Size mInputImageSize = cv::Size(0,0);
        cv::Mat mXMap, mYMap;

        // Parameters for the functioning of the class
        bool mIsInputInfoChanged = true;
}; 

#endif // CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_SPHERICALWARPERTOOLS_H_ 