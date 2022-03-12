/*
This class can be used to project an image to an sphere of an specific radius.
    * Parameters needed:
        - radiusSphericalScreen: radius of the sphere where the image will be projected.
        - cameraInfoK: intrinsic parameters matrix K. This is a matrix that depends on the camera.
        - cameraInfoR: rotation matrix R. This is a matrix that depends on the camera.
This script cannot be run directly and needs to be used as an import.
*/


#include "sphericalWarperTool.hpp"

SphericalWarperTool::SphericalWarperTool(const int radiusSphericalScreen, const cv::Mat cameraInfoK,
    const cv::Mat cameraInfoR) : SphericalWarper(radiusSphericalScreen)
{
    // Initialize attributes
    mRadiusSphericalScreen = radiusSphericalScreen;
    mCameraInfoK = cameraInfoK;
    mCameraInfoR = cameraInfoR;
    mIsInputInfoChanged = true;

    // Empty mapping
    cv::Mat empty;
    empty.copyTo(mXMap);
    empty.copyTo(mYMap);
}

SphericalWarperTool::~SphericalWarperTool(){
}

void SphericalWarperTool::warpImage(const cv::Mat inputImage, cv::Mat &rOutputImage)
{
    /*
    Method that projects an image to an sphere of radius inputed in the class construction.
    * Input parameters:
        - inputImage: image that will be projected into the sphere.
        - rOutputImage: where the image projected into the sphere will be saved. THIS VALUE WILL BE OVERWRITTEN.
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();

    ROS_DEBUG_STREAM("SPHERIC SCREEN PROJECTION TOOL: Projecting the image to an spheric screen of radius: " << mRadiusSphericalScreen 
        << ". K: " << mCameraInfoK << ". R: " << mCameraInfoR << ".");
    
    // Warp image
    warp(inputImage, mCameraInfoK, mCameraInfoR, cv::INTER_LINEAR, cv::BORDER_CONSTANT, rOutputImage);

    // Empty Mapping
    if (mInputImageSize != inputImage.size()) {
        mInputImageSize = inputImage.size();
        // Empty mapping
        cv::Mat empty;
        empty.copyTo(mXMap);
        empty.copyTo(mYMap);
    }
    mIsInputInfoChanged = false;

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("SPHERIC SCREEN PROJECTION TOOL: Projecting the image to an spheric screen done in " 
        << duration.count() << " seconds.");
}

int SphericalWarperTool::unwarpImage(const cv::Mat inputWarpedImage, cv::Mat &rOutputUnwarpedImage)
{
    /*
    Method that projects an image to an sphere of radius inputed in the class construction.
    * Input parameters:
        - inputWarpedImage: image that will be warped back from the sphere.
        - rOutputUnwarpedImage: where the image that will be warped back from the sphere will be saved. THIS VALUE WILL BE OVERWRITTEN.
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();

    if (mIsInputInfoChanged) {
        ROS_WARN_STREAM("SPHERIC SCREEN PROJECTION TOOL: Please run function warpImage first, in order to know the input image information.");
        return 1;
    }

    ROS_DEBUG_STREAM("SPHERIC SCREEN PROJECTION TOOL: Projecting the image from an spheric screen of radius: " << mRadiusSphericalScreen 
        << ". K: " << mCameraInfoK << ". R: " << mCameraInfoR << ".");
    // Unwarped
    warpBackward(inputWarpedImage, mCameraInfoK, mCameraInfoR, cv::INTER_LINEAR, 0, mInputImageSize, rOutputUnwarpedImage);
    
    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("SPHERIC SCREEN PROJECTION TOOL: Projecting the image from an spheric screen done in " << duration.count() << " seconds.");
    return 0;
}

double SphericalWarperTool::getDepthSphericalTargetByRadius(const double radiusOfSphereInRealLife, const double radiusOfSphereFoundInWarped)
{
    /*
    Method that allows to calculate the depth of the sphere in the unwarped image.
    It can be calculated from the radius of the sphere in the warped image, 
        the radius of the spherical screen and the actual radius of the sphere.
    * Input parameres:
        - radiusOfSphereInRealLife: actual radius of the sphere in meters.
        - radiusOfSphereFoundInWarped: radius in pixels of the sphere found in the warped image.
    */
    double depth;
    depth = radiusOfSphereInRealLife / sin(radiusOfSphereFoundInWarped/mRadiusSphericalScreen);
    return depth;
}

double SphericalWarperTool::getRadiusInImageSphericalTargetByDepth(const double radiusOfSphereInRealLife, const double depthInRealLife)
{
    /*
    Method that allows to calculate the radius in pixels of the sphere found in the warped image.
    * Input parameres:
        - radiusOfSphereInRealLife: actual radius of the sphere in meters.
        - depthInRealLife: depth of the sphere in real life.
    */
    double radiusOfSphereFoundInWarped;
    radiusOfSphereFoundInWarped = asin(radiusOfSphereInRealLife / depthInRealLife) * mRadiusSphericalScreen;
    return radiusOfSphereFoundInWarped;
}

double SphericalWarperTool::getRadiusSphericalTargetByDepthAndWarpedRadius(const double radiusOfSphereFoundInWarped, const double depthInRealLife)
{
    /*
    Method that allows to calculate the radius real life of the spherical target.
    * Input parameres:
        - radiusOfSphereFoundInWarped: radius in pixels of the sphere found in the warped image.
        - depthInRealLife: depth of the sphere in real life.
    */
    double radiusOfSphereInRealLife;
    radiusOfSphereInRealLife = depthInRealLife * sin(radiusOfSphereFoundInWarped / mRadiusSphericalScreen);
    return radiusOfSphereInRealLife;
}

int SphericalWarperTool::unwarpPoint(const double u, const double v, double &rX, double &rY)
{
    /*
    Method that gives the corresponding point in the original image from a point in the warped image.
    * Input parameters:
        - (u, v): point in the warped image.
        - (rX, rY): corresponding point of (u,v) in the original unwarped image. THEIR VALUE WILL BE OVERWRITTEN.
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();

    if (mIsInputInfoChanged) {
        ROS_WARN_STREAM("SPHERIC SCREEN PROJECTION TOOL: Please run function warpImage first, in order to know the input image information.");
        return 1;
    }

    // Build map between the points in the screen and the points in the input image
    if (mXMap.size() == cv::Size(0,0) || mYMap.size() == cv::Size(0,0)) {
        ROS_DEBUG_STREAM("SPHERIC SCREEN PROJECTION TOOL: Building map of pixels between input image and spherical screen."
            << " Spheric screen of radius: " << mRadiusSphericalScreen << ". K: " << mCameraInfoK << ". R: " << mCameraInfoR << ".");
        buildMaps(mInputImageSize, mCameraInfoK, mCameraInfoR, mXMap, mYMap);
    }
    ROS_DEBUG_STREAM("SPHERIC SCREEN PROJECTION TOOL: Map of pixels between input image and spherical screen already calculated."
        << " X map size: " << mXMap.size() << " and Y map size: " << mYMap.size() << ".");

    // Get the desired point
    rX = mXMap.at<float>(v, u);
    rY = mYMap.at<float>(v, u);

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("SPHERIC SCREEM PROJECTION TOOL: Unwarping point done in " << duration.count() << " seconds.");
    return 0;
}