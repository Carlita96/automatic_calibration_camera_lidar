/*
This class can be used to find a spherical target from an image.
For that, define the intrinsic camera information at setCameraIntrinsicParameters.
Then, call getCenterSphericalTarget to get the center both in the image and in 3D.
*/

#include "findSphericalTargetImage.hpp"

FindSphericalTargetImage::FindSphericalTargetImage(const ros::NodeHandle &rNodeHandle)
{
    // Read the ros parameters that define the parameters for the problem
    mNodeHandle = rNodeHandle;
    initParameters(mNodeHandle);

    // Image transport
    image_transport::ImageTransport it(mNodeHandle);     

    // Initialize publishers
    mSphericalTargetMarkerPublisher = mNodeHandle.advertise<visualization_msgs::Marker>("sphere_marker_image", 1);
    mWarpedImage = it.advertise("warped_image", 1);
    mWarpedImageWithCircle = it.advertise("warped_image_with_circle", 1);
    mUnwarpedImageWithCircle = it.advertise("unwarped_image_with_circle", 1);
    mImageHoughCirclesPublisher = it.advertise("hough_circles", 1);
    mImageHoughAndBestCirclePublisher = it.advertise("hough_and_best_circles", 1);
    mImageCannyEdgesPublisher = it.advertise("canny_edges", 1);

    ROS_INFO_STREAM("Find Spherical Target Image class init OK");
}

FindSphericalTargetImage::~FindSphericalTargetImage()
{
    delete mGeneralProcessingTool;
    delete mSphericalWarperTool;
}

void FindSphericalTargetImage::initParameters(ros::NodeHandle &nodeHandle) 
{
    /*
    Method that initialize the parameters declared in ROS. These are:
        - showImageViewer: image viewer window independent of Rviz used for debugging.
        - sphericalTarget/radius: radius in meters of the spherical target used to calibrate the sensors.
        - image: parameters used for finding the spherical target in the image data.
    For more detail, read main comment on header file.
    */
    // Spherical target information
    nodeHandle.param<double>( "calibration/sphericalTarget/radius", mSphericalTargetRadius, 0.25 );
    // Image
    // Sphere screen projection
    nodeHandle.param<int>( "calibration/image/sphericalWarper/sphereRadius", mSphericalWarperSphereRadius, 900 );
    // Median blur 
    nodeHandle.param<int>( "calibration/image/medianBlur/kSize", mMedianBlurKSize, 5 );
    // Canny edges
    nodeHandle.param<int>( "calibration/image/cannyEdges/minThreshold", mCannyEdgesMinThreshold, 30 );
    nodeHandle.param<int>( "calibration/image/cannyEdges/maxThreshold", mCannyEdgesMaxThreshold, 120 );
    nodeHandle.param<int>( "calibration/image/cannyEdges/kSizeSobel", mCannyEdgesKSizeSobel, 5 );
    // Hough Circles 
    nodeHandle.param<int>( "calibration/image/houghCircles/accumulatorResolution", mHoughCirclesAccumulatorResolution, 2 );
    nodeHandle.param<int>( "calibration/image/houghCircles/minimumDistanceBetweenCircles", mHoughCirclesMinimumDistanceBetweenCircles, 5 );
    nodeHandle.param<int>( "calibration/image/houghCircles/cannyHighThreshold", mHoughCirclesCannyHighThreshold, 100 );
    nodeHandle.param<int>( "calibration/image/houghCircles/minimumVotes", mHoughCirclesMinimumVotes, 100 );
    nodeHandle.param<int>( "calibration/image/houghCircles/minDepth", mHoughCirclesMinDepth, 1.0 );
    nodeHandle.param<int>( "calibration/image/houghCircles/maxDepth", mHoughCirclesMaxDepth, 15.0 );
    // Get best circles
    nodeHandle.param<int>( "calibration/image/optimizeCircles/pixelThresholdHorizontalEdge", mOptimizeCirclesPixelThresholdHorizontalEdge, 100 );
    nodeHandle.param<int>( "calibration/image/optimizeCircles/minIntensityPixelsEdgesOptimization", 
            mOptimizecirclesMinIntensityPixelsEdgesOptimization, 200 );
    nodeHandle.param<int>( "calibration/image/optimizeCircles/numberPixelsAroundEdgesOptimization", 
            mOptimizecirclesNumberPixelAroundRadiusEdgesOptimization, 3 );
    nodeHandle.param<int>( "calibration/image/optimizeCircles/minIntensityPixelInliers", mOptimizecirclesMinIntensityPixelInliers, 150 );
    nodeHandle.param<double>( "calibration/image/optimizeCircles/minMSP", mOptimizeCirclesMinMSP, 50.0 );
    nodeHandle.param<int>( "calibration/image/optimizeCircles/numberPixelAroundRadiusEdgesBest", mOptimizecirclesNumberPixelAroundRadiusEdgesBest, 3 );
    nodeHandle.param<double>( "calibration/image/optimizeCircles/minRateSuperPixelInCircleByAround", mOptimizeCirclesMinRateSuperPixelInCircleByAround, 0.5 );
}

void FindSphericalTargetImage::initTools()
{
    /*
    Method where the tools used are initialized.
    */
    mGeneralProcessingTool = new GeneralProcessingTool();
    mSphericalWarperTool = new SphericalWarperTool(mSphericalWarperSphereRadius, mIntrinsicMatrixKCamera, mIntrinsicMatrixRCamera);
}

void FindSphericalTargetImage::setCameraIntrinsicParameters(const cv::Mat intrinsicMatrixKCamera, const cv::Mat intrinsicMatrixRCamera) 
{
    /*
    Method that saves the camera intrinsic parameters into the class.
    * Input parameters:
        - intrinsicMatrixKCamera: K matrix of intrinsic parameters of camera.
        - intrinsicMatrixRCamera: R matrix of intrinsic parameters of camera.
    */
    // Save params in class
    mIntrinsicMatrixKCamera = intrinsicMatrixKCamera;
    mIntrinsicMatrixRCamera = intrinsicMatrixRCamera;

    // Start tools now as some need from these camera parameters to be initialized.
    initTools();
    mIsToolInitialized = true;

    return;
}

int FindSphericalTargetImage::getCenterSphericalTarget(const cv_bridge::CvImageConstPtr pImage, Eigen::Vector2d& rPixelsCenterSphereImage,
        Eigen::Vector3d& rPointCenterSphereImage) 
{
    /*
    Method that calculates the center of the spherical target from the image data.
    * Input parameters:
        - pImage: image that will be processed and where the sphere will be searched.
    * Output parameters:
        - rPixelsCenterSphereImage: output that contains (u, v) of center of spherical target in image.
        - rPointCenterSphereImage: output that contains (x, y, z) of center of spherical target from camera 
                calculated from the image.
    */
    // To check status. 
    int status = 0;
    if (!mIsToolInitialized) {
        ROS_ERROR_STREAM("FINDING SPHERICAL TARGET IMAGE: Some tools do not seem to be initialized. K and R matrix from camera"
                << " are needed. Please call setCameraIntrinsicParameters.");
        return 1;
    }

    // Time the process 
    auto start = std::chrono::system_clock::now();
    ROS_INFO_STREAM("FINDING SPHERICAL TARGET IMAGE: Running process of finding spheric target center in image data.");

    // Define output
    Eigen::Vector2d pixelsCenterSphereImage = Eigen::Vector2d(0,0);
    Eigen::Vector3d pointCenterSphereImage = Eigen::Vector3d(0,0,0);

    // Get input image
    cv::Mat inputImage = pImage->image;

    // Define images and best circle
    cv::Mat inputImageWithCircleCenter;
    cv::Mat wrapedImageWithBestSphere;
    cv::Mat wrapedImage;
    cv::Vec3d bestHoughCircle;

    // STEP 1: Wrap image into spheric screen
    mSphericalWarperTool->warpImage(inputImage, wrapedImage);
    publishImage(wrapedImage, mWarpedImage);

    // Copy input image or warped image to debug images
    inputImage.copyTo(inputImageWithCircleCenter);
    wrapedImage.copyTo(wrapedImageWithBestSphere);

    // STEP 2: Change image to gray scale
    mGeneralProcessingTool->setInputImage(wrapedImage);
    mGeneralProcessingTool->convertToGrayImage();
    if (status == 0){
        // STEP 3: Blur image
        status = mGeneralProcessingTool->blurImage(mMedianBlurKSize, true);
        if (status == 0) {
            // STEP 4: Get Hough circles
            double houghCirclesMinRadius = mSphericalWarperTool->getRadiusInImageSphericalTargetByDepth(mSphericalTargetRadius, mHoughCirclesMaxDepth);
            double houghCirclesMaxRadius = mSphericalWarperTool->getRadiusInImageSphericalTargetByDepth(mSphericalTargetRadius, mHoughCirclesMinDepth);
            status = mGeneralProcessingTool->calculateHoughCircles(mHoughCirclesAccumulatorResolution, mHoughCirclesMinimumDistanceBetweenCircles,
                mHoughCirclesCannyHighThreshold, mHoughCirclesMinimumVotes, houghCirclesMinRadius, houghCirclesMaxRadius);
            if (status == 0) {
                // STEP 5: Calculate Canny edges
                status = mGeneralProcessingTool->doCannyEdges(mCannyEdgesMinThreshold, mCannyEdgesMaxThreshold);
                if (status == 0) { 
                    // STEP 6: Optimize position and radius of circle
                    status = mGeneralProcessingTool->optimizeHoughCircles(mOptimizecirclesMinIntensityPixelsEdgesOptimization,
                            mOptimizecirclesNumberPixelAroundRadiusEdgesOptimization, mOptimizeCirclesPixelThresholdHorizontalEdge);
                    if (status == 0) { 
                        // STEP 7: Calculate best circle
                        status = mGeneralProcessingTool->calculateBestOptimizedHoughCircles(mOptimizecirclesMinIntensityPixelInliers, 
                                    mOptimizeCirclesMinMSP, mOptimizecirclesNumberPixelAroundRadiusEdgesBest,
                                    mOptimizeCirclesMinRateSuperPixelInCircleByAround);
                        if (status == 0) {
                            // STEP 7.1 Get best circle
                            bestHoughCircle = mGeneralProcessingTool->getBestHoughCircle();

                            // Calculate depth of sphere
                            double depth = mSphericalWarperTool->getDepthSphericalTargetByRadius(mSphericalTargetRadius, bestHoughCircle[2]);

                            // STEP 8: Project back center of circle to input image
                            double centerX, centerY;
                            status = mSphericalWarperTool->unwarpPoint(bestHoughCircle[0], bestHoughCircle[1], centerX, centerY);
                            if (status == 0) {
                                // STEP 9: Calculate X and Y of spheric target
                                double xCenterCircle = (centerX - mIntrinsicMatrixKCamera.at<float>(0,2)) * depth / mIntrinsicMatrixKCamera.at<float>(0,0);
                                double yCenterCircle = (centerY - mIntrinsicMatrixKCamera.at<float>(1,2)) * depth / mIntrinsicMatrixKCamera.at<float>(1,1);

                                // Publish input image with center of circle
                                cv::Point center = cv::Point(centerX, centerY);
                                cv::circle(inputImageWithCircleCenter, center, 1, cv::Scalar(0, 0, 255), 10, cv::LINE_AA);
                                publishImage(inputImageWithCircleCenter, mUnwarpedImageWithCircle);
                                // Publish warped image
                                mGeneralProcessingTool->drawBestHoughCirclesInInputImage(wrapedImageWithBestSphere);
                                publishImage(wrapedImageWithBestSphere, mWarpedImageWithCircle);

                                // Save parameters
                                pixelsCenterSphereImage = Eigen::Vector2d(centerX, centerY);
                                pointCenterSphereImage = Eigen::Vector3d(xCenterCircle, yCenterCircle, depth);

                                publishSphereMarker(depth, -xCenterCircle, -yCenterCircle, 
                                        2*mSphericalTargetRadius, mSphericalTargetMarkerPublisher, 
                                        "camera_cam_rear_link", 1.0f, 1.0f, 0.0f);
                            }
                        }
                    }
                } 
            }            
        }
    }
    // Publishing Hough circles 
    cv::Mat houghCirclesImage;
    wrapedImage.copyTo(houghCirclesImage);
    int statusDebug = mGeneralProcessingTool->drawHoughCirclesInInputImage(houghCirclesImage);
    if (statusDebug != 0) {
        ROS_ERROR_STREAM("FINDING SPHERICAL TARGET IMAGE: Not possible to get image with Hough circles.");
    }
    publishImage(houghCirclesImage, mImageHoughCirclesPublisher);

    // Publish Hough circles and best circle for debugging
    cv::Mat imageWithHoughCirclesAndBestSelection;
    wrapedImage.copyTo(imageWithHoughCirclesAndBestSelection);
    mGeneralProcessingTool->drawHoughCirclesInInputImage(imageWithHoughCirclesAndBestSelection);
    mGeneralProcessingTool->drawBestHoughCirclesInInputImage(imageWithHoughCirclesAndBestSelection);
    publishImage(imageWithHoughCirclesAndBestSelection, mImageHoughAndBestCirclePublisher);
    // Publish Canny edges for debugging
    cv::Mat cannyEdges = mGeneralProcessingTool->getCannyEdgesImage();
    publishGrayScaleImage(cannyEdges, mImageCannyEdgesPublisher);

    if (status != 0) {
        ROS_ERROR_STREAM("FINDING SPHERICAL TARGET IMAGE: Process for finding the spheric target in the image data failed.");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_INFO_STREAM("FINDING SPHERICAL TARGET IMAGE: Finding the spheric target in image data was successful. It took " << duration.count()
        << " seconds. Sphere center is at X: " << pointCenterSphereImage[0] << ", Y: " << pointCenterSphereImage[1] << " and Z: " 
        << pointCenterSphereImage[2] << " meters. Pixel u: " << pixelsCenterSphereImage[0] << " and v: " << pixelsCenterSphereImage[1] << ".");

    // Save to output
    rPixelsCenterSphereImage = pixelsCenterSphereImage;
    rPointCenterSphereImage = pointCenterSphereImage;
    return status;
}