/*
This class can be used to do general processes on an image.

This script cannot be run directly and needs to be used as an import.
*/


#include "generalProcessingTool.hpp"

GeneralProcessingTool::GeneralProcessingTool()
{
    mCircleParametersOptimization = new CircleParametersOptimization();
}

GeneralProcessingTool::~GeneralProcessingTool(){
    delete mCircleParametersOptimization;
}

void GeneralProcessingTool::setInputImage(const cv::Mat inputImage)
{
    /*
    Method that defines the input image in the class. 
    The other images calculated from the input image are emptied.
    */
    // Set input image
    inputImage.copyTo(mInputImage);
    mSizeInputImage = mInputImage.size();

    // Initialize the other images
    cv::Mat emptyImage;
    emptyImage.copyTo(mGrayImage);
    emptyImage.copyTo(mBlurImage);
    emptyImage.copyTo(mGrayBlurImage);
    emptyImage.copyTo(mCannyEdgesImage);
    emptyImage.copyTo(mSobelImage);
    mHoughCircles.clear();
    mOptimizedHoughCirclesParams.clear();
    mBestHoughCircleParams = cv::Vec3d(0, 0, 0);

    ROS_DEBUG_STREAM("GENERAL PROCESSING TOOL: Input image has size: " << mInputImage.size() << ".");
}

int GeneralProcessingTool::convertToGrayImage() 
{
    /*
    Method that converts the input image to gray scale. 
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();
    // Check input image
    if (mInputImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Input image has size: " << mInputImage.size() << ". Set using setInputImage.");
        return 1;
    }
    // Check if it already exists
    if (mGrayImage.size() != cv::Size(0,0)) {
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Gray image already calculated. Call getGrayImage to access it.");
        return 0;
    }

    cv::cvtColor(mInputImage, mGrayImage, cv::COLOR_BGR2GRAY);

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("GENERAL PROCESSING TOOL: Image with size: " << mGrayImage.size() 
        << " converted to gray scale. It took " << duration.count() << " seconds.");
    return 0;
}

int GeneralProcessingTool::blurImage(const int kSize, const bool fromGrayScale) 
{
    /*
    Method that blurs the image. 
    * Input parameters:
        - kSize: median blur will be done with matrix of size kSize x kSize.
        - fromGrayScale: if true, the gray image will be blured. If false, the input image will be blured. Default false.
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();
    ROS_DEBUG_STREAM("GENERAL PROCESSING TOOL: Blurring image with K size: " << kSize << ". Blurring gray image: " << fromGrayScale << ".");
    // Check K size param
    if ((kSize % 2 != 1) || (kSize <=2)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Blurring image with K size: " << kSize     
            << ". Wrong number as it should be even and more than 2.");
        return 1;
    }
    // Check input image
    if (mInputImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Input image has size: " << mInputImage.size() << ". Set using setInputImage.");
        return 1;
    }

    // Check if image already exists 
    if (fromGrayScale) {
        if (mGrayBlurImage.size() != cv::Size(0,0)) {
            ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Gray blur image already calculated. Call getBlurGrayImage to access it.");
            return 0;
        }
    } else { // Blur input image
        if (mBlurImage.size() != cv::Size(0,0)) {
            ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Blur image already calculated. Call getBlurImage to access it.");
            return 0;
        }
    }

    // Blur gray image
    cv::Mat blurImage;
    if (fromGrayScale) {
        if (mGrayImage.size() == cv::Size(0,0)) {
            ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Gray image was not calculated. Calling convertGrayImage() for user.");
            convertToGrayImage();
        }
        cv::medianBlur(mGrayImage, blurImage, kSize);
        blurImage.copyTo(mGrayBlurImage);
    } else { // Blur input image
        cv::medianBlur(mInputImage, blurImage, kSize);
        blurImage.copyTo(mBlurImage);
    }

    if (blurImage.size() != mInputImage.size()) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Blured image with size: " << blurImage.size() << " and input image: " << mInputImage.size() << ".");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("GENERAL PROCESSING TOOL: Blurring image done in " << duration.count() << " seconds.");
    return 0;
}

int GeneralProcessingTool::doCannyEdges(const int cannyMinThreshold, const int cannyMaxThreshold) 
{
    /*
    Method that gets the edges of an image by Canny algorithm.
    * Input parameters: 
        - cannyMinHighThreshold: lower threshold for the internal Canny edge detector
            (sensitivity; how strong the edges of the circles need to be).
        - cannyMaxHighThreshold: upper threshold for the internal Canny edge detector
            (sensitivity; how strong the edges of the circles need to be).
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();
    ROS_DEBUG_STREAM("CANNY EDGE TOOL: Getting Canny edges with min threshold: " << cannyMinThreshold 
        << " and max threshold: " << cannyMaxThreshold << ".");
    // Check input image
    if (mInputImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Input image has size: " << mInputImage.size() << ". Set using setInputImage.");
        return 1;
    }
    // Check blur gray image
    if (mGrayBlurImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Gray blur image has size: " << mGrayBlurImage.size() 
            << ". Call using convertToGrayImage and blurImage.");
        return 1;
    }
    // Check if this process has already been done
    if (mCannyEdgesImage.size() != cv::Size(0,0)){
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Canny edges already calculated. Call getCannyEdgesImage or getSobelImage to access them.");
    }

    // Get Canny edges
    cv::Mat cannyEdgesImage;
    cv::Canny(mGrayBlurImage, cannyEdgesImage, cannyMinThreshold, cannyMaxThreshold);

    if (cannyEdgesImage.size() == cv::Size(0,0)){
        ROS_ERROR_STREAM("CANNY EDGE TOOL: Size of image outputed is " << cannyEdgesImage.size() << ".");
        return 1;
    }

    // Save images
    mCannyEdgesImage = cannyEdgesImage;

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("CANNY EDGE TOOL: Canny edges done in " << duration.count() << " seconds.");
    return 0;
}

int GeneralProcessingTool::doSobel(const int kSizeSobel) 
{
    /*
    Method that gets the edges of an image by Canny algorithm.
    * Input parameters: 
        - kSizeSobel: size of matrix for sobel.
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();
    ROS_DEBUG_STREAM("SOBEL TOOL: Getting Sobel image with K size: " << kSizeSobel << ".");
    // Check K size param
    if ((kSizeSobel % 2 != 1) || (kSizeSobel <=2)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Getting sobel edges image with K size: " << kSizeSobel     
            << ". Wrong number as it should be even and more than 2.");
        return 1;
    }
    // Check input image
    if (mInputImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Input image has size: " << mInputImage.size() << ". Set using setInputImage.");
        return 1;
    }
    // Check blur gray image
    if (mGrayBlurImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Gray blur image has size: " << mGrayBlurImage.size() 
            << ". Call using convertToGrayImage and blurImage.");
        return 1;
    }
    // Check if this process has already been done
    if (mSobelImage.size() != cv::Size(0,0)){
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Sobel already calculated. Call getSobelImage to access them.");
    }

    // Do Sobel
    cv::Mat sobelImage;
    cv::Sobel(mGrayBlurImage, sobelImage, -1, 1, 1, kSizeSobel, 1, 0, cv::BORDER_DEFAULT);

    if (sobelImage.size() == cv::Size(0,0)){
        ROS_ERROR_STREAM("SOBEL TOOL: Size of image for Sobel outputed is " << sobelImage.size() << ".");
        return 1;
    }

    // Save images
    mSobelImage = sobelImage;

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("SOBEL TOOL: Sobel done in " << duration.count() << " seconds.");
    return 0;
}

int GeneralProcessingTool::calculateHoughCircles(int accumulatorResolution, int minimumDistanceBetweenCircles, 
    int cannyHighThreshold, int minimumVotes, int minRadius, int maxRadius) 
{
    /*
    Method that does gets circles by Hough Circles method.
    * Input parameters: 
        - accumulatorResolution:the inverse ratio of resolution.
        - minimumDistanceBetweenCircles: minimum distance between detected centers.
        - cannyHighThreshold: upper threshold for the internal Canny edge detector
            (sensitivity; how strong the edges of the circles need to be).
        - minimumVotes: threshold for center detection 
            (how many edge points it needs to find to declare that it's found a circle).
        - minRadius & maxRadius: min and max radius values for a circle to be retrieved.
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();
    ROS_DEBUG_STREAM("HOUGH CIRCLES TOOL: Doing Hough circles with accumulator resolution: " << accumulatorResolution 
        << ", minimum distance between circles: " << minimumDistanceBetweenCircles << ", Canny high threshold: "
        << cannyHighThreshold << ", minimum number of votes: " << minimumVotes << ", minimum radius: "
        << minRadius << " and maximum radius: " << maxRadius << ".");
    // Check input image
    if (mInputImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Input image has size: " << mInputImage.size() << ". Set using setInputImage.");
        return 1;
    }
    // Check blur gray image
    if (mGrayBlurImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Gray blur image has size: " << mGrayBlurImage.size() 
            << ". Call using convertToGrayImage and blurImage.");
        return 1;
    }
    // Check if this process has already been done
    if (mHoughCircles.size() != 0){
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Hough circles already calculated. Call getHoughCircles to access them.");
    }

    // Get hough circles
    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(mGrayBlurImage, circles, cv::HOUGH_GRADIENT, 
                accumulatorResolution,
                minimumDistanceBetweenCircles, 
                cannyHighThreshold, 
                minimumVotes,
                minRadius, maxRadius 
    );

    if (circles.size() == 0){
        ROS_ERROR_STREAM("HOUGH CIRCLES TOOL: " << circles.size() << " number of circles have been found.");
        return 1;
    }

    // Save circles
    mHoughCircles = circles;

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("HOUGH CIRCLES TOOL: Hough circles done in " << duration.count() << " seconds. " << mHoughCircles.size() 
        << " number of circles have been found.");
    return 0;
}

int GeneralProcessingTool::optimizeHoughCircles(const int minIntensityPixelsEdgesOptimization, const int numberPixelAroundRadiusEdgesOptimization,
        const int pixelThresholdHorizontalEdge) 
{
    /*
    Method that optimizes the parameters of Hough Circles method. MSP = Super pixels/total pixels. 
    Super pixel: pixel that is edge in Sobel with intensity higher than the threshold setup
    * Input parameters:
        - minIntensityPixelsEdgesOptimization: minimum intensities for pixels to be considered edges sent to optimization.
        - numberPixelAroundRadiusEdgesOptimization: number of pixels inside and outside the circle for searching for edges.
        - pixelThresholdHorizontalEdge: number of pixels of margin to the horizontal edge. The circles found in this range will be discarted. 
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();
    ROS_DEBUG_STREAM("OPTIMIZE HOUGH CIRCLES TOOL: Getting best circles of Hough circles. Minimum intensity for optimization: " 
        << minIntensityPixelsEdgesOptimization << ", number of pixels around circle: " << numberPixelAroundRadiusEdgesOptimization << ".");
    // Check input image
    if (mInputImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Input image has size: " << mInputImage.size() << ". Set using setInputImage.");
        return 1;
    }
    // Check blur gray image
    if (mGrayBlurImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Gray blur image has size: " << mGrayBlurImage.size() 
            << ". Call using convertToGrayImage and blurImage.");
        return 1;
    }
    // Check if the Hough circle has already been done
    if (mHoughCircles.size() == 0){
        ROS_WARN_STREAM("HOUGH CIRCLES TOOL: Hough circles not calculated. Call calculateHoughCircles to calculate them.");
        return 1;
    }
    // Check if Canny edges have been found e
    if (mCannyEdgesImage.size() == cv::Size(0,0)){
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Canny edges not calculated. Call doCannyEdges to get it.");
    }   
    // Check min intensity parameters
    if (minIntensityPixelsEdgesOptimization > 255) {
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Min intensity pixels edges optimization is bigger than 255. 255 is maximum.");
        return 1;
    }

    // Create needed parameters
    double a, b, r, x, y;
    std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > samplePointsEdgesForCircle;
    std::vector<cv::Vec3d> optimizedCircleParams;
    bool statusOptimization;
    double intensityPixel;
	double xCenterCircle;
	double yCenterCircle;
	double radiusCircle;

    // Go through all Hough circles
    // Go through circles with radius [-3+r, 3+r] and get the pixels in this range that are edge in canny edges
    for (size_t i = 0; i < mHoughCircles.size(); i++) {  
        // Check if circle is within desired bounds in image
        if (mHoughCircles[i][0] < pixelThresholdHorizontalEdge || 
            mHoughCircles[i][0] > mSizeInputImage.width-pixelThresholdHorizontalEdge) {
            continue;
        }      
        // Define different ranges of radius
        for (int j = -numberPixelAroundRadiusEdgesOptimization; j <= numberPixelAroundRadiusEdgesOptimization; j++) {
            // Get params of circle
            a = mHoughCircles[i][0];
            b = mHoughCircles[i][1];
            r = mHoughCircles[i][2] + j;

            // For each circle, get the points of the circle by the formula
            // x = a + r cos(θ) ---- y = b + r sin(θ)
            for (double h = -180.0 ; h <= 180.0 ; h+=0.1) {
                x = a + r * cos(h *3.14159/180);
                y = b + r * sin(h *3.14159/180);

                // Get intensity of pixel and add it to list if bigger than a threshold
                intensityPixel = (int)mCannyEdgesImage.at<uchar>(y, x);
                if (intensityPixel > minIntensityPixelsEdgesOptimization) {
	                samplePointsEdgesForCircle.push_back(Eigen::Vector2d(x, y));
                }
            }
        }

        // Optimize the parameters of the circle by the edges in canny edges found close to the circle
        statusOptimization = mCircleParametersOptimization->getCircleParametersFromPoints(samplePointsEdgesForCircle, 
                xCenterCircle, yCenterCircle, radiusCircle);
        // Save the optimized parameters if successful or save old parameters if failed
        if (statusOptimization == 0) {
            cv::Vec3d circleParams = cv::Vec3d(xCenterCircle, yCenterCircle, radiusCircle);
            optimizedCircleParams.push_back(circleParams);
        } else {
            cv::Vec3d circleParams = cv::Vec3d(a,b,r);
            optimizedCircleParams.push_back(circleParams);
        }

        // Clear the list of points for the optimization problem
        samplePointsEdgesForCircle.clear();
    }

    // Check output
    if (optimizedCircleParams.size() == 0){
        ROS_ERROR_STREAM("OPTIMIZE HOUGH CIRCLES TOOL: Circle optimization failed.");
        return 1;
    }

    // Save 
    mOptimizedHoughCirclesParams = optimizedCircleParams;

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("OPTIMIZE HOUGH CIRCLES TOOL: Optimization done in " << duration.count() << " seconds." << mHoughCircles.size() 
        << " number circles before and " << mOptimizedHoughCirclesParams.size() << " number after.");
    return 0;
}

int GeneralProcessingTool::calculateBestOptimizedHoughCircles(const int minIntensityPixelsEdgesInliers, const double minMsp,
        const int numberPixelAroundRadiusEdgesBest, const double minRateSuperPixelInCircleByAround) 
{
    /*
    Method that does gets circles by Hough Circles method. MSP = Super pixels/total pixels. 
    Super pixel: pixel that is edge in Sobel with intensity higher than the threshold setup
    * Input parameters:
        - minIntensityPixelsEdgesInliers: minimum intensities for pixels to be considered super pixels.
        - minMsp: super pixels / total pixels in circle * 100.
        - numberPixelAroundRadiusEdgesBest: number of pixels around circle for searching for super pixels out of circle
        - minRateSuperPixelInCircleByAround: super pixels in circle / super pixels around the circle.
    */
    // Time the process 
    auto start = std::chrono::system_clock::now();
    ROS_DEBUG_STREAM("BEST CIRCLES TOOL: Getting best circle of the optimized Hough circles. Minimum intensity for inliers: "
                << minIntensityPixelsEdgesInliers << " and min MSP: " << minMsp << ".");
    // Check input image
    if (mInputImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Input image has size: " << mInputImage.size() << ". Set using setInputImage.");
        return 1;
    }
    // Check blur gray image
    if (mGrayBlurImage.size() == cv::Size(0,0)) {
        ROS_ERROR_STREAM("GENERAL PROCESSING TOOL: Gray blur image has size: " << mGrayBlurImage.size() 
            << ". Call using convertToGrayImage and blurImage.");
        return 1;
    }
    // Check if the Hough circle has already been done
    if (mHoughCircles.size() == 0){
        ROS_WARN_STREAM("HOUGH CIRCLES TOOL: Hough circles not calculated. Call calculateHoughCircles to calculate them.");
        return 1;
    }
    // Check if Canny edges have been found e
    if (mCannyEdgesImage.size() == cv::Size(0,0)){
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Optimized Hough Circles is empty. Call optimizeHoughCircles to get it.");
    }
    // Check min intensity parameters
    if (minIntensityPixelsEdgesInliers > 255) {
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Min intensity pixels edges inliers is bigger than 255. 255 is maximum.");
        return 1;
    }

    // Create needed parameters
    double a, b, r, x, y;
    double intensityPixel;
    int numberOfCirclePixels = 0;
    int numberOfCircleSuperPixels = 0;
    int numberOfAroundPixels = 0;
    int numberOfAroundSuperPixels = 0;
    double msp;
    double bestMsp = 0;
    double rateSuperPixelInCircleByAround;
    cv::Vec3d bestCircleParams = cv::Vec3d(0, 0, 0);

    // Go through the optimized circles and choose the one with best edge inliers rate
    // and less outliers
    for (size_t i = 0; i < mOptimizedHoughCirclesParams.size(); i++) {  
        // Set to 0 the counters 
        numberOfCirclePixels = 0;
        numberOfCircleSuperPixels = 0;     
        numberOfAroundPixels = 0;
        numberOfAroundSuperPixels = 0;     
        // Get i pixels around the circle 
        for (int j = -numberPixelAroundRadiusEdgesBest; j <= numberPixelAroundRadiusEdgesBest; j++) {
            // Get params of circle
            a = mOptimizedHoughCirclesParams[i][0];
            b = mOptimizedHoughCirclesParams[i][1];
            r = mOptimizedHoughCirclesParams[i][2] + j;

            // For each circle, get the points of the circle by the formula
            // x = a + r cos(θ) ---- y = b + r sin(θ)
            for (double h = -180.0 ; h <= 180.0 ; h+=0.1) {
                x = a + r * cos(h *3.14159/180);
                y = b + r * sin(h *3.14159/180);

                // Get intensity of pixel
                intensityPixel = (int)mCannyEdgesImage.at<uchar>(y, x);

                // In case it is the circle defined by the parameters
                if (j == 0) {
                    // Count super pixels and number of pixels
                    if (intensityPixel > minIntensityPixelsEdgesInliers) {
                        numberOfCircleSuperPixels++;
                    }
                    numberOfCirclePixels++;
                // In case it is around circle defined by the parameters
                } else {
                    // Count around super pixels and number of around pixels
                    if (intensityPixel > minIntensityPixelsEdgesInliers) {
                        numberOfAroundSuperPixels++;
                    }
                    numberOfAroundPixels++;
                }
            }
        }

        // Get rate of super pixels from the circle pixels (MSP)
        msp = numberOfCircleSuperPixels * 100.0 / numberOfCirclePixels;
        rateSuperPixelInCircleByAround = numberOfCircleSuperPixels * 1.0 / numberOfAroundSuperPixels;
        // Save best circle parameters
        if ((msp >= minMsp) && (msp >= bestMsp) && (rateSuperPixelInCircleByAround >= minRateSuperPixelInCircleByAround)) {
            bestCircleParams = mOptimizedHoughCirclesParams[i];
            bestMsp = msp;
        }
    }
    // Check output
    if (bestCircleParams == cv::Vec3d(0, 0, 0)){
        ROS_ERROR_STREAM("BEST CIRCLES TOOL: No circles past the thresholds set in the best circle algorithm.");
        return 1;
    }

    // Save 
    mBestHoughCircleParams = bestCircleParams;

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("BEST CIRCLES TOOL: Best circle tool done in " << duration.count() << " seconds." 
        << " Best circle has parameteres. A: " << mBestHoughCircleParams[0] << " B: " <<mBestHoughCircleParams[1]
        << " and R: " << mBestHoughCircleParams[2] << ".");
    return 0;
}

int GeneralProcessingTool::drawHoughCirclesInInputImage(cv::Mat &rOutputImage)
{
    /*
    Method that draws the Hough circles found in the input image and retrieves them.
    * Input parameters: 
        - rOutputImage: image where the circles will be drawn
    */
    // Check if the Hough circle has already been done
    if (mHoughCircles.size() == 0){
        ROS_WARN_STREAM("HOUGH CIRCLES TOOL: Hough circles not calculated. Call calculateHoughCircles to access them.");
        return 1;
    }

    // Draw all the circles found
    for( size_t i = 0; i < mHoughCircles.size(); i++ )
    {
        cv::Vec3i circleParameters = mHoughCircles[i];
        cv::Point center = cv::Point(circleParameters[0], circleParameters[1]);
        // Draw circle center
        circle( rOutputImage, center, 1, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
        // Draw circle outline
        int radius = circleParameters[2];
        circle( rOutputImage, center, radius, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);
    }

    return 0;
}

int GeneralProcessingTool::drawOptimizedHoughCirclesInInputImage(cv::Mat &rOutputImage)
{
    /*
    Method that draws the Hough circles found in the input image and retrieves them.
    * Input parameters: 
        - rOutputImage: image where the circles will be drawn
    */
    // Check if the Hough circle has already been done
    if (mOptimizedHoughCirclesParams.size() == 0){
        ROS_WARN_STREAM("HOUGH CIRCLES TOOL: Hough circles not optimized. Call optimizeHoughCircles to access them.");
        return 1;
    }

    // Draw all the circles found
    for( size_t i = 0; i < mOptimizedHoughCirclesParams.size(); i++ )
    {
        cv::Vec3d circleParameters = mOptimizedHoughCirclesParams[i];
        cv::Point center = cv::Point(circleParameters[0], circleParameters[1]);
        // Draw circle center
        circle( rOutputImage, center, 1, cv::Scalar(0,255,0), 3, cv::LINE_AA);
        // Draw circle outline
        int radius = circleParameters[2];
        circle( rOutputImage, center, radius, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
    }

    return 0;
}

int GeneralProcessingTool::drawBestHoughCirclesInInputImage(cv::Mat &rOutputImage)
{
    /*
    Method that draws the best Hough circle found in the input image and retrieves them.
    * Input parameters: 
        - rOutputImage: image where the circles will be drawn
    */
    // Check if the Hough circle has already been done
    if (mBestHoughCircleParams == cv::Vec3d(0, 0, 0)){
        ROS_ERROR_STREAM("BEST CIRCLES TOOL: No circles past the thresholds set in the best circle algorithm.");
        return 1;
    }

    // Draw the circle found
    cv::Point center = cv::Point(mBestHoughCircleParams[0], mBestHoughCircleParams[1]);
    // Draw circle center
    circle( rOutputImage, center, 1, cv::Scalar(0,255,0), 10, cv::LINE_AA);
    // Draw circle outline
    int radius = mBestHoughCircleParams[2];
    circle( rOutputImage, center, radius, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);

    return 0;
}

cv::Mat GeneralProcessingTool::getGrayImage() 
{
    if (mGrayImage.size() == cv::Size(0,0)) {
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Gray image non-existant.");
    }
    return mGrayImage;
}
cv::Mat GeneralProcessingTool::getBlurImage() 
{
    if (mBlurImage.size() == cv::Size(0,0)) {
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Blur image non-existant.");
    }
    return mBlurImage;
}
cv::Mat GeneralProcessingTool::getBlurGrayImage() 
{
    if (mGrayBlurImage.size() == cv::Size(0,0)) {
        ROS_WARN_STREAM("GENERAL PROCESSING TOOL: Blur gray image non-existant.");
    }
    return mGrayBlurImage;
}
std::vector<cv::Vec3f> GeneralProcessingTool::getHoughCircles() 
{
    if (mHoughCircles.size() == 0) {
        ROS_WARN_STREAM("HOUGH CIRCLES TOOL: Hough circles non-existant.");
    }
    return mHoughCircles;
}
std::vector<cv::Vec3d> GeneralProcessingTool::getOptimizedHoughCircles() 
{
    if (mOptimizedHoughCirclesParams.size() == 0) {
        ROS_WARN_STREAM("OPTIMIZE HOUGH CIRCLES TOOL: Hough circles optimized non-existant.");
    }
    return mOptimizedHoughCirclesParams;
}
cv::Vec3d GeneralProcessingTool::getBestHoughCircle() 
{
    if (mBestHoughCircleParams == cv::Vec3d(0, 0, 0)) {
        ROS_WARN_STREAM("BEST HOUGH CIRCLE TOOL: Best Hough circle has not been calculated. Call calculateBestOptimizedHoughCircles");
    }
    return mBestHoughCircleParams;
}
cv::Mat GeneralProcessingTool::getCannyEdgesImage() 
{
    if (mCannyEdgesImage.size() == cv::Size(0,0)) {
        ROS_WARN_STREAM("CANNY EDGE TOOL: Canny edge image non-existant.");
    }
    return mCannyEdgesImage;
}
cv::Mat GeneralProcessingTool::getSobelImage() 
{
    if (mSobelImage.size() == cv::Size(0,0)) {
        ROS_WARN_STREAM("CANNY EDGE TOOL: Sobel edge image non-existant.");
    }
    return mSobelImage;
}