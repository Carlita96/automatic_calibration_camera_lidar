/*
This class can be used to do general processes on an image.

This script cannot be run directly and needs to be used as an import.
*/

#ifndef CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_GENERALPROCESSINGTOOLS_H_
#define CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_GENERALPROCESSINGTOOLS_H_

// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/console.h> 
// ROS libraries
#include <ros/console.h>
// OpenCV libraries
#include <opencv2/imgproc.hpp>

// Optimization of Hough circles tool
#include "optimizationProblemTools/circleParametersOptimization.hpp"


class GeneralProcessingTool
{
    public:
        GeneralProcessingTool();

        ~GeneralProcessingTool();
        
        void setInputImage(const cv::Mat inputImage);

        int convertToGrayImage();

        int blurImage(const int kSize, const bool fromGrayScale=false);

        int calculateHoughCircles(const int accumulatorResolution, const int minimumDistanceBetweenCircles,
            const int cannyHighThreshold, const int minimumVotes, const int minRadius, const int maxRadius);

        int doCannyEdges(const int cannyMinThreshold, const int cannyMaxThreshold);

        int doSobel(const int kSizeSobel);

        int optimizeHoughCircles(const int minIntensityPixelsEdgesOptimization, const int numberPixelAroundRadiusEdgesOptimization,
                const int pixelThresholdHorizontalEdge=0);

        int calculateBestOptimizedHoughCircles(const int minIntensityPixelsEdgesInliers, const double minMsp,
                const int numberPixelAroundRadiusEdgesBest, const double minRateSuperPixelInCircleByAround);

        int drawHoughCirclesInInputImage(cv::Mat &rOutputImage); 

        int drawOptimizedHoughCirclesInInputImage(cv::Mat &rOutputImage);

        int drawBestHoughCirclesInInputImage(cv::Mat &rOutputImage);

        cv::Mat getGrayImage(); 
        cv::Mat getBlurImage(); 
        cv::Mat getBlurGrayImage(); 
        std::vector<cv::Vec3f> getHoughCircles(); 
        std::vector<cv::Vec3d> getOptimizedHoughCircles();
        cv::Vec3d getBestHoughCircle();
        cv::Mat getCannyEdgesImage();
        cv::Mat getSobelImage();

    private:
        // Initialization of images and image parameters
        cv::Mat mInputImage;
        cv::Size mSizeInputImage;
        cv::Mat mGrayImage;
        cv::Mat mBlurImage;
        cv::Mat mGrayBlurImage;
        cv::Mat mCannyEdgesImage;
        cv::Mat mSobelImage;
        std::vector<cv::Vec3f> mHoughCircles;
        std::vector<cv::Vec3d> mOptimizedHoughCirclesParams;
        cv::Vec3d mBestHoughCircleParams = cv::Vec3d(0, 0, 0);

        // Optimizer
        CircleParametersOptimization *mCircleParametersOptimization;
}; 

#endif // CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_GENERALPROCESSINGTOOLS_H_ 