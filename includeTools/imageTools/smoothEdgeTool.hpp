/*
This class can be used to get smooth edges in image. First the image edges are calculated. 
Then, the edges are smoothed with an inverse distance transform from a pixel to the sharpest closest edge. 
Using formula: Dij = a*Eij + (1-a) * max(Exy) * gamma^(max(abs(x-i), abs(y-j))) 

* Input parameters:
    - kSizeEdgeCalculation: Size of matrix of closest points for calculating the edge points.
    - kSizeEdgeSmoothness: Size of matrix of pixels that will be smoothed.
    - alpha and gamma: parameters of the formula for the inverse distance transform.

This script cannot be run directly and needs to be used as an import.
*/

#ifndef CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_SMOOTHEDGETOOLS_H_
#define CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_SMOOTHEDGETOOLS_H_

// Import necessary libraries
// General libraries
#include <chrono>
// ROS libraries
#include <ros/console.h> 
// ROS libraries
#include <ros/console.h>
// OpenCV libraries
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>


class SmoothEdgeTool
{
    public:
        SmoothEdgeTool(const int kSizeEdgeCalculation, const int kSizeEdgeSmoothness, 
                const double alphaEdgeSmoothness, const double gammaEdgeSmoothness);

        ~SmoothEdgeTool();
        
        int getSmoothEdge(const cv::Mat inputImage, cv::Mat &rOutputImage);

    private:
        int mKSizeEdgeCalculation = 3;
        double mAlphaEdgeSmoothness = 1.0/3;
        double mGammaEdgeSmoothness = 0.98; 
        int mKSizeEdgeSmoothness = 4;
}; 

#endif // CALIBRATION_INCLUDETOOLS_POINTCLOUDTOOLS_SMOOTHEDGETOOLS_H_ 