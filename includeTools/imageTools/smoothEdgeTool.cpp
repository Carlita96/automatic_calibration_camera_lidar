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


#include "smoothEdgeTool.hpp"

SmoothEdgeTool::SmoothEdgeTool(const int kSizeEdgeCalculation, const int kSizeEdgeSmoothness, 
                const double alphaEdgeSmoothness, const double gammaEdgeSmoothness)
{
    // Save parameters
    mKSizeEdgeCalculation = kSizeEdgeCalculation;
    mKSizeEdgeSmoothness = kSizeEdgeSmoothness;
    mAlphaEdgeSmoothness = alphaEdgeSmoothness;
    mGammaEdgeSmoothness = gammaEdgeSmoothness;
}

SmoothEdgeTool::~SmoothEdgeTool(){
}

int SmoothEdgeTool::getSmoothEdge(const cv::Mat inputImage, cv::Mat &rOutputImage)
{
    /*
    Method that gets the edges of an image. First it would get the edges and it would
    smooth them with an inverse distance transform to the sharpest edge.
    * Input parameters:
        - inputImage: Input image which edges will be calculated. It needs to be gray scale.
    * Ouput parameters:
        - outputImage: Image calculated with the smoothed edges.
    */
    ROS_DEBUG_STREAM("SMOOTH EDGE TOOL: Getting smooth edge image with K size for edge calculation " 
        << mKSizeEdgeCalculation << ", K size for smoothing edges: " << mKSizeEdgeSmoothness 
        << " alpha for smoothing edges: " << mAlphaEdgeSmoothness << " gamma for smoothing edges: " << mGammaEdgeSmoothness << ".");
    // Check that input image is in gray scale.
    if (inputImage.channels() != 1) {
        ROS_ERROR_STREAM("SMOOTH EDGE IMAGE: Input image has " << inputImage.channels() << " number of channels. 1 needed (gray scale image).");
        return 1;
    }

    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Create images necessary
    cv::Mat edgeImage(inputImage.size().height, inputImage.size().width, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat smoothEdgeImage(inputImage.size().height, inputImage.size().width, CV_8UC3, cv::Scalar(0, 0, 0));

    // Save the maximum intensity of the image in order to set it to 255 and change other values with respect to that
    // in next steps
    int maxIntensityImage = 0;
    // STEP 1: Get edges by checking the maximum difference between one pixel to its neighbours.
    // Go through each pixel, calculate the max difference between neighbours and save to new image
    for(int i = 0; i < inputImage.size().height; ++i) {
        for (int j = 0; j < inputImage.size().width; ++j) {
            int maxDiffIntensityPixel = 0;
            int intensityMainPixel = (int)inputImage.at<uchar>(i, j);
            // Go through neighbours
            for (int nI = -mKSizeEdgeCalculation/2; nI < mKSizeEdgeCalculation/2; ++nI) {
                for (int nJ = -mKSizeEdgeCalculation/2; nJ < mKSizeEdgeCalculation/2; ++nJ) {
                    // Check that pixels are within image margin
                    if ((j+nJ > 0) && (j+nJ < inputImage.size().width) && 
                            (i+nI > 0) && (i+nI < inputImage.size().height)) {
                        // Get max difference between pixels
                        int intensityPoolingPixel = (int)inputImage.at<uchar>(i+nI, j+nJ);
                        maxDiffIntensityPixel = std::max(abs(intensityMainPixel-intensityPoolingPixel), 
                                                        maxDiffIntensityPixel);
                    }
                }
            }
            // Save max difference between pixel and neighbours into new image
            edgeImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, maxDiffIntensityPixel);
            // Save the max intensity of the whole image 
            maxIntensityImage = std::max(maxDiffIntensityPixel, maxIntensityImage);
        }
    }

    // Parameters
    int kSize = 5;
    int cannyMinThreshold = 110;
    int cannyMaxThreshold = 140;
    // STEP 2: Blur image and get Canny Edge image
    cv::Mat blurImage;
    cv::Mat cannyEdgesImage;
    cv::medianBlur(inputImage, blurImage, kSize);
    cv::Canny(blurImage, cannyEdgesImage, cannyMinThreshold, cannyMaxThreshold);

    // STEP 3: Set Canny edges to 255 in edge image
    for(int i = 0; i < cannyEdgesImage.size().height; ++i) {
        for (int j = 0; j < cannyEdgesImage.size().width; ++j) {
            int intensityPixel = (int)cannyEdgesImage.at<uchar>(i, j);
            if (intensityPixel == 255) {
                // Save max difference between pixel and neighbours into new image
                edgeImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);
            }
        }
    }

    // Create necessary parameters
    int maxIntensityNeighbour, maxNeighbourI, maxNeighbourJ;
    // STEP 2: Do an inverse distance transform to smooth the edges
    // STEP 2.1: The max intensity of the whole image will be set to 255 and the rest re-scale to that.
    for(int i = 0; i < edgeImage.size().height; ++i) {
        for (int j = 0; j < edgeImage.size().width; ++j) {
            maxIntensityNeighbour = 0;


            // // New approach
            // int mainIntensityNeighbourPixel = (int)edgeImage.at<cv::Vec3b>(i, j)[2];

            // // DELETE
            // // Get h list with possible angles
            // std::vector<double> possibleRayAngle;
            // std::vector<int> previousIntensityInRay;
            // for (double h = -180.0 ; h <= 180.0 ; h+=20.0) {
            //     possibleRayAngle.push_back(h);
            //     previousIntensityInRay.push_back(mainIntensityNeighbourPixel);
            // }

            // std::vector<int> deleteRayPosition;
            // int maxIntensity = 0;
            // int xMaxIntensity = i;
            // int yMaxIntensity = j;
            // for (int r = 1; r <= mKSizeEdgeSmoothness; r++) {
            //     // For each circle, get the points of the circle by the formula
            //     // x = a + r cos(θ) ---- y = b + r sin(θ)
            //     for (auto it = std::begin(possibleRayAngle); it != std::end(possibleRayAngle); ++it) {
            //         // Get pixel intensity in the rays
            //         int itRay = it - possibleRayAngle.begin();
            //         double h = possibleRayAngle[itRay];
            //         int previousIntensityRay = previousIntensityInRay[itRay];
            //         int x = i + r * cos(h *3.14159/180);
            //         int y = j + r * sin(h *3.14159/180);
            //         int intensityNeighbourPixel = (int)edgeImage.at<cv::Vec3b>(x, y)[2];

            //         if (intensityNeighbourPixel > previousIntensityRay && intensityNeighbourPixel > maxIntensity) {
            //             previousIntensityInRay[itRay] = intensityNeighbourPixel;
            //             maxIntensity = intensityNeighbourPixel;
            //             xMaxIntensity = x;
            //             yMaxIntensity = y;
            //         } else if (intensityNeighbourPixel < previousIntensityRay) {
            //             deleteRayPosition.push_back(itRay);
            //         }
            //     }
            // }
            
            // for (auto it = std::begin(deleteRayPosition); it != std::end(deleteRayPosition); ++it) {
            //     int itRay = it - deleteRayPosition.begin();
		    //     possibleRayAngle.erase(possibleRayAngle.begin() + itRay);
		    //     previousIntensityInRay.erase(previousIntensityInRay.begin() + itRay);
            // }

            // // Calculate new value of pixel with inverse distante transform to the sharpest edge close to the pixel
            // // FORMULA: Dij = a*Eij + (1-a) * max(Exy) * gamma^(max(abs(x-i), abs(y-j))) 
            // int intensityMainPixel = (int)edgeImage.at<cv::Vec3b>(i, j)[2];
            // int valueInSmoothImage = mAlphaEdgeSmoothness * intensityMainPixel + (1 - mAlphaEdgeSmoothness) * maxIntensity * 
            //             pow(mGammaEdgeSmoothness, std::max(abs(xMaxIntensity-i), abs(yMaxIntensity-j)));
            // smoothEdgeImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, valueInSmoothImage);

            // Go through neighbours
            for (int nJ = -mKSizeEdgeSmoothness/2; nJ < mKSizeEdgeSmoothness/2; ++nJ) {
                // Check that pixels are within image margin
                if ((j+nJ > 0) && (j+nJ < edgeImage.size().width)) {
                    // Get intensity of neightboor
                    int intensityNeighbourPixel = (int)edgeImage.at<cv::Vec3b>(i, j+nJ)[2];
                    // Save the maximum intensity of the neighbours
                    if (intensityNeighbourPixel > maxIntensityNeighbour) {
                        maxIntensityNeighbour = std::max(intensityNeighbourPixel, maxIntensityNeighbour);
                        maxNeighbourI = i;
                        maxNeighbourJ = j+nJ;
                    }
                }
            }
            for (int nI = -mKSizeEdgeSmoothness/2; nI < mKSizeEdgeSmoothness/2; ++nI) {
                // Check that pixels are within image margin
                if ((i+nI > 0) && (i+nI < edgeImage.size().height)) {
                    // Get intensity of neightboor
                    int intensityNeighbourPixel = (int)edgeImage.at<cv::Vec3b>(i+nI, j)[2];
                    // Save the maximum intensity of the neighbours
                    if (intensityNeighbourPixel > maxIntensityNeighbour) {
                        maxIntensityNeighbour = std::max(intensityNeighbourPixel, maxIntensityNeighbour);
                        maxNeighbourI = i+nI;
                        maxNeighbourJ = j;
                    }
                }
            }
            // Calculate new value of pixel with inverse distante transform to the sharpest edge close to the pixel
            // FORMULA: Dij = a*Eij + (1-a) * max(Exy) * gamma^(max(abs(x-i), abs(y-j))) 
            int intensityMainPixel = (int)edgeImage.at<cv::Vec3b>(i, j)[2];
            int valueInSmoothImage = mAlphaEdgeSmoothness * intensityMainPixel + (1 - mAlphaEdgeSmoothness) * maxIntensityNeighbour * 
                        pow(mGammaEdgeSmoothness, std::max(abs(maxNeighbourI-i), abs(maxNeighbourJ-j)));
            smoothEdgeImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, valueInSmoothImage);
        }
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    // Save image to output
    rOutputImage = smoothEdgeImage;

    ROS_DEBUG_STREAM("SMOOTH EDGE TOOL: Smooth edges found in " << duration.count() << " seconds.");

    return 0;
}