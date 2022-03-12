/**
Cost funtion that will be called with each new value of the quaternions and translation values.
The tool will try to reduce the residual. 
The residual is the distance between the point of point cloud projected to the image and the pixel in the image.
*/

#ifndef CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_FUNCTOR3DTO2D_H_
#define CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_FUNCTOR3DTO2D_H_

// Import necessary libraries
// General libraries
#include <iostream>
#include <vector>
// Eigen libraries
#include <eigen3/Eigen/Core>
#include <Eigen/Dense>
#include <eigen3/Eigen/Geometry>
// Ceres libraries
#include <ceres/ceres.h>


using namespace std;


class CostFunctionCalibration3Dto2D
{
public:
    CostFunctionCalibration3Dto2D(Eigen::Vector2d pixelInImage, Eigen::Matrix3d intrinsicMatrixK):
        mPixelInImage(pixelInImage), mIntrinsicMatrixK(intrinsicMatrixK){}

    template<typename T>
    bool operator()(const T* const quaternion, const T* const translation,
                    const T* const point, T* residuals)const
    {  
        /*
        Method that will be called with each new value of the quaternions and translation values.
        The tool will try to reduce the residual. 
        The residual is the distance between the point of point cloud projected to the image and the pixel in the image.
        * Input parameters:
            - quaternion: actual value of quaternions for rotation point cloud point to image.
            - translation: actual value of translation for translating point cloud point to image.
            - point: point of point cloud.
        * Output:
            - residuals: error of actual values. In this case, distance of projecter point cloud point and image pixel.
        */

        // Get calibration params
        Eigen::Map<const Eigen::Quaternion<T>> rotationQuaternion(quaternion);
        Eigen::Map<const Eigen::Matrix<T,3,1>> translationVector(translation);

        // Get the 3D point from point cloud from image
        Eigen::Map<const Eigen::Matrix<T,3,1>> pointPointCloud(point);
        Eigen::Matrix<T,3,1> pointImage = rotationQuaternion * pointPointCloud + translationVector;

        // Transform the 3D point from the image to 2D and rest the actual value of the 2D point
        T fx = T(mIntrinsicMatrixK(0,0));
        T fy = T(mIntrinsicMatrixK(1,1));
        T cx = T(mIntrinsicMatrixK(0,2));
        T cy = T(mIntrinsicMatrixK(1,2));

        residuals[0] = T(fx * pointImage(0,0) / pointImage(2, 0) + cx - T(mPixelInImage(0)));
        residuals[1] = T(fy * pointImage(1,0) / pointImage(2, 0) + cy - T(mPixelInImage(1)));

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector2d pixelInImage, const Eigen::Matrix3d intrinsicMatrixK){
        return (new ceres::AutoDiffCostFunction<CostFunctionCalibration3Dto2D,2,4,3,3>(
                new CostFunctionCalibration3Dto2D(pixelInImage, intrinsicMatrixK))); 
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    Eigen::Vector2d mPixelInImage;
    Eigen::Matrix3d mIntrinsicMatrixK;
};


#endif //CALIBRATION_INCLUDETOOLS_OPTIMIZATIONTOOLS_FUNCTOR3DTO2D_H_