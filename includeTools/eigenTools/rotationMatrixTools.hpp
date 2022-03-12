/*
In this file, functions related to rotation matrix or Euler Angles.
They can be imported and used as wanted.
*/

#ifndef CALIBRATION_INCLUDETOOLS_EIGENTOOLS_ROTATIONTOOLS_H_
#define CALIBRATION_INCLUDETOOLS_EIGENTOOLS_ROTATIONTOOLS_H_

// Eigen libraries
#include <Eigen/Eigen>

Eigen::Matrix3d getRotationMatrixFromRpy(const Eigen::Vector3d& rollPithYawVector);

Eigen::Matrix3d getRotationMatrixFromRpyRadians(const Eigen::Vector3d& rollPithYawVector);

Eigen::Matrix3d getRotationMatrixFromYpr(const Eigen::Vector3d& rollPithYawVector);

Eigen::Matrix3d getRotationMatrixFromQuaternions(const Eigen::Vector4d& quaternions);

#endif // CALIBRATION_INCLUDETOOLS_EIGENTOOLS_ROTATIONTOOLS_H_