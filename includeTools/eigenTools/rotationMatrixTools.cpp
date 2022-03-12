/*
In this file, functions related to rotation matrix or Euler Angles.
They can be imported and used as wanted.
*/

#include "rotationMatrixTools.hpp"

Eigen::Matrix3d getRotationMatrixFromRpy(const Eigen::Vector3d& rollPithYawVector)
{
    /*
    Method that can calculate the rotation matrix from Roll Pitch Yaw angles in degrees.
    * Input:
        - rollPitchYaw: vector containing roll[0], pitch[0] and yaw[0].
    * Output:
        - rotation matrix: of size 3x3. Calculated by Rroll * Rpitch * Ryaw.
    */
    // Calculate roll pitch yaw matrixes. 
    // Angle is changed to degrees
    Eigen::Matrix3d rollMatrix;
    rollMatrix <<   1,                      0,                                              0,
                    0,                      cos(rollPithYawVector[0] * M_PI / 180.0),       -sin(rollPithYawVector[0] * M_PI / 180.0),
                    0,                      sin(rollPithYawVector[0] * M_PI / 180.0),       cos(rollPithYawVector[0] * M_PI / 180.0);
    Eigen::Matrix3d pitchMatrix;
    pitchMatrix <<  cos(rollPithYawVector[1] * M_PI / 180.0),       0,                      sin(rollPithYawVector[1] * M_PI / 180.0),
                    0,                                              1,                      0,
                    -sin(rollPithYawVector[1] * M_PI / 180.0),      0,                      cos(rollPithYawVector[1] * M_PI / 180.0); 
    Eigen::Matrix3d yawMatrix;
    yawMatrix <<    cos(rollPithYawVector[2] * M_PI / 180.0),     -sin(rollPithYawVector[2] * M_PI / 180.0),    0,
                    sin(rollPithYawVector[2] * M_PI / 180.0),     cos(rollPithYawVector[2] * M_PI / 180.0),     0,
                    0,                                              0,                                          1;   

    // Calculate rotation matrix
    Eigen::Matrix3d rotationMatrix = rollMatrix * pitchMatrix * yawMatrix;
    return rotationMatrix;
};

Eigen::Matrix3d getRotationMatrixFromRpyRadians(const Eigen::Vector3d& rollPithYawVector)
{
    /*
    Method that can calculate the rotation matrix from Roll Pitch Yaw angles in radians.
    * Input:
        - rollPitchYaw: vector containing roll[0], pitch[0] and yaw[0].
    * Output:
        - rotation matrix: of size 3x3. Calculated by Rroll * Rpitch * Ryaw.
    */
    // Calculate roll pitch yaw matrixes.
    Eigen::Matrix3d rollMatrix;
    rollMatrix <<   1,                      0,                              0,
                    0,                      cos(rollPithYawVector[0]),      -sin(rollPithYawVector[0]),
                    0,                      sin(rollPithYawVector[0]),      cos(rollPithYawVector[0]);
    Eigen::Matrix3d pitchMatrix;
    pitchMatrix <<  cos(rollPithYawVector[1]),     0,                      sin(rollPithYawVector[1]),
                    0,                             1,                      0,
                    -sin(rollPithYawVector[1]),    0,                      cos(rollPithYawVector[1]); 
    Eigen::Matrix3d yawMatrix;
    yawMatrix <<    cos(rollPithYawVector[2]),     -sin(rollPithYawVector[2]),    0,
                    sin(rollPithYawVector[2]),     cos(rollPithYawVector[2]),     0,
                    0,                             0,                             1;   

    // Calculate rotation matrix
    Eigen::Matrix3d rotationMatrix = rollMatrix * pitchMatrix * yawMatrix;
    return rotationMatrix;
};

Eigen::Matrix3d getRotationMatrixFromYpr(const Eigen::Vector3d& rollPithYawVector)
{
    /*
    Method that can calculate the rotation matrix from Roll Pitch Yaw angles in degrees.
    * Input:
        - rollPitchYaw: vector containing roll[0], pitch[0] and yaw[0].
    * Output:
        - rotation matrix: of size 3x3. Calculated by Rroll * Rpitch * Ryaw.
    */
    // Calculate roll pitch yaw matrixes. 
    // Angle is changed to degrees
    Eigen::Matrix3d rollMatrix;
    rollMatrix <<   1,                      0,                                              0,
                    0,                      cos(rollPithYawVector[0] * M_PI / 180.0),       -sin(rollPithYawVector[0] * M_PI / 180.0),
                    0,                      sin(rollPithYawVector[0] * M_PI / 180.0),       cos(rollPithYawVector[0] * M_PI / 180.0);
    Eigen::Matrix3d pitchMatrix;
    pitchMatrix <<  cos(rollPithYawVector[1] * M_PI / 180.0),       0,                      sin(rollPithYawVector[1] * M_PI / 180.0),
                    0,                                              1,                      0,
                    -sin(rollPithYawVector[1] * M_PI / 180.0),      0,                      cos(rollPithYawVector[1] * M_PI / 180.0); 
    Eigen::Matrix3d yawMatrix;
    yawMatrix <<    cos(rollPithYawVector[2] * M_PI / 180.0),     -sin(rollPithYawVector[2] * M_PI / 180.0),    0,
                    sin(rollPithYawVector[2] * M_PI / 180.0),     cos(rollPithYawVector[2] * M_PI / 180.0),     0,
                    0,                                              0,                                          1;   

    // Calculate rotation matrix
    Eigen::Matrix3d rotationMatrix = yawMatrix * pitchMatrix * rollMatrix;
    return rotationMatrix;
};

Eigen::Matrix3d getRotationMatrixFromQuaternions(const Eigen::Vector4d& quaternions)
{
    /*
    Method that can calculate the rotation matrix from quaternions.
    * Input:
        - quaternions: vector containing qx[0], qy[0], qz[0] and qw[0].
    * Output:
        - rotation matrix: of size 3x3.
    */
    // Get rotation matrix
    // Row 0
    double r00 = 1 - (2 * (quaternions[1]  * quaternions[1] + quaternions[2] * quaternions[2]));
    double r01 = 2 * (quaternions[0]  * quaternions[1] - quaternions[2] * quaternions[3]);
    double r02 = 2 * (quaternions[0]  * quaternions[2] + quaternions[1] * quaternions[3]);
    // Row 1
    double r10 = 2 * (quaternions[0]  * quaternions[1] + quaternions[2] * quaternions[3]);
    double r11 = 1 - (2 * (quaternions[0]  * quaternions[0] + quaternions[2] * quaternions[2]));
    double r12 = 2 * (quaternions[1]  * quaternions[2] - quaternions[0] * quaternions[3]);
    // Row 2
    double r20 = 2 * (quaternions[0]  * quaternions[2] - quaternions[1] * quaternions[3]);
    double r21 = 2 * (quaternions[1]  * quaternions[2] + quaternions[0] * quaternions[3]);
    double r22 = 1 - (2 * (quaternions[0]  * quaternions[0] + quaternions[1] * quaternions[1]));

    // Calculate rotation matrix
    Eigen::Matrix3d rotationMatrix;
    rotationMatrix << r00, r01, r02, r10, r11, r12, r20, r21, r22;
    return rotationMatrix;
};