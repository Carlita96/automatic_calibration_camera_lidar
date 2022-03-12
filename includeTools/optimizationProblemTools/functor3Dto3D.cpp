/*
This struct uses a Levenberg-Marquardt algorithm for calibrating a camera considering:
    [cameraPoint3D] = R * [LidarPoint3D] + T + error
This script cannot be run directly and needs to be used as an import.
*/

// Eigen libraries
#include <Eigen/Eigen>

// Rotation Matrix tool
#include "eigenTools/rotationMatrixTools.hpp"

struct Functor3Dto3D
{
    // Number of pairs of observations -> 'm' pairs of (x, f(x))
    // In this case, the center points of the sphere
	Eigen::MatrixXd measuredValues; // This would be [xC, yC, zC, xL, yL, zL] Matrix [m, 6]

	// Compute 'm' errors, one for each data point, for the given parameter values in 'calibrationParams'
	int operator()(const Eigen::VectorXd &calibrationParams, Eigen::VectorXd &fvec) const
	{
		// 'calibrationParams' has dimensions n x 1 (6 x 1)
		// It contains the current estimates for the parameters
        // (alpha, beta, theta, t1, t2, t3)

		// 'fvec' has dimensions m x 1 (pairsObservations x 1)
		// It will contain the error for each data point.

		double alphaRotation = calibrationParams(0);
		double betaRotation = calibrationParams(1);
		double thetaRotation = calibrationParams(2);
		double xTranslation = calibrationParams(3);
		double yTranslation = calibrationParams(4);
		double zTranslation = calibrationParams(5);

        // Get rotation matrix with (alpha, beta, theta)
        Eigen::Vector3d rollPithYawVector; rollPithYawVector << alphaRotation, betaRotation, thetaRotation;
        Eigen::Matrix3d rotationMatrix = getRotationMatrixFromRpy(rollPithYawVector);
        Eigen::Vector3d tranlationVector; tranlationVector << xTranslation, yTranslation, zTranslation;

        // Calculate the error for each measurement (pairs of observations)
        Eigen::Vector3d lidarVector;
        Eigen::Vector3d cameraVector;
		for (int i = 0; i < values(); i++) {
            // Get measured data
			double xCamera = measuredValues(i, 0);
			double yCamera = measuredValues(i, 1);
			double zCamera = measuredValues(i, 2);
			double xLidar = measuredValues(i, 3);
			double yLidar = measuredValues(i, 4);
			double zLidar = measuredValues(i, 5);

            // Calculate error
            lidarVector =  Eigen::Vector3d(xLidar, yLidar, zLidar);
            cameraVector = rotationMatrix * lidarVector + tranlationVector;
			fvec(i) = sqrt(pow(xCamera - cameraVector[0], 2) + pow(yCamera - cameraVector[1], 2) + pow(zCamera - cameraVector[2], 2));
		}
		return 0;
	}

	// Compute the jacobian of the errors
	int df(const Eigen::VectorXd &calibrationParams, Eigen::MatrixXd &fjac) const
	{
		// 'calibrationParams' has dimensions n x 1 (6 x 1)
		// It contains the current estimates for the parameters.
        // (alpha, beta, theta, t1, t2, t3)

		// 'fjac' has dimensions m x n (pairsObservations x 6)
		// It will contain the jacobian of the errors, calculated numerically in this case.

		double epsilon;
		epsilon = 1e-5f;

		for (int i = 0; i < calibrationParams.size(); i++) {
			Eigen::VectorXd calibrationParamsPlus(calibrationParams);
			calibrationParamsPlus(i) += epsilon;
			Eigen::VectorXd calibrationParamsMinus(calibrationParams);
			calibrationParamsMinus(i) -= epsilon;

			Eigen::VectorXd fvecPlus(values());
			operator()(calibrationParamsPlus, fvecPlus);

			Eigen::VectorXd fvecMinus(values());
			operator()(calibrationParamsMinus, fvecMinus);

			Eigen::VectorXd fvecDiff(values());
			fvecDiff = (fvecPlus - fvecMinus) / (2.0f * epsilon);

			fjac.block(0, i, values(), 1) = fvecDiff;
		}

		return 0;
	}

	// Number of data points, i.e. values.
	int m;

	// Returns 'm', the number of values.
	int values() const { return m; }

	// The number of parameters, i.e. inputs.
	int n;

	// Returns 'n', the number of inputs.
	int inputs() const { return n; }

};