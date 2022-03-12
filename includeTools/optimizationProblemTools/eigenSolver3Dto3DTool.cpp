/*
This class can be used to solve an optimization problem of a 3D-3D calibration.
The problem is a minimization problem. In this minimization problem, the rotation matrix R and translation vector T
are wanted.
The distance error from a 3D lidar point transformed by R and T 
to the 3D point of the image data has to be minimized.
*/

#include "eigenSolver3Dto3DTool.hpp"

EigenSolver3Dto3DTool::EigenSolver3Dto3DTool()
{ 
}

EigenSolver3Dto3DTool::~EigenSolver3Dto3DTool(){
}

int EigenSolver3Dto3DTool::computeCalibrationParams(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud,
            const std::vector<Eigen::Vector3d> listPointCenterSphericalImage, const int numberCalibrationParameters,
            Eigen::VectorXd &rOutputCalibrationParams) 
{
    /*
    Method that does an Levenberg-Maquardt optimization of points in image and point cloud in order to
        calibrate a camera an a LiDAR.
    * Input parameters: 
        - listPositionCenterSphericalTargetPointCloud: list with points in 3D from the point cloud data.
        - listPointCenterSphericalImage: list with points in 3D from the image data.
            NOTICE: They must be paired. So point [0] in image is point [0] in point cloud.
        - numberCalibrationParameters: number of calibration parameters (alpha, beta, theta, x, y, z) -> 6
        - rOutputCalibrationParams: calibration parameters after optimization.
    */
    // Check input 
    if (listPointCenterSphericalTargetPointCloud.size() != listPointCenterSphericalImage.size()) {
        ROS_ERROR_STREAM("EIGEN SOLVER 3D TO 3D TOOL: Number of points from image (" << listPointCenterSphericalImage.size() 
        << ") and from pointcloud (" << listPointCenterSphericalTargetPointCloud.size() << ") is not the same. Same number is needed.");
        return 1;
    }
	// 'n' is the number of parameters in the function.
	// In this case, the calibration parameters: (alpha, beta, theta, t1, t2, t3)
	int n = numberCalibrationParameters;
    // Get number of observation pairs
    int m = listPointCenterSphericalTargetPointCloud.size();
    ROS_DEBUG_STREAM("EIGEN SOLVER 3D TO 3D TOOL: Doing Eigen Levenberg-Maquardt with " << m << " number of paired observations"
        << " and looking for " <<  n << " number of parameters. Doing calibration 3D-3D.");

    // Time the process 
    auto start = std::chrono::system_clock::now();

	// Move the points from camera and lidar into an Eigen Matrix.
	// The first three column have the points in the camera. The last three column the points in the lidar.
    Eigen::Vector3d calibrationPointcloud;
    Eigen::Vector3d calibrationImage;
    int i, j;
	Eigen::MatrixXd measuredValues(m, 6);
    for (auto it = std::begin(listPointCenterSphericalTargetPointCloud); it != std::end(listPointCenterSphericalTargetPointCloud); ++it) {
        // Get first spherical target position from point cloud and image
        i = it - listPointCenterSphericalTargetPointCloud.begin();
        calibrationPointcloud = listPointCenterSphericalTargetPointCloud[i];
        calibrationImage = listPointCenterSphericalImage[i];
		measuredValues(i, 0) = calibrationImage[0];
		measuredValues(i, 1) = calibrationImage[1];
		measuredValues(i, 2) = calibrationImage[2];
		measuredValues(i, 3) = calibrationPointcloud[0];
		measuredValues(i, 4) = calibrationPointcloud[1];
		measuredValues(i, 5) = calibrationPointcloud[2];
	}

	// 'calibrationParams' is vector of length 'n' containing the initial values for the parameters.
	// The parameters 'calibrationParams' are also referred to as the 'inputs' in the context of LM optimization.
	// The LM optimization inputs should not be confused with the calibrationParams input values.
	Eigen::VectorXd calibrationParams(n);
	calibrationParams(0) = 0.0;             // initial value for 'alpha rotation'
	calibrationParams(1) = 0.0;             // initial value for 'beta rotation'
	calibrationParams(2) = 0.0;             // initial value for 'theta rotation'
	calibrationParams(3) = 0.0;             // initial value for 'x translation'
	calibrationParams(4) = 0.0;             // initial value for 'y translation'
	calibrationParams(5) = 0.0;             // initial value for 'z translation'

	//
	// Run the LM optimization
	// Create a LevenbergMarquardt object and pass it the functor.

	Functor3Dto3D functor;
	functor.measuredValues = measuredValues;
	functor.m = m;
	functor.n = n;

	Eigen::LevenbergMarquardt<Functor3Dto3D, double> lm(functor); 
	lm.parameters.maxfev = 100000;
	lm.parameters.xtol = 1.0e-16;
	lm.parameters.ftol = 1.0e-16;
	lm.parameters.gtol = 1.0e-16;
	Eigen::LevenbergMarquardtSpace::Status status = lm.minimize(calibrationParams);

		//  STATUS OPTIONS
        //  NotStarted = -2,
        //  Running = -1,
        //  ImproperInputParameters = 0,
        //  RelativeReductionTooSmall = 1,
        //  RelativeErrorTooSmall = 2,
        //  RelativeErrorAndReductionTooSmall = 3,
        //  CosinusTooSmall = 4,
        //  TooManyFunctionEvaluation = 5,
        //  FtolTooSmall = 6,
        //  XtolTooSmall = 7,
        //  GtolTooSmall = 8,
        //  UserAsked = 9

    // Check output
    if (status <= 0) {
        ROS_ERROR_STREAM("EIGEN SOLVER 3D TO 3D TOOL: LM optimization retrieved status " << status << ".");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("EIGEN SOLVER 3D TO 3D TOOL: Eigen Levenberg-Marquardt algorithm done in " << duration.count() << " seconds.");
	
	// Results
	ROS_DEBUG_STREAM("EIGEN SOLVER 3D TO 3D TOOL: 3D-3D Optimization results");
	ROS_DEBUG_STREAM("\talpha rotation: " << calibrationParams(0));
	ROS_DEBUG_STREAM("\tbeta rotation: " << calibrationParams(1));
	ROS_DEBUG_STREAM("\ttheta rotation: " << calibrationParams(2));
	ROS_DEBUG_STREAM("\tx translation: " << calibrationParams(3));
	ROS_DEBUG_STREAM("\ty translation: " << calibrationParams(4));
	ROS_DEBUG_STREAM("\tz translation: " << calibrationParams(5));

	// Save to output
	rOutputCalibrationParams = calibrationParams;
    return 0;
}