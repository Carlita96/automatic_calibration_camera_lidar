/*
This class can be used to solve an optimization problem of a 3D-2D calibration.
The problem is a minimization problem. In this minimization problem, the rotation matrix R and translation vector T
are wanted.
The distance error from a 3D lidar point projected in the image plane
with the R and a the T to the 2D point of the image has to be minimized.
*/

#include "ceresSolver3Dto2DTool.hpp"

CeresSolver3Dto2DTool::CeresSolver3Dto2DTool()
{ 
}

CeresSolver3Dto2DTool::~CeresSolver3Dto2DTool(){
}

void CeresSolver3Dto2DTool::buildOptimizationProblem(const std::vector<Eigen::Vector2d> &rPointsImage,
                              const Eigen::Matrix3d intrinsicCameraMatrix,
                              std::vector<Eigen::Vector3d> &rPointsPointCloud,
                              Eigen::Quaterniond &rQuaternionsRotation,
                              Eigen::Vector3d &rTranslationVector,
                              ceres::Problem* pProblem)
{
    /*
    Method that does an Levenberg-Maquardt optimization of points in image and point cloud in order to
        calibrate a 3D sensor to a camera.
    * Input parameters: 
        - rPointsImage: list with points in 2D from the image data.
		- intrinsicCameraMatrix: intrinsic parameters camera matrix. [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        - rPointsPointCloud: list with points in 3D from the point cloud data.
        - rQuaternionsRotation: reference to the quaternions value as calibration params.
        - rTranslationVector: reference to the translation vector value as calibration params.
        - pProblem: Ceres Solver Problem from Eigen.
    */  

    // Create loss function
    ceres::LossFunction* lossFunction = new ceres::HuberLoss(1.0);

    // Define the parametization as quaternions
    ceres::LocalParameterization* quaternionLocalParameterization =
            new ceres::EigenQuaternionParameterization;

    // Define a problem for each observation
    for (int i = 0; i < rPointsPointCloud.size(); ++i)
    {
        // Define the functor and
        ceres::CostFunction* costFunction =
                CostFunctionCalibration3Dto2D::Create(rPointsImage[i], intrinsicCameraMatrix);
        pProblem->AddResidualBlock(costFunction,
                                  lossFunction,
                                  rQuaternionsRotation.coeffs().data(),
                                  rTranslationVector.data(),
                                  rPointsPointCloud[i].data());
        // Define which parameters are the quaternions
        pProblem->SetParameterization(rQuaternionsRotation.coeffs().data(),
                                     quaternionLocalParameterization);
    }

    // Block the point cloud points as they should not change
    for (int j = 0; j < rPointsPointCloud.size(); ++j)
    {
        pProblem->SetParameterBlockConstant(rPointsPointCloud[j].data());
    }
}

int CeresSolver3Dto2DTool::computeCalibrationParams(const std::vector<Eigen::Vector3d> listPositionCenterSphericalTargetPointCloud,
            const std::vector<Eigen::Vector2d> listPixelCenterSphericalImage, const int numberCalibrationParameters, 
			const Eigen::Vector3d startingEulerAngleVector, const Eigen::Vector3d startingTranslationVector, 
            const Eigen::Matrix3d intrinsicCameraMatrix, Eigen::VectorXd &rOutputCalibrationParams, Eigen::Quaterniond &rOutputQuaternions)
{
    /*
    Method that does an Levenberg-Maquardt optimization of points in image and point cloud in order to
        calibrate a camera an a LiDAR.
    * Input parameters: 
        - listPositionCenterSphericalTargetPointCloud: list with points in 3D from the point cloud data.
        - listPixelCenterSphericalImage: list with points in 2D from the image data.
            NOTICE: They must be paired. So point [0] in image is point [0] in point cloud.
        - numberCalibrationParameters: number of calibration parameters (alpha, beta, theta, x, y, z) -> 6
        - startingEulerAngleVector: euler angles Lidar->camera from which the problem will start to optimize the problem.
        - startingTranslationVector: translation vector Lidar->camera from which the problem will start to optimize the problem.
		- intrinsicCameraMatrix: intrinsic parameters camera matrix. [fx, 0, cx; 0, fy, cy; 0, 0, 1]
        - rOutputCalibrationParams: calibration parameters after optimization.
        - rOutputQuaternions: quaternions after optimization.
    */    
    // Check input 
    if (listPositionCenterSphericalTargetPointCloud.size() != listPixelCenterSphericalImage.size()) {
        ROS_ERROR_STREAM("CERES SOLVER 3D TO 2D TOOL: Number of points from image (" << listPixelCenterSphericalImage.size() 
        << ") and from pointcloud (" << listPositionCenterSphericalTargetPointCloud.size() << ") is not the same. Same number is needed.");
        return 1;
    }
    // Get number of observation pairs
	int n = numberCalibrationParameters;
    int m = listPositionCenterSphericalTargetPointCloud.size();
    ROS_DEBUG_STREAM("CERES SOLVER 3D TO 2D TOOL: Doing Ceres solver with " << m << " number of paired observations"
        << " and looking for " <<  n << " number of parameters. Doing calibration 3D-3D.");

    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Transform points to Double
    std::vector<Eigen::Vector3d> pointsPointcloud;
    std::vector<Eigen::Vector2d> pointsImage;
    pointsPointcloud = listPositionCenterSphericalTargetPointCloud;
    pointsImage = listPixelCenterSphericalImage;

    // Define the optimization parameters (Calibration params)
    Eigen::Matrix3d rotationMatrix = getRotationMatrixFromRpy(startingEulerAngleVector);
    Eigen::Quaterniond quaternions(rotationMatrix);
    Eigen::Vector3d translationVector = startingTranslationVector;
    ceres::Problem problem;

    // Build Ceres Solver Problem
    buildOptimizationProblem(pointsImage, intrinsicCameraMatrix, pointsPointcloud, quaternions, translationVector, &problem); 

    // Run the solver!
    ceres::Solver::Options options;
    //options.minimizer_progress_to_stdout = true;
	//options.minimizer_type = ceres::LINE_SEARCH;
    options.max_num_iterations = 5000;
    options.linear_solver_type = ceres::DENSE_QR;
    options.function_tolerance = 1e-16;
    options.gradient_tolerance = 1e-16;
    options.parameter_tolerance = 1e-16;
	//options.line_search_type = ceres::ARMIJO; // WOLFE for BFGS/LBGS, ARMIJO
	//options.linear_solver_type = ceres::ITERATIVE_SCHUR; // DENSE_QR , SPARSE_NORMAL_CHOLESKY, DENSE_NORMAL_CHOLESKY, SPARSE_SCHUR, ITERATIVE_SCHUR, CGNR
	//options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT; // NONLINEAR_CONJUGATE_GRADIENT, BFGS, LBFGS
	//options.nonlinear_conjugate_gradient_type = ceres::POLAK_RIBIERE; // POLAK_RIBIERE, HESTENES_STIEFEL
	//options.preconditioner_type = ceres::SCHUR_JACOBI;//IDENTITY, JACOBI, SCHUR_JACOBI (ITERATIVE_SCHUR), CLUSTER_JACOBI, CLUSTER_TRIDIAGONAL, SUBSET
	//options.trust_region_strategy_type = ceres::DOGLEG;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);


    // Check output 
    if (summary.termination_type != ceres::CONVERGENCE) {
        ROS_ERROR_STREAM("CERES SOLVER 3D TO 2D TOOL: Ceres solver optimization did not converged. "
                << "It retrieved termination type: " << summary.termination_type << ".");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("CERES SOLVER 3D TO 2D TOOL: Ceres solver done in " << duration.count() << " seconds.");
	// Results
    Eigen::Vector3d outputEulerAngles = quaternions.toRotationMatrix().eulerAngles(2, 1, 0);
	ROS_DEBUG_STREAM("CERES SOLVER 3D TO 2D TOOL: 3D-2D Optimization results");
	ROS_DEBUG_STREAM("\talpha rotation: " << outputEulerAngles[2] / 3.14159 * 180);
	ROS_DEBUG_STREAM("\tbeta rotation: " << outputEulerAngles[1] / 3.14159 * 180);
	ROS_DEBUG_STREAM("\ttheta rotation: " << outputEulerAngles[0] / 3.14159 * 180);
	ROS_DEBUG_STREAM("\tx translation: " << translationVector[0]);
	ROS_DEBUG_STREAM("\ty translation: " << translationVector[1]);
	ROS_DEBUG_STREAM("\tz translation: " << translationVector[2]);
    Eigen::VectorXd outputCalibrationParams(numberCalibrationParameters); 
    outputCalibrationParams << outputEulerAngles[2] / 3.14159 * 180, outputEulerAngles[1] / 3.14159 * 180, 
            outputEulerAngles[0] / 3.14159 * 180, translationVector[0],  translationVector[1], translationVector[2];
    rOutputCalibrationParams = outputCalibrationParams;
    rOutputQuaternions = quaternions;
    return 0;
}