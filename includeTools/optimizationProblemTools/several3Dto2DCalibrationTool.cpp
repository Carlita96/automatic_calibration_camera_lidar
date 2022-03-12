/*
This class can be used to solve an optimization problem of a 3D-2D calibration.
Process is:
    1. From a list of N numbers of observations, M numbers of optimization problems are done.
        This problem is run on L<N number of random observations from the N observations. 
    2. The distance error sum of all observations in each M optimization problem is calculated.
    3. The problem m from M with lowest distance error sum is considerd the best calibration fit.
    4. All points that are within a threshold right in this best calibration are considered inliers.
    5. Run a last optimization problem with the inliers of the best calibration.
*/

#include "several3Dto2DCalibrationTool.hpp"

Several3Dto2DCalibrationTool::Several3Dto2DCalibrationTool(const int numberOfCalibration, const int observationsPerCalibration,
		const double maxDistanceErrorCalibrationForInliers, const Eigen::Matrix3d intrinsicCameraMatrixK)
{ 
	mNumberOfCalibration = numberOfCalibration;
	mObservationsPerCalibration = observationsPerCalibration;
	mMaxDistanceErrorCalibrationForInliers = maxDistanceErrorCalibrationForInliers;
	mIntrinsicCameraMatrixK = intrinsicCameraMatrixK;
	mNumberOfCalibrationParameters = 6;

    mCeresSolver3Dto2DTool = new CeresSolver3Dto2DTool();
}

Several3Dto2DCalibrationTool::~Several3Dto2DCalibrationTool()
{
    delete mCeresSolver3Dto2DTool;
}

int Several3Dto2DCalibrationTool::getFinalCalibration(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud, 
			const std::vector<Eigen::Vector2d> listPixelCenterSphericalTargetImage,
			Eigen::Vector3d &rEulerAngleVector, Eigen::Vector3d &rTranslationVector, Eigen::Quaterniond &rQuaternions) 
{
    /*
    Method that runs the optimization algorithm to find the calibration 
		parameters. It is run several times with random samples from the list of observations.
		Then, the best calibration parameters is found and observations inliers of it are saved. 
		A last optimization problem is done with the inlier observations of the best calibration.
    * Input parameters:
        - listPointCenterSphericalTargetPointCloud: list with points in 3D from the point cloud data.
        - listPixelCenterSphericalTargetImage: list with points in 2D from the image data.
            NOTICE: They must be paired. So point [0] in image is point [0] in point cloud.
		- rEulerAngleVector: Euler angle from which to optimize.
		- rTranslationVector: translation vector from which to optimize.
    */
	int status = 0;
    auto start = std::chrono::system_clock::now();
    // Check input 
    if (listPointCenterSphericalTargetPointCloud.size() != listPixelCenterSphericalTargetImage.size()) {
        ROS_ERROR_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: Number of points from image (" << listPixelCenterSphericalTargetImage.size() 
        << ") and from pointcloud (" << listPointCenterSphericalTargetPointCloud.size() << ") is not the same. Same number is needed.");
        return 1;
    }

	ROS_INFO_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: Running " << mNumberOfCalibration << " calibrations with " << mObservationsPerCalibration
		<< " observations per calibration and max distance error for inliers " << mMaxDistanceErrorCalibrationForInliers 
		<< ". List of points from image has size: " << listPixelCenterSphericalTargetImage.size() 
		<< " and from point cloud has size: " << listPointCenterSphericalTargetPointCloud.size() << ".");

	// Define needed parameters
	Eigen::Vector3d startingEulerAngleVector = rEulerAngleVector;
	Eigen::Vector3d startingTranslationVector = rTranslationVector;
	Eigen::VectorXd calibrationParams(mNumberOfCalibrationParameters);
	std::vector<Eigen::VectorXd> listCalibrationParameters;
	std::vector<int> listIndexFinalCalibration;
	Eigen::Quaterniond quaternions;

	// STEP 1: Implement several calibrations with random samples of observations
	status = implementSeveralCalibration(listPointCenterSphericalTargetPointCloud, listPixelCenterSphericalTargetImage, 
				startingEulerAngleVector, startingTranslationVector, listCalibrationParameters);
	if (status == 0) {
		// STEP 2: Get inliers of best calibration
		status = getInliersBestCalibration(listPointCenterSphericalTargetPointCloud, listPixelCenterSphericalTargetImage, listCalibrationParameters, listIndexFinalCalibration);
		if (status == 0) {
			// STEP 2.1: Get list of points of inliers 
			// Get list of observation from list of inliers
			std::vector<Eigen::Vector3d> listObservationsInliersPointCloud;
			std::vector<Eigen::Vector2d> listObservationsInliersImage;
			for (auto it = std::begin(listIndexFinalCalibration); it != std::end(listIndexFinalCalibration); ++it) { 
				listObservationsInliersPointCloud.push_back(listPointCenterSphericalTargetPointCloud[*it]);
				listObservationsInliersImage.push_back(listPixelCenterSphericalTargetImage[*it]);
			}
		
			// STEP 4: Do finel calibration
			status = mCeresSolver3Dto2DTool->computeCalibrationParams(listObservationsInliersPointCloud, listObservationsInliersImage, 
					mNumberOfCalibrationParameters, startingEulerAngleVector, startingTranslationVector,
					mIntrinsicCameraMatrixK, calibrationParams, quaternions);
		}
	} 

	// Check output 
    if (status != 0) {
        ROS_ERROR_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: Several calibration tool failed.");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_INFO_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: End calibration is:\n\t- Alpha rotation: " << calibrationParams(0)
		<< "\n\t- Beta rotation: " << calibrationParams(1) << "\n\t- Theta rotation: " << calibrationParams(2)
		<< "\n\t- X translation: " << calibrationParams(3) << "\n\t- Y translation: " << calibrationParams(4) 
		<< "\n\t- Z translation: " << calibrationParams(5) << "\n\t- Qx: " << quaternions.x() << "\n\t- Qy: "
		<< quaternions.y() << "\n\t- Qz: " << quaternions.z() << "\n\t- Qw: " << quaternions.w()
		<< ". Done in " << duration.count() << " seconds.");

	// Save output
	rEulerAngleVector = Eigen::Vector3d(calibrationParams(0), calibrationParams(1), calibrationParams(2));
	rTranslationVector = Eigen::Vector3d(calibrationParams(3), calibrationParams(4), calibrationParams(5));
	rQuaternions = quaternions;
    return 0;
}

int Several3Dto2DCalibrationTool::implementSeveralCalibration(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud, 
			const std::vector<Eigen::Vector2d> listPixelCenterSphericalTargetImage,
			const Eigen::Vector3d startingEulerAngleVector, const Eigen::Vector3d startingTranslationVector, 
			std::vector<Eigen::VectorXd> &rListCalibrationParameters) 
{
    /*
	Method that runs the Levenberg-Marquardt optimization algorithm to find the calibration 
		parameters. It is run several times with random samples from the list of observations.
    * Input parameters:
        - listPointCenterSphericalTargetPointCloud: list with points in 3D from the point cloud data.
        - listPixelCenterSphericalTargetImage: list with points in 2D from the image data.
            NOTICE: They must be paired. So point [0] in image is point [0] in point cloud.
		- startingEulerAngleVector: starting Euler angle from which to optimize.
		- startingTranslationVector: starting translation vector from which to optimize.
		- listCalibrationParameters: list with calibration parameters calculated.
    */
	int status = 0;
    auto start = std::chrono::system_clock::now();
    // Check input 
    if (listPointCenterSphericalTargetPointCloud.size() != listPixelCenterSphericalTargetImage.size()) {
        ROS_ERROR_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: Number of points from image (" << listPixelCenterSphericalTargetImage.size() 
        << ") and from pointcloud (" << listPointCenterSphericalTargetPointCloud.size() << ") is not the same. Same number is needed.");
        return 1;
    }

	// Do calibration several times
	std::vector<Eigen::Vector3d> listObservationsPointCloud;
    std::vector<Eigen::Vector2d> listObservationsImage;
	std::vector<int> randomObservationsIndex;
	Eigen::VectorXd calibrationParams(mNumberOfCalibrationParameters);
	Eigen::Quaterniond quaternions;

	// Outlier removal will make the observations per calibration be lower than expected. 
	// Check and save maximum value or value received
	int numberOfObservationsPerCalibration;
	if (listPointCenterSphericalTargetPointCloud.size() < mObservationsPerCalibration) {
		numberOfObservationsPerCalibration = listPointCenterSphericalTargetPointCloud.size();
	} else {
		numberOfObservationsPerCalibration = mObservationsPerCalibration;
	}

	// Do calibration
	for (int i=0; i<mNumberOfCalibration; i++) {
		// Clear list of observations
		listObservationsPointCloud.clear();
		listObservationsImage.clear();
		
		// Get list of random indexes
		randomObservationsIndex = getRandomObservationsIndex(numberOfObservationsPerCalibration, listPointCenterSphericalTargetPointCloud.size());
		// Get list of observations from the random indexes
		for (auto it = std::begin(randomObservationsIndex); it != std::end(randomObservationsIndex); ++it) { 
			listObservationsPointCloud.push_back(listPointCenterSphericalTargetPointCloud[*it]);
			listObservationsImage.push_back(listPixelCenterSphericalTargetImage[*it]);
		}

		// Run calibration and save output calibration parameters
		status = mCeresSolver3Dto2DTool->computeCalibrationParams(listObservationsPointCloud, listObservationsImage, 
				mNumberOfCalibrationParameters, startingEulerAngleVector, startingTranslationVector,
				mIntrinsicCameraMatrixK, calibrationParams, quaternions);
		rListCalibrationParameters.push_back(calibrationParams);
	}

	// Check output 
    if (rListCalibrationParameters.size() == 0) {
        ROS_ERROR_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: No calibration parameters have been received.");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_INFO_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: Done " << mNumberOfCalibration << " calibrations in " << duration.count() << " seconds.");
    return 0;
}

int Several3Dto2DCalibrationTool::getInliersBestCalibration(const std::vector<Eigen::Vector3d> listPointCenterSphericalTargetPointCloud,
            const std::vector<Eigen::Vector2d> listPixelCenterSphericalTargetImage, const std::vector<Eigen::VectorXd> listCalibrationParameters,
			std::vector<int> &rListIndexFinalCalibration) 
{
    /*
    Method that finds the best calibration. Then, inlier observations are found. The inliers are observations that have a 
		lower error than the threshold defined when using the best calibration.
    * Input parameters: 
        - listPointCenterSphericalTargetPointCloud: list with points in 3D from the point cloud data.
        - listPixelCenterSphericalTargetImage: list with points in 3D from the image data.
            NOTICE: They must be paired. So point [0] in image is point [0] in point cloud.
		- listCalibrationParameters: list of calibration parameters where the best will be found and which inliers will be found.
	* Output parameters:
		- rListIndexFinalCalibration: list of inliers in best calibration parameters.
    */
	int status = 0;
    auto start = std::chrono::system_clock::now();
    // Check input 
    if (listPointCenterSphericalTargetPointCloud.size() != listPixelCenterSphericalTargetImage.size()) {
        ROS_ERROR_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: Number of points from image (" << listPixelCenterSphericalTargetImage.size() 
        << ") and from pointcloud (" << listPointCenterSphericalTargetPointCloud.size() << ") is not the same. Same number is needed.");
        return 1;
    }
	if (listCalibrationParameters.size() == 0) {
        ROS_ERROR_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: No calibration parameters have been received. Run implementSeveralCalibration before.");
        return 1;
    }

	// Initialize needed params
	double minDistanceErrorCalibrationParams = 1000000.0;
	double totalDistanceErrorCalibrationParams = 0.0;
	double distanceErrorObservation;
	Eigen::VectorXd calibrationParams;
	Eigen::Vector3d rollPithYaw;
	Eigen::Matrix3d rotationMatrix;
	Eigen::Vector3d translationMatrix;
    Eigen::Vector3d pointPointCloud;
    Eigen::Vector2d pointImage;
	int i;
	// List to save inliers of best calibration parameters
	std::vector<int> listInliersIndexBestCalibrationParams;
	std::vector<int> listInliersIndexCalibrationParams;

	// Get the sum of distance errors of all observations with all calibration params
	// Save then the inliers of the best calibration -> inliers if distance error is less than max set in class
	for (auto it = std::begin(listCalibrationParameters); it != std::end(listCalibrationParameters); ++it) { 
		// Reset error
		totalDistanceErrorCalibrationParams = 0.0;
		listInliersIndexCalibrationParams.clear();
		// Get rotation and translation matrix
		calibrationParams = *it;
		rollPithYaw = Eigen::Vector3d(calibrationParams(0), calibrationParams(1), calibrationParams(2)); 
		rotationMatrix = getRotationMatrixFromYpr(rollPithYaw);
		translationMatrix = Eigen::Vector3d(calibrationParams(3), calibrationParams(4), calibrationParams(5));
		
		// Get distance error
		for (auto it1 = std::begin(listPointCenterSphericalTargetPointCloud); it1 != std::end(listPointCenterSphericalTargetPointCloud); ++it1) { 
			// Get observation points
			i = it1 - listPointCenterSphericalTargetPointCloud.begin();
			pointPointCloud = listPointCenterSphericalTargetPointCloud[i];
			pointImage = listPixelCenterSphericalTargetImage[i];

			// Add error to total error of this calibration parameters
			distanceErrorObservation = getAbsoluteDistanceErrorTransformation3Dto2D(pointPointCloud, pointImage, mIntrinsicCameraMatrixK,
					rotationMatrix, translationMatrix);
			totalDistanceErrorCalibrationParams += distanceErrorObservation;
			
			// Save inliers if the distance is smaller than max
			if (distanceErrorObservation < mMaxDistanceErrorCalibrationForInliers) {
				listInliersIndexCalibrationParams.push_back(i);
			}
		}

		// When error is less than saved error, save inliers as best calibration inliers
		if (totalDistanceErrorCalibrationParams < minDistanceErrorCalibrationParams) {
			listInliersIndexBestCalibrationParams = listInliersIndexCalibrationParams;
		}
	}

	// Check output 
    if (listInliersIndexBestCalibrationParams.size() == 0) {
        ROS_ERROR_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: No inliers where saved in the best calibration found.");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_INFO_STREAM("SEVERAL 3D TO 2D CALIBRATION TOOL: " << listInliersIndexBestCalibrationParams.size() 
		<< " number of inliers found for the best calibration. Done in " << duration.count() << " seconds.");

	// Save output
	rListIndexFinalCalibration = listInliersIndexBestCalibrationParams;
    return 0;
}

std::vector<int> Several3Dto2DCalibrationTool::getRandomObservationsIndex(int numberOfObservationsPerCalibration, int maxNumber, int minNumber)
{
	/*
	Method that gets numberOfObservationsPerCalibration number of random numbers between min and max number.
	* Input parameters:
		- numberOfObservationsPerCalibration: number of observations.
		- minNumber: minimum number from the random selection.
		- maxNumber: maximum number from the random selection.
	*/
	std::vector<int> randomIndexesList;
	std::vector<int> possibleIndexesList;

	// Get a list of possible indexes
	for (int i=minNumber; i<maxNumber; i++) {
		possibleIndexesList.push_back(i);
	}

	// While loop to get a random index from the list of possible index and delete it from the list
	int randomIndexInPossibleIndexesList;
	while (randomIndexesList.size() != numberOfObservationsPerCalibration) {
		// Get random index in the list
		randomIndexInPossibleIndexesList = rand() % possibleIndexesList.size();

		// Push it to the vector
		randomIndexesList.push_back(possibleIndexesList[randomIndexInPossibleIndexesList]);

		// Remove it from the other vector
		possibleIndexesList.erase(possibleIndexesList.begin() + randomIndexInPossibleIndexesList);
	}
	return randomIndexesList;
}