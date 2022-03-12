/*
This class can be used to calibrate two sensors, a LiDAR and a camera.
The process can be divided as following:
    1. Get camera information and then subscribe to point cloud and image data.
    2. Receive point cloud data and image data synchronized.
        - Point cloud:
            1. Go through row of data of point cloud and segment. Keeps the ones smaller than 
                half of the perimeter of the sphere.
            2. Cluster the segments that are together to each other.
            3. Fit sphere to every cluster and keep the one with best inliers/outliers rate.
            4. Save parameters of sphere as 3D. (X, Y, Z) of the center.
        - Image:
            1. Project the image to a spherical screen so that spheres are seen as circles.
            2. Get gray image.
            3. Blur image.
            4. Calculate Hough Circles.
            5. Calculate Canny edges.
            6. Optimize position and radius of Hough circles thanks to canny edges.
            7. Choose the best circle.
            8. Calculate X, Y and Z of the center of the sphere and save as 3D data. (X, Y, Z) of the center.
            9. Save U, V of center of sphere in the image plane.
    3. Repeat 2 until the number of observations needed is reached.
    4. Get calibration parameters with Levenberg Marquardt on the 3D-3D data.
        4.1. Get random observations from the ones obtained.
        4.2. Obtain calibration parameters with LM algorithm in the 3D-3D data.
        4.3. Calculate the sum of error of all observations with each calibration parameters.
        4.4. Get inliers from the best calibration parameters. 
        4.5. Run a last LM algorithm with those inliers. 
    5. Get calibration parameters with Levenberg Marquardt on the 3D-2D data.
        5.1. Set the output of 4.5 as the starting point of the optimization problem.
        5.2. Get random observations from the ones obtained.
        5.3. Obtain calibration parameters with LM algorithm in the 3D-3D data.
        5.4. Calculate the sum of error of all observations with each calibration parameters.
        5.5. Get inliers from the best calibration parameters. 
        5.6. Run a last LM algorithm with those inliers. 
*/

#include "simulateCalibrateSensors.hpp"

CalibrateSensors::CalibrateSensors()
{
    datetime();    
    // Read the ros parameters for the tools
    initParameters(nodeHandle);
    initTools();

    if (mParametersDefinedWrongly) {
        // Subscribe to images, point clouds and camera information
        mCameraInfoSubscriber = nodeHandle.subscribe<sensor_msgs::CameraInfo>( mCameraInfoTopicName, 1, &CalibrateSensors::cameraInfoCallback, this );
        mImageSubscriber.subscribe(nodeHandle, mImageTopicName, 1);
        mPointCloudSubscriber.subscribe(nodeHandle, mPointcloudTopicName, 1);
        // Synchronize the data
        mpSync.reset(new Sync(MySyncPolicy(10), mImageSubscriber, mPointCloudSubscriber));

        // Publishers
        // Image transport
        image_transport::ImageTransport it(nodeHandle);     

        // Initialize publishers
        mCloudProjectedImagePublisher = it.advertise("cloud_projected_to_image", 1);

        // Service of spheric target pose
        mSphericalTargetModelStateServiceClient = nodeHandle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        //  Image window
        if (mSeeProjectPointCloudToImage) {
            cv::namedWindow("Projection");
        }

        ROS_INFO_STREAM("Calibration node init OK");
    }
}

CalibrateSensors::~CalibrateSensors()
{
    // Image window
    if (mSeeProjectPointCloudToImage) {
        cv::destroyWindow("Projection");
    }

    delete mFindSphericalTargetImage;
    delete mFindSphericalTargetPointCloud;
    delete mRemoveOutliersListTool;
    delete mSeveral3Dto3DCalibrationTool;
    delete mSeveral3Dto2DCalibrationTool;
}

void CalibrateSensors::initParameters(ros::NodeHandle &nodeHangle) 
{
    /*
    Method that initialize the parameters declared in ROS. These are:
        - analysisData/calibrationParametersOutput: do several calibration and save parameters to analyze calibration params.
        - analysisData/accuracySensorsCenterSphericalTarget: save the center of the spherical target for analyzing.
        - sphericalTarget/radius: radius in meters of the spherical target used to calibrate the sensors.
        - output_files_directory: directory where text files with positions of spheres or calibration parameters will be saved.
        - calibration/numberPositions: number of positions of sphere recorded for the calibration process.
        - pointcloudTopicName: topic to which subscribe to get point clouds.
        - pointcloudNumberOfMessages: number of messages that will be concatenated from the pointcloud topic.
        - imageTopicName: topic to which subscribe to get point clouds.
        - calibration: in calibration, parameters for the calibration process are defined such as paramters for the LM algorithm.
    For more detail, read main comment on header file.
    */
    // Setup
    nodeHandle.param<std::string>( "spherical_target_gazebo_model_name", mSphericalTargetGazeboModelName, std::string( "spherical_target" ));
    // Analysis of data
    nodeHandle.param<bool>( "calibration/analysisData/testing", mTesting, true );
    nodeHandle.param<bool>( "calibration/analysisData/seeProjectPointCloudToImage", mSeeProjectPointCloudToImage, false );
    nodeHandle.param<bool>( "calibration/analysisData/calibrationParametersOutput", mAnalysisDataCalibrationParams, false );
    nodeHandle.param<int>( "calibration/analysisData/numberOfCalibrations", mNumberOfCalibrationsToDo, 300 );
    nodeHandle.param<bool>( "calibration/analysisData/accuracySensorsCenterSphericalTarget", mAnalysisDataAccuracySensors, false );
    // Spherical target information
    nodeHandle.param<double>( "calibration/sphericalTarget/radius", mSphericalTargetRadius, 0.5 );
    // Output files directory
    nodeHandle.param<std::string>( "output_files_irectory", mOutputFilesDirectory, std::string( "./" ));
    // Calibration parameters
    nodeHandle.param<int>( "calibration/numberPositions", mNumberPositions, 300 );
    // Listening to topics
    nodeHandle.param<std::string>( "calibration/pointcloudTopicName", mPointcloudTopicName, std::string( "/velodyne_points" ));
    nodeHandle.param<std::string>( "calibration/imageTopicName", mImageTopicName, std::string( "/camera_cam_rear/image_raw" ));
    nodeHandle.param<std::string>( "calibration/cameraInfoTopicName", mCameraInfoTopicName, std::string( "/camera_cam_rear/camera_info" ));
    // Calibration
    // Outlier removal
    nodeHandle.param<double>( "calibration/calibration/outlierRemovalList/kTimeStdDev", mOutlierRemovalKTimesStdDev, 1 );
    // Several calibration 3D-3D
    nodeHandle.param<int>( "calibration/calibration/severalCalibration3Dto3D/numberOfCalibration", mSeveralCalibrationNumberOfCalibration3D, 1 );
    nodeHandle.param<int>( "calibration/calibration/severalCalibration3Dto3D/observationsPerCalibration", mSeveralCalibrationObservationsPerCalibration3D, 150 );
    nodeHandle.param<double>( "calibration/calibration/severalCalibration3Dto3D/maxDistanceErrorCalibrationForInliers", 
            mSeveralCalibrationMaxDistanceErrorCalibrationForInliers3D, 2.0 );
    // Several calibration 3D-2D
    nodeHandle.param<int>( "calibration/calibration/severalCalibration3Dto2D/numberOfCalibration", mSeveralCalibrationNumberOfCalibration2D, 1 );
    nodeHandle.param<int>( "calibration/calibration/severalCalibration3Dto2D/observationsPerCalibration", mSeveralCalibrationObservationsPerCalibration2D, 150 );
    nodeHandle.param<double>( "calibration/calibration/severalCalibration3Dto2D/maxDistanceErrorCalibrationForInliers", 
            mSeveralCalibrationMaxDistanceErrorCalibrationForInliers2D, 2.0 );

    // Check that parameters make sense
    if (mNumberPositions < mSeveralCalibrationObservationsPerCalibration3D) {
        ROS_ERROR_STREAM("DEFINITION OF PARAMETERS: Parameter numberPosition (" << mNumberPositions 
            << ") should be higher than observationsPerCalibration (" << mSeveralCalibrationObservationsPerCalibration3D << ").");
        mParametersDefinedWrongly = false;
    }
    if (mNumberPositions < mSeveralCalibrationObservationsPerCalibration2D) {
        ROS_ERROR_STREAM("DEFINITION OF PARAMETERS: Parameter numberPosition (" << mNumberPositions 
            << ") should be higher than observationsPerCalibration (" << mSeveralCalibrationObservationsPerCalibration2D << ").");
        mParametersDefinedWrongly = false;
    }
}

void CalibrateSensors::initTools()
{
    /*
    Method where the tools used are initialized.
    */
    mFindSphericalTargetImage = new FindSphericalTargetImage(nodeHandle);
    mFindSphericalTargetPointCloud = new FindSphericalTargetPointCloud(nodeHandle);

    mRemoveOutliersListTool = new RemoveOutliersListTool();
    mSeveral3Dto3DCalibrationTool = new Several3Dto3DCalibrationTool(mSeveralCalibrationNumberOfCalibration3D, mSeveralCalibrationObservationsPerCalibration3D,
            mSeveralCalibrationMaxDistanceErrorCalibrationForInliers3D);
}

void CalibrateSensors::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& pCameraInfoMsg)
{
    /*
    Callback for the camera info:
    * Input parameters:
        - pCameraInfoMsg: It is the pointer to the camera info message received.
    */

    // Read K and R matrix and save in Image format
    cv::Mat intrinsicMatrixKCamera;
    cv::Mat intrinsicMatrixRCamera;
    intrinsicMatrixKCamera = (cv::Mat_<float>(3,3) << 
               pCameraInfoMsg->K[0],       pCameraInfoMsg->K[1],      pCameraInfoMsg->K[2],
               pCameraInfoMsg->K[3],       pCameraInfoMsg->K[4],      pCameraInfoMsg->K[5],
               pCameraInfoMsg->K[6],       pCameraInfoMsg->K[7],      pCameraInfoMsg->K[8]
               );
    intrinsicMatrixRCamera = (cv::Mat_<float>(3,3) << 
               pCameraInfoMsg->R[0],       pCameraInfoMsg->R[1],      pCameraInfoMsg->R[2],
               pCameraInfoMsg->R[3],       pCameraInfoMsg->R[4],      pCameraInfoMsg->R[5],
               pCameraInfoMsg->R[6],       pCameraInfoMsg->R[7],      pCameraInfoMsg->R[8]
               );
    // Save K as Eigen matrix too
    mIntrinsicMatrixKCameraEigen << pCameraInfoMsg->K[0], 0, pCameraInfoMsg->K[2], 0, pCameraInfoMsg->K[4], pCameraInfoMsg->K[5], 0, 0, 1;

    // Initialize tools that needed from K or R matrix
    mFindSphericalTargetImage->setCameraIntrinsicParameters(intrinsicMatrixKCamera, intrinsicMatrixRCamera);
    mSeveral3Dto2DCalibrationTool = new Several3Dto2DCalibrationTool(mSeveralCalibrationNumberOfCalibration2D, 
            mSeveralCalibrationObservationsPerCalibration2D,
            mSeveralCalibrationMaxDistanceErrorCalibrationForInliers2D, mIntrinsicMatrixKCameraEigen);

    // Subscribe to the image and point cloud data now
    mpSync->registerCallback(boost::bind(&CalibrateSensors::dataCallback, this, _1, _2));
    // Unsubscribing to this topic as info has been saved
    mCameraInfoSubscriber.shutdown();


    ROS_INFO_STREAM("CALIBRATION: Camera information received. K: " << intrinsicMatrixKCamera
        << " and R: " << intrinsicMatrixRCamera << ". Unsubscribing from topic.");
    return;
}

void CalibrateSensors::dataCallback(const sensor_msgs::ImageConstPtr& pImageMsg, const sensor_msgs::PointCloud2::ConstPtr& pPointcloudMsg)
{
    /*
    Callback for the images and point clouds:
    * Input parameters:
        - pImageMsg: It is the pointer to the image received. 
        - pointcloudMsg: It is the pointer to the cloud received. 
    */
    int status = 0;
    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Define with 0s the Vectors of the center of the spherical target that want to be obtained
    Eigen::Vector3d pointCenterSphereGroundTruth = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d pointCenterSpherePointCloud = Eigen::Vector3d(0,0,0);
    Eigen::Vector3d pointCenterSphereImage = Eigen::Vector3d(0,0,0);
    Eigen::Vector2d pixelsCenterSphereImage = Eigen::Vector2d(0,0);

    // GROUND TRUTH
    // Get ground truth position of spherical target
    // Get position from demo robot
    gazebo_msgs::GetModelState sphericalTargetState;
    sphericalTargetState.request.model_name = (std::string) (mSphericalTargetGazeboModelName);
    sphericalTargetState.request.relative_entity_name = (std::string) ("demo_robot");
    if (mSphericalTargetModelStateServiceClient.call(sphericalTargetState))
    {
        ROS_DEBUG_STREAM("Position of spherical target: " << sphericalTargetState.response.pose.position << ".");
        // Get X, Y, Z from demo_robot and save to vector
        pointCenterSphereGroundTruth = Eigen::Vector3d(sphericalTargetState.response.pose.position.x,
                                sphericalTargetState.response.pose.position.y,
                                sphericalTargetState.response.pose.position.z);
    }
    else
    {
        ROS_ERROR_STREAM("SPHERIC TARGET GROUNDTRUTH: Failed to call service /gazebo/get_model_state");
        return;
    }

    // POINT CLOUD
    // Finding the center of the spherical target in the pointcloud
    // Transform the message to a pcl::PCLPointCloud2 and then to PCL point cloud  
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );  
    pcl::fromROSMsg(*pPointcloudMsg, *cloud);
    int statusPointCloud = mFindSphericalTargetPointCloud->getCenterSphericalTarget(cloud, pointCenterSpherePointCloud);
    mFailPointCloud += statusPointCloud;
    if (statusPointCloud != 0) {
        ROS_ERROR_STREAM("FINDING SPHERICAL TARGET: Process for finding the spherical target in the point cloud data failed.");
        status = 1;
    }

    // IMAGE
    // Finding the center of the spherical target in the image
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(pImageMsg, "bgr8");
    int statusImage = mFindSphericalTargetImage->getCenterSphericalTarget(image, pixelsCenterSphereImage, pointCenterSphereImage);
    mFailImage += statusImage;

    if (statusImage != 0) {
        ROS_ERROR_STREAM("FINDING SPHERICAL TARGET: Process for finding the spherical target in the image data failed.");
        status = 1;
    }

    // Project Point cloud to image and show in CV window
    projectPointCloudToImage(cloud, image, eulerAnglesCalibrationVector, translationCalibrationVector);

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    if (status != 0) {
        ROS_ERROR_STREAM("FINDING SPHERICAL TARGET: Finding spherical target process failed. It took " << duration.count() << " seconds.");
        return;
    }
    ROS_INFO_STREAM("FINDING SPHERICAL TARGET: Finding spherical target process in pointcloud with timestamp: " << pPointcloudMsg->header.stamp
        << " and image with timestamp: " << pImageMsg->header.stamp << " done successfully. It took " << duration.count() << " seconds.");


    if (pointCenterSpherePointCloud != Eigen::Vector3d(0,0,0) && pointCenterSphereImage != Eigen::Vector3d(0,0,0)
            && pixelsCenterSphereImage != Eigen::Vector2d(0,0)) {
        mListPointsCenterSphericalTargetGroundTruth.push_back(pointCenterSphereGroundTruth);
        mListPointsCenterSphericalTargetPointCloud.push_back(pointCenterSpherePointCloud);
        mListPointsCenterSphericalTargetImage.push_back(pointCenterSphereImage);
        mListPixelsCenterSphericalTargetImage.push_back(pixelsCenterSphereImage);
        ROS_INFO_STREAM("FINDING SPHERICAL TARGET: There are: " << mListPointsCenterSphericalTargetGroundTruth.size() 
                << " spherical positions in list. Missing: " << mNumberPositions-mListPointsCenterSphericalTargetGroundTruth.size() << ".");
        // The size of the three lists should be the same so that the observations are paired together. Otherwise clear.
        // This should not happen but it is a security measure
        if (mListPointsCenterSphericalTargetPointCloud.size() != mListPointsCenterSphericalTargetImage.size() || 
                            mListPointsCenterSphericalTargetPointCloud.size() != mListPixelsCenterSphericalTargetImage.size()) {
            ROS_ERROR_STREAM("CALIBRATION: Number of positions of spherical targets found by image and pointcloud are not the same."
                << " Emptying list to start again as the matching pairs then cannot be known.");
            mListPointsCenterSphericalTargetGroundTruth.clear();
            mListPointsCenterSphericalTargetPointCloud.clear();
            mListPointsCenterSphericalTargetImage.clear();
            mListPixelsCenterSphericalTargetImage.clear();
        // Calibrate sensors if number of positions of sphere found is enough
        } else if (mListPointsCenterSphericalTargetPointCloud.size() == mNumberPositions && mListPointsCenterSphericalTargetImage.size() == mNumberPositions
                                && mListPixelsCenterSphericalTargetImage.size() == mNumberPositions) {
            // Save output of center if boolean is true
            if (mAnalysisDataAccuracySensors) {
                calculatePerformanceSensorFindingTarget();
            }

            ROS_INFO_STREAM("CALIBRATION: The spherical target has been found in " << mNumberPositions << " positions. Calibrating.");
            ROS_INFO_STREAM("CALIBRATION: Point cloud process failed "
                    << mFailPointCloud << " times. Image process failed " << mFailImage << " times.");
            status = calibrateSensors();
            // Check status after calibration and unsubscribe and shut down process
            if (status == 0) {
                // In case of success: Do several calibration if it is analysis process
                if ((mTesting)) {
                    mListPointsCenterSphericalTargetGroundTruth.clear();
                    mListPointsCenterSphericalTargetPointCloud.clear();
                    mListPointsCenterSphericalTargetImage.clear();
                    mListPixelsCenterSphericalTargetImage.clear();
                    mNumberOfCalibrationsDone++;
                    ROS_INFO_STREAM("CALIBRATION: " << mNumberOfCalibrationsDone << " number of calibrations done.");
                // Or stop
                } else {
                    mImageSubscriber.unsubscribe();
                    mPointCloudSubscriber.unsubscribe();
                    ROS_INFO_STREAM("CALIBRATION: Calibration process succeded. Unsubscribing to topic and shuting down process.");
                    ros::shutdown();
                    mRequestShutdown = true;
                }
            // If it failed, clear list and re-start
            } else {
                mListPointsCenterSphericalTargetGroundTruth.clear();
                mListPointsCenterSphericalTargetPointCloud.clear();
                mListPointsCenterSphericalTargetImage.clear();
                mListPixelsCenterSphericalTargetImage.clear();
                ROS_ERROR_STREAM("CALIBRATION: Calibration process failed. Clearing list of points and starting again.");
            }
        }
    }

    return;
}

int CalibrateSensors::calibrateSensors()
{
    /*
    Method that computes the calibration parameters.
    */
    int status = 0;
    // Time the process 
    auto start = std::chrono::system_clock::now();
    ROS_INFO_STREAM("CALIBRATION: Computating the calibration parameters.");

    // Remove outliers from lists
    std::vector<Eigen::Vector3d> listPointCloudWoOutliers;
    std::vector<Eigen::Vector3d> listPointCloudWoOutliers2;
    std::vector<Eigen::Vector3d> listImageWoOutliers;
    std::vector<Eigen::Vector2d> listImagePixelWoOutliers;
    status = mRemoveOutliersListTool->removeOutliers3d3d(mListPointsCenterSphericalTargetPointCloud, mListPointsCenterSphericalTargetImage, 
                    mOutlierRemovalKTimesStdDev, listPointCloudWoOutliers, listImageWoOutliers);
    status = mRemoveOutliersListTool->removeOutliers3d2d(mListPointsCenterSphericalTargetPointCloud, mListPixelsCenterSphericalTargetImage, 
                    mOutlierRemovalKTimesStdDev, listPointCloudWoOutliers2, listImagePixelWoOutliers);


    // 3D-3D Get calibration optimized by doing several calibrations
    Eigen::VectorXd calibrationParams(6);
    status = mSeveral3Dto3DCalibrationTool->getFinalCalibration(listPointCloudWoOutliers, listImageWoOutliers, calibrationParams);

    if (status == 0) {
        Eigen::Vector3d eulerAngleRotation = Eigen::Vector3d(calibrationParams(0), calibrationParams(1), calibrationParams(2));
        Eigen::Vector3d translationVector = Eigen::Vector3d(calibrationParams(3), calibrationParams(4), calibrationParams(5));
        Eigen::Quaterniond quaternions;
        // 3D-2D Get final calibration optimized by doing several calibrations
        status = mSeveral3Dto2DCalibrationTool->getFinalCalibration(listPointCloudWoOutliers2, listImagePixelWoOutliers,
                eulerAngleRotation, translationVector, quaternions);
        if ((status == 0) && (mAnalysisDataCalibrationParams)) {
            // Save params to file 
            std::string outputFile = mOutputFilesDirectory;
            outputFile += "calibrationParameters_" + mStartingDatetime + ".txt";
            mOutfile.open(outputFile, std::ios_base::app);
            mOutfile << eulerAngleRotation[0] << " " << eulerAngleRotation[1] << " " << eulerAngleRotation[2] << " "
                << translationVector[0] << " " << translationVector[1] << " " << translationVector[2] <<"\n" ; 
            mOutfile.close();
            ROS_INFO_STREAM("Calibration succeded!!");
        }
        eulerAnglesCalibrationVector = eulerAngleRotation;
        translationCalibrationVector = translationVector;
        mIsCalibrationDone = true;
    }

    if (status != 0) {
        ROS_ERROR_STREAM("CALIBRATION: Computation of calibration parameters failed.");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_INFO_STREAM("CALIBRATION: Computation of calibration parameters succesful. It took " << duration.count()
        << " seconds. ");

    return 0;
}

int CalibrateSensors::calculatePerformanceSensorFindingTarget()
{
    /*
    Method that saves the center position of the spherical target in different file formats so they can be analyzed.
    */
    Eigen::Vector3d centerSphericalTargetGroundTruth;
    Eigen::Vector3d centerSphericalTargetPointCloud;
    Eigen::Vector3d centerSphericalTargetImage;
    Eigen::Vector2d pixelCenterSphericalTargetImage;
    int i;
    for (auto it = std::begin(mListPointsCenterSphericalTargetGroundTruth); it != std::end(mListPointsCenterSphericalTargetGroundTruth); ++it) { 
        // Get point from list
        i = it - mListPointsCenterSphericalTargetGroundTruth.begin();
        centerSphericalTargetGroundTruth = mListPointsCenterSphericalTargetGroundTruth[i];
        centerSphericalTargetPointCloud = mListPointsCenterSphericalTargetPointCloud[i];
        centerSphericalTargetImage = mListPointsCenterSphericalTargetImage[i];
        pixelCenterSphericalTargetImage = mListPixelsCenterSphericalTargetImage[i];

        // Transform position of spheric target from velodyne to base_link
        try {
            // To check the error of finding the center of the sphere in 3D both in Point cloud and Camera
            // PointCloud
            geometry_msgs::PointStamped pointTargetPointCloud;
            pointTargetPointCloud.point.x = centerSphericalTargetPointCloud[0];
            pointTargetPointCloud.point.y = centerSphericalTargetPointCloud[1];
            pointTargetPointCloud.point.z = centerSphericalTargetPointCloud[2];
            pointTargetPointCloud.header.frame_id = "/velodyne";
            pointTargetPointCloud.header.stamp = ros::Time();
            // Transform to base_link
            geometry_msgs::PointStamped pointTargetFromRobotPointCloud;
            mTfListener.transformPoint("base_link", pointTargetPointCloud, pointTargetFromRobotPointCloud);
            
            // Image
            geometry_msgs::PointStamped pointTargetImage;
            pointTargetImage.point.x = centerSphericalTargetImage[0];
            pointTargetImage.point.y = centerSphericalTargetImage[1];
            pointTargetImage.point.z = centerSphericalTargetImage[2];
            pointTargetImage.header.frame_id = "/camera_cam_rear_optical_link"; // camera_cam_rear_link
            pointTargetImage.header.stamp = ros::Time();
            // Transform to base_link
            geometry_msgs::PointStamped pointTargetFromRobotImage;
            mTfListener.transformPoint("base_link", pointTargetImage, pointTargetFromRobotImage);

            // Save file
            // Center of target from base_link of ground truth, point cloud and 3D image
            std::string outputFile = mOutputFilesDirectory;
            outputFile += "sphericalTargetPositionSensors_" + mStartingDatetime + ".txt";
            mOutfile.open(outputFile, std::ios_base::app);
            mOutfile << centerSphericalTargetGroundTruth[0] << " " << centerSphericalTargetGroundTruth[1] 
                << " " << centerSphericalTargetGroundTruth[2] << " "
                << pointTargetFromRobotPointCloud.point.x << " " << pointTargetFromRobotPointCloud.point.y 
                << " " << pointTargetFromRobotPointCloud.point.z << " " << pointTargetFromRobotImage.point.x 
                << " " << pointTargetFromRobotImage.point.y << " " << pointTargetFromRobotImage.point.z <<"\n" ; 
            mOutfile.close();

            // Save file
            // Center of target from base_link of ground truth, point cloud and 3D image
            std::string outputFile1 = mOutputFilesDirectory;
            outputFile1 += "errorLidar_" + mStartingDatetime + ".txt";
            mOutfile.open(outputFile1, std::ios_base::app);
            double absoluteError = sqrt(pow(centerSphericalTargetGroundTruth[0]-pointTargetFromRobotPointCloud.point.x, 2)
             + pow(centerSphericalTargetGroundTruth[1]-pointTargetFromRobotPointCloud.point.y, 2)
			 + pow(centerSphericalTargetGroundTruth[2]-pointTargetFromRobotPointCloud.point.z, 2));
            mOutfile << absoluteError  <<"\n" ; 
            mOutfile.close();

            // To check the error of finding the center of the sphere in 2D in image
            // Get ground truth from camera so it can be projected onto the image
            // Create point
            geometry_msgs::PointStamped targetInImage;
            targetInImage.point.x = centerSphericalTargetGroundTruth[0];
            targetInImage.point.y = centerSphericalTargetGroundTruth[1];
            targetInImage.point.z = centerSphericalTargetGroundTruth[2];
            targetInImage.header.frame_id = "base_link"; // camera_cam_rear_link
            targetInImage.header.stamp = ros::Time();
            // Transform to base_link
            geometry_msgs::PointStamped pointTargetInImage;
            mTfListener.transformPoint("/camera_cam_rear_optical_link", targetInImage, pointTargetInImage);

            // Save file
            std::string outputFile2 = mOutputFilesDirectory;
            outputFile2 += "targetImageError_" + mStartingDatetime + ".txt";
            mOutfile.open(outputFile2, std::ios_base::app);
            mOutfile << pointTargetInImage.point.x << " " << pointTargetInImage.point.y << " " << pointTargetInImage.point.z
                << " " << pixelCenterSphericalTargetImage[0] << " " << pixelCenterSphericalTargetImage[1] <<"\n" ;  
            mOutfile.close();

            double uPoint = (pointTargetInImage.point.x * mIntrinsicMatrixKCameraEigen(0, 0) 
                        / pointTargetInImage.point.z) + mIntrinsicMatrixKCameraEigen(0, 2);
            double vPoint = (pointTargetInImage.point.y * mIntrinsicMatrixKCameraEigen(1, 1) 
                        / pointTargetInImage.point.z) + mIntrinsicMatrixKCameraEigen(1, 2);
            double absoluteError2 = sqrt(pow(pixelCenterSphericalTargetImage[0]-uPoint, 2)
             + pow(pixelCenterSphericalTargetImage[1]-vPoint, 2));

            // Save file
            std::string outputFile3 = mOutputFilesDirectory;
            outputFile3 += "errorImage_" + mStartingDatetime + ".txt";
            mOutfile.open(outputFile3, std::ios_base::app);
            mOutfile << absoluteError2 <<"\n" ;  
            mOutfile.close();
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            return 1;
        }
    }
    return 0;
}

void CalibrateSensors::projectPointCloudToImage(pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, cv_bridge::CvImageConstPtr pImage,
        Eigen::Vector3d rollPithYawVector, Eigen::Vector3d translationVector)
{
    /*
    Method that projects the point cloud received to the image received by the RPY vector and translation vector.
    * Input parameters:
        - pCloud: cloud that will be projected onto the image.
        - pImage: image where the cloud will be projected.
        - rollPitchYawVector: vector of 3 doubles with Roll, Pitch and Yaw angles in degrees. 
                The point cloud will be rotated in order ZYX by these angles.
        - translationVector: vector of 3 doubles with translation in X, Y and Z.
                The point cloud will be translated by this vector. 
    */

    // Define parameters
    Eigen::Vector3d lidarPointVector, cameraPointVector;
    Eigen::Matrix3d rotationMatrix;
    double u, v, depth;
    std::vector<double> listUPixels;
    std::vector<double> listVPixels;
    std::vector<double> listDepth;
    double minDepth = 1000.0;
    double maxDepth = -1000.0;

    if (!mIsCalibrationDone) {
        rollPithYawVector = Eigen::Vector3d(0, -90, 0);
        translationVector = Eigen::Vector3d(0, 0, 0);
    }

    // Read image
    cv::Mat image = pImage->image;

    for(std::size_t i=0; i<pCloud->width; i++){
        for(std::size_t j=0; j<pCloud->height; j++){
            // Not consider NaN values of points
            if (isnan(pCloud->at(i,j).x) || isnan(pCloud->at(i,j).y) || isnan(pCloud->at(i,j).z) ) {
                continue;
            }

            // Go through all points in cloud
            lidarPointVector = Eigen::Vector3d(pCloud->at(i,j).x, pCloud->at(i,j).y, pCloud->at(i,j).z);

            // Find the point from camera frame
            rotationMatrix = getRotationMatrixFromYpr(rollPithYawVector);    
            cameraPointVector = rotationMatrix * lidarPointVector + translationVector;

            // Project point into image plane
            u = mIntrinsicMatrixKCameraEigen(0, 0) * cameraPointVector[0] / cameraPointVector[2] + mIntrinsicMatrixKCameraEigen(0, 2);
            v = mIntrinsicMatrixKCameraEigen(1, 1) * cameraPointVector[1] / cameraPointVector[2] + mIntrinsicMatrixKCameraEigen(1, 2);

            if ((u > 0) && (u < mIntrinsicMatrixKCameraEigen(0, 2)*2) && (v > 0) && (v < mIntrinsicMatrixKCameraEigen(1, 2)*2)
                        && (cameraPointVector[2] > 0)) {
                if (cameraPointVector[2] < minDepth) {
                    minDepth = cameraPointVector[2];
                }
                if (cameraPointVector[2] > maxDepth) {
                    maxDepth = cameraPointVector[2];
                }
                listUPixels.push_back(u);
                listVPixels.push_back(v);
                listDepth.push_back(cameraPointVector[2]);
            }
        }
    }

    // Put color in pixels in image by depth range
    int i;
    float color;
    for (auto it = std::begin(listUPixels); it != std::end(listUPixels); ++it) { 
        i = it - listUPixels.begin();
		u = listUPixels[i];
		v = listVPixels[i];
        depth = listDepth[i];

        // Calculate colour by depth range
        color = abs(255 - ((depth - minDepth) / (maxDepth - minDepth)) * (255 - 0));
        // Draw
        cv::Point center = cv::Point(u, v);
        // Draw circle center
        circle(image, center, 1, cv::Scalar(color, color, color), 2, cv::LINE_AA);
    }

    // Show
    if (mSeeProjectPointCloudToImage) {
        try {
            cv::imshow("Projection", image);
            cv::waitKey(30);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR_STREAM("Could not convert from '%s' to 'bgr8'.");
        }
    }
    
    publishImage(image, mCloudProjectedImagePublisher);
}

void CalibrateSensors::datetime()
{
    /*
    Method that saves the datetime of the start of calibration in the class. 
    */
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    // Get time
    time (&rawtime);
    timeinfo = localtime(&rawtime);

    // Save datetime in class attribute with certain format
    strftime(buffer,80,"%Y-%m-%d_%H-%M-%S",timeinfo);
    mStartingDatetime = std::string(buffer);
}

int main (int argc, char** argv)
{
    ros::init( argc, argv, "calibrateSensorsNode" );

    // Create the class to go through the process
    CalibrateSensors calibrateSensors;

    // Define a multi thread
    ros::MultiThreadedSpinner spinner(4);

    // Do while loop. Needed for proper shut down
    while (!calibrateSensors.mRequestShutdown)
    {
        // Normal way to spin
        ros::spinOnce();
        usleep(100000);
    }

    // Stop ROS node
    ros::shutdown();

    return 0;
}