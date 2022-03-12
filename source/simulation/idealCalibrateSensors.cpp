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
            4. Get Hough Circles.
            5. ...? ToDo
            6. Calculate X, Y and Z of the center of the sphere and save as 3D data. (X, Y, Z) of the center.
            7. Save U, V of center of sphere in the image plane.
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

#include "idealCalibrateSensors.hpp"

CalibrateSensors::CalibrateSensors()
{
    datetime();    
    // Read the ros parameters for the tools
    initParameters(nodeHandle);
    initTools();

    if (mParametersDefinedWrongly) {
        // Subscribe to images, point clouds and camera information
        mCameraInfoSubscriber = nodeHandle.subscribe<sensor_msgs::CameraInfo>( mCameraInfoTopicName, 10000, &CalibrateSensors::cameraInfoCallback, this );
        mImageSubscriber.subscribe(nodeHandle, mImageTopicName, 1);
        mPointCloudSubscriber.subscribe(nodeHandle, mPointcloudTopicName, 1);
        // Synchronize the data
        mpSync.reset(new Sync(MySyncPolicy(10), mImageSubscriber, mPointCloudSubscriber));

        // Service of spheric target pose
        mSphericalTargetModelStateServiceClient = nodeHandle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

        ROS_INFO_STREAM("Calibration node init OK");
    }
}

CalibrateSensors::~CalibrateSensors()
{
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
    nodeHandle.param<bool>( "calibration/analysisData/calibrationParametersOutput", mAnalysisDataCalibrationParams, false );
    nodeHandle.param<int>( "calibration/analysisData/numberOfCalibrations", mNumberOfCalibrationsToDo, 300 );
    nodeHandle.param<double>( "calibration/analysisData/timeSyncError", mTimeSyncError, 0.05 );
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
    nodeHandle.param<int>( "calibration/calibration/outlierRemovalList/kTimeStdDev", mOutlierRemovalKTimesStdDev, 1 );
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
    mIntrinsicMatrixKCameraEigen << pCameraInfoMsg->K[0], 0, pCameraInfoMsg->K[2], 
                0, pCameraInfoMsg->K[4], pCameraInfoMsg->K[5], 
                0, 0, 1;

    // Initialize tools that needed from K or R matrix
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
    Eigen::Vector3d pointCenterSphereGroundTruthDelayed = Eigen::Vector3d(0,0,0);
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

    // Sleep
    int millisec = 1000 * mTimeSyncError;
    std::this_thread::sleep_for(std::chrono::milliseconds(millisec));
    if (mSphericalTargetModelStateServiceClient.call(sphericalTargetState))
    {
        ROS_DEBUG_STREAM("Position of spherical target: " << sphericalTargetState.response.pose.position << ".");
        // Get X, Y, Z from demo_robot and save to vector
        pointCenterSphereGroundTruthDelayed = Eigen::Vector3d(sphericalTargetState.response.pose.position.x,
                                sphericalTargetState.response.pose.position.y,
                                sphericalTargetState.response.pose.position.z);
    }
    else
    {
        ROS_ERROR_STREAM("SPHERIC TARGET GROUNDTRUTH: Failed to call service /gazebo/get_model_state");
        return;
    }

    // POINT CLOUD
    // IMAGE
    // Transform position of spheric target from velodyne to base_link
    try {
        // To check the error of finding the center of the sphere in 3D both in Point cloud and Camera
        // PointCloud
        geometry_msgs::PointStamped pointTargetPointCloud;
        pointTargetPointCloud.point.x = pointCenterSphereGroundTruthDelayed[0];
        pointTargetPointCloud.point.y = pointCenterSphereGroundTruthDelayed[1];
        pointTargetPointCloud.point.z = pointCenterSphereGroundTruthDelayed[2];
        pointTargetPointCloud.header.frame_id = "/base_link";
        pointTargetPointCloud.header.stamp = ros::Time();
        // Transform to base_link
        geometry_msgs::PointStamped pointTargetFromLidarPointCloud;
        mTfListener.transformPoint("velodyne", pointTargetPointCloud, pointTargetFromLidarPointCloud);
        pointCenterSpherePointCloud = Eigen::Vector3d(pointTargetFromLidarPointCloud.point.x, 
                        pointTargetFromLidarPointCloud.point.y,
                        pointTargetFromLidarPointCloud.point.z);
        
        // Image
        geometry_msgs::PointStamped pointTargetImage;
        pointTargetImage.point.x = pointCenterSphereGroundTruth[0];
        pointTargetImage.point.y = pointCenterSphereGroundTruth[1];
        pointTargetImage.point.z = pointCenterSphereGroundTruth[2];
        pointTargetImage.header.frame_id = "/base_link"; // camera_cam_rear_link
        pointTargetImage.header.stamp = ros::Time();
        // Transform to base_link
        geometry_msgs::PointStamped pointTargetFromCameraImage;
        mTfListener.transformPoint("camera_cam_rear_optical_link", pointTargetImage, pointTargetFromCameraImage);
        pointCenterSphereImage = Eigen::Vector3d(pointTargetFromCameraImage.point.x, 
                        pointTargetFromCameraImage.point.y,
                        pointTargetFromCameraImage.point.z);

        // Project to pixels
        pixelsCenterSphereImage[0] = (pointTargetFromCameraImage.point.x * mIntrinsicMatrixKCameraEigen(0, 0) / 
                    pointTargetFromCameraImage.point.z) + mIntrinsicMatrixKCameraEigen(0, 2);
        pixelsCenterSphereImage[1] = (pointTargetFromCameraImage.point.y * mIntrinsicMatrixKCameraEigen(1, 1) / 
                    pointTargetFromCameraImage.point.z) + mIntrinsicMatrixKCameraEigen(1, 2);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        return;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    if (status != 0) {
        ROS_ERROR_STREAM("FINDING SPHERICAL TARGET: Finding spherical target process failed. It took " << duration.count() << " seconds.");
        return;
    }
    ROS_INFO_STREAM("FINDING SPHERICAL TARGET: Finding spherical target process in pointcloud with timestamp: " << pPointcloudMsg->header.stamp
        << " and image with timestamp: " << pImageMsg->header.stamp << " done successfully. It took " << duration.count() << " seconds.");

    // Save position of sphere
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
            ROS_INFO_STREAM("CALIBRATION: The spherical target has been found in " << mNumberPositions << " positions. Calibrating.");
            status = calibrateSensors();
            // Check status after calibration and unsubscribe and shut down process
            if (status == 0) {
                // In case of success: Do several calibration if it is analysis process
                if (mAnalysisDataCalibrationParams && (mNumberOfCalibrationsDone < mNumberOfCalibrationsToDo)) {
                    mListPointsCenterSphericalTargetGroundTruth.clear();
                    mListPointsCenterSphericalTargetPointCloud.clear();
                    mListPointsCenterSphericalTargetImage.clear();
                    mListPixelsCenterSphericalTargetImage.clear();
                    mNumberOfCalibrationsDone++;
                    ROS_INFO_STREAM("CALIBRATION: " << mNumberOfCalibrationsDone << "number of calibrations done.");
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
    ros::init( argc, argv, "idealCalibrateSensorsNode" );

    // Create the class to go through the process
    CalibrateSensors calibrateSensors;

    //  Image window
    cv::namedWindow("view");

    // Define a multi thread
    ros::MultiThreadedSpinner spinner(2);

    // Do while loop. Needed for proper shut down
    while (!calibrateSensors.mRequestShutdown)
    {
        // Normal way to spin
        ros::spinOnce();
        usleep(100000);
    }

    // Image window
    cv::destroyWindow("view");

    // Stop ROS node
    ros::shutdown();

    return 0;
}