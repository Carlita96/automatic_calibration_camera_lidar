/*
*/

#include "checkMiscalibrationSensors.hpp"

CheckMiscalibrationSensors::CheckMiscalibrationSensors()
{
    datetime();    
    // Read the ros parameters for the tools
    initParameters(nodeHandle);
    initTools();

    // Publish static TF
    publishStaticTf(mTx, mTy, mTz, mQx, mQy, mQz, mQw, 
        "cam_rear", "velodyne", mStaticBroadcaster);
    ROS_INFO_STREAM("CHECK MISCALIBRATION: Creating static TF with: Tx: " << mTx << ", Ty: " << mTy 
        << " and Tz: " << mTz << ". Qx: " << mQx << ", Qy: " << mQy << ", Qz: " << mQz << " and Qw: " << mQw << ".");

    // Subscribe to images, point clouds and camera information
    mCameraInfoSubscriber = nodeHandle.subscribe<sensor_msgs::CameraInfo>( mCameraInfoTopicName, 1, &CheckMiscalibrationSensors::cameraInfoCallback, this );
    mImageSubscriber.subscribe(nodeHandle, mImageTopicName, 1);
    mPointCloudSubscriber.subscribe(nodeHandle, mPointcloudTopicName, 1);
    // Synchronize the data
    mpSync.reset(new Sync(MySyncPolicy(10), mImageSubscriber, mPointCloudSubscriber));

    // Publishers
    // Image transport
    image_transport::ImageTransport it(nodeHandle);

    // Initialize publishers
    mEdgesCloudProjectedImagePublisher = it.advertise("edges_cloud_projected_to_image", 1);
    mCloudProjectedImageDepthPublisher = it.advertise("cloud_projected_to_image_depth", 1);
    mCloudProjectedImageIntensityPublisher = it.advertise("cloud_projected_to_image_intensity", 1);
    mEdgeSmoothImagePublisher = it.advertise("edge_smooth_image", 1);
    mEdgeImagePublisher = it.advertise("edge_image", 1);
    mBothEdgesImagePublisher = it.advertise("edges_cloud_and_image", 1);
    mImageProjectedCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("image_projected_cloud", 1);  
    mEdgesCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("edges_cloud", 1);

    ROS_INFO_STREAM("Miscalibration detection node init OK");
}

CheckMiscalibrationSensors::~CheckMiscalibrationSensors()
{
    delete mGeneralProcessingTool;
    delete mClusterTool;
    delete mSmoothEdgeTool;
    delete mRangeImageTool;
}

void CheckMiscalibrationSensors::initParameters(ros::NodeHandle &nodeHandle) 
{
    /*
    For more detail, read main comment on header file.
    */
    // Consecutive miscalibrations
    nodeHandle.param<int>( "calibration/kSizeCloudPointCalibrationRate", mKSizeCloudPointCalibrationRate, 15 );
    nodeHandle.param<double>( "calibration/minCalibrationRate", mMinCalibrationRate, 15 );
    nodeHandle.param<int>( "calibration/consecutiveMiscalibrationsAllowed", mConsecutiveMiscalibrationsAllowed, 15 );
    // Listening to topics
    nodeHandle.param<std::string>( "calibration/pointcloudTopicName", mPointcloudTopicName, std::string( "/velodyne_points" ));
    nodeHandle.param<std::string>( "calibration/imageTopicName", mImageTopicName, std::string( "/camera_cam_rear/image_raw" ));
    nodeHandle.param<std::string>( "calibration/cameraInfoTopicName", mCameraInfoTopicName, std::string( "/camera_cam_rear/camera_info" ));
    // Process Point cloud
    // Cluster 
    nodeHandle.param<double>( "calibration/pointCloud/segment/maxDepthDiffSegment", mSegmentMaxDepthDiffSegment, 0.2 );
    // Edges
    nodeHandle.param<int>( "calibration/pointCloud/edges/minNumber", mMinNumberEdgesInCloud, 15 );
    // Process Image
    // Median blur 
    nodeHandle.param<int>( "calibration/image/medianBlur/kSize", mMedianBlurKSize, 5 );
    // Canny edges
    nodeHandle.param<int>( "calibration/image/cannyEdges/minThreshold", mCannyEdgesMinThreshold, 30 );
    nodeHandle.param<int>( "calibration/image/cannyEdges/maxThreshold", mCannyEdgesMaxThreshold, 120 );
    // Sobel
    nodeHandle.param<int>( "calibration/image/sobel/kSizeSobel", mCannyEdgesKSizeSobel, 5 );
    // Smoth edge
    nodeHandle.param<int>( "calibration/image/smoothEdge/kSizeEdgeCalculation", mSmoothEdgeKSizeEdgeCalculation, 2 );
    nodeHandle.param<int>( "calibration/image/smoothEdge/kSizeSmoothEdge", mSmoothEdgeKSizeEdgeSmooth, 5 );
    nodeHandle.param<double>( "calibration/image/smoothEdge/alpha", mSmoothEdgeAlpha, 0.333 );
    nodeHandle.param<double>( "calibration/image/smoothEdge/gamma", mSmoothEdgeGamma, 0.98 );
    // Calibration
    // Calibration parameter
    nodeHandle.param<double>( "Tx", mTx, 0);
    nodeHandle.param<double>( "Ty", mTy, 0);
    nodeHandle.param<double>( "Tz", mTz, 0);
    nodeHandle.param<double>( "Qx", mQx, 0);
    nodeHandle.param<double>( "Qy", mQy, 0);
    nodeHandle.param<double>( "Qz", mQz, 0);
    nodeHandle.param<double>( "Qw", mQw, 0);
}

void CheckMiscalibrationSensors::initTools()
{
    /*
    Method where the tools used are initialized.
    */
    mGeneralProcessingTool = new GeneralProcessingTool();
    mClusterTool = new ClusterTool();
    mSmoothEdgeTool = new SmoothEdgeTool(mSmoothEdgeKSizeEdgeCalculation, mSmoothEdgeKSizeEdgeSmooth, mSmoothEdgeAlpha, mSmoothEdgeGamma);
    mRangeImageTool = new RagenImageTool();
}

void CheckMiscalibrationSensors::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& pCameraInfoMsg)
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
    if ((pCameraInfoMsg->K[0] == 0) || (pCameraInfoMsg->K[4] == 0) || (pCameraInfoMsg->K[2] == 0) || (pCameraInfoMsg->K[5] == 0)) {
        ROS_ERROR_STREAM("CALIBRATION: Fx, Fy, Cx and Cy values received are 0. Waiting for not null values.");
        return;
    }
    intrinsicMatrixRCamera = (cv::Mat_<float>(3,3) << 
               pCameraInfoMsg->R[0],       pCameraInfoMsg->R[1],      pCameraInfoMsg->R[2],
               pCameraInfoMsg->R[3],       pCameraInfoMsg->R[4],      pCameraInfoMsg->R[5],
               pCameraInfoMsg->R[6],       pCameraInfoMsg->R[7],      pCameraInfoMsg->R[8]
               );

    // Save K as Eigen matrix too
    mIntrinsicMatrixKCameraEigen << pCameraInfoMsg->K[0], 0, pCameraInfoMsg->K[2], 0, pCameraInfoMsg->K[4], pCameraInfoMsg->K[5], 0, 0, 1;

    // Subscribe to the image and point cloud data now
    mpSync->registerCallback(boost::bind(&CheckMiscalibrationSensors::dataCallback, this, _1, _2));
    // Unsubscribing to this topic as info has been saved
    mCameraInfoSubscriber.shutdown();


    ROS_INFO_STREAM("CALIBRATION: Camera information received. K: " << intrinsicMatrixKCamera
        << " and R: " << intrinsicMatrixRCamera << ". Unsubscribing from topic.");
    return;
}

void CheckMiscalibrationSensors::dataCallback(const sensor_msgs::ImageConstPtr& pImageMsg, const sensor_msgs::PointCloud2::ConstPtr& pPointcloudMsg)
{
    /*
    Callback for the images and point clouds:
    * Input parameters:
        - pImageMsg: It is the pointer to the image received. 
        - pointcloudMsg: It is the pointer to the cloud received. 
    */

    int statusPointcloud = 0;
    int statusImage = 0;
    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Define clouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudEdges = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudEdgesInPictureMargin = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );
    // Define images
    cv::Mat grayImage;
    cv::Mat cannyEdgesImage;
    cv::Mat sobelImage;
    cv::Mat smoothEdgeImage;

    // POINT CLOUD
    // Finding the center of the spherical target in the pointcloud
    // Transform the message to a pcl::PCLPointCloud2 and then to PCL point cloud  
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );  
    pcl::fromROSMsg(*pPointcloudMsg, *cloud);

    ROS_INFO_STREAM("POINT CLOUD PROCESS: Cloud of size " << cloud->points.size() << " read. Starting process.");
    // STEP 1: Segment cloud by depth of points
    statusPointcloud = mClusterTool->segment(cloud, mSegmentMaxDepthDiffSegment, 1000); 
    if (statusPointcloud == 0) {
        // STEP 2: Get edges of segments
        statusPointcloud = mClusterTool->getEdgesSegmentsCloud(cloudEdges);
        ROS_INFO_STREAM("POINT CLOUD PROCESS: Number of edges of segments gotten is " << cloudEdges->points.size()
                << ".");
        if (statusPointcloud == 0) {
            // STEP 3: Get edges that are in camera side
            statusPointcloud = getPointsInCloudWithinImageMargin(cloudEdges, Eigen::Vector3d(mTx, mTy, mTz), 
                    Eigen::Vector4d(mQx, mQy, mQz, mQw), mIntrinsicMatrixKCameraEigen, cloudEdgesInPictureMargin);
            ROS_INFO_STREAM("POINT CLOUD PROCESS: " << cloudEdgesInPictureMargin->points.size()
                    << " number of these edge points are within image margin.");

            if (cloudEdgesInPictureMargin->points.size() < mMinNumberEdgesInCloud) {
                ROS_ERROR_STREAM("POINT CLOUD PROCESS: Only " << cloudEdgesInPictureMargin->points.size()
                        << " number of edge points in cloud found. At least " << mMinNumberEdgesInCloud << " are needed.");
                return;
            }
        }
    }
    // mRangeImageTool->getRangeImage(cloudEdgesInPictureMargin);


    if (statusPointcloud != 0) {
        ROS_ERROR_STREAM("POINT CLOUD PROCES: Point cloud process failed.");
        return;
    }

    // IMAGE
    // Finding the center of the spherical target in the image
    ROS_INFO_STREAM("IMAGE PROCESS: Image read. Starting process.");
    cv_bridge::CvImageConstPtr image = cv_bridge::toCvShare(pImageMsg, "bgr8");

    // STEP 1: Get gray image
    mGeneralProcessingTool->setInputImage(image->image);
    statusImage = mGeneralProcessingTool->convertToGrayImage();
    grayImage = mGeneralProcessingTool->getGrayImage();
    if (statusImage == 0) {
        if (statusImage == 0) {
            // STEP 2: Smooth edges
            statusImage = mSmoothEdgeTool->getSmoothEdge(grayImage, smoothEdgeImage);
        }
    }

    if (statusImage != 0) {
        ROS_ERROR_STREAM("IMAGE PROCES: Image process failed.");
        return;
    }

    // PUBLISH INFORMATION
    // Project Point cloud to image and show in CV window
    projectPointCloudToImage(cloud, image->image, mCloudProjectedImageDepthPublisher, 
            mCloudProjectedImageIntensityPublisher, mImageProjectedCloudPublisher, mIntrinsicMatrixKCameraEigen,
            Eigen::Vector3d(mTx, mTy, mTz), Eigen::Vector4d(mQx, mQy, mQz, mQw));
    // Project Point cloud to image in exact color
    projectPointCloudToImageInColor(cloudEdgesInPictureMargin, image->image, 255, 0, 0, mEdgesCloudProjectedImagePublisher, 
            mIntrinsicMatrixKCameraEigen, Eigen::Vector3d(mTx, mTy, mTz), Eigen::Vector4d(mQx, mQy, mQz, mQw));
    publishPointCloud(cloudEdgesInPictureMargin, mEdgesCloudPublisher, "velodyne");
    publishGrayScaleImage(cannyEdgesImage, mEdgeImagePublisher);
    publishImage(smoothEdgeImage, mEdgeSmoothImagePublisher);
    // Project edge of cloud in edge image
    projectPointCloudToImageInColor(cloudEdgesInPictureMargin, smoothEdgeImage, 0, 255, 0, mBothEdgesImagePublisher,
            mIntrinsicMatrixKCameraEigen, Eigen::Vector3d(mTx, mTy, mTz), Eigen::Vector4d(mQx, mQy, mQz, mQw));

    // Check calibration
    getCalibrationRate(cloudEdgesInPictureMargin, smoothEdgeImage, Eigen::Vector3d(mTx, mTy, mTz), Eigen::Vector4d(mQx, mQy, mQz, mQw));

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_INFO_STREAM("MISCALIBRATION DETECTION: Checking miscalibration between pointcloud with timestamp: " << pPointcloudMsg->header.stamp
        << " and image with timestamp: " << pImageMsg->header.stamp << ". It took " << duration.count() << " seconds.");

    return;
}


int CheckMiscalibrationSensors::getCalibrationRate(const pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, const cv::Mat smoothEdgeImage,
        const Eigen::Vector3d translationVector, const Eigen::Vector4d quaternionsVector)
{
    /*
    Method that gets the points inside a point cloud that are within the margins of the image.
    * Input parameters:
        - pCloud: cloud which points will go throw the filter.
        - smoothEdgeImage: image with smooth edges.
        - translationVector and quaternionsVector: vectors defining the calibration parameters between sensors.
    */
    // Define parameters
    Eigen::Vector3d lidarPointVector, cameraPointVector;
    Eigen::Matrix3d rotationMatrix;
    double u, v; 
    double intensityRateSum = 0;

    // Get points in image and save depth values
    for(std::size_t i=0; i<pCloud->points.size(); i++){
        // Not consider NaN values of points
        if (isnan(pCloud->points[i].x) || isnan(pCloud->points[i].y) || isnan(pCloud->points[i].z) ) {
            continue;
        }
        // Restart max
        int maxIntensityPixel = 0;

        // Go through all points in cloud
        lidarPointVector = Eigen::Vector3d(pCloud->points[i].x, pCloud->points[i].y, pCloud->points[i].z);

        // Find the point from camera frame
        rotationMatrix = getRotationMatrixFromQuaternions(quaternionsVector);    
        cameraPointVector = rotationMatrix * lidarPointVector + translationVector;

        // Project point into image plane
        u = mIntrinsicMatrixKCameraEigen(0, 0) * cameraPointVector[0] / cameraPointVector[2] + mIntrinsicMatrixKCameraEigen(0, 2);
        v = mIntrinsicMatrixKCameraEigen(1, 1) * cameraPointVector[1] / cameraPointVector[2] + mIntrinsicMatrixKCameraEigen(1, 2);

        // Read intensity of the point in image
        for (int i = -mKSizeCloudPointCalibrationRate/2; i <= mKSizeCloudPointCalibrationRate/2; ++i) {
            for (int j = -mKSizeCloudPointCalibrationRate/2; j <= mKSizeCloudPointCalibrationRate/2; ++j) {
                // Check that pixels are within image margin
                if ((u+j > 0) && (u+j < smoothEdgeImage.size().width) && 
                        (v+i > 0) && (v+i < smoothEdgeImage.size().height)) {
                    // Get max difference between pixels
                    int intensityPixel = (int)smoothEdgeImage.at<cv::Vec3b>(v+i, u+j)[2];
                    maxIntensityPixel = std::max(intensityPixel, maxIntensityPixel);
                }
            }
        }
        intensityRateSum += maxIntensityPixel / 255.0;
    }

    double calibrationRate = intensityRateSum / pCloud->points.size();

    ROS_INFO_STREAM("MISCALIBRATION DETECTION: Calibration rate calculated is: " << calibrationRate << ".");

    if (calibrationRate < mMinCalibrationRate) {
        mMiscalibrationsConsecutive++;
        if (mMiscalibrationsConsecutive > mConsecutiveMiscalibrationsAllowed) {
            ROS_ERROR_STREAM("MISCALIBRATION DETECTION: Sensors seem to be miscalibration!!!");
        }
    } else {
        mMiscalibrationsConsecutive = 0;
    }

    return 0;
}


void CheckMiscalibrationSensors::datetime()
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
    ros::init( argc, argv, "checkMiscalibrationSensorsNode" );

    // Create the class to go through the process
    CheckMiscalibrationSensors checkMiscalibrationSensors;

    // Define a multi thread
    ros::MultiThreadedSpinner spinner(4);

    // Do while loop. Needed for proper shut down
    while (!checkMiscalibrationSensors.mRequestShutdown)
    {
        // Normal way to spin
        ros::spinOnce();
        usleep(100000);
    }

    // Stop ROS node
    ros::shutdown();

    return 0;
}