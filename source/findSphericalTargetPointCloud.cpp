/*
This class can be used to find the center of a spherical target from a point cloud.
Then, call getCenterSphericalTarget to get the center in 3D.
*/

#include "findSphericalTargetPointCloud.hpp"

FindSphericalTargetPointCloud::FindSphericalTargetPointCloud(const ros::NodeHandle &rNodeHandle)
{
    // Read the ros parameters that define the parameters for the problem
    mNodeHandle = rNodeHandle;
    initParameters(mNodeHandle);
    initTools();

    // Initialize publishers
    mSphericalTargetCloudPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("sphere_cloud", 1);
    mClustersCloudPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("clusters_cloud", 1);
    mSegmentsCloudPublisher = mNodeHandle.advertise<sensor_msgs::PointCloud2>("segments_cloud", 1);
    mSphericalTargetMarkerPublisher = mNodeHandle.advertise<visualization_msgs::Marker>("sphere_marker_point_cloud", 1);

    ROS_INFO_STREAM("Find Spherical Target Point Cloud class init OK");
}

FindSphericalTargetPointCloud::~FindSphericalTargetPointCloud()
{
    delete mNormalEstimationTool;
    delete mSphereSegmentationTool;
    delete mClusterTool;
}

void FindSphericalTargetPointCloud::initParameters(ros::NodeHandle &nodeHandle) 
{
    /*
    Method that initialize the parameters declared in ROS. These are:
        - sphericalTarget/radius: radius in meters of the spherical target used to calibrate the sensors.
        - pointCloud: parameters used for finding the spherical target in the pointcloud data.
    For more detail, read main comment on header file.
    */
    // Spherical target information
    nodeHandle.param<double>( "calibration/sphericalTarget/radius", mSphericalTargetRadius, 0.5 );
    // Cluster 
    nodeHandle.param<double>( "calibration/pointCloud/cluster/maxDepthDiffSegment", mClusterMaxDepthDiffSegment, 0.2 );
    nodeHandle.param<double>( "calibration/pointCloud/cluster/maxDistanceCenterSegment", mClusterMaxDistanceCenterSegment, 1.0 );
    nodeHandle.param<double>( "calibration/pointCloud/cluster/thresholdSegmentPerimeter", mClysterThresholdSegmentPerimeter, 0.2 );
    // Normal Estimation 
    nodeHandle.param<int>( "calibration/pointCloud/normalEstimation/kSearch", mNormalEstimationKSearch, 50 );
    // Sphere Segmentation
    nodeHandle.param<int>( "calibration/pointCloud/sphereSegmentation/minNumberPoints", mSphereSegmentationMinNumberPoints, 50 );
    nodeHandle.param<double>( "calibration/pointCloud/sphereSegmentation/normalDistanceWeight", mSphereSegmentationNormalDistanceWeight, 0.1 );
    nodeHandle.param<double>( "calibration/pointCloud/sphereSegmentation/distanceThreshold", mSphereSegmentationDistanceThreshold, 0.03 );
    nodeHandle.param<int>( "calibration/pointCloud/sphereSegmentation/maxIterations", mSphereSegmentationMaxIterations, 100 );
    nodeHandle.param<double>( "calibration/pointCloud/sphereSegmentation/thresholdRadius", mSphereSegmentationThresholdRadius, 0.01 );
    // Position Filter
    nodeHandle.param<double>( "calibration/pointCloud/positionFilter/minValue", mPositionFilterMinValue, 0.5 );
    nodeHandle.param<double>( "calibration/pointCloud/positionFilter/maxValue", mPositionFilterMaxValue, 15.0 );
}

void FindSphericalTargetPointCloud::initTools()
{
    /*
    Method where the tools used are initialized.
    */
    mClusterTool = new ClusterTool();
    mNormalEstimationTool = new NormalEstimationTool(mNormalEstimationKSearch);
    mSphereSegmentationTool = new SphereSegmentationTool(mSphereSegmentationNormalDistanceWeight, 
        mSphereSegmentationDistanceThreshold, mSphereSegmentationMaxIterations, 
        mSphericalTargetRadius-mSphereSegmentationThresholdRadius, mSphericalTargetRadius+mSphereSegmentationThresholdRadius);
}

int FindSphericalTargetPointCloud::getCenterSphericalTarget(const pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, 
        Eigen::Vector3d& rPointCenterSpherePointCloud) 
{
    /*
    Method that finds the spherical target in the point cloud and obtains its center.
    * Input parameters:
        - pCloud: cloud that will be processed and where the spherical target will be searched.
    * Output parameters:
        - rPointCenterSphereImage: output that contains (x, y, z) of center of spherical target from the sensor 
                calculated from the Point cloud data.
    */
    int status = 0;
    int statusCluster = 0;
    // Time the process 
    auto start = std::chrono::system_clock::now();
    ROS_INFO_STREAM("FINDING SPHERICAL TARGET: Running process of finding spheric target center in point cloud data.");

    // Define output
    Eigen::Vector3d pointCenterSpherePointCloud = Eigen::Vector3d(0,0,0);
    double radiusSpherePointCloud = 0;

    // Create pointclouds
    pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );
    pcl::PointCloud<pcl::Normal>::Ptr clusterCloudNormals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr sphereCloud = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );
    // Create list of clusters
    std::vector<pcl::PointCloud<pcl::PointXYZI>> listClusterClouds;
    std::vector<pcl::PointCloud<pcl::PointXYZI>> listSegmentClouds;
    pcl::ModelCoefficients::Ptr coefficientsSphere = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
    double maxRateInliersOutliersInListClusters = 0;

    // STEP 1: Segment cloud by depth of points
    status = mClusterTool->segment(pCloud, mClusterMaxDepthDiffSegment, M_PI * mSphericalTargetRadius + mClysterThresholdSegmentPerimeter); 
    // Max length segment: Perimeter/2 = 2/2 * PI * r = PI * r
    if (status == 0) {
        // STEP 2: Cluster the segments close to each other together
        status = mClusterTool->cluster(mClusterMaxDistanceCenterSegment);
        if (status == 0) {
            // STEP 3: Fit a sphere in each cloud found in cluster
            listSegmentClouds = mClusterTool->getListSegmentsClouds();
            listClusterClouds = mClusterTool->getListClustersClouds();
            // Create a cloud with clusters in different colors and publish
            publishListsClouds(listSegmentClouds, mSegmentsCloudPublisher);
            publishListsClouds(listClusterClouds, mClustersCloudPublisher);
            for (auto it2 = std::begin(listClusterClouds); it2 != std::end(listClusterClouds); ++it2) {
                // STEP 3.1: Calculate normals
                *clusterCloud = *it2;
                if (clusterCloud->points.size() >= mSphereSegmentationMinNumberPoints) {
                    statusCluster = mNormalEstimationTool->getNormalsCloud(clusterCloud, clusterCloudNormals);
                    if (statusCluster == 0) {
                        // STEP 3.2: Fit a sphere to each cluster
                        statusCluster = mSphereSegmentationTool->segmentCloud(clusterCloud, clusterCloudNormals);
                        if (statusCluster == 0) {
                            // STEP 3.3: Save the sphere only if it has a better inlier/outlier rate than before fittings
                            if (mSphereSegmentationTool->getRateInliersOutliers() > maxRateInliersOutliersInListClusters && 
                                    isPositionOfSphereCorrect(mSphereSegmentationTool->getSphereCoefficients())){
                                *coefficientsSphere = mSphereSegmentationTool->getSphereCoefficients();
                                *sphereCloud = mSphereSegmentationTool->getSegmentedCloud(true);
                                maxRateInliersOutliersInListClusters = mSphereSegmentationTool->getRateInliersOutliers();
                            }
                        }
                    }
                }
            }
        }
    }
    // Check that a sphere has been saved or set status = 1
    if (maxRateInliersOutliersInListClusters == 0) {
        ROS_ERROR_STREAM("FINDING SPHERICAL TARGET: When fitting spheres in list of clusters found. No cluster was suitable.");
        return 1;
    }

    if (status != 0) {
        ROS_ERROR_STREAM("FINDING SPHERICAL TARGET: Process for finding the spheric target in the point cloud data failed.");
        return 1;
    }

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    // Save spherical target parameters found in the point cloud data
    pointCenterSpherePointCloud = Eigen::Vector3d(coefficientsSphere->values[0], coefficientsSphere->values[1], coefficientsSphere->values[2]);
    radiusSpherePointCloud = coefficientsSphere->values[3];
    // Publish to Rviz
    publishPointCloud(sphereCloud, mSphericalTargetCloudPublisher, "velodyne");
    publishSphereMarker(coefficientsSphere->values[0], coefficientsSphere->values[1],
        coefficientsSphere->values[2], 2*coefficientsSphere->values[3], mSphericalTargetMarkerPublisher, 
        "velodyne", 0.0f, 1.0f, 1.0f);

    ROS_INFO_STREAM("FINDING SPHERICAL TARGET: Finding the spheric target in point cloud data was successful. It took " << duration.count()
        << " seconds. Sphere center is at X: " << pointCenterSpherePointCloud[0] << ", Y: " << pointCenterSpherePointCloud[1] 
        << " and Z: " << pointCenterSpherePointCloud[2] << " meters. Radius of sphere is: " << radiusSpherePointCloud << " meters.");
        
    // Save in output
    rPointCenterSpherePointCloud = pointCenterSpherePointCloud;
    return status;
}

bool FindSphericalTargetPointCloud::isPositionOfSphereCorrect(const pcl::ModelCoefficients sphereCoefficients)
{
    /*
    Method that returns a boolean specifying if the sphere received is within the desired bounds.
    * Input parameters:
        - sphereCoefficients: coefficients of the sphere model, with order: centerX, centerY, centerZ and radius.
    * Output paramteres:
        - true if sphere is within bounds, false otherwise.
    */
    if ((sphereCoefficients.values[0] > -mPositionFilterMinValue && sphereCoefficients.values[0] < mPositionFilterMinValue) && 
        (sphereCoefficients.values[1] > -mPositionFilterMinValue && sphereCoefficients.values[1] < mPositionFilterMinValue) && 
        (sphereCoefficients.values[2] > -mPositionFilterMinValue && sphereCoefficients.values[2] < mPositionFilterMinValue))
    {
        ROS_WARN_STREAM("FINDING SPHERICAL TARGET: Spherical target fit is at position X: " << sphereCoefficients.values[0]
                << ", Y: " << sphereCoefficients.values[1] << " and Z: " << sphereCoefficients.values[2] 
                << ". Not considered a possible position of the spherical target.");
        return false;
    }
    if ((sphereCoefficients.values[0] < -mPositionFilterMaxValue || sphereCoefficients.values[0] > mPositionFilterMaxValue) || 
        (sphereCoefficients.values[1] < -mPositionFilterMaxValue || sphereCoefficients.values[1] > mPositionFilterMaxValue) || 
        (sphereCoefficients.values[2] < -mPositionFilterMaxValue || sphereCoefficients.values[2] > mPositionFilterMaxValue))
    {
        ROS_WARN_STREAM("FINDING SPHERICAL TARGET: Spherical target fit is at position X: " << sphereCoefficients.values[0]
                << ", Y: " << sphereCoefficients.values[1] << " and Z: " << sphereCoefficients.values[2] 
                << ". Not considered a possible position of the spherical target.");
        return false;
    }
    return true;
}

void FindSphericalTargetPointCloud::publishListsClouds(const std::vector<pcl::PointCloud<pcl::PointXYZI>> listClusterClouds,
        ros::Publisher publisher)
{
    /*
    Method that publishes the clusters of point clouds in different color.
    * Input parameters:
        - listClusterClouds: list with the point cloud of each cluster.
        - publisher: publisher where to publish the clouds.
    */
    
    // Initialize clouds and parameters
    pcl::PointCloud<pcl::PointXYZI> clusterCloud;
    pcl::PointXYZRGB colorPointDummy;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterCloudColor = pcl::PointCloud<pcl::PointXYZRGB>::Ptr( new pcl::PointCloud<pcl::PointXYZRGB>() );
    int r, g, b;

    // Go through each cluster
    for (auto it2 = std::begin(listClusterClouds); it2 != std::end(listClusterClouds); ++it2) {
        // Get cloud and transform to colored
        clusterCloud = *it2;

        // Get random color for cluster
        r = rand() % 255 + 1;
        g = rand() % 255 + 1;
        b = rand() % 255 + 1;

        // Add color to all points in cloud
        for (size_t i = 0; i < clusterCloud.points.size(); i++) {
            colorPointDummy.x = clusterCloud.points[i].x;
            colorPointDummy.y = clusterCloud.points[i].y;
            colorPointDummy.z = clusterCloud.points[i].z;
            colorPointDummy.r = r;
            colorPointDummy.g = g;
            colorPointDummy.b = b;
            clusterCloudColor->push_back(colorPointDummy);
        }
    }

    publishColorPointCloud(clusterCloudColor, publisher, "velodyne"); 
}