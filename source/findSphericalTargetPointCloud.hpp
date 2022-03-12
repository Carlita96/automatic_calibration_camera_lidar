/*
This class can be used to find the center a spherical target in a point cloud.
    * ROS Parameters that can be used:
        * sphericalTarget/radius: radius in meters of the spherical target used to calibrate the sensors.
        * pointCloud: parameters used for finding the spherical target in the pointcloud data.
            - cluster: tool used to cluster the points together in different pointclouds depending on depth.
                - maxDepthDiffSegment: maximum depth difference between points in same row for them to be considered from the same segment.
                - maxDistanceCenterSegment: maximum absolute distance between the center of the segments for them to be considered from the same cluster.
            - normalEstimation: tool used to calculate normals on the clouds and recognize surfaces.
                - kSearch: number of close neighbours points usef to calculate the normal.
            - sphereSegmentation: tool used to find the spherical target on the point cloud.
                - normalDistanceWeight: relative weight (0-1) to give to the angular distance (0-pi/2) between point normals and the sphere normal. 
                - distanceThreshold: how close a point must be to the model in order to be considered an inlier.
                - maxIterations: maximum number of iterations trying to fit the sphere. 
                - thresholdRadius: look for sphere with radius [sphericalTarget/radius-thresholdRadius, sphericalTarget/radius+thresholdRadius].
*/

#ifndef CALIBRATION_SOURCE_FINDSPHERICALTARGETPOINTCLOUD_H_
#define CALIBRATION_SOURCE_FINDSPHERICALTARGETPOINTCLOUD_H_

// Import necessary libraries
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
// Eigen libraries
#include <Eigen/Eigen>
// ROS Tools
#include "../includeTools/rosTools/publishTools.hpp"
// Point cloud Tools
#include "pointCloudTools/cropboxTool.hpp"
#include "pointCloudTools/normalEstimationTool.hpp"
#include "pointCloudTools/planeSegmentationTool.hpp"
#include "pointCloudTools/sphereSegmentationTool.hpp"
#include "pointCloudTools/clusterTool.hpp"
#include "rosTools/publishTools.hpp"

class FindSphericalTargetPointCloud
{
    public:
        FindSphericalTargetPointCloud(const ros::NodeHandle &rNodeHandle);

        ~FindSphericalTargetPointCloud();

        int getCenterSphericalTarget(const pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, 
                Eigen::Vector3d& rPointCenterSpherePointCloud);

    private:
        // Functions
        void initParameters(ros::NodeHandle &nodeHangle);

        void initTools();

        bool isPositionOfSphereCorrect(const pcl::ModelCoefficients sphereCoefficients);

        void publishListsClouds(const std::vector<pcl::PointCloud<pcl::PointXYZI>> listClusterClouds, ros::Publisher publisher);

        // Attributes
        // Create ros objects necessary for the class
        ros::NodeHandle mNodeHandle;

        // Publishers
        ros::Publisher mSphericalTargetCloudPublisher;
        ros::Publisher mClustersCloudPublisher;
        ros::Publisher mSegmentsCloudPublisher;
        ros::Publisher mSphericalTargetMarkerPublisher;

        // Create attributes necessary for saving ROS parameters
        // Spherical target
        double mSphericalTargetRadius;
        // Process Cluster
        // Cluster
        double mClusterMaxDepthDiffSegment;
        double mClusterMaxDistanceCenterSegment;
        double mClysterThresholdSegmentPerimeter;
        // Normal Estimation
        int mNormalEstimationKSearch;
        // Sphere Segmentation
        int mSphereSegmentationMinNumberPoints;
        double mSphereSegmentationNormalDistanceWeight;
        double mSphereSegmentationDistanceThreshold;
        int mSphereSegmentationMaxIterations;
        double mSphereSegmentationThresholdRadius;
        // Position Filter
        double mPositionFilterMinValue;
        double mPositionFilterMaxValue;

        // Tools used
        NormalEstimationTool *mNormalEstimationTool;
        SphereSegmentationTool *mSphereSegmentationTool;
        ClusterTool *mClusterTool;
}; 

#endif // CALIBRATION_SOURCE_FINDSPHERICALTARGETPOINTCLOUD_H_ 