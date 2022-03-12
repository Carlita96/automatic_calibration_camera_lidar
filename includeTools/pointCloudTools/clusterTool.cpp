/*
This class can be used to segment an organized cloud into different parts with similar depth. Then it can
cluster these segments together if they are close to each other.

This script cannot be run directly and needs to be used as an import.
*/

#include "clusterTool.hpp"

ClusterTool::ClusterTool()
{ 
}

ClusterTool::~ClusterTool(){
}

int ClusterTool::segment(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud, 
    const double maxDepthDiffSegment, const double maxLengthSegment) 
{
    /*
    Method that finds segments in the cloud that have similar depth and are smaller than a size.
    * Input parameters: 
        - pInputCloud: cloud that will be segmented by depth difference between points. 
            It needs to be organized.
        - maxDepthDiffSegment: maximum depth difference between points for them to be considered
            from the same segment.
        - maxLengthSegment: maximum length segment to be saved.
    */
    ROS_DEBUG_STREAM("CLUSTER TOOL: Segmenting organized cloud into segments with max depth " 
        << " difference between points of the same segment of: " << maxDepthDiffSegment 
        << " and maximum length of segments: " << maxLengthSegment << ".");
    // Check input cloud
    if (pInputCloud->points.size() == 0) {
        ROS_ERROR_STREAM("CLUSTER TOOL: Input cloud has size " << pInputCloud->points.size() << ".");
        return 1;
    }
    if (!pInputCloud->isOrganized()) {
        ROS_ERROR_STREAM("CLUSTER TOOL: Input cloud needs to be organized cloud.");
        return 1;
    }
    // Time the process 
    auto start = std::chrono::system_clock::now();

    // Go through rows in point cloud and divide in segments with similar depth
    // Save first segment to check if first and last segment can be saved together
    // Save only segments that have less than maximum length
    // Information of last point read in row
    pcl::PointXYZI lastPoint;
    // Segment information
    double distanceBetweenPoints;
    double lengthSegment = 0;
    pcl::PointCloud<pcl::PointXYZI> segmentCloud;
    // First segment could be connected to last. Information to keep about first segment
    pcl::PointXYZI firstSegmentRowPoint;
    bool isFirstSegmentRow = true;
    double lengthFirstSegmentRow = 0;
    pcl::PointCloud<pcl::PointXYZI> firstSegmentRowCloud;
    // List where segments will be saved
    std::vector<pcl::PointCloud<pcl::PointXYZI>> listSegmentClouds;

    // Go through rows and columns
    for(std::size_t i=0; i<pInputCloud->width; i++){
        // Clean parameters about last row
        lastPoint.x = 1000;
        lastPoint.y = 1000;
        lastPoint.z = 1000;
        firstSegmentRowPoint.x = 1000;
        firstSegmentRowPoint.y = 1000;
        firstSegmentRowPoint.z = 1000;
        lengthSegment = 0;
        lengthFirstSegmentRow = 0;
        isFirstSegmentRow = true;
        for(std::size_t j=0; j<pInputCloud->height; j++){
            // Not consider NaN values of points
            if (isnan(pInputCloud->at(i,j).x) || isnan(pInputCloud->at(i,j).y) || isnan(pInputCloud->at(i,j).z) ) {
                continue;
            }
            // First point saves parameters and defines last point
            if (firstSegmentRowPoint.x == 1000) {
                firstSegmentRowPoint = pInputCloud->at(i,j);
                lastPoint = pInputCloud->at(i,j);
                continue;
            }

            // Check if difference of depth is small enough to consider the point in the same segment as the point before
            distanceBetweenPoints = getAbsoluteDistanceBetweenPoints(lastPoint.x, lastPoint.y, lastPoint.z, pInputCloud->at(i,j).x, 
                        pInputCloud->at(i,j).y, pInputCloud->at(i,j).z);
            if ((distanceBetweenPoints > - maxDepthDiffSegment) && (distanceBetweenPoints < maxDepthDiffSegment)) {
                lengthSegment += getAbsoluteDistanceBetweenPoints(lastPoint.x, lastPoint.y, lastPoint.z, pInputCloud->at(i,j).x, 
                        pInputCloud->at(i,j).y, pInputCloud->at(i,j).z);
                segmentCloud.push_back(pInputCloud->at(i,j));
            } else {
                // If distance is too large, close the segment and add it to the list

                // If it is the first segment of the row, do not add to list and wait for end of row
                if (isFirstSegmentRow) {
                    firstSegmentRowCloud = segmentCloud;
                    isFirstSegmentRow = false;
                    lengthFirstSegmentRow = lengthSegment;
                } else if (lengthSegment < maxLengthSegment && lengthSegment > 0) {
                    // Add to the list if it has a good length
                    listSegmentClouds.push_back(segmentCloud);
                }

                // Clean the segment information
                segmentCloud.clear();
                lengthSegment = 0;
            }

            // Only in the last point of row. Check if first and last segments can merge together
            if (j == pInputCloud->height-1) {
                // Add them together if they have right length and are similar in depth
                distanceBetweenPoints = getAbsoluteDistanceBetweenPoints(lastPoint.x, lastPoint.y, lastPoint.z, pInputCloud->at(i,j).x, 
                        pInputCloud->at(i,j).y, pInputCloud->at(i,j).z);
                if ((distanceBetweenPoints > - maxDepthDiffSegment) && (distanceBetweenPoints < maxDepthDiffSegment)) {
                    lengthSegment += lengthFirstSegmentRow + getAbsoluteDistanceBetweenPoints(firstSegmentRowPoint.x, firstSegmentRowPoint.y,
                            firstSegmentRowPoint.z, pInputCloud->at(i,j).x, pInputCloud->at(i,j).y, pInputCloud->at(i,j).z);
                    if (lengthSegment < maxLengthSegment && lengthSegment > 0) {
                        firstSegmentRowCloud += segmentCloud;
                        listSegmentClouds.push_back(firstSegmentRowCloud);
                    }
                } else { // Add them separately if they have right size independently
                    if (lengthFirstSegmentRow < maxLengthSegment && lengthFirstSegmentRow > 0) {
                        listSegmentClouds.push_back(firstSegmentRowCloud);
                    }
                    if (lengthSegment < maxLengthSegment && lengthSegment > 0) {
                        listSegmentClouds.push_back(segmentCloud);
                    }
                }
            }

            // Set last point
            lastPoint = pInputCloud->at(i,j);
        }
    }

    // Check that segments have been found
    if (listSegmentClouds.size() == 0) {
        ROS_WARN_STREAM("CROPBOX TOOL: No segments were found.");
        return 1;
    }
    mListSegmentClouds = listSegmentClouds;

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("CLUSTER TOOL: Segmentation done in " << duration.count() << " seconds. "
        << listSegmentClouds.size() << " number of segments found.");
    return 0;
}

int ClusterTool::cluster(const double maxDistanceBetweenCentersSegmentForCluster) 
{
    /*
    Method that clusters together the segments that are close to each other.
    * Input parameters: 
        - maxDistanceBetweenCentersSegmentForCluster: maximum absolute distance between the center 
            of the segments for them to be considered from the same cluster.
    */
    ROS_DEBUG_STREAM("CLUSTER TOOL: Clustering the segments found together with a maximum" 
        << " absolute distance between the centers of the segments for them to be considered"
        << " of the same cluster: " << maxDistanceBetweenCentersSegmentForCluster << ".");
    // Check segments are saved
    if (mListSegmentClouds.size() == 0) {
        ROS_ERROR_STREAM("CLUSTER TOOL: No segments are saved in class. Run segment successfully.");
        return 1;
    }
    // Time the process 
    auto start = std::chrono::system_clock::now();

    //  First, get center of each segment averaging X, Y and Z
    pcl::PointXYZI center;
    double averageX, averageY, averageZ;
    // List where centers will be saved
    std::vector<pcl::PointXYZI> listSegmentCenterClouds;

    for (auto it = std::begin(mListSegmentClouds); it != std::end(mListSegmentClouds); ++it) {
        averageX = 0;
        averageY = 0;
        averageZ = 0;
        for (size_t i = 0; i < it->points.size(); i++) {
            averageX += it->points[i].x;
            averageY += it->points[i].y;
            averageZ += it->points[i].z;
        }
        center.x = averageX / it->points.size();
        center.y = averageY / it->points.size();
        center.z = averageZ / it->points.size();
        listSegmentCenterClouds.push_back(center);
    }

    // Second, get which center of which cloud is close to which center of which cloud and merge them
    pcl::PointCloud<pcl::PointXYZI> clusterCloud;
    int i, j;
    // Keep a list of index of clouds added to the list of clusters to avoid duplicates
    std::vector<int> listItAddedToCluster;
    // List of clusters found
    std::vector<pcl::PointCloud<pcl::PointXYZI>> listClusterClouds;

    for (auto it1 = std::begin(listSegmentCenterClouds); it1 != std::end(listSegmentCenterClouds); ++it1) {
        // Check if segment is already added to cluster, continue
        i = it1 - listSegmentCenterClouds.begin();
        if (std::count(listItAddedToCluster.begin(), listItAddedToCluster.end(), i)) {
            continue;
        }

        // Add cloud to cluster and list of clouds added
        clusterCloud += mListSegmentClouds[i];
        listItAddedToCluster.push_back(i);
        // Find clouds close to this cloud and add to cluster
        for (auto it2 = std::begin(listSegmentCenterClouds); it2 != std::end(listSegmentCenterClouds); ++it2) {
            // Check if segment is already added to cluster, continue
            j = it2 - listSegmentCenterClouds.begin();
            if (i == j || std::count(listItAddedToCluster.begin(), listItAddedToCluster.end(), j)) {
                continue;
            }

            // Get distance between the centers. If less than specified, add to cluster
            if (getAbsoluteDistanceBetweenPoints(it1->x, it1->y, it1->z, it2->x, it2->y, it2->z) < maxDistanceBetweenCentersSegmentForCluster) {
                clusterCloud += mListSegmentClouds[j];
                listItAddedToCluster.push_back(j);
            }
        }
        // Add cluster cloud to list
        listClusterClouds.push_back(clusterCloud);
        // Clean cluster information
        clusterCloud.clear();
    }

    // Check that segments have been found
    if (listClusterClouds.size() == 0) {
        ROS_WARN_STREAM("CROPBOX TOOL: No clusters were made.");
        return 1;
    }
    mListClusterClouds = listClusterClouds;
    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("CLUSTER TOOL: Clustering of segments done in " << duration.count() << " seconds. "
        << listClusterClouds.size() << " number of clusters made.");
    return 0;
}

int ClusterTool::getEdgesSegmentsCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr rOutputCloud)
{
    /*
    It retrieves the edges of the segments. Returns one single cloud with all edges.
    * Output: 
        - rOutputCloud: Cloud containing the edges of the segments. 
    */

    // Check that edges segments have been found
    if (mListSegmentClouds.size() == 0) {
        ROS_ERROR_STREAM("CLUSTER PROCESS: Number of segments is 0. " << mListSegmentClouds.size()
                << ". A not-null number of segments is needed.");
        return 1;
    }

    // Define point clouds
    int i;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::PointCloud<pcl::PointXYZI> outputCloud;

    // Go through list and get first and last in each segment
    for (auto it = std::begin(mListSegmentClouds); it != std::end(mListSegmentClouds); ++it) {
        // Get point from list
        i = it - mListSegmentClouds.begin();
        cloud = mListSegmentClouds[i];

        // Add to output cloud the first and last point
        outputCloud.points.push_back(cloud.points[0]);
        outputCloud.points.push_back(cloud.points[cloud.points.size() - 1]);
    }
    
    // Check that edges segments have been found
    if (outputCloud.size() == 0) {
        ROS_ERROR_STREAM("CLUSTER PROCESS: Getting edges points from segments. " << outputCloud.size()
                << " number of edge points gotten.");
        return 1;
    }

    // Save data
    *rOutputCloud = outputCloud;

    return 0;
}

std::vector<pcl::PointCloud<pcl::PointXYZI>> ClusterTool::getListSegmentsClouds() 
{
    return mListSegmentClouds;
}
std::vector<pcl::PointCloud<pcl::PointXYZI>> ClusterTool::getListClustersClouds() 
{
    return mListClusterClouds;
}