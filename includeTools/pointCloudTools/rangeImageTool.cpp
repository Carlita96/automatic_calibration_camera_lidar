/*
This class can be used to ...

This script cannot be run directly and needs to be used as an import.
*/

#include "rangeImageTool.hpp"

RagenImageTool::RagenImageTool()
{ 
}

RagenImageTool::~RagenImageTool(){
}

int RagenImageTool::getRangeImage(const pcl::PointCloud<pcl::PointXYZI>::Ptr pInputCloud) 
{
    /*
    Method that projects a point cloud into a range image.
    * Input parameters: 
        - pInputCloud: cloud that will be projected.
    */
    ROS_DEBUG_STREAM("RANGE IMAGE TOOL: Getting the range image to cloud of size " << pInputCloud->points.size() << ".");
    // Check input cloud
    if (pInputCloud->points.size() == 0) {
        ROS_ERROR_STREAM("RANGE IMAGE TOOL: Input cloud has size " << pInputCloud->points.size() << ".");
        return 1;
    }
    // Time the process 
    auto start = std::chrono::system_clock::now();

	// Angular resolution is the angular distance between pixels.
	// Kinect: 57° horizontal FOV, 43° vertical FOV, 640x480 (chosen here).
	// Xtion: 58° horizontal FOV, 45° vertical FOV, 640x480.
    float angularResolutionX = (float)(57.0f / 640.0f * (M_PI / 180.0f));
	float angularResolutionY = (float)(43.0f / 480.0f * (M_PI / 180.0f));
    ROS_INFO_STREAM("RANGE IMAGE TOOL: Angular resolution defined.");
	// Maximum horizontal and vertical angles. For example, for a full panoramic scan,
	// the first would be 360º. Choosing values that adjust to the real sensor will
	// decrease the time it takes, but don't worry. If the values are bigger than
	// the real ones, the image will be automatically cropped to discard empty zones.
	float maxAngleX = (float)(60.0f * (M_PI / 180.0f));
	float maxAngleY = (float)(50.0f * (M_PI / 180.0f));
    ROS_INFO_STREAM("RANGE IMAGE TOOL: Max angles defined.");
	// Sensor pose. Thankfully, the cloud includes the data.
	// Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(-0.0234669, -0.178055, -0.142186)) *
	// 							 Eigen::Affine3f(Eigen::Quaternionf(0.454809, -0.443706, 0.54286, 0.54916));
	Eigen::Affine3f sensorPose = Eigen::Affine3f(Eigen::Translation3f(0, 0, 0)) *
								 Eigen::Affine3f(Eigen::Quaternionf(0, 0, 0, 0));
    ROS_INFO_STREAM("RANGE IMAGE TOOL: Affine transformation defined.");
	// Noise level. If greater than 0, values of neighboring points will be averaged.
	// This would set the search radius (e.g., 0.03 == 3cm).
	float noiseLevel = 0.05f;
	// Minimum range. If set, any point closer to the sensor than this will be ignored.
	float minimumRange = 0.0f;
	// Border size. If greater than 0, a border of "unobserved" points will be left
	// in the image when it is cropped.
	int borderSize = 1;
    ROS_INFO_STREAM("RANGE IMAGE TOOL: Random parameters defined.");

	// Range image object.
	pcl::RangeImage rangeImage;
	rangeImage.createFromPointCloud(*pInputCloud, angularResolutionX, angularResolutionY,
									maxAngleX, maxAngleY, sensorPose, pcl::RangeImage::CAMERA_FRAME,
									noiseLevel, minimumRange, borderSize);

    ROS_INFO_STREAM("RANGE IMAGE TOOL: Range image created.");


	// // Visualize the image.
	// pcl::visualization::RangeImageVisualizer viewer("Range image");
	// viewer.showRangeImage(rangeImage);
	// while (!viewer.wasStopped())
	// {
	// 	viewer.spinOnce();
	// 	// Sleep 100ms to go easy on the CPU.
	// 	pcl_sleep(0.1);
	// }
    // ROS_INFO_STREAM("RANGE IMAGE TOOL: PCL viewer shown created.");

    // Time the process 
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<float> duration = end - start;

    ROS_DEBUG_STREAM("RANGE IMAGE TOOL: Projection done in " << duration.count() << " seconds.");
    return 0;
}
