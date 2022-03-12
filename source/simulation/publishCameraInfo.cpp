/*
*/

// Import necessary libraries
// ROS libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>


int main (int argc, char** argv)
{
    ros::init( argc, argv, "publishCameraInfoNode" );

    // Create publisher
    ros::NodeHandle nh;
    std::string cameraInfoTopicName;
    nh.param<std::string>( "calibration/cameraInfoTopicName", cameraInfoTopicName, std::string( "/camera_cam_rear/camera_info" ));
    ros::Publisher cameraInfoPublisher = nh.advertise<sensor_msgs::CameraInfo>(cameraInfoTopicName, 1);

    // Create message
    sensor_msgs::CameraInfo cameraInfoMessage;
    boost::array<double, 9> K = {998.0229234911538, 0, 964.1975906475012, 
               0, 995.708687304811, 595.5671753731642,
               0, 0, 1};
    cameraInfoMessage.K = K;
    boost::array<double, 9> R = {1, 0, 0, 
               0, 1, 0,
               0, 0, 1};
    cameraInfoMessage.R = R;

    // Do while loop. Needed for proper shut down
    int published = 0;
    while (published < 100)
    {
        // Publish
        cameraInfoMessage.header.stamp = ros::Time::now();
        cameraInfoPublisher.publish(cameraInfoMessage);
        published++;

        // Normal way to spin
        ros::spinOnce();
        usleep(100000);
    }

    // Stop ROS node
    ros::shutdown();

    return 0;
}