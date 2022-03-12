/*
In this file, functions to publish an image, pointcloud or marker are defined.
They can be imported and used as wanted.
*/

#include "publishTools.hpp"

void publishPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr pCloud, const ros::Publisher publisher, 
        const std::string frame) 
{
    /*
    Function that takes a PCL cloud and publishes in a frame.
    * Input parameters:
        - pCloud: pointer to cloud that will be published.
        - publisher: publisher that will send the cloud.
        - frame: frame_id that will be defined as the frame of the cloud when publishing it.
    */
    // Convert PCL cloud to a ROS message type
    sensor_msgs::PointCloud2 outputMsg;
    pcl::toROSMsg(*pCloud, outputMsg);

    // Add information as timestamp and frame
    outputMsg.header.stamp = ros::Time::now();
    outputMsg.header.frame_id = frame;
    
    // Publish
    publisher.publish(outputMsg);
}

void publishColorPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pCloud, const ros::Publisher publisher, 
        const std::string frame) 
{
    /*
    Function that takes a PCL cloud and publishes in a frame.
    * Input parameters:
        - pCloud: pointer to cloud that will be published.
        - publisher: publisher that will send the cloud.
        - frame: frame_id that will be defined as the frame of the cloud when publishing it.
    */
    // Convert PCL cloud to a ROS message type
    sensor_msgs::PointCloud2 outputMsg;
    pcl::toROSMsg(*pCloud, outputMsg);

    // Add information as timestamp and frame
    outputMsg.header.stamp = ros::Time::now();
    outputMsg.header.frame_id = frame;
    
    // Publish
    publisher.publish(outputMsg);
}

void publishImage(const cv::Mat image, const image_transport::Publisher publisher) 
{
    /*
    Function that publishes an image in a publisher.
    * Input parameters:
        - image: image that will be published.
        - publisher: publisher that will send the image.
    */
    // Create message with image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    // Publish message
    publisher.publish(msg);
}

void publishGrayScaleImage(const cv::Mat image, const image_transport::Publisher publisher) 
{
    /*
    Function that publishes an image in a publisher.
    * Input parameters:
        - image: image that will be published.
        - publisher: publisher that will send the image.
    */
    // Create message with image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    // Publish message
    publisher.publish(msg);
}

void publishSphereMarker(const double positionX, const double positionY, const double positionZ, const double size, 
    const ros::Publisher publisher, const std::string frame, const float red, const float green, const float blue) 
{
    /*
    Function that publishes a sphere marker for Rviz.
    * Input parameters:
        - positionX, positionY, positionZ: define the center of the sphere to be placed in Rviz.
        - radius: define the radius of the sphere to be placed in Rviz.
        - publisher: publisher that will send the sphere.
        - frame: frame_id that will be defined as the frame of the sphere when publishing it.
        - red, green, blue: defining the color in which it will be published. Values between: [0-255].
    */

    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = frame;
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::SPHERE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = positionX;
    marker.pose.position.y = positionY;
    marker.pose.position.z = positionZ;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = size;
    marker.scale.y = size;
    marker.scale.z = size;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    publisher.publish(marker);
}

void publishStaticTf(const double tx, const double ty, const double tz, const double qx, const double qy, 
    const double qz, const double qw, const std::string sourceFrame, const std::string targetFrame,
    tf2_ros::StaticTransformBroadcaster broadcaster) 
{
    // Publish TF
    geometry_msgs::TransformStamped staticTransformStamped;
    staticTransformStamped.header.stamp = ros::Time::now();
    staticTransformStamped.header.frame_id = sourceFrame;
    staticTransformStamped.child_frame_id = targetFrame;
    staticTransformStamped.transform.translation.x = tx;
    staticTransformStamped.transform.translation.y = ty;
    staticTransformStamped.transform.translation.z = tz;
    staticTransformStamped.transform.rotation.x = qx;
    staticTransformStamped.transform.rotation.y = qy;
    staticTransformStamped.transform.rotation.z = qz;
    staticTransformStamped.transform.rotation.w = qw;
    broadcaster.sendTransform(staticTransformStamped);
}