#ifndef AR_TAG_TO_PIXEL_H
#define AR_TAG_TO_PIXEL_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/PointStamped.h>
#include <cv_bridge/cv_bridge.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


#include <opencv2/opencv.hpp>

using namespace sensor_msgs;

class ARTagtoPixel
{
    public:
    ARTagtoPixel();
    ~ARTagtoPixel();

    private:
    ros::NodeHandle nh_;
    ros::Subscriber camera_info_sub;
    ros::Subscriber image_sub;
    ros::Publisher pixel_point_pub;
    ros::Publisher image_pub;

    tf::TransformListener* tf_listener;
    ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg;
    CameraInfo camera_info;
    void cameraInfoCallback(const CameraInfo::ConstPtr& msg);
    void arTagCallback(const sensor_msgs::ImageConstPtr& img);
};

#endif  // AR_TAG_TO_PIXEL_H