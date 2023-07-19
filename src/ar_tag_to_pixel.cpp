#include "ar_tag_to_pixel/ar_tag_to_pixel.h"

ARTagtoPixel::ARTagtoPixel():nh_("~")
{
  camera_info_sub = nh_.subscribe("/camera/color/camera_info", 1, &ARTagtoPixel::cameraInfoCallback,this);
  image_sub = nh_.subscribe("/camera/color/image_raw", 1, &ARTagtoPixel::arTagCallback,this);
  pixel_point_pub = nh_.advertise<geometry_msgs::PointStamped>("/ar_tag/pixel_point", 1);
  image_pub = nh_.advertise<sensor_msgs::Image>("/ar_tag/image_with_point", 1);
  tf_listener = new tf::TransformListener();
}

ARTagtoPixel::~ARTagtoPixel()
{
  delete tf_listener;
  tf_listener = nullptr;

}

void ARTagtoPixel::cameraInfoCallback(const CameraInfo::ConstPtr& msg)
{
  camera_info = *msg;
}

void ARTagtoPixel::arTagCallback(const ImageConstPtr& img)
{
  
  
  // 获取当前帧图像转为cv
  cv_bridge::CvImagePtr cv_ptr =  cv_bridge::toCvCopy(img, image_encodings::RGB8); 

  // 获取ATTag信息
  
  // msg = boost::make_shared<ar_track_alvar_msgs::AlvarMarkers>();
  // ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg;
  
  ar_track_alvar_msgs::AlvarMarkers::ConstPtr msg = ros::topic::waitForMessage<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker", ros::Duration(5));
  for (const auto& marker : msg->markers)
  {
    
    // 访问AR标记的ID
    int id = marker.id;

    // 访问AR标记的位姿(相对于父坐标camera_link)
    geometry_msgs::Pose pose = marker.pose.pose;
    geometry_msgs::Point position = pose.position;
    geometry_msgs::Quaternion orientation = pose.orientation;


    // 获取相机坐标与图像坐标的变换阵
    // 相机坐标系           图像坐标系
    //    y                     y
    //    |                   /
    //    |                  /
    //    |______x          /________ z
    //   /                  |
    //  /                   |
    // z                    x
    
    tf::StampedTransform transform;
    try
    {
      tf_listener->lookupTransform("camera_color_optical_frame", marker.header.frame_id, ros::Time(0), transform);
    }
    catch (tf::TransformException& e)
    {
      ROS_WARN("Failed to lookup transform: %s", e.what());
      return;
    }
    // ROS_INFO("Transform:");
    // ROS_INFO("Translation (x, y, z): %.2f, %.2f, %.2f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
    // ROS_INFO("Rotation (x, y, z, w): %.2f, %.2f, %.2f, %.2f", transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
    // ROS_INFO("Child Frame ID: %s", transform.child_frame_id_.c_str());
    // ROS_INFO("Parent Frame ID: %s", transform.frame_id_.c_str());
    // ROS_INFO("Timestamp: %f", transform.stamp_.toSec());

    tf::Point ar_position(marker.pose.pose.position.x, marker.pose.pose.position.y, marker.pose.pose.position.z);
    tf::Point uv_position = transform * ar_position;
    // ROS_INFO("ar_position (x, y, z): %.2f, %.2f, %.2f", position.x, position.y, position.z);
    // ROS_INFO("camera_position (x, y, z): %.2f, %.2f, %.2f", uv_position.x(), uv_position.y(), uv_position.z());

    // 三维坐标点转换至像素坐标系中
    double fx = camera_info.K[0];  // 相机内参矩阵的fx
    double fy = camera_info.K[4];  // 相机内参矩阵的fy
    double cx = camera_info.K[2];  // 相机内参矩阵的cx
    double cy = camera_info.K[5];  // 相机内参矩阵的cy
    
    double u = fx * uv_position.x() / uv_position.z() + cx;
    double v = fy * uv_position.y() / uv_position.z() + cy;

    ROS_INFO("uv_position (u, v): %.2f, %.2f", u, v);
    // 在原始图像上标记像素点
    cv::Point p_point(u, v);
    cv::circle(cv_ptr->image, p_point, 5, cv::Scalar(0, 0, 255), -1);
    // 发布带有标记的图像
    geometry_msgs::PointStamped pixel_point;
    pixel_point.header = msg->header;
    pixel_point.point.x = u;
    pixel_point.point.y = v;
    pixel_point.point.z = 0.0;

    pixel_point_pub.publish(pixel_point);



    // 打印AR标记的ID和位姿信息
    // ROS_INFO("AR Marker ID: %d", id);
    // ROS_INFO("Position (x, y, z): %.2f, %.2f, %.2f", ar_position.x, ar_position.y, ar_position.z);
    // ROS_INFO("Orientation (x, y, z, w): %.2f, %.2f, %.2f, %.2f", ar_orientation.x, ar_orientation.y, ar_orientation.z, ar_orientation.w);
  }
  image_pub.publish(cv_ptr->toImageMsg());
  // msg.reset(new ar_track_alvar_msgs::AlvarMarkers());


}