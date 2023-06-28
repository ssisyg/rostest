/*
 * OpenCV Example using ROS and CPP
 */

// Include the ROS library
#include <ros/ros.h>



#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
// Include CvBridge, Image Transport, Image msg
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV Window Name
static const std::string OPENCV_WINDOW = "Image window";

// Topics
static const std::string IMAGE_TOPIC = "/camera/color/image_raw";
// /camera/depth/image_rect_raw
static const std::string DEPTH_IMAGE_TOPIC = "/camera/depth/image_rect_raw";




void image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  std_msgs::Header msg_header = msg->header;
  std::string frame_id = msg_header.frame_id.c_str();
  ROS_INFO_STREAM("New Image from " << frame_id);
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(3);
}


void depth_image_cb(const sensor_msgs::ImageConstPtr& msg)
{
  std_msgs::Header msg_header = msg->header;
  std::string frame_id = msg_header.frame_id.c_str();
  ROS_INFO_STREAM("New Depth Image from " << frame_id);
  //ROS_INFO_STREAM("Local: " << std::put_time(std::localtime(&t), "%c %Z"));
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  cv::imshow("DEPTH IMAGE", cv_ptr->image);
  cv::waitKey(3);
}
int main(int argc, char** argv)
{
  // Initialize the ROS Node "roscpp_opencv"
  ros::init(argc, argv, "roscpp_opencv");
  ros::NodeHandle nh;
  
  // Print "Hello" message with node name to the terminal and ROS log file
  ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

  // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a callback function of image_cb
  ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, image_cb);
  ros::Subscriber depth_sub = nh.subscribe(DEPTH_IMAGE_TOPIC, 1, depth_image_cb);

  // Initialize an OpenCV Window
  cv::namedWindow(OPENCV_WINDOW);

  // loop to keep ROS subscriber and publisher threads open
  ros::spin();

  // Program successful
  return 0;
}