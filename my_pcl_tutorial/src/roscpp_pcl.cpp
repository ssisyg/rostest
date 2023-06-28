#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Laser Cloud Viewer"));
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // 将激光雷达数据转换为PCL点云数据
  cloud->clear();
  for (int i = 0; i < msg->ranges.size(); ++i)
  {
    pcl::PointXYZ point;
    point.x = msg->ranges[i] * std::cos(msg->angle_min + i * msg->angle_increment);
    point.y = msg->ranges[i] * std::sin(msg->angle_min + i * msg->angle_increment);
    point.z = 0.0;
    cloud->push_back(point);
  }

  // 更新点云显示
  viewer->removeAllPointClouds();
  viewer->addPointCloud(cloud, "laser_cloud");
  viewer->spinOnce(10);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_visualizer");
  ros::NodeHandle nh;

  // 订阅激光雷达数据
  ros::Subscriber sub = nh.subscribe("/scan", 1, scanCallback);

  while (ros::ok() && !viewer->wasStopped())
  {
    // 处理ROS回调函数
    ros::spinOnce();
  }

  return 0;
}
