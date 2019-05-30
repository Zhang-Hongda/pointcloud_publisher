#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include "getfile.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_publisher");
  ROS_INFO("pointcloud_publisher START!");
  ros::NodeHandle n;
  std::string topic, path, frame_id;
  int rate;
  ros::param::param<std::string>("~topic_name", topic, "/kinect2/sd/points");
  ros::param::param<std::string>("~frame_id", frame_id, "kinect2_link");
  ros::param::param("~rate", rate, 10);
  path = ros::package::getPath("pointcloud_publisher") + "/pcd";
  ROS_INFO("through topic: %s", topic.c_str());
  ROS_INFO("pcdfiles path: %s", path.c_str());
  ROS_INFO("frame id: %s", frame_id.c_str());
  ROS_INFO("rate: %d", rate);
  file_names pcd_files = get_files(path, "pcd");

  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>(topic, 1);
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud_set;
  std::vector<sensor_msgs::PointCloud2> output_set;
  for (std::vector<std::string>::const_iterator it = pcd_files.ext_files.begin(); it != pcd_files.ext_files.end(); it++)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    sensor_msgs::PointCloud2 output;
    pcl::io::loadPCDFile<pcl::PointXYZRGB>(*it, *cloud);
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = frame_id;
    output.header.stamp = ros::Time::now();

    output_set.push_back(output);
    cloud_set.push_back(cloud);
  }
  ros::Rate loop_rate(rate);
  ROS_INFO("Publishing......");
  while (ros::ok())
  {
    for (std::vector<sensor_msgs::PointCloud2>::const_iterator it = output_set.begin(); it != output_set.end(); it++)
    {
      pub.publish(*it);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}
