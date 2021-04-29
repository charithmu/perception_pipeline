#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <chrono>
#include <cstring>
#include <iostream>
#include <string>

ros::Publisher sensor1_pub, sensor2_pub, sensor3_pub, sensor4_pub;

void sensor1callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  sensor_msgs::PointCloud2 newMsg;
  newMsg = *msg;
  newMsg.header.stamp.fromNSec(std::chrono::nanoseconds{0}.count());
  newMsg.header.frame_id = "sensor1_frame";
  // ROS_INFO("I heard from sensor 1: [%s]", msg->header.frame_id.c_str());
  sensor1_pub.publish(newMsg);
}

void sensor2callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  sensor_msgs::PointCloud2 newMsg;
  newMsg = *msg;
  newMsg.header.stamp.fromNSec(std::chrono::nanoseconds{0}.count());
  newMsg.header.frame_id = "sensor2_frame";
  // ROS_INFO("I heard from sensor 2: [%s]", msg->header.frame_id.c_str());
  sensor2_pub.publish(newMsg);
}

void sensor3callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  sensor_msgs::PointCloud2 newMsg;
  newMsg = *msg;
  newMsg.header.stamp.fromNSec(std::chrono::nanoseconds{0}.count());
  newMsg.header.frame_id = "sensor3_frame";
  // ROS_INFO("I heard from sensor 3: [%s]", msg->header.frame_id.c_str());
  sensor3_pub.publish(newMsg);
}

void sensor4callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  sensor_msgs::PointCloud2 newMsg;
  newMsg = *msg;
  newMsg.header.stamp.fromNSec(std::chrono::nanoseconds{0}.count());
  newMsg.header.frame_id = "sensor4_frame";
  // ROS_INFO("I heard from sensor 3: [%s]", msg->header.frame_id.c_str());
  sensor4_pub.publish(newMsg);
}

int main(int argc, char **argv)
{
  // init node
  ros::init(argc, argv, "sensor_playback");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  ros::Subscriber sensor1_sub = nh.subscribe("/env/sensor1/points", 10, sensor1callback);
  ros::Subscriber sensor2_sub = nh.subscribe("/env/sensor2/points", 10, sensor2callback);
  ros::Subscriber sensor3_sub = nh.subscribe("/env/sensor3/points", 10, sensor3callback);
  ros::Subscriber sensor4_sub = nh.subscribe("/env/sensor4/points", 10, sensor4callback);

  sensor1_pub = nh.advertise<sensor_msgs::PointCloud2>("/sensor1_os_cloud_node/points", 10);
  sensor2_pub = nh.advertise<sensor_msgs::PointCloud2>("/sensor2_os_cloud_node/points", 10);
  sensor3_pub = nh.advertise<sensor_msgs::PointCloud2>("/sensor3_os_cloud_node/points", 10);
  sensor4_pub = nh.advertise<sensor_msgs::PointCloud2>("/sensor4_os_cloud_node/points", 10);

  ros::spin();

  return 0;
}
