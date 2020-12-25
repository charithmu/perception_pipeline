#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>

#include <cstring>
#include <iostream>
#include <string>
#include <chrono>

ros::Publisher sensor1_pub, sensor2_pub, sensor3_pub;

void sensor1callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 newMsg;
  newMsg = *msg;
  newMsg.header.stamp.fromNSec(std::chrono::nanoseconds{0}.count());

  //ROS_INFO("I heard from sensor 1: [%s]", msg->header.frame_id.c_str());
  sensor1_pub.publish(newMsg);
}

void sensor2callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 newMsg;
  newMsg = *msg;
  newMsg.header.stamp.fromNSec(std::chrono::nanoseconds{0}.count());

  //ROS_INFO("I heard from sensor 2: [%s]", msg->header.frame_id.c_str());
  sensor2_pub.publish(newMsg);
}

void sensor3callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  sensor_msgs::PointCloud2 newMsg;
  newMsg = *msg;
  newMsg.header.stamp.fromNSec(std::chrono::nanoseconds{0}.count());

  //ROS_INFO("I heard from sensor 3: [%s]", msg->header.frame_id.c_str());
  sensor3_pub.publish(newMsg);
}

int main(int argc, char** argv)
{
  // init node
  ros::init(argc, argv, "sensor_playback");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  ros::Subscriber sensor1_sub = nh.subscribe("/env/sensor1/points", 1000, sensor1callback);
  ros::Subscriber sensor2_sub = nh.subscribe("/env/sensor2/points", 1000, sensor2callback);
  ros::Subscriber sensor3_sub = nh.subscribe("/env/sensor3/points", 1000, sensor3callback);

  sensor1_pub = nh.advertise<sensor_msgs::PointCloud2>("/env/sensor1/points/synced", 1000);
  sensor2_pub = nh.advertise<sensor_msgs::PointCloud2>("/env/sensor2/points/synced", 1000);
  sensor3_pub = nh.advertise<sensor_msgs::PointCloud2>("/env/sensor3/points/synced", 1000);

  ros::spin();

  return 0;
}