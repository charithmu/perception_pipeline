// other
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
// msgs
#include <perception_pipeline/Quadcloud.h>
#include <sensor_msgs/PointCloud2.h>
// ros
#include <ros/console.h>
#include <ros/package.h>
#include <ros/ros.h>
// c++
#include <statistics.h>

#include <array>
#include <iostream>

typedef sensor_msgs::PointCloud2 ROSPointCloud;
typedef sensor_msgs::PointCloud2::Ptr ROSPointCloudPtr;

typedef perception_pipeline::Quadcloud QuadCloud;

/*
 * Global variables
 */

// Publishers
ros::Publisher quadcloud_pub;

/*
 * Synchronized message callback
 */
void callback(const sensor_msgs::PointCloud2ConstPtr &raw_cloud1, const sensor_msgs::PointCloud2ConstPtr &raw_cloud2,
              const sensor_msgs::PointCloud2ConstPtr &raw_cloud3, const sensor_msgs::PointCloud2ConstPtr &raw_cloud4)
{
  QuadCloud quad;
  quad.header = (*raw_cloud1).header;
  quad.cloud1 = *raw_cloud1;
  quad.cloud2 = *raw_cloud2;
  quad.cloud3 = *raw_cloud3;
  quad.cloud4 = *raw_cloud4;

  quadcloud_pub.publish(quad);
}

/*
 * Main function
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sync_node");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  auto world_frame = nh.param<std::string>("world_frame", "world_frame");
  auto sensor1_frame = nh.param<std::string>("sensor1_frame", "sensor1_frame");
  auto sensor2_frame = nh.param<std::string>("sensor2_frame", "sensor2_frame");
  auto sensor3_frame = nh.param<std::string>("sensor3_frame", "sensor3_frame");
  auto sensor4_frame = nh.param<std::string>("sensor4_frame", "sensor4_frame");

  auto sensor1_topic = nh.param<std::string>("sensor1_topic", "env/sensor1/points");
  auto sensor2_topic = nh.param<std::string>("sensor2_topic", "env/sensor2/points");
  auto sensor3_topic = nh.param<std::string>("sensor3_topic", "env/sensor3/points");
  auto sensor4_topic = nh.param<std::string>("sensor4_topic", "env/sensor4/points");

  auto sync_policy = nh.param<std::string>("sync_policy", "approx");

  quadcloud_pub = nh.advertise<QuadCloud>("env/syncedClouds", 1);

  message_filters::Subscriber<ROSPointCloud> s1(nh, sensor1_topic, 1);
  message_filters::Subscriber<ROSPointCloud> s2(nh, sensor2_topic, 1);
  message_filters::Subscriber<ROSPointCloud> s3(nh, sensor3_topic, 1);
  message_filters::Subscriber<ROSPointCloud> s4(nh, sensor4_topic, 1);

  ROS_INFO("Node: sync_node:: Messege sync subscribers created.");

  typedef message_filters::sync_policies::ExactTime<ROSPointCloud, ROSPointCloud, ROSPointCloud, ROSPointCloud>
      ExactTimePolicy;
  typedef message_filters::sync_policies::ApproximateTime<ROSPointCloud, ROSPointCloud, ROSPointCloud, ROSPointCloud>
      ApproxTimePolicy;

  message_filters::Synchronizer<ExactTimePolicy> exactTimeSync(ExactTimePolicy(5), s1, s2, s3, s4);
  message_filters::Synchronizer<ApproxTimePolicy> approxTimeSync(ApproxTimePolicy(5), s1, s2, s3, s4);

  message_filters::TimeSynchronizer<ROSPointCloud, ROSPointCloud, ROSPointCloud, ROSPointCloud> simpleTimeSync(
      s1, s2, s3, s4, 5);

  ROS_INFO("Node: sync_node:: Messege synchronizer created.");

  if (sync_policy == "simple")
  {
    simpleTimeSync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  }
  else if (sync_policy == "exact")
  {
    exactTimeSync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  }
  else if (sync_policy == "approx")
  {
    approxTimeSync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));
  }
  else
  {
    ROS_ERROR("Node: sync_node:: Unidentified messege synchronizer specified.");
  }

  ROS_INFO("Node: sync_node:: Messege sync callback created and listning for sensor messeges.");

  ros::spin();

  return 0;
}
