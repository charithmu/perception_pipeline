// pcl
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
// other
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
// msgs
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
// ros
#include <ros/console.h>
#include <ros/ros.h>
// c++
#include <statistics.h>

#include <array>
#include <iostream>

typedef pcl::PointCloud<pcl::PointXYZI> PCLPointCloud;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PCLPointCloudPtr;
typedef sensor_msgs::PointCloud2 ROSPointCloud;
typedef sensor_msgs::PointCloud2::Ptr ROSPointCloudPtr;

/*
 * Global variables
 */

// Statistics
Statistics stats;

// Parameters used gloablly
std::string world_frame;
float voxel_leaf_size;
float x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;

// Publishers
ros::Publisher sensor1_pub, sensor2_pub, sensor3_pub, sensor4_pub, combined_pub, voxelfilter_pub, passfilter_pub;

// static transforms
geometry_msgs::TransformStamped transformStamped1, transformStamped2, transformStamped3, transformStamped4;

/*
 * Utility Functions
 */

// Identity-transform creator
geometry_msgs::TransformStamped identityTransform(std::string parent_frame, std::string child_frame)
{
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> mat = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d rot;

  geometry_msgs::TransformStamped empty_transform;

  empty_transform.header.stamp = ros::Time::now();
  empty_transform.header.frame_id = parent_frame;
  empty_transform.child_frame_id = child_frame;

  empty_transform.transform.translation.x = mat(0, 3);
  empty_transform.transform.translation.y = mat(1, 3);
  empty_transform.transform.translation.z = mat(2, 3);

  rot = mat.block<3, 3>(0, 0);

  const Eigen::Quaterniond q(rot);

  empty_transform.transform.rotation.x = q.x();
  empty_transform.transform.rotation.y = q.y();
  empty_transform.transform.rotation.z = q.z();
  empty_transform.transform.rotation.w = q.w();

  std::cout << "rot: " << rot << std::endl << q.x() << " " << q.y() << " " << q.z() << " " << std::endl;

  ROS_INFO("transformation: %s to world", child_frame.c_str());

  return empty_transform;
}

/*
 * Synchronized message callback
 */
void callback(const sensor_msgs::PointCloud2ConstPtr &raw_cloud1, const sensor_msgs::PointCloud2ConstPtr &raw_cloud2,
              const sensor_msgs::PointCloud2ConstPtr &raw_cloud3)
{
  // start stat measurements
  stats.startCycle();

  /*
   * Transform and publish
   */
  // Transform the pointclouds based on tf
  sensor_msgs::PointCloud2 transformed_cloud1, transformed_cloud2, transformed_cloud3, transformed_cloud4;

  pcl_ros::transformPointCloud(world_frame, transformStamped1.transform, *raw_cloud1, transformed_cloud1);
  pcl_ros::transformPointCloud(world_frame, transformStamped2.transform, *raw_cloud2, transformed_cloud2);
  pcl_ros::transformPointCloud(world_frame, transformStamped3.transform, *raw_cloud3, transformed_cloud3);

  // Publish individually transformed pointclouds
  sensor1_pub.publish(transformed_cloud1);
  sensor2_pub.publish(transformed_cloud2);
  sensor3_pub.publish(transformed_cloud3);

  // Convert pointcloud ROS->PCL for further processing
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud1, pcl_cloud2, pcl_cloud3, pcl_cloud4, pcl_cloud_full;

  pcl::fromROSMsg(transformed_cloud1, pcl_cloud1);
  pcl::fromROSMsg(transformed_cloud2, pcl_cloud2);
  pcl::fromROSMsg(transformed_cloud3, pcl_cloud3);
  // pcl::fromROSMsg(transformed_cloud4, pcl_cloud4);

  // Concatenate pointclouds on point basis
  pcl_cloud_full = pcl_cloud1 + pcl_cloud2 + pcl_cloud3;  // + pcl_cloud4;

  // Convert pointcloud PCL->ROS
  sensor_msgs::PointCloud2::Ptr ros_cloud_full(new sensor_msgs::PointCloud2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_full_ptr(new pcl::PointCloud<pcl::PointXYZI>(pcl_cloud_full));

  pcl::toROSMsg(*pcl_cloud_full_ptr, *ros_cloud_full);

  ros_cloud_full->header.frame_id = world_frame;
  ros_cloud_full->header.stamp = ros::Time::now();

  // Publish combined pointcloud
  combined_pub.publish(ros_cloud_full);

  stats.finishCycle();

  // voxel grid filter
  float voxel_leaf_size = 0.01;  // millimeters

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr = pcl_cloud_full_ptr;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_voxel_filtered(new pcl::PointCloud<pcl::PointXYZI>());

  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

  voxel_filter.setInputCloud(cloud_ptr);
  voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
  voxel_filter.filter(*cloud_voxel_filtered);

  // ROS_INFO_STREAM("Original cloud  had " << cloud_ptr->size() << " points");
  // ROS_INFO_STREAM("Downsampled cloud  with " << cloud_voxel_filtered->size() << " points");

  ///////////// Convert pointcloud PCL->ROS & Publish combined pointcloud
  sensor_msgs::PointCloud2::Ptr ros_cloud_voxel(new sensor_msgs::PointCloud2);

  pcl::toROSMsg(*cloud_voxel_filtered, *ros_cloud_voxel);
  ros_cloud_voxel->header.frame_id = world_frame;
  ros_cloud_voxel->header.stamp = ros::Time::now();
  voxelfilter_pub.publish(ros_cloud_voxel);
  /////////////

  // pass through filters
  float x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;
  pcl::PointCloud<pcl::PointXYZI> xf_cloud, yf_cloud, zf_cloud;

  // x
  pcl::PassThrough<pcl::PointXYZI> pass_x;
  pass_x.setInputCloud(cloud_voxel_filtered);  // input from last filter
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(x_filter_min, x_filter_max);
  pass_x.filter(xf_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr xf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(xf_cloud));
  // y
  pcl::PassThrough<pcl::PointXYZI> pass_y;
  pass_y.setInputCloud(xf_cloud_ptr);
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(y_filter_min, y_filter_max);
  pass_y.filter(yf_cloud);

  pcl::PointCloud<pcl::PointXYZI>::Ptr yf_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>(yf_cloud));
  // z
  pcl::PassThrough<pcl::PointXYZI> pass_z;
  pass_z.setInputCloud(yf_cloud_ptr);
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(z_filter_min, z_filter_max);
  pass_z.filter(zf_cloud);

  ///////////// Convert pointcloud PCL->ROS & Publish combined pointcloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_passthorugh_ptr(new pcl::PointCloud<pcl::PointXYZI>(zf_cloud));
  sensor_msgs::PointCloud2::Ptr ros_cloud_passthrough(new sensor_msgs::PointCloud2);

  pcl::toROSMsg(*pcl_cloud_passthorugh_ptr, *ros_cloud_passthrough);
  ros_cloud_passthrough->header.frame_id = world_frame;
  ros_cloud_passthrough->header.stamp = ros::Time::now();
  passfilter_pub.publish(ros_cloud_passthrough);
  /////////////
}

// main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "transformpipe_node");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  std::string sensor1_frame, sensor2_frame, sensor3_frame, sensor4_frame;
  std::string sensor1_topic, sensor2_topic, sensor3_topic, sensor4_topic;
  std::string sensor1_topic_res, sensor2_topic_res, sensor3_topic_res, sensor4_topic_res;

  world_frame = priv_nh_.param<std::string>("world_frame", "world_frame");

  sensor1_frame = priv_nh_.param<std::string>("sensor1_frame", "sensor1_frame");
  sensor2_frame = priv_nh_.param<std::string>("sensor2_frame", "sensor2_frame");
  sensor3_frame = priv_nh_.param<std::string>("sensor3_frame", "sensor3_frame");
  sensor4_frame = priv_nh_.param<std::string>("sensor4_frame", "sensor4_frame");

  sensor1_topic = priv_nh_.param<std::string>("sensor1_topic", "env/sensor1/points");
  sensor2_topic = priv_nh_.param<std::string>("sensor2_topic", "env/sensor2/points");
  sensor3_topic = priv_nh_.param<std::string>("sensor3_topic", "env/sensor3/points");
  sensor4_topic = priv_nh_.param<std::string>("sensor4_topic", "env/sensor4/points");

  sensor1_pub = nh.advertise<sensor_msgs::PointCloud2>("env/sensor1/points/aligned", 1);
  sensor2_pub = nh.advertise<sensor_msgs::PointCloud2>("env/sensor2/points/aligned", 1);
  sensor3_pub = nh.advertise<sensor_msgs::PointCloud2>("env/sensor3/points/aligned", 1);
  sensor4_pub = nh.advertise<sensor_msgs::PointCloud2>("env/sensor4/points/aligned", 1);

  combined_pub = nh.advertise<sensor_msgs::PointCloud2>("env/combined/points", 1);

  voxelfilter_pub = nh.advertise<sensor_msgs::PointCloud2>("env/combined/voxelfilter", 1);
  passfilter_pub = nh.advertise<sensor_msgs::PointCloud2>("env/combined/passfilter", 1);

  voxel_leaf_size = priv_nh_.param<float>("voxel_leaf_size", 0.1);
  x_filter_min = priv_nh_.param<float>("x_filter_min", -2.5);
  x_filter_max = priv_nh_.param<float>("x_filter_max", 2.5);
  y_filter_min = priv_nh_.param<float>("y_filter_min", -2.5);
  y_filter_max = priv_nh_.param<float>("y_filter_max", 2.5);
  z_filter_min = priv_nh_.param<float>("z_filter_min", -2.5);
  z_filter_max = priv_nh_.param<float>("z_filter_max", 2.5);

  // Transform listner
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  // tfListener = new tf2_ros::TransformListener(tfBuffer);

  // Get the transformation for each pointcloud
  try
  {
    transformStamped1 = tfBuffer.lookupTransform(world_frame, sensor1_frame, ros::Time(0), ros::Duration(1));
    transformStamped2 = tfBuffer.lookupTransform(world_frame, sensor2_frame, ros::Time(0), ros::Duration(1));
    transformStamped3 = tfBuffer.lookupTransform(world_frame, sensor3_frame, ros::Time(0), ros::Duration(1));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    // continue;
  }

  sensor1_topic_res = nh.resolveName(sensor1_topic);
  sensor2_topic_res = nh.resolveName(sensor2_topic);
  sensor3_topic_res = nh.resolveName(sensor3_topic);
  sensor4_topic_res = nh.resolveName(sensor4_topic);

  message_filters::Subscriber<sensor_msgs::PointCloud2> s1(nh, sensor1_topic_res, 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> s2(nh, sensor2_topic_res, 100);
  message_filters::Subscriber<sensor_msgs::PointCloud2> s3(nh, sensor3_topic_res, 100);

  // typedef message_filters::sync_policies::ExactTime<PointCloud2, PointCloud2, PointCloud2> ExactTimePolicy;
  // typedef message_filters::sync_policies::ApproximateTime<PointCloud2, PointCloud2, PointCloud2> ApproxTimePolicy;

  // message_filters::Synchronizer<ApproxTimePolicy> policysync(ApproxTimePolicy(10), s1, s2, s3);
  // policysync.registerCallback(boost::bind(&callback, _1, _2, _3));

  message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2, sensor_msgs::PointCloud2>
      timesync(s1, s2, s3, 100);
  timesync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}
