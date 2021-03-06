// pcl
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

// tf
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

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

typedef pcl::PointCloud<pcl::PointXYZI> PCLPointCloud;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PCLPointCloudPtr;
typedef sensor_msgs::PointCloud2 ROSPointCloud;
typedef sensor_msgs::PointCloud2::Ptr ROSPointCloudPtr;

typedef perception_pipeline::Quadcloud QuadCloud;

/*
 * Global variables
 */

// Statistics
Statistics stats;

// Parameters used gloablly
std::string world_frame;

float voxel_leaf_size;  // in mm
float x_filter_min, x_filter_max, y_filter_min, y_filter_max, z_filter_min, z_filter_max;

bool save_pcd, enable_octreefilter, enable_voxelfilter, enable_passfilter;
float save_interval;
std::string save_path_full;

// static transforms
geometry_msgs::TransformStamped transformStamped1, transformStamped2, transformStamped3, transformStamped4;

// Publishers
ros::Publisher sensor1_pub, sensor2_pub, sensor3_pub, sensor4_pub, combined_pub;
ros::Publisher octreefilter_pub, voxelfilter_pub, passfilter_pub;

// last frame time
ros::Time lastSavedTimeStamp;

/*
 * Synchronized clouds callback
 */
void quadCloudCallback(const QuadCloud::ConstPtr &quadcloud)
{
  // start stat measurements
  stats.startCycle();

  ROSPointCloud raw_cloud1, raw_cloud2, raw_cloud3, raw_cloud4;

  raw_cloud1 = (*quadcloud).cloud1;
  raw_cloud2 = (*quadcloud).cloud2;
  raw_cloud3 = (*quadcloud).cloud3;
  raw_cloud4 = (*quadcloud).cloud4;

  /*
   * TRANSFORM POINTCLOUDS AND PUBLISH SEPARATELY
   */
  ROSPointCloud ros_cloud1_transformed, ros_cloud2_transformed, ros_cloud3_transformed, ros_cloud4_transformed;

  pcl_ros::transformPointCloud(world_frame, transformStamped1.transform, raw_cloud1, ros_cloud1_transformed);
  pcl_ros::transformPointCloud(world_frame, transformStamped2.transform, raw_cloud2, ros_cloud2_transformed);
  pcl_ros::transformPointCloud(world_frame, transformStamped3.transform, raw_cloud3, ros_cloud3_transformed);
  pcl_ros::transformPointCloud(world_frame, transformStamped4.transform, raw_cloud4, ros_cloud4_transformed);

  sensor1_pub.publish(ros_cloud1_transformed);
  sensor2_pub.publish(ros_cloud2_transformed);
  sensor3_pub.publish(ros_cloud3_transformed);
  sensor4_pub.publish(ros_cloud4_transformed);

  /*
   * CONVERT POINTCLOUDS ROS->PCL
   */
  PCLPointCloud pcl_cloud1, pcl_cloud2, pcl_cloud3, pcl_cloud4, pcl_cloud_full;

  pcl::fromROSMsg(ros_cloud1_transformed, pcl_cloud1);
  pcl::fromROSMsg(ros_cloud2_transformed, pcl_cloud2);
  pcl::fromROSMsg(ros_cloud3_transformed, pcl_cloud3);
  pcl::fromROSMsg(ros_cloud4_transformed, pcl_cloud4);

  /*
   * CONCATNATE POINTCLOUDS INTO SINGLE COMBINED POINTCLOUD
   */
  pcl_cloud_full = pcl_cloud1 + pcl_cloud2 + pcl_cloud3 + pcl_cloud4;
  // set timestamp of full pointcloud to current timestamp
  pcl_conversions::toPCL(ros::Time::now(), pcl_cloud_full.header.stamp);

  /*
   * PUBLISH COMBINED POINTCLOUD
   */
  ROSPointCloudPtr ros_cloud_full_ptr(new ROSPointCloud);
  PCLPointCloudPtr pcl_cloud_full_ptr(new PCLPointCloud(pcl_cloud_full));
  pcl::toROSMsg(*pcl_cloud_full_ptr, *ros_cloud_full_ptr);
  combined_pub.publish(ros_cloud_full_ptr);

  PCLPointCloudPtr downsampleFilterOutput = pcl_cloud_full_ptr;

  if (enable_octreefilter)
  {
    /*
     * OCTREE VOXEL CENTROID FILTER AND PUBLISH
     */
    PCLPointCloudPtr pcl_cloud_octree_input = pcl_cloud_full_ptr;

    pcl::octree::OctreePointCloudVoxelCentroid<pcl::PointXYZI> octree(voxel_leaf_size);
    octree.setInputCloud(pcl_cloud_octree_input);
    octree.defineBoundingBox();
    octree.addPointsFromInputCloud();

    pcl::PointCloud<pcl::PointXYZI>::VectorType voxelCentroids;
    octree.getVoxelCentroids(voxelCentroids);

    PCLPointCloud pcl_cloud_octree_filtered;
    pcl_cloud_octree_filtered.width = voxelCentroids.size();
    pcl_cloud_octree_filtered.height = 1;
    pcl_cloud_octree_filtered.is_dense = true;
    pcl_cloud_octree_filtered.points.resize(pcl_cloud_octree_filtered.width * pcl_cloud_octree_filtered.height);
    pcl_cloud_octree_filtered.points = voxelCentroids;
    pcl_cloud_octree_filtered.header = pcl_cloud_full.header;

    ROSPointCloudPtr ros_cloud_octree_filtered(new ROSPointCloud);
    pcl::toROSMsg(pcl_cloud_octree_filtered, *ros_cloud_octree_filtered);
    octreefilter_pub.publish(ros_cloud_octree_filtered);

    PCLPointCloudPtr ros_cloud_octree_filtered_ptr(new PCLPointCloud(pcl_cloud_octree_filtered));
    downsampleFilterOutput = ros_cloud_octree_filtered_ptr;
  }

  if (enable_voxelfilter)
  {
    /*
     * VOXEL GRID FILTER AND PUBLISH
     */
    PCLPointCloudPtr pcl_cloud_voxel_input = pcl_cloud_full_ptr;
    PCLPointCloudPtr pcl_cloud_voxel_filtered(new PCLPointCloud());

    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;

    voxel_filter.setInputCloud(pcl_cloud_voxel_input);
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter(*pcl_cloud_voxel_filtered);

    ROSPointCloudPtr ros_cloud_voxel_filtered(new ROSPointCloud);
    pcl::toROSMsg(*pcl_cloud_voxel_filtered, *ros_cloud_voxel_filtered);
    voxelfilter_pub.publish(ros_cloud_voxel_filtered);

    downsampleFilterOutput = pcl_cloud_voxel_filtered;
  }

  PCLPointCloudPtr passFilterOutput = downsampleFilterOutput;

  if (enable_passfilter)
  {
    /*
     * PASS THROUGH FILTERS FOR X,Y,Z CROP AND PUBLISH
     */
    PCLPointCloud pcl_cloud_xf_out, pcl_cloud_yf_out, pcl_cloud_zf_out;
    pcl::PassThrough<pcl::PointXYZI> pass_x, pass_y, pass_z;
    // x
    PCLPointCloudPtr pcl_cloud_xf_in = downsampleFilterOutput;
    pass_x.setInputCloud(pcl_cloud_xf_in);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(x_filter_min, x_filter_max);
    pass_x.filter(pcl_cloud_xf_out);
    // y
    PCLPointCloudPtr pcl_cloud_yf_in(new PCLPointCloud(pcl_cloud_xf_out));
    pass_y.setInputCloud(pcl_cloud_yf_in);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(y_filter_min, y_filter_max);
    pass_y.filter(pcl_cloud_yf_out);
    // z
    PCLPointCloudPtr pcl_cloud_zf_in(new PCLPointCloud(pcl_cloud_yf_out));
    pass_z.setInputCloud(pcl_cloud_zf_in);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(z_filter_min, z_filter_max);
    pass_z.filter(pcl_cloud_zf_out);

    PCLPointCloudPtr pcl_cloud_passthorugh_ptr(new PCLPointCloud(pcl_cloud_zf_out));
    ROSPointCloudPtr ros_cloud_passthrough(new ROSPointCloud);

    pcl::toROSMsg(*pcl_cloud_passthorugh_ptr, *ros_cloud_passthrough);
    passfilter_pub.publish(ros_cloud_passthrough);

    passFilterOutput = pcl_cloud_passthorugh_ptr;
  }

  // end stat measurements
  stats.finishCycle();

  /*
   * SAVE TO *.PCD FILE ACCORDING TO GIVEN INTERVAL
   */

  // get the timestamp of the frame processed in this cycle
  ros::Time currentTimeStamp = ros_cloud_full_ptr->header.stamp;
  ros::Duration timeSinceLast = currentTimeStamp - lastSavedTimeStamp;

  if (save_pcd == true && timeSinceLast >= ros::Duration(save_interval))
  {
    // save to PCD && reset lastTimeStamp
    lastSavedTimeStamp = currentTimeStamp;
    std::string file_name = save_path_full + "/" + "pcd-" + std::to_string(currentTimeStamp.toNSec()) + ".pcd";
    pcl::io::savePCDFileASCII(file_name, *passFilterOutput);
    ROS_INFO("Saved pointcloud with %lu points to %s", (*passFilterOutput).size(), file_name.c_str());
  }
}

/*
 * Main function
 */
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "perception_node");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  auto world_frame = nh.param<std::string>("world_frame", "world_frame");
  auto sensor1_frame = nh.param<std::string>("sensor1_frame", "sensor1_frame");
  auto sensor2_frame = nh.param<std::string>("sensor2_frame", "sensor2_frame");
  auto sensor3_frame = nh.param<std::string>("sensor3_frame", "sensor3_frame");
  auto sensor4_frame = nh.param<std::string>("sensor4_frame", "sensor4_frame");

  sensor1_pub = nh.advertise<ROSPointCloud>("env/sensor1/points/aligned", 1);
  sensor2_pub = nh.advertise<ROSPointCloud>("env/sensor2/points/aligned", 1);
  sensor3_pub = nh.advertise<ROSPointCloud>("env/sensor3/points/aligned", 1);
  sensor4_pub = nh.advertise<ROSPointCloud>("env/sensor4/points/aligned", 1);

  combined_pub = nh.advertise<ROSPointCloud>("env/combined/points", 1);

  auto downsample_filter = priv_nh_.param<std::string>("downsample_filter", "none");
  auto resize_filter = priv_nh_.param<std::string>("resize_filter", "none");

  if (downsample_filter == "octree")
  {
    enable_octreefilter = true;
    octreefilter_pub = nh.advertise<ROSPointCloud>("env/combined/octreefilter", 1);
    voxel_leaf_size = priv_nh_.param<float>("voxel_leaf_size", 0.1);
  }
  else if (downsample_filter == "voxelgrid")
  {
    enable_voxelfilter = true;
    voxelfilter_pub = nh.advertise<ROSPointCloud>("env/combined/voxelfilter", 1);
    voxel_leaf_size = priv_nh_.param<float>("voxel_leaf_size", 0.1);
  }
  else
  {
    enable_octreefilter = false;
    enable_voxelfilter = false;
  }

  if (resize_filter == "passthrough")
  {
    enable_passfilter = true;
    passfilter_pub = nh.advertise<ROSPointCloud>("env/combined/passfilter", 1);

    x_filter_min = priv_nh_.param<float>("x_filter_min", -2.5);
    x_filter_max = priv_nh_.param<float>("x_filter_max", 2.5);
    y_filter_min = priv_nh_.param<float>("y_filter_min", -2.5);
    y_filter_max = priv_nh_.param<float>("y_filter_max", 2.5);
    z_filter_min = priv_nh_.param<float>("z_filter_min", -2.5);
    z_filter_max = priv_nh_.param<float>("z_filter_max", 2.5);
  }
  else
  {
    enable_passfilter = false;
  }

  save_pcd = priv_nh_.param<bool>("save_pcd", false);
  save_interval = priv_nh_.param<float>("save_interval", 2.0);
  auto save_path = priv_nh_.param<std::string>("save_path", "output");

  save_path_full = ros::package::getPath("perception_pipeline") + "/" + save_path;

  // Transform listner
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // Get the transformation for each pointcloud
  try
  {
    transformStamped1 = tfBuffer.lookupTransform(world_frame, sensor1_frame, ros::Time(0), ros::Duration(5));
    transformStamped2 = tfBuffer.lookupTransform(world_frame, sensor2_frame, ros::Time(0), ros::Duration(5));
    transformStamped3 = tfBuffer.lookupTransform(world_frame, sensor3_frame, ros::Time(0), ros::Duration(5));
    transformStamped4 = tfBuffer.lookupTransform(world_frame, sensor4_frame, ros::Time(0), ros::Duration(5));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_ERROR("Node: perception_node:: Trasnformations lookup failed.");
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  ROS_INFO("Node: perception_node:: Trasnformations lookup was successful.");

  ros::Subscriber quadcloud_sub = nh.subscribe("env/syncedClouds", 1, quadCloudCallback);

  ros::spin();

  return 0;
}
