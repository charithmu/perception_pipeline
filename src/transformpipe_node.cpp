#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>

#include "pcl_ros/transforms.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class CloudTransformer
{
public:
  CloudTransformer(ros::NodeHandle &nh, ros::NodeHandle &priv_nh_);

  void run(ros::NodeHandle &nh);

  geometry_msgs::TransformStamped identityTransform(std::string parent_frame, std::string child_frame);

private:
  // Parameters
  std::string world_frame;
  std::string sensor1_frame, sensor2_frame, sensor3_frame, sensor4_frame;
  std::string sensor1_topic, sensor2_topic, sensor3_topic, sensor4_topic;
  std::array<std::string, 4> sensor_frames, sensor_topics;

  // Publishers
  ros::Publisher sensor1_pub, sensor2_pub, sensor3_pub, sensor4_pub, combined_pub;

  sensor_msgs::PointCloud2 transformed_cloud1, transformed_cloud2, transformed_cloud3, transformed_cloud4;

  // Transform listner
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener *tfListener;
};

// Constructor
CloudTransformer::CloudTransformer(ros::NodeHandle &nh, ros::NodeHandle &priv_nh_)
{
  tfListener = new tf2_ros::TransformListener(tfBuffer);

  world_frame = priv_nh_.param<std::string>("world_frame", "world_frame");

  sensor1_frame = priv_nh_.param<std::string>("sensor1_frame", "sensor1_frame");
  sensor2_frame = priv_nh_.param<std::string>("sensor2_frame", "sensor2_frame");
  sensor3_frame = priv_nh_.param<std::string>("sensor3_frame", "sensor3_frame");
  sensor4_frame = priv_nh_.param<std::string>("sensor4_frame", "sensor4_frame");

  sensor_frames = { sensor1_frame, sensor2_frame, sensor3_frame, sensor4_frame };

  sensor1_topic = priv_nh_.param<std::string>("sensor1_topic", "env/sensor1/points");
  sensor2_topic = priv_nh_.param<std::string>("sensor2_topic", "env/sensor2/points");
  sensor3_topic = priv_nh_.param<std::string>("sensor3_topic", "env/sensor3/points");
  sensor4_topic = priv_nh_.param<std::string>("sensor4_topic", "env/sensor4/points");

  sensor_topics = { sensor1_topic, sensor2_topic, sensor3_topic, sensor4_topic };

  sensor1_pub = nh.advertise<sensor_msgs::PointCloud2>("env/sensor1/alignedpoints", 1);
  sensor2_pub = nh.advertise<sensor_msgs::PointCloud2>("env/sensor2/alignedpoints", 1);
  sensor3_pub = nh.advertise<sensor_msgs::PointCloud2>("env/sensor3/alignedpoints", 1);
  sensor4_pub = nh.advertise<sensor_msgs::PointCloud2>("env/sensor4/alignedpoints", 1);

  combined_pub = nh.advertise<sensor_msgs::PointCloud2>("env/combinedpoints", 1);
}

// Main program
void CloudTransformer::run(ros::NodeHandle &nh)
{
  while (ros::ok())
  {
    std::string sensor1topic = nh.resolveName(sensor1_topic);
    std::string sensor2topic = nh.resolveName(sensor2_topic);
    std::string sensor3topic = nh.resolveName(sensor3_topic);
    std::string sensor4topic = nh.resolveName(sensor4_topic);

    ROS_INFO_STREAM("Cloud service called; waiting for a PointCloud2 on topics " << std::endl
                                                                                 << sensor1_topic << std::endl
                                                                                 << sensor2_topic << std::endl
                                                                                 << sensor3_topic << std::endl
                                                                                 << sensor4_topic);

    sensor_msgs::PointCloud2::ConstPtr raw_cloud1 =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(sensor1topic, nh);

    sensor_msgs::PointCloud2::ConstPtr raw_cloud2 =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(sensor2topic, nh);

    sensor_msgs::PointCloud2::ConstPtr raw_cloud3 =
        ros::topic::waitForMessage<sensor_msgs::PointCloud2>(sensor3topic, nh);

    geometry_msgs::TransformStamped transformStamped1, transformStamped2, transformStamped3, transformStamped4,
        transformStamped5;

    // geometry_msgs::TransformStamped transformStamped1 =
    //     *(ros::topic::waitForMessage<geometry_msgs::TransformStamped>("env/transform/sensor1", nh));

    try
    {
      transformStamped2 = tfBuffer.lookupTransform(sensor1_frame, sensor2_frame, ros::Time(0), ros::Duration(1));
      transformStamped3 = tfBuffer.lookupTransform(sensor1_frame, sensor3_frame, ros::Time(0), ros::Duration(1));
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    transformStamped1 = identityTransform("sensor1_frame", "sensor1_frame");

    pcl_ros::transformPointCloud(world_frame, transformStamped1.transform, *raw_cloud1, transformed_cloud1);
    pcl_ros::transformPointCloud(world_frame, transformStamped2.transform, *raw_cloud2, transformed_cloud2);
    pcl_ros::transformPointCloud(world_frame, transformStamped3.transform, *raw_cloud3, transformed_cloud3);

    sensor1_pub.publish(transformed_cloud1);
    sensor2_pub.publish(transformed_cloud2);
    sensor3_pub.publish(transformed_cloud3);
  }
}

// Identity-transform creator
geometry_msgs::TransformStamped CloudTransformer::identityTransform(std::string parent_frame, std::string child_frame)
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

// main
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "transformpipe_node");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  CloudTransformer transformerpipe(nh, priv_nh_);

  ros::Duration(1).sleep();  // wait to initialize

  transformerpipe.run(nh);

  return 0;
}