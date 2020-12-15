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
  CloudTransformer(ros::NodeHandle &nh, ros::NodeHandle &priv_nh_)
  {
    tfListener1 = new tf2_ros::TransformListener(tfBuffer1);
    // tfListener2 = new tf2_ros::TransformListener(tfBuffer2);
    // tfListener3 = new tf2_ros::TransformListener(tfBuffer3);
    // tfListener4 = new tf2_ros::TransformListener(tfBuffer4);

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

  void run(ros::NodeHandle &nh)
  {
    while (ros::ok())
    {
      /*
       * LISTEN FOR POINTCLOUD
       */
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

      // set to also pointcloud 3 for testing purposes
      // sensor_msgs::PointCloud2::ConstPtr raw_cloud4 =
      //     ros::topic::waitForMessage<sensor_msgs::PointCloud2>(sensor3topic, nh);

      /*
       * TRANSFORM POINTCLOUD FROM CAMERA FRAME TO WORLD FRAME
       */

      geometry_msgs::TransformStamped transformStamped1, transformStamped2, transformStamped3, transformStamped4,
          transformStamped5;

      // ROS_INFO_STREAM("Cloud service called; waiting for a transforms on topics env/transform/sensor1,2,3 " << std::endl);

      // geometry_msgs::TransformStamped transformStamped1 =
      //     *(ros::topic::waitForMessage<geometry_msgs::TransformStamped>("env/transform/sensor1", nh));

      // geometry_msgs::TransformStamped transformStamped2 =
      //     *(ros::topic::waitForMessage<geometry_msgs::TransformStamped>("env/transform/sensor2", nh));

      // geometry_msgs::TransformStamped transformStamped3 =
      //     *(ros::topic::waitForMessage<geometry_msgs::TransformStamped>("env/transform/sensor3", nh));

      // geometry_msgs::TransformStamped transformStamped4 =
      //     *(ros::topic::waitForMessage<geometry_msgs::TransformStamped>("env/transform/sensor4", nh));

      try
      {
        transformStamped1 = tfBuffer1.lookupTransform(world_frame, sensor1_frame, ros::Time(0), ros::Duration(1));
        transformStamped2 = tfBuffer1.lookupTransform(world_frame, sensor2_frame, ros::Time(0), ros::Duration(1));
        // transformStamped3 = tfBuffer1.lookupTransform(world_frame, sensor3_frame, ros::Time(0), ros::Duration(1));
        // transformStamped4 = tfBuffer1.lookupTransform(world_frame, sensor4_frame, ros::Time(0), ros::Duration(1));
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      sensor_msgs::PointCloud2 transformed_cloud1, transformed_cloud2, transformed_cloud3, transformed_cloud4;

      pcl_ros::transformPointCloud(world_frame, transformStamped1.transform, *raw_cloud1, transformed_cloud1);
      pcl_ros::transformPointCloud(world_frame, transformStamped2.transform, *raw_cloud2, transformed_cloud2);
      // pcl_ros::transformPointCloud(world_frame, transformStamped3.transform, *raw_cloud3, transformed_cloud3);
      // pcl_ros::transformPointCloud(world_frame, transformStamped4.transform, *raw_cloud4, transformed_cloud4);

      /*
       * CONVERT POINTCLOUD ROS->PCL
       */
      // pcl::PointCloud<pcl::PointXYZ> cloud;
      // pcl::fromROSMsg(transformed_cloud, cloud);

      /*
       * CONVERT POINTCLOUD PCL->ROS
       */
      // sensor_msgs::PointCloud2::Ptr pc2_cloud(new sensor_msgs::PointCloud2);
      // pcl::toROSMsg(*prism_filtered_cloud, *pc2_cloud);

      // pc2_cloud->header.frame_id = world_frame;
      // pc2_cloud->header.stamp = ros::Time::now();
      // object_pub.publish(pc2_cloud);

      sensor1_pub.publish(transformed_cloud1);
      sensor2_pub.publish(transformed_cloud2);
      // sensor3_pub.publish(transformed_cloud3);
      // sensor4_pub.publish(transformed_cloud4);
    }
  }

private:
  /*
   * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
   */
  std::string world_frame;
  std::string sensor1_frame, sensor2_frame, sensor3_frame, sensor4_frame;
  std::string sensor1_topic, sensor2_topic, sensor3_topic, sensor4_topic;
  std::array<std::string, 4> sensor_frames, sensor_topics;

  /*
   * SETUP PUBLISHERS
   */
  ros::Publisher sensor1_pub, sensor2_pub, sensor3_pub, sensor4_pub, combined_pub;

  /*
   * Static transform publishers
   */
  tf2_ros::Buffer tfBuffer1;                // tfBuffer2 tfBuffer3, tfBuffer4, tfBufferTest;
  tf2_ros::TransformListener *tfListener1;  // *tfListener2, *tfListener3, *tfListener4, *tfListenerTest;
};

int main(int argc, char *argv[])
{
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "transformpipe_node");

  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  CloudTransformer transformerpipe(nh, priv_nh_);

  ros::Duration(1).sleep();  // wait to initialize

  transformerpipe.run(nh);

  return 0;
}