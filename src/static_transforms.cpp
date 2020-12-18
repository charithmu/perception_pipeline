
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>

geometry_msgs::TransformStamped get_static_transform(std::string parent_frame, std::string child_frame,
                                                     Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &mat);

int main(int argc, char **argv)
{
  // init node
  ros::init(argc, argv, "static_transforms");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  // Setup Parameters
  std::vector<double> trans_world_param, trans12_param, trans13_param, trans14_param;

  if (!priv_nh_.getParam("world_sensor1_transform", trans_world_param))
    ROS_ERROR("Node: static_transforms:: Failed to get parameter sensor12_transform from server.");

  if (!priv_nh_.getParam("sensor1_sensor2_transform", trans12_param))
    ROS_ERROR("Node: static_transforms:: Failed to get parameter sensor12_transform from server.");

  if (!priv_nh_.getParam("sensor1_sensor3_transform", trans13_param))
    ROS_ERROR("Node: static_transforms:: Failed to get parameter sensor13_transform from server.");

  if (!priv_nh_.getParam("sensor1_sensor4_transform", trans14_param))
    ROS_ERROR("Node: static_transforms:: Failed to get parameter sensor14_transform from server.");

  // Populate transform matrices from parameters
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> transworld(trans_world_param.data());
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> trans12(trans12_param.data());
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> trans13(trans13_param.data());
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> trans14(trans14_param.data());

  //std::cout << trans12 << std::endl << trans13 << std::endl << trans14 << std::endl;

  // Define static tf publisher
  static tf2_ros::StaticTransformBroadcaster static_transform_broadcaster;

  geometry_msgs::TransformStamped world_transformStamped, sensor12_transformStamped, sensor13_transformStamped, sensor14_transformStamped;

  // Get tf messages built for all the sensors
  world_transformStamped = get_static_transform("world_frame", "sensor1_frame", transworld);
  sensor12_transformStamped = get_static_transform("sensor1_frame", "sensor2_frame", trans12);
  sensor13_transformStamped = get_static_transform("sensor1_frame", "sensor3_frame", trans13);
  sensor14_transformStamped = get_static_transform("sensor1_frame", "sensor4_frame", trans14);

  // Publish all tf messeges to tf_static
  const std::vector<geometry_msgs::TransformStamped> alltransforms{world_transformStamped,
                                                                   sensor12_transformStamped,
                                                                   sensor13_transformStamped,
                                                                   sensor14_transformStamped};
  static_transform_broadcaster.sendTransform(alltransforms);

  ros::spin();

  return 0;
}

// TF messege builder
geometry_msgs::TransformStamped get_static_transform(std::string parent_frame, std::string child_frame,
                                                     Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &mat)
{
  geometry_msgs::TransformStamped stat_trans;

  stat_trans.header.stamp = ros::Time::now();
  stat_trans.header.frame_id = parent_frame;
  stat_trans.child_frame_id = child_frame;

  stat_trans.transform.translation.x = mat(0, 3);
  stat_trans.transform.translation.y = mat(1, 3);
  stat_trans.transform.translation.z = mat(2, 3);

  Eigen::Matrix3d rot;
  rot = mat.block<3, 3>(0, 0);

  //std::cout << "rot: " << rot << std::endl;

  const Eigen::Quaterniond q(rot);

  stat_trans.transform.rotation.x = q.x();
  stat_trans.transform.rotation.y = q.y();
  stat_trans.transform.rotation.z = q.z();
  stat_trans.transform.rotation.w = q.w();

  //ROS_INFO("Publishing transformation: %s to %s", child_frame.c_str(), parent_frame.c_str());

  return stat_trans;
}