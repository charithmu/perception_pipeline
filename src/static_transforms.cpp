#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <Eigen/Dense>

geometry_msgs::TransformStamped publish_static_transform(std::string child_frame,
                                                         Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &mat);

int main(int argc, char **argv)
{
  /*
   * INITIALIZE ROS NODE
   */
  ros::init(argc, argv, "static_transforms");
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh_("~");

  /*
   * SET UP PARAMETERS (COULD BE INPUT FROM LAUNCH FILE/TERMINAL)
   */
  std::vector<double> trans12_param, trans13_param, trans14_param;

  if (!priv_nh_.getParam("/sensor12_transform", trans12_param))
    ROS_ERROR("Failed to get parameter sensor12_transform from server.");

  if (!priv_nh_.getParam("/sensor13_transform", trans13_param))
    ROS_ERROR("Failed to get parameter sensor13_transform from server.");

  if (!priv_nh_.getParam("/sensor14_transform", trans14_param))
    ROS_ERROR("Failed to get parameter sensor14_transform from server.");

  /*
   * SETUP PUBLISHERS
   */
  // ros::Publisher sensor1_transform, sensor2_transform, sensor3_transform, sensor4_transform;

  // sensor1_transform = nh.advertise<geometry_msgs::TransformStamped>("env/transform/sensor1", 1, true);
  // sensor2_transform = nh.advertise<geometry_msgs::TransformStamped>("env/transform/sensor2", 1, true);
  // sensor3_transform = nh.advertise<geometry_msgs::TransformStamped>("env/transform/sensor3", 1, true);
  // sensor4_transform = nh.advertise<geometry_msgs::TransformStamped>("env/transform/sensor4", 1, true);

  /*
   * populate the transform matrices
   */
  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> trans12(trans12_param.data()), trans13(trans13_param.data()),
      trans14(trans14_param.data());

  std::cout << trans12 << std::endl << trans13 << std::endl << trans14 << std::endl;

  tf2_ros::TransformBroadcaster transform_broadcaster;

  geometry_msgs::TransformStamped sensor12_transformStamped, sensor13_transformStamped, sensor14_transformStamped;

  // ros::Rate loop_rate(10);

  // while (ros::ok())
  // {
  //   sensor1_transform.publish(sensor12_transformStamped);
  //   sensor2_transform.publish(sensor13_transformStamped);
  //   sensor3_transform.publish(sensor14_transformStamped);

  //   loop_rate.sleep();
  // }

  ros::Rate rate(10.0);

  while (ros::ok())
  {
    sensor12_transformStamped = publish_static_transform("sensor2_frame", trans12);
    // sensor13_transformStamped = publish_static_transform("sensor3_frame", trans13);
    // sensor14_transformStamped = publish_static_transform("sensor4_frame", trans14);

    //const std::vector<geometry_msgs::TransformStamped> alltransforms{ sensor12_transformStamped,
                                                                      // sensor13_transformStamped,
                                                                      // sensor14_transformStamped };
    transform_broadcaster.sendTransform(sensor12_transformStamped);
    rate.sleep();
  }

  // ros::spin();

  return 0;
}

geometry_msgs::TransformStamped publish_static_transform(std::string child_frame,
                                                         Eigen::Matrix<double, 4, 4, Eigen::RowMajor> &mat)
{
  geometry_msgs::TransformStamped stat_trans;

  stat_trans.header.stamp = ros::Time::now();
  stat_trans.header.frame_id = "sensor1_frame";
  stat_trans.child_frame_id = child_frame;

  stat_trans.transform.translation.x = mat(0, 3);
  stat_trans.transform.translation.y = mat(1, 3);
  stat_trans.transform.translation.z = mat(2, 3);

  Eigen::Matrix3d rot;
  rot = mat.block<3, 3>(0, 0);

  std::cout << "rot: " << rot << std::endl;

  const Eigen::Quaterniond q(rot);

  stat_trans.transform.rotation.x = q.x();
  stat_trans.transform.rotation.y = q.y();
  stat_trans.transform.rotation.z = q.z();
  stat_trans.transform.rotation.w = q.w();

  ROS_INFO("Publishing transformation: %s to world", child_frame.c_str());

  return stat_trans;
}