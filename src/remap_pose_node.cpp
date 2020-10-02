//
// Created by kehan on 19-7-5.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>
#include <tf2_eigen/tf2_eigen.h>

#include <gflags/gflags.h>

DEFINE_string(vio_topic_type, "Odometry", "Available types: PoseStamped, PoseWithCovarianceStamped, Odometry");
DEFINE_string(vio_topic_name, "/vio_pose", "VIO pose output. In flu coordinate");
DEFINE_string(mavros_vision_topic_name, "/mavros/vision_pose/pose_cov", "mavros VIO pose interface. In flu coordinate");
DEFINE_string(mavros_state_topic_name, "/mavros/state", "mavros px4 state interface");
DEFINE_double(translation_x, 0., "transform camera to fcu");
DEFINE_double(translation_y, 0., "transform camera to fcu");
DEFINE_double(translation_z, 0., "transform camera to fcu");
DEFINE_double(rotation_r, 0., "transform camera to fcu, rpy format");
DEFINE_double(rotation_p, 0., "transform camera to fcu, rpy format");
DEFINE_double(rotation_y, 0., "transform camera to fcu, rpy format");

ros::Publisher vision_pose_pub;

mavros_msgs::State px4_current_state;

nav_msgs::Odometry current_vio_odom;
geometry_msgs::PoseStamped current_vio_pose;
geometry_msgs::PoseWithCovarianceStamped current_vio_pose_with_cov;

Eigen::Isometry3d T_fcu_camera;

void px4_state_cb(const mavros_msgs::State::ConstPtr &msg) {
  px4_current_state = *msg;
}

void vio_odom_cb(const nav_msgs::Odometry::ConstPtr &msg) {
  current_vio_odom = *msg;

  geometry_msgs::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = ros::Time::now();
  pose_cov.pose = current_vio_odom.pose;

  Eigen::Isometry3d src_pose;
  tf2::fromMsg(pose_cov.pose.pose, src_pose);
  auto target_pose = T_fcu_camera * src_pose;
  pose_cov.pose.pose = tf2::toMsg(target_pose);

  vision_pose_pub.publish(pose_cov);
}

void vio_pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  current_vio_pose = *msg;

  geometry_msgs::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = ros::Time::now();
  pose_cov.pose.pose = current_vio_pose.pose;

  Eigen::Isometry3d src_pose;
  tf2::fromMsg(pose_cov.pose.pose, src_pose);
  auto target_pose = T_fcu_camera * src_pose;
  pose_cov.pose.pose = tf2::toMsg(target_pose);

  vision_pose_pub.publish(pose_cov);
}

void vio_pose_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  current_vio_pose_with_cov = *msg;

  geometry_msgs::PoseWithCovarianceStamped pose_cov;
  pose_cov.header.stamp = ros::Time::now();
  pose_cov.pose = current_vio_pose_with_cov.pose;

  Eigen::Isometry3d src_pose;
  tf2::fromMsg(pose_cov.pose.pose, src_pose);
  auto target_pose = T_fcu_camera * src_pose;
  pose_cov.pose.pose = tf2::toMsg(target_pose);

  vision_pose_pub.publish(pose_cov);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "remap_pose_node");
  ros::NodeHandle nh("~");

  google::ParseCommandLineFlags(&argc, &argv, true);

  // Transform initial
  T_fcu_camera = Eigen::Isometry3d::Identity();

  Eigen::AngleAxisd rollAngle(FLAGS_rotation_r, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(FLAGS_rotation_p, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(FLAGS_rotation_y, Eigen::Vector3d::UnitZ());
  Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

  // ASSERT
  auto tmp_rpy = q.toRotationMatrix().eulerAngles(0, 1, 2);
  BOOST_ASSERT(tmp_rpy.x() == FLAGS_rotation_r);
  BOOST_ASSERT(tmp_rpy.y() == FLAGS_rotation_p);
  BOOST_ASSERT(tmp_rpy.z() == FLAGS_rotation_y);

  T_fcu_camera.prerotate(q);
  T_fcu_camera.pretranslate(Eigen::Vector3d(FLAGS_translation_x, FLAGS_translation_y, FLAGS_translation_z));


  // ROS Publisher and Subscriber
  ros::Subscriber px4_state_sub = nh.subscribe<mavros_msgs::State>(FLAGS_mavros_state_topic_name, 1, px4_state_cb);
  vision_pose_pub =
      nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(FLAGS_mavros_vision_topic_name, 1);
  ros::Subscriber vio_data_sub;
  if (FLAGS_vio_topic_type == "PoseStamped") {
    vio_data_sub = nh.subscribe<geometry_msgs::PoseStamped>(FLAGS_vio_topic_name, 1, vio_pose_cb);
  } else if (FLAGS_vio_topic_type == "PoseWithCovarianceStamped") {
    vio_data_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(FLAGS_vio_topic_name, 1, vio_pose_cov_cb);
  } else if (FLAGS_vio_topic_type == "Odometry") {
    vio_data_sub = nh.subscribe<nav_msgs::Odometry>(FLAGS_vio_topic_name, 1, vio_odom_cb);
  }


  // Wait for px4 connect
  ros::Rate loop_rate(10);
  while (ros::ok() && !px4_current_state.connected) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "\033[32m" << "Start vision pose information message!"
            << "\033[0m" << std::endl;


  // spin
  ros::spin();
  return 0;
}

