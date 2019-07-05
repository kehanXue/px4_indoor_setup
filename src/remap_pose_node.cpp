//
// Created by kehan on 19-7-5.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>

mavros_msgs::State px4_current_state;
nav_msgs::Odometry rs_current_odom;

void px4_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    px4_current_state = *msg;
}

void rs_odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    rs_current_odom = *msg;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remap_pose_node");
    ros::NodeHandle nh("~");

    ros::Subscriber px4_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, px4_state_cb);
    ros::Subscriber rs_odom_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1, rs_odom_cb);
    ros::Publisher  vision_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov", 1);

    ros::Rate loop_rate(50);


    while (ros::ok() && !px4_current_state.connected)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    geometry_msgs::PoseWithCovarianceStamped cur_pose_cov;
    while (ros::ok())
    {
        cur_pose_cov.header.frame_id = "camera_pose_frame";
        cur_pose_cov.header.stamp = ros::Time::now();
        cur_pose_cov.pose = rs_current_odom.pose;
        vision_pose_pub.publish(cur_pose_cov);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}
