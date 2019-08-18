//
// Created by kehan on 19-7-5.
//

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>

mavros_msgs::State px4_current_state;

nav_msgs::Odometry vision_current_odom;

void px4_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    px4_current_state = *msg;
}


void vision_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    vision_current_odom = *msg;
}

int main(int argc, char** argv)
{

    sleep(15);

    ros::init(argc, argv, "remap_pose_node");
    ros::NodeHandle nh("~");

    const std::string &node_name = ros::this_node::getName();
    std::string odom_topic_name = "/odom/example";


    if (nh.hasParam("odom_topic_name"))
    {
        nh.getParam("odom_topic_name", odom_topic_name);
        ROS_INFO("%s, use odom_topic_name %s", node_name.c_str(), odom_topic_name.c_str());
    }
    else
    {
        ROS_WARN("%s, use the default model %s", node_name.c_str(), odom_topic_name.c_str());
    }


    ros::Subscriber px4_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, px4_state_cb);
    ros::Subscriber vision_odom_sub = nh.subscribe<nav_msgs::Odometry>(odom_topic_name, 1, vision_odom_cb);
    ros::Publisher  vision_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov", 1);

    ros::Rate loop_rate(45);


    while (ros::ok() && !px4_current_state.connected)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    geometry_msgs::PoseWithCovarianceStamped cur_pose_cov;

    std::cout << "\033[32m" << "Start vision pose information message!"
              << "\033[0m" << std::endl;

    while (ros::ok())
    {
        cur_pose_cov.header.frame_id = vision_current_odom.header.frame_id;
        cur_pose_cov.header.stamp = ros::Time::now();
        cur_pose_cov.pose = vision_current_odom.pose;
        vision_pose_pub.publish(cur_pose_cov);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}

