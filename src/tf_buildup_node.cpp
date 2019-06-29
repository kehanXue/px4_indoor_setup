/*
 * Author Kehan Xue
 * Date 2019/06/29
 */


#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/State.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <cmath>
#include <deque>


mavros_msgs::State px4_current_state;
std::deque<double_t> que_yaw_data;


void px4_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    px4_current_state = *msg;
}


void px4_local_enu_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    double_t q0 = msg->pose.orientation.w;
    double_t q1 = msg->pose.orientation.x;
    double_t q2 = msg->pose.orientation.y;
    double_t q3 = msg->pose.orientation.z;

    double_t current_yaw = std::atan2( 2.0 * (q0*q3 + q1*q2), 1.0-2.0*(q2*q2 + q3*q3) );
    que_yaw_data.push_back(current_yaw);
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "tf_buildup_node");
    ros::NodeHandle nh("~");

    ros::Subscriber px4_state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, px4_state_cb);
    ros::Subscriber px4_local_enu_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 1, px4_local_enu_pose_cb);



    std::string px4_enu_frame("px4_local_origin_enu");
    std::string vo_flu_frame("camera_odom_frame");

    if (nh.hasParam("px4_enu_frame"))
    {
        nh.getParam("px4_enu_frame", px4_enu_frame);
        ROS_INFO("Use px4_enu_frame %s", px4_enu_frame.c_str());
    }
    else
    {
        ROS_WARN("Use the default px4_enu_frame: %s", px4_enu_frame.c_str());
    }

    if (nh.hasParam("vo_flu_frame"))
    {
        nh.getParam("vo_flu_frame", vo_flu_frame);
        ROS_INFO("Use vo_flu_frame %s", vo_flu_frame.c_str());
    }
    else
    {
        ROS_WARN("Use the default vo_flu_frame: %s", vo_flu_frame.c_str());
    }


    ros::Rate loop_rate(30);
    while (ros::ok() && !px4_current_state.connected)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::Time collect_yaw_data_begin_time = ros::Time::now();
    while (que_yaw_data.size() < 150)
    {
        ros::spinOnce();
        if ( (ros::Time::now() - collect_yaw_data_begin_time) > ros::Duration(7.0))
        {
            ROS_ERROR("Collect yaw data time out!");
            return -1;
        }

        loop_rate.sleep();
    }

    std::sort(que_yaw_data.begin(), que_yaw_data.end());
    double_t yaw_diff = que_yaw_data.at(que_yaw_data.size()/2);

    ROS_ERROR("Select yaw: %f", yaw_diff);



    tf2_ros::StaticTransformBroadcaster static_tf2_broadcaster;
    geometry_msgs::TransformStamped static_tf_enu_flu;
    static_tf_enu_flu.header.stamp = ros::Time::now();
    static_tf_enu_flu.header.frame_id = px4_enu_frame;
    static_tf_enu_flu.child_frame_id = vo_flu_frame;

    static_tf_enu_flu.transform.translation.x = 0;
    static_tf_enu_flu.transform.translation.y = 0;
    static_tf_enu_flu.transform.translation.z = 0;


    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw_diff);
    quaternion.normalize();
    static_tf_enu_flu.transform.rotation.x = quaternion.x();
    static_tf_enu_flu.transform.rotation.y = quaternion.y();
    static_tf_enu_flu.transform.rotation.z = quaternion.z();
    static_tf_enu_flu.transform.rotation.w = quaternion.w();

    static_tf2_broadcaster.sendTransform(static_tf_enu_flu);
    ROS_INFO("Init successfully");

    ros::spin();

    return 0;
}

