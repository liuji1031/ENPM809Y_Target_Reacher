# include<chrono>
# include<memory>
# include<functional>

# include "rclcpp/rclcpp.hpp"
# include "geometry_msgs/msg/transform_stamped.hpp"
# include "tf2_ros/transform_broadcaster.h"
# include "nav_msgs/msg/odometry.hpp"
# include "odom_updater\odom_updater.h"

void OdomUpdater::sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    // broadcast onto tf topic
    // first create msg
    geometry_msgs::msg::TransformStamped t_msg;
    t_msg.header.stamp = this->get_clock()->now();
    t_msg.header.frame_id = "robot1/odom";
    t_msg.child_frame_id = "robot1/base_footprint";

    // copy translation
    t_msg.transform.translation.x = msg->pose.pose.position.x;
    t_msg.transform.translation.y = msg->pose.pose.position.y;
    t_msg.transform.translation.z = msg->pose.pose.position.z;

    // copy orientation
    t_msg.transform.rotation.x = msg->pose.pose.orientation.x;
    t_msg.transform.rotation.y = msg->pose.pose.orientation.y;
    t_msg.transform.rotation.z = msg->pose.pose.orientation.z;
    t_msg.transform.rotation.w = msg->pose.pose.orientation.w;

    // broadcast transformation
    m_tf_broadcaster->sendTransform(t_msg);
}

