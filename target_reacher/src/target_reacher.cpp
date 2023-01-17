#include <math.h>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "target_reacher/target_reacher.h"
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/time.h>
#include <tf2/exceptions.h>


void TargetReacher::goal_callback(const std_msgs::msg::Bool::SharedPtr msg){
    if(msg->data==true){ // marker found

        // need to first figure out which goal the bot is at
        geometry_msgs::msg::TransformStamped t;
        t = m_currloc_tf_buffer->lookupTransform("robot1/odom","robot1/base_footprint",
            tf2::TimePointZero);
        
        double curr_x{t.transform.translation.x};
        double curr_y{t.transform.translation.y};

        auto marker_x = this->get_parameter("aruco_target.x").as_double();
        auto marker_y = this->get_parameter("aruco_target.y").as_double();

        // if currently at goal #1 or the marker location
        if(abs(curr_x-marker_x)<0.1 && abs(curr_y-marker_y)<0.1){ 
            // spin in place
            if(m_marker_reached==0)m_marker_reached = 1;
            if(m_final_des_found==0){
                geometry_msgs::msg::Twist vel_msg;
                vel_msg.angular.z = 0.75;
                m_vel_pub->publish(vel_msg);
                RCLCPP_INFO(this->get_logger(),"Spinning in place");
            }
            return;
        }

        // if at the final destination
        if(abs(curr_x-m_final_des_x)<0.1 && abs(curr_y-m_final_des_y)<0.1){ 
            // reached final destination
            m_final_des_reached = 1;
            RCLCPP_INFO(this->get_logger(),"Final destination reached!");
            return;
        }        
    }
}

void TargetReacher::marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
    // check which marker was read
    int marker_id = static_cast<int>(msg->marker_ids.at(0));
    if(m_final_des_found==0)m_bot_controller->stop();
    if(m_final_des_found==1)return;
    // get relative coordinates according to marker_id
    std::string s{"final_destination.aruco_"};

    RCLCPP_INFO(this->get_logger(),"Marker %d found!",marker_id);
    m_final_des_found = 1;

    // publish the static frame final_destination under originx
    geometry_msgs::msg::TransformStamped t_msg;
    t_msg.header.stamp = this->get_clock()->now();
    t_msg.header.frame_id = this->get_parameter("final_destination.frame_id").as_string();
    t_msg.child_frame_id = "final_destination";

    // translation
    double x = this->get_parameter(s+std::to_string(marker_id)+".x").as_double();
    double y = this->get_parameter(s+std::to_string(marker_id)+".y").as_double();
    t_msg.transform.translation.x = x;
    t_msg.transform.translation.y = y;
    t_msg.transform.translation.z = 0;
    RCLCPP_INFO(this->get_logger(),"x %f, y %f in origin frame",x,y);
    // orientation
    t_msg.transform.rotation.x = 0;
    t_msg.transform.rotation.y = 0;
    t_msg.transform.rotation.z = 0;
    t_msg.transform.rotation.w = 1;
    m_stat_broadcaster->sendTransform(t_msg);

}

void TargetReacher::timer_move_bot_callback(){
    if(m_marker_reached==0){ // has not reached aruco marker
        double x{this->get_parameter("aruco_target.x").as_double()};
        double y{this->get_parameter("aruco_target.y").as_double()};
        if(x!=m_curr_goal_x || y!=m_curr_goal_y)
        {   
            m_curr_goal_x = x;
            m_curr_goal_y = y;
            // set the marker location as the goal
            m_bot_controller->set_goal(m_curr_goal_x,m_curr_goal_y);
        }
        
    }
    else{ // has reached aruco marker
        if(m_final_des_found==1 && m_final_des_reached!=1){
            // get the frame location from /tf_static
            geometry_msgs::msg::TransformStamped t;
            try{
                t = m_des_tf_buffer->lookupTransform("robot1/odom","final_destination",tf2::TimePointZero);
            } catch(const tf2::TransformException &ex){
                RCLCPP_INFO(this->get_logger(),"final_destination not found");
                return;
            }
            // command bot to the final destination
            m_final_des_x = t.transform.translation.x;
            m_final_des_y = t.transform.translation.y;
            //m_bot_controller->set_goal(m_final_des_x,m_final_des_y);
            if(m_final_des_x!=m_curr_goal_x||m_final_des_y!=m_curr_goal_y){
                m_curr_goal_x = m_final_des_x;
                m_curr_goal_y = m_final_des_y;
                // set the marker location as the goal
                m_bot_controller->set_goal(m_curr_goal_x,m_curr_goal_y);
            }
        }
    }
}

