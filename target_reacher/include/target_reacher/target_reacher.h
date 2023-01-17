#pragma once

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>


class TargetReacher : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Target Reacher object
     * @details declares node parameters and initialize member subscribers,
     * publishers and timers
     * 
     * @param bot_controller the reference to the constant shared pointer of 
     * the bot controller
     */
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher"),
    m_marker_reached{0},m_final_des_found{0},m_final_des_reached{0},
    m_final_des_x{0.0},m_final_des_y{0.0},m_curr_goal_x{0.0},m_curr_goal_y{0.0}
    {
        // initialize bot controller
        m_bot_controller = bot_controller;

        // declare node parameters
        this->declare_parameter<double>("aruco_target.x");
        this->declare_parameter<double>("aruco_target.y");
        this->declare_parameter<std::string>("final_destination.frame_id");
        this->declare_parameter<double>("final_destination.aruco_0.x");
        this->declare_parameter<double>("final_destination.aruco_0.y");
        this->declare_parameter<double>("final_destination.aruco_1.x");
        this->declare_parameter<double>("final_destination.aruco_1.y");
        this->declare_parameter<double>("final_destination.aruco_2.x");
        this->declare_parameter<double>("final_destination.aruco_2.y");
        this->declare_parameter<double>("final_destination.aruco_3.x");
        this->declare_parameter<double>("final_destination.aruco_3.y");

        // initialize subscribers and publishers
        m_goal_reach_sub = this->create_subscription<std_msgs::msg::Bool>("/goal_reached",10,
            std::bind(&TargetReacher::goal_callback,this,std::placeholders::_1));
        m_marker_sub = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("/aruco_markers",10,
            std::bind(&TargetReacher::marker_callback,this,std::placeholders::_1));
        m_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/robot1/cmd_vel",1);

        // initialize tf listeners and broadcasters
        m_currloc_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_currloc_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_currloc_tf_buffer);
        m_des_tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        m_des_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_des_tf_buffer);
        m_stat_broadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // initialize timers
        m_timer_move_bot = this->create_wall_timer(std::chrono::milliseconds(500),
            std::bind(&TargetReacher::timer_move_bot_callback,this));
    }


private:
    // attributes
    /**
     * @brief m_marker_reached equals 1 if the bot has reached
     * the location to scan the aruco marker. equals 0 if otherwise
     */
    int m_marker_reached;
    /**
     * @brief m_final_des_found equals 1 if the the aruco marker is successfully
     * scanned and the final destination coordinate retrieved. equals 0 otherwise
     */
    int m_final_des_found;
    /**
     * @brief m_final_des_reached equals 1 if the bot has reached the final
     * destination. equals 0 if otherwise
     */
    int m_final_des_reached;
    /**
     * @brief m_final_des_x stores the x coordinate of the final destination
     * in the fixed (odom) frame
     */
    double m_final_des_x;
    /**
     * @brief m_final_des_y stores the y coordinate of the final destination
     * in the fixed (odom) frame
     */
    double m_final_des_y;
    /**
     * @brief m_curr_goal_x stores the current target x coordinate for the bot controller's
     * set_goal() method
     */
    double m_curr_goal_x;
    /**
     * @brief m_curr_goal_y stores the current target y coordinate for the bot controller's
     * set_goal() method
     */
    double m_curr_goal_y;
    /**
     * @brief pointer to the bot controller
     */
    std::shared_ptr<BotController> m_bot_controller;
    /**
     * @brief subscriber to the /goal_reached topic
     */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_goal_reach_sub;
    /**
     * @brief publisher that publishes bot velocity to /robot1/cmd_vel
     * eg. make bot spin in place
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_vel_pub;
    /**
     * @brief subscriber to the /aruco_markers topic
     * 
     */
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr m_marker_sub;
    /**
     * @brief tf buffer to look up the current location of the bot (robot1/odom->robot1/base_footprint)
     */
    std::unique_ptr<tf2_ros::Buffer> m_currloc_tf_buffer;
    /**
     * @brief tf listener for the current location of the bot (robot1/odom->robot1/base_footprint)
     */
    std::shared_ptr<tf2_ros::TransformListener> m_currloc_tf_listener;
    /**
     * @brief tf buffer to look up robot1/odom -> final_destination
     */
    std::unique_ptr<tf2_ros::Buffer> m_des_tf_buffer;
    /**
     * @brief tf listener to look up robot1/odom -> final_destination
     */
    std::shared_ptr<tf2_ros::TransformListener> m_des_tf_listener;
    /**
     * @brief static tf broadcaster that broadcast the final destination frame under
     * the origin frame as specified in the node param (final_destination.frame_id)
     */
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> m_stat_broadcaster;
    /**
     * @brief timer to drive the bot to goals
     */
    rclcpp::TimerBase::SharedPtr m_timer_move_bot;
    
    // private member functions
    /**
     * @brief callback for arriving at goal positions. 
     * @details when arriving at goal #1 (aruco marker), the callback makes the bot
     * spin in place. when arriving at goal #2 (final destination), simply log and print
     * message to screen.
     * 
     * @param msg msg->data true for arriving at goal
     */
    void goal_callback(const std_msgs::msg::Bool::SharedPtr msg);
    /**
     * @brief callback for retrieving marker information
     * @details the callback uses marker_id to determine the final destination location
     * and broadcast this frame under the correct origin frame
     * @param msg 
     */
    void marker_callback(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief timer callback for driving the bot to goal 1 and goal 2 location
     */
    void timer_move_bot_callback();


};