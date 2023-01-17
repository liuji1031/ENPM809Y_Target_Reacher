# include<chrono>
# include<memory>
# include<functional>

# include "rclcpp/rclcpp.hpp"
# include "geometry_msgs/msg/transform_stamped.hpp"
# include "tf2_ros/transform_broadcaster.h"
# include "nav_msgs/msg/odometry.hpp"

class OdomUpdater : public rclcpp::Node
{   public: 
        
    /**
     * @brief Construct a new Odom Updater object 
     * 
     */
    OdomUpdater(): Node("odom_updater")
    {
        // initialize broadcaster
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // m_timer = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&OdomUpdater::timer_callback,this));
        m_odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/robot1/odom",10,std::bind(&OdomUpdater::sub_callback,this,std::placeholders::_1));
    }

    private:
    /**
     * @brief the subscriber callback listens to the robot1/odom topic
     * and publish the transformation between robot1/odom frame and 
     * robot1/base_footprint on /tf topic
     * 
     * @param msg 
     */
    void sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    /**
     * @brief transformation broadcaster (robot1/odom -> robot1/base_footprint)
     * 
     */
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    /**
     * @brief subscriber to the robot1/odom topic
     * 
     */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr m_odom_sub;
};
