#include <iostream>
#include <cmath>
#include <mutex>
#include <fstream>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class LimoController : public rclcpp::Node {
    public:
        LimoController();

        void set_goal_pose(double x, double y, double theta);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void publish_cmd_vel();
        void linear_traversal(double curr_x, double curr_y, double curr_theta, geometry_msgs::msg::Twist &cmd_vel_msg);
        void angular_traversal(double curr_theta, double desired_theta, geometry_msgs::msg::Twist &cmd_vel_msg);

    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
        rclcpp::TimerBase::SharedPtr timer_;

        geometry_msgs::msg::Pose goal_pose_;
        std::mutex msg_mutex_;

        nav_msgs::msg::Odometry::SharedPtr msg_;
        double linear_integral_ = 0;
        double angular_integral_ = 0;

        double k_p_linear_ = 0.06;
        double k_i_linear_ = 0.004;
        double k_p_angular_ = 0.1;
        double k_i_angular_ = 0.5;

        double angular_offset_ = 0.3;
        int state_ = 0;
};