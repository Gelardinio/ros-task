#include "limo_controller.hpp"

LimoController::LimoController() : Node("limo_controller") {
    this->set_goal_pose(10, 10, 1.57);

    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&LimoController::odom_callback, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10
    );

    timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&LimoController::publish_cmd_vel, this)
    );
}

void LimoController::set_goal_pose(double x, double y, double theta) {
    goal_pose_.position.x = x;
    goal_pose_.position.y = y;
    goal_pose_.position.z = 0.0;
    goal_pose_.orientation.x = 0.0;
    goal_pose_.orientation.y = 0.0;
    goal_pose_.orientation.z = sin(theta / 2);
    goal_pose_.orientation.w = cos(theta / 2);
}

void LimoController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    msg_mutex_.lock();
    msg_ = msg;
    msg_mutex_.unlock();
}

void LimoController::publish_cmd_vel() {
    if (msg_ == nullptr) {
        publisher_->publish(geometry_msgs::msg::Twist());
        return;
    }

    msg_mutex_. lock();
    double curr_x = msg_->pose.pose.position.x;
    double curr_y = msg_->pose.pose.position.y;
    double curr_theta = tf2::getYaw(msg_->pose.pose.orientation) + angular_offset_;
    msg_mutex_.unlock();

    geometry_msgs::msg::Twist cmd_vel_msg;
    switch(state_) {
        case 0:
            angular_traversal(curr_theta, atan2(goal_pose_.position.y, goal_pose_.position.x), cmd_vel_msg);
            break;
        case 1:
            linear_traversal(curr_x, curr_y, curr_theta, cmd_vel_msg);
            break;
        case 2:
            angular_traversal(curr_theta, tf2::getYaw(goal_pose_.orientation), cmd_vel_msg);
            break; 
        default:
            cmd_vel_msg.linear.x = 0.0;
            cmd_vel_msg.angular.z = 0.0;
            break;
    }
    
    publisher_->publish(cmd_vel_msg);
}

void LimoController::linear_traversal(double curr_x, double curr_y, double curr_theta, geometry_msgs::msg::Twist &cmd_vel_msg) {
    double goal_x = goal_pose_.position.x;
    double goal_y = goal_pose_.position.y;
    double origin_to_goal = sqrt(goal_x * goal_x + goal_y * goal_y);
    double angle_to_goal = atan2(goal_y, goal_x);
    double x_dist_left = origin_to_goal * cos(angle_to_goal - angular_offset_) - curr_x;

    if (x_dist_left > 0.01) {
        linear_integral_ += x_dist_left * 0.02;
        cmd_vel_msg.linear.x = k_p_linear_ * x_dist_left + k_i_linear_ * linear_integral_;
    } else {
        cmd_vel_msg.linear.x = 0.0;
        linear_integral_ = 0.0;
        state_++;
    }

    // std::ofstream file;
    // file.open("./euclidean_log.txt", std::ios::app);
    // if (file.is_open()) {
    //     file << this->now().seconds() << ", " << curr_x / cos(angle_to_goal - angular_offset_) << ", " << origin_to_goal - curr_x / cos(angle_to_goal - angular_offset_) << std::endl;
    //     file.close();
    // }
}

void LimoController::angular_traversal(double curr_theta, double desired_theta, geometry_msgs::msg::Twist &cmd_vel_msg) {
    double error_theta = desired_theta - curr_theta;
    error_theta = std::remainder(error_theta, 2 * M_PI);

    if (fabs(error_theta) > 0.01) {
        angular_integral_ += error_theta * 0.02;
        cmd_vel_msg.angular.z = k_p_angular_ * error_theta + k_i_angular_ * angular_integral_;
    } else {
        cmd_vel_msg.angular.z = 0.0;
        angular_integral_ = 0;
        state_++;
    }

    // std::ofstream angFile;
    // angFile.open("./angular_log.txt", std::ios::app);
    // if (angFile.is_open()) {
    //     angFile << this->now().seconds() << ", " << curr_theta << ", " << error_theta << std::endl;
    // }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LimoController>());
    rclcpp::shutdown();
    
    return 0;
}