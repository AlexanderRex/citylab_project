#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "patrol_interfaces/action/go_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class GoToPose : public rclcpp::Node {
public:
  using GoToPoseAction = patrol_interfaces::action::GoToPose;
  using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_action_server", options) {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<GoToPoseAction>(
        this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }

private:
  rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;

  double prev_error_ = 0;
  double integral_error_ = 0;
  const double Kp_ = 0.5;
  const double Ki_ = 0.01;
  const double Kd_ = 0.1;
  const double MAX_ANGULAR_SPEED = 0.5; // Limit for angular speed

  // Update robot's position and orientation based on odometry data
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.theta = yaw;
  }

  // Handle incoming goal requests
  rclcpp_action::GoalResponse
  handle_goal(std::shared_ptr<const GoToPoseAction::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with position x: %f, y: %f, theta: %f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    desired_pos_ = goal->goal_pos;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // Handle request to cancel the goal
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> /* goal_handle */) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Handle accepted goals and start execution
  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    std::thread{[this, goal_handle]() { this->execute(goal_handle); }}.detach();
  }

  // Utility function to clamp values
  double clamp(double val, double min_val, double max_val) {
    return std::max(min_val, std::min(val, max_val));
  }

  // Main function to drive the robot towards the goal
  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    geometry_msgs::msg::Twist cmd_vel;

    const double position_error_margin = 0.1;
    const double angle_error_margin = 0.1;
    const double angle_deadzone = 0.05; // A small deadzone for angle

    // Stage 1: Drive towards the target while adjusting the angle
    while (rclcpp::ok()) {
      double diff_x = desired_pos_.x - current_pos_.x;
      double diff_y = desired_pos_.y - current_pos_.y;
      double angle_to_goal = atan2(diff_y, diff_x);
      double error = angle_to_goal - current_pos_.theta;

      // Wrap the error to the range -pi to pi
      while (error > M_PI)
        error -= 2.0 * M_PI;
      while (error < -M_PI)
        error += 2.0 * M_PI;

      // If error is within the deadzone, consider it as zero
      if (abs(error) < angle_deadzone)
        error = 0;

      integral_error_ += error;
      double derivative_error = error - prev_error_;
      double angular_speed =
          Kp_ * error + Ki_ * integral_error_ + Kd_ * derivative_error;
      prev_error_ = error;

      if (abs(diff_x) < position_error_margin &&
          abs(diff_y) < position_error_margin) {
        break;
      }

      cmd_vel.linear.x = 0.2;
      cmd_vel.angular.z =
          clamp(angular_speed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
      cmd_vel_pub_->publish(cmd_vel);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Stage 2: Adjust orientation after reaching the target position
    cmd_vel.linear.x = 0.0;
    while (rclcpp::ok()) {
      double angle_to_goal = desired_pos_.theta;
      double error = angle_to_goal - current_pos_.theta;

      // Wrap the error to the range -pi to pi
      while (error > M_PI)
        error -= 2.0 * M_PI;
      while (error < -M_PI)
        error += 2.0 * M_PI;

      // If error is within the deadzone, consider it as zero
      if (abs(error) < angle_deadzone)
        error = 0;

      if (abs(error) < angle_error_margin) {
        break;
      }

      double angular_speed = Kp_ * error;
      cmd_vel.angular.z =
          clamp(angular_speed, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);
      cmd_vel_pub_->publish(cmd_vel);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Stop the robot after reaching the goal
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
    auto result = std::make_shared<GoToPoseAction::Result>();
    result->status = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal reached");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoToPose>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
