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
        this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
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

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPoseAction::Goal> goal) {
    (void)uuid; // Чтобы избавиться от предупреждения
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with position x: %f, y: %f, theta: %f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    desired_pos_ = goal->goal_pos;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    (void)goal_handle; // Чтобы избавиться от предупреждения
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    // Stop the robot
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(stop_cmd);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    using namespace std::placeholders;
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    auto &current_pos_feedback = feedback->current_pos;
    geometry_msgs::msg::Twist cmd_vel;

    // Set an acceptable error margin to determine if the robot reached the goal
    const double position_error_margin = 0.1; // 10cm
    const double angle_error_margin = 0.1;    // ~5.7 degrees

    // Control loop
    while (rclcpp::ok()) {
      // Compute the difference between current and desired positions
      double diff_x = desired_pos_.x - current_pos_.x;
      double diff_y = desired_pos_.y - current_pos_.y;
      double angle_to_goal = atan2(diff_y, diff_x);

      // Compute the required angular speed
      double angular_speed = angle_to_goal - current_pos_.theta;

      if (goal_handle->is_canceling()) {
        auto result = std::make_shared<GoToPoseAction::Result>();
        result->status = false;
        goal_handle->canceled(
            result); // Notify that the action has been canceled
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        return;
      }

      // Send the control command
      cmd_vel.linear.x = 0.2;
      cmd_vel.angular.z = angular_speed;
      cmd_vel_pub_->publish(cmd_vel);

      // Populate feedback
      current_pos_feedback.x = current_pos_.x;
      current_pos_feedback.y = current_pos_.y;
      current_pos_feedback.theta = current_pos_.theta;
      goal_handle->publish_feedback(feedback);

      // Check if the robot is close enough to the desired position
      if (abs(diff_x) < position_error_margin &&
          abs(diff_y) < position_error_margin &&
          abs(angular_speed) < angle_error_margin) {
        break;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);

    // Notify completion of the action
    auto result = std::make_shared<GoToPoseAction::Result>();
    result->status = true;
    goal_handle->succeed(result); // Передайте результат в succeed()
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
