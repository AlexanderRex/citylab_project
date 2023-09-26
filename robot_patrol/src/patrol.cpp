#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

class RobotPatrol : public rclcpp::Node {
public:
  RobotPatrol() : Node("robot_patrol") {
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&RobotPatrol::laserCallback, this, std::placeholders::_1));

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    left_ray_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "/left_ray_marker", 10);
    right_ray_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "/right_ray_marker", 10);
    direction_marker_pub_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "/direction_marker", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&RobotPatrol::publishCmd, this));
  }

private:
  float direction_ = 0.0;
  const float safety_distance = 0.35; // Distance to obstacle

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<std::pair<size_t, size_t>> gaps;
    bool in_gap = false;
    size_t start_index = 0;

    for (size_t i = 0; i < msg->ranges.size(); i++) {
      if (msg->ranges[i] > safety_distance && !in_gap) {
        in_gap = true;
        start_index = i;
      } else if ((msg->ranges[i] <= safety_distance ||
                  i == msg->ranges.size() - 1) &&
                 in_gap) {
        in_gap = false;
        gaps.push_back({start_index, i - 1});
      }
    }

    size_t max_gap_length = 0;
    std::pair<size_t, size_t> max_gap = {0, 0};
    for (const auto &gap : gaps) {
      size_t gap_length = gap.second - gap.first;
      if (gap_length > max_gap_length) {
        max_gap_length = gap_length;
        max_gap = gap;
      }
    }

    size_t center_index = (max_gap.first + max_gap.second) / 2;
    direction_ = msg->angle_min + center_index * msg->angle_increment;

    publishVectorMarker(left_ray_marker_pub_,
                        msg->angle_min + max_gap.first * msg->angle_increment,
                        msg->ranges[max_gap.first]);
    publishVectorMarker(right_ray_marker_pub_,
                        msg->angle_min + max_gap.second * msg->angle_increment,
                        msg->ranges[max_gap.second]);
    publishVectorMarker(direction_marker_pub_, direction_,
                        msg->ranges[center_index]);
  }

  void publishCmd() {
    geometry_msgs::msg::Twist cmd_msg;
    cmd_msg.linear.x = 0.1;
    cmd_msg.angular.z = direction_ / 2.0;
    cmd_vel_pub_->publish(cmd_msg);
  }

  void publishVectorMarker(
      rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub,
      float angle, float length) {
    auto marker_msg = std::make_shared<visualization_msgs::msg::Marker>();
    marker_msg->header.frame_id = "base_link";
    marker_msg->header.stamp = this->now();
    marker_msg->id = marker_id_++;
    marker_msg->type = visualization_msgs::msg::Marker::ARROW;
    marker_msg->action = visualization_msgs::msg::Marker::ADD;

    marker_msg->points.resize(2);
    marker_msg->points[0].x = 0;
    marker_msg->points[0].y = 0;
    marker_msg->points[1].x = length * cos(angle);
    marker_msg->points[1].y = length * sin(angle);

    marker_msg->scale.x = 0.02;
    marker_msg->scale.y = 0.05;
    marker_msg->scale.z = 0.1;
    marker_msg->color.r = 1.0;
    marker_msg->color.g = 0.0;
    marker_msg->color.b = 0.0;
    marker_msg->color.a = 1.0;
    marker_msg->lifetime = rclcpp::Duration(1.0);

    marker_pub->publish(*marker_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      left_ray_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      right_ray_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      direction_marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int marker_id_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotPatrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
