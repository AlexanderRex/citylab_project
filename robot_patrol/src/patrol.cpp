#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1));
  }

private:
  // Callback to process laser scan data
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    geometry_msgs::msg::Twist twist_msg;
    float safest_distance = msg->ranges[360];
    direction_ = 0;

    // Find the safest direction
    for (int i = 0; i < 720; ++i) {
      if (msg->ranges[i] > safest_distance) {
        safest_distance = msg->ranges[i];
        direction_ = i < 360 ? -(360 - i) : i - 360;
      }
    }

    // Decide motion based on safest direction
    if (safest_distance < 1.0) {
      twist_msg.linear.x = 0.0;
    } else {
      twist_msg.linear.x = 0.1;
      twist_msg.angular.z = direction_ / 2.0;
    }

    publisher_->publish(twist_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  float direction_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}
