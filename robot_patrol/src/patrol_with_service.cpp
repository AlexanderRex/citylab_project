#include "geometry_msgs/msg/twist.hpp"
#include "patrol_interfaces/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class RobotPatrol : public rclcpp::Node {
public:
  RobotPatrol() : Node("robot_patrol") {
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&RobotPatrol::laserCallback, this, std::placeholders::_1));

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    client_ = this->create_client<patrol_interfaces::srv::GetDirection>(
        "/direction_service");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Client<patrol_interfaces::srv::GetDirection>::SharedPtr client_;

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    sendServiceRequest(msg);
  }

  void sendServiceRequest(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto request =
        std::make_shared<patrol_interfaces::srv::GetDirection::Request>();
    request->laser_data = *msg;

    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for the service to appear...");
    }

    auto result_future = client_->async_send_request(
        request,
        [this](
            rclcpp::Client<patrol_interfaces::srv::GetDirection>::SharedFuture
                future) {
          if (future.valid()) {
            auto response = future.get();
            geometry_msgs::msg::Twist cmd_msg;

            if (response->direction == "front") {
              cmd_msg.linear.x = 0.1;
              cmd_msg.angular.z = 0.0;
            } else if (response->direction == "left") {
              cmd_msg.linear.x = 0.1;
              cmd_msg.angular.z = 0.5;
            } else if (response->direction == "right") {
              cmd_msg.linear.x = 0.1;
              cmd_msg.angular.z = -0.5;
            } else {
              RCLCPP_ERROR(this->get_logger(),
                           "Неверное направление получено от сервиса.");
              return;
            }
            cmd_vel_pub_->publish(cmd_msg);
          }
        });
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotPatrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
