#include "patrol_interfaces/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {
    service_ = this->create_service<patrol_interfaces::srv::GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::handleDirectionRequest, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handleDirectionRequest(
      const std::shared_ptr<patrol_interfaces::srv::GetDirection::Request> req,
      std::shared_ptr<patrol_interfaces::srv::GetDirection::Response> res) {

    const sensor_msgs::msg::LaserScan &scan = req->laser_data;

    std::string direction = analyzeLaserData(scan);

    res->direction = direction;
  }

  std::string analyzeLaserData(const sensor_msgs::msg::LaserScan &scan) {
    double total_dist_sec_right = 0;
    double total_dist_sec_front = 0;
    double total_dist_sec_left = 0;

    for (size_t i = 0; i < scan.ranges.size(); i++) {
      double angle = scan.angle_min + i * scan.angle_increment;

      if (angle >= -M_PI / 2 && angle < -M_PI / 6) {
        total_dist_sec_left += scan.ranges[i];
      } else if (angle >= M_PI / 6 && angle <= M_PI / 2) {
        total_dist_sec_right += scan.ranges[i];
      } else if (angle >= -M_PI / 6 && angle < M_PI / 6) {
        total_dist_sec_front += scan.ranges[i];
      }
    }

    if (total_dist_sec_front >= total_dist_sec_left &&
        total_dist_sec_front >= total_dist_sec_right) {
      return "forward";
    } else if (total_dist_sec_left >= total_dist_sec_front &&
               total_dist_sec_left >= total_dist_sec_right) {
      return "left";
    } else {
      return "right";
    }
  }

  rclcpp::Service<patrol_interfaces::srv::GetDirection>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
