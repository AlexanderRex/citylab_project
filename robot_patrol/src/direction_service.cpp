#include "geometry_msgs/msg/twist.hpp"
#include "patrol_interfaces/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {
    service_ = this->create_service<patrol_interfaces::srv::GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::handleDirectionRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "sector_boundaries", 10);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&DirectionService::visualizeSectorBoundaries, this));
  }

private:
  rclcpp::Service<patrol_interfaces::srv::GetDirection>::SharedPtr service_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

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
      double weight = 1 / (scan.ranges[i] + 0.01);

      if (angle >= -M_PI / 2 && angle < -M_PI / 6) {
        total_dist_sec_left += weight * scan.ranges[i];
      } else if (angle >= M_PI / 6 && angle <= M_PI / 2) {
        total_dist_sec_right += weight * scan.ranges[i];
      } else if (angle >= -M_PI / 6 && angle < M_PI / 6) {
        total_dist_sec_front += weight * scan.ranges[i];
      }
    }

    if (total_dist_sec_front >= total_dist_sec_left &&
        total_dist_sec_front >= total_dist_sec_right) {
      return "front";
    } else if (total_dist_sec_left > total_dist_sec_right) {
      return "left";
    } else {
      return "right";
    }
  }

  void visualizeSectorBoundaries() {
    // Left sector
    visualization_msgs::msg::Marker left_sector =
        createSectorMarker(0, -M_PI / 2, -M_PI / 6, 1.0, 0.0, 0.0);
    marker_pub_->publish(left_sector);

    // Front sector
    visualization_msgs::msg::Marker front_sector =
        createSectorMarker(1, -M_PI / 6, M_PI / 6, 0.0, 1.0, 0.0);
    marker_pub_->publish(front_sector);

    // Right sector
    visualization_msgs::msg::Marker right_sector =
        createSectorMarker(2, M_PI / 6, M_PI / 2, 0.0, 0.0, 1.0);
    marker_pub_->publish(right_sector);
  }

  visualization_msgs::msg::Marker createSectorMarker(int id, double start_angle,
                                                     double end_angle, float r,
                                                     float g, float b) {
    visualization_msgs::msg::Marker sector;
    sector.header.frame_id = "base_link";
    sector.ns = "sectors";
    sector.id = id;
    sector.type = visualization_msgs::msg::Marker::LINE_STRIP;
    sector.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point start_point;
    start_point.x = std::cos(start_angle);
    start_point.y = std::sin(start_angle);

    geometry_msgs::msg::Point end_point;
    end_point.x = std::cos(end_angle);
    end_point.y = std::sin(end_angle);

    sector.points.push_back(start_point);
    sector.points.push_back(geometry_msgs::msg::Point());
    sector.points.push_back(end_point);

    sector.scale.x = 0.1;
    sector.color.a = 1.0;
    sector.color.r = r;
    sector.color.g = g;
    sector.color.b = b;

    return sector;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
