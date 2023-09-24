#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"

class RobotPatrol : public rclcpp::Node {
public:
  RobotPatrol() : Node("robot_patrol"), direction_(0.0) {
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
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int num_rays = (msg->angle_max - msg->angle_min) / msg->angle_increment + 1;

    // Смещаем индексы на четверть вправо и влево
    int start_index = num_rays / 4;
    int end_index = 3 * num_rays / 4;

    float max_distance = msg->range_min;
    int max_distance_index = start_index;

    for (int i = start_index; i < end_index; i++) {
      if (msg->ranges[i] > max_distance && msg->ranges[i] < msg->range_max) {
        max_distance = msg->ranges[i];
        max_distance_index = i;
      }
    }

    RCLCPP_INFO(this->get_logger(), "Max distance in ranges: %f", max_distance);

    direction_ = msg->angle_min + max_distance_index * msg->angle_increment;

    RCLCPP_INFO(this->get_logger(), "Safest direction: %f", direction_);

    // Визуализация
    publishVectorMarker(left_ray_marker_pub_, -M_PI_2,
                        msg->ranges[start_index]);
    publishVectorMarker(right_ray_marker_pub_, M_PI_2,
                        msg->ranges[end_index - 1]);
    publishVectorMarker(direction_marker_pub_, direction_, max_distance);
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
    marker_msg->id = marker_id_;
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
    marker_id_++;
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
  float direction_;
  int marker_id_ = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotPatrol>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
