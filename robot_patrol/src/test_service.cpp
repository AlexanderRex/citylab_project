#include "patrol_interfaces/srv/get_direction.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ServiceClientNode : public rclcpp::Node {
public:
  ServiceClientNode() : Node("service_client_node") {
    // Создаем клиента для сервиса /direction_service
    client_ = this->create_client<patrol_interfaces::srv::GetDirection>(
        "/direction_service");

    // Подписываемся на топик /scan для получения данных
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
          // Когда приходят данные из топика /scan, вызываем функцию для
          // отправки запроса
          sendServiceRequest(msg);
        });
  }

private:
  // Функция для отправки запроса к сервису
  void sendServiceRequest(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Создаем запрос к сервису
    auto request =
        std::make_shared<patrol_interfaces::srv::GetDirection::Request>();
    request->laser_data = *msg;

    // Ожидаем, пока сервис станет доступным
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for the service to appear...");
    }

    // Отправляем запрос к сервису и ожидаем ответ асинхронно
    auto result_future = client_->async_send_request(
        request,
        [this](
            rclcpp::Client<patrol_interfaces::srv::GetDirection>::SharedFuture
                future) {
          if (future.valid()) {
            auto response = future.get();
            if (!response->direction.empty()) {
              RCLCPP_INFO(this->get_logger(), "Direction received: %s",
                          response->direction.c_str());
              rclcpp::shutdown();
            } else {
              RCLCPP_ERROR(this->get_logger(),
                           "Empty direction received from the service.");
              rclcpp::shutdown();
            }
          } else {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
            rclcpp::shutdown();
          }
        });
  }

  rclcpp::Client<patrol_interfaces::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Создаем ноду и добавляем ее в executor
  auto node = std::make_shared<ServiceClientNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
