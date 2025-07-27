#include "minim5robo_bridge/minim5robo_bridge_component.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MiniM5Robo::Minim5RoboBridge>());
  rclcpp::shutdown();
  return 0;
}
