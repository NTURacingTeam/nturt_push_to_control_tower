// std include
#include <memory>

// ros2 include
#include "rclcpp/rclcpp.hpp"

// nturt include
#include "nturt_push_to_control_tower/push_to_control_tower.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  rclcpp::NodeOptions options;

  rclcpp::Node::SharedPtr push_to_control_tower_node =
      std::make_shared<PushToControlTower>(options);

  executor.add_node(push_to_control_tower_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
