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

  auto push_to_control_tower_node =
      std::make_shared<PushToControlTower>(options);
  push_to_control_tower_node->register_can_callback();

  executor.add_node(push_to_control_tower_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
