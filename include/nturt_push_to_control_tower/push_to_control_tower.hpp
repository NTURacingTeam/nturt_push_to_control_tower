/**
 * @file nturt_push_to_control_tower_core.hpp
 * @author Jack b10502016@ntu.edu.tw
 * @brief ROS2 package to get data from CAN sensor and GPS sensor, then push
 * then to the sever. The server package repo is
 * [here](https://github.com/NTURacingTeam/ROS_server)
 */

#ifndef NTURT_PUSH_TO_CONTROL_TOWER_HPP
#define NTURT_PUSH_TO_CONTROL_TOWER_HPP

// glibc include
#include <stdint.h>

// std include
#include <array>
#include <memory>
#include <sstream>
#include <string>

// boost include
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

// ros2 include
#include <rclcpp/rclcpp.hpp>

// ros2 message include
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

// nturt include
#include "nturt_can_config.h"
#include "nturt_can_config/battery_utils.hpp"
#include "nturt_can_config_logger-binutil.h"
#include "nturt_ros_interface/msg/system_stats.hpp"

namespace http = boost::beast::http;
namespace websocket = boost::beast::websocket;
namespace net = boost::asio;
using tcp = boost::asio::ip::tcp;

/**
 * @author Jack b10502016@ntu.edu.tw
 * @brief Class for sending data to remote server.
 */
class PushToControlTower : public rclcpp::Node {
 public:
  /// @brief Constructor of PushToControlTower.
  PushToControlTower(rclcpp::NodeOptions options);

  /// @brief Register codedbc callback function for can signal.
  void register_can_callback();

 private:
  /// @brief Callback function when receiving message from "/from_can_bus".
  void onCan(const std::shared_ptr<can_msgs::msg::Frame> msg);

  /// @brief Callback function when receiving message from "/fix".
  void onGpsFix(const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg);

  /// @brief Callback function when receiving message from "/vel".
  void onGpsVel(const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg);

  /// @brief Callback function when receiving message from "/system_stats".
  void onSystemStats(
      const std::shared_ptr<nturt_ros_interface::msg::SystemStats> msg);

  /// @brief Timed callback function for periodically checking can receive
  /// timeout.
  void check_can_timer_callback();

  /// @brief Timed callback function for sending fast data to control tower.
  void send_fast_data_timer_callback();

  /// @brief Timed callback function for sending slow data to control tower.
  void send_slow_data_timer_callback();

  /// @brief Timed callback function for checking websockt connection is
  /// established. If not, try to reconnect.
  void check_ws_connection_timer_callback();

  /// @brief Function for connecting to websoccket.
  int connect_to_ws();

  /* coder dbc callback function ---------------------------------------------*/
  uint32_t get_tick();

  /// @brief ROS2 sbscriber to "/from_can_bus", for receiving can signal.
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;

  /// @brief ROS2 sbscriber to "/fix", for receiving GPS signal.
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_sub_;

  /// @brief ROS2 sbscriber to "/vel", for receiving GPS signal.
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      gps_vel_sub_;

  /// @brief ROS2 sbscriber to "/system_stats", for receiving system stats
  /// information.
  rclcpp::Subscription<nturt_ros_interface::msg::SystemStats>::SharedPtr
      system_stats_sub_;

  /// @brief ROS2 timer for periodically checking can receive timeout.
  rclcpp::TimerBase::SharedPtr check_can_timer_;

  /// @brief ROS2 timer for sending fast data to control tower.
  rclcpp::TimerBase::SharedPtr send_fast_data_timer_;

  /// @brief ROS2 timer for sending slow data to control tower.
  rclcpp::TimerBase::SharedPtr send_slow_data_timer_;

  /// @brief ROS2 timer for reconnecting to websocket if disconnected.
  rclcpp::TimerBase::SharedPtr check_ws_connection_timer_;

  /// @brief Io_context for resolver_ and ws_. See detail in
  /// [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
  net::io_context ioc_;

  /// @brief Resolver. See detail in
  /// [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
  tcp::resolver resolver_{ioc_};

  /// @brief Websocket holder. See detail in
  /// [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
  websocket::stream<tcp::socket> ws_{ioc_};

  /// @brief I don't know what this is. See detail in
  /// [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
  tcp::endpoint ep_;

  /// @brief The IP of the control tower.
  std::string ws_ip_;

  /// @brief The port of the control tower.
  std::string ws_port_;

  /// @brief Struct for storing can frame data.
  nturt_can_config_logger_rx_t can_rx_;

  /// @brief Struct for storing "/fix" message data.
  sensor_msgs::msg::NavSatFix gps_fix_;

  /// @brief Struct for storing "/vel" message data.
  geometry_msgs::msg::TwistStamped gps_vel_;

  /// @brief Struct for storing "/system_stats" message data.
  nturt_ros_interface::msg::SystemStats system_stats_;

  /// @brief Struct for storing battery data.
  BatteryData battery_data_;

  /// @brief String stream for sending data to control tower.
  std::stringstream ss_;
};

#endif  // NTURT_PUSH_TO_CONTROL_TOWER_HPP
