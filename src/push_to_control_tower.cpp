#include "nturt_push_to_control_tower/push_to_control_tower.hpp"

// glibc include
#include <string.h>

// std include
#include <fstream>
#include <functional>
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
#include "nturt_can_config_logger-binutil.h"
#include "nturt_ros_interface/msg/system_stats.hpp"

namespace http = boost::beast::http;
namespace websocket = boost::beast::websocket;
namespace net = boost::asio;
using namespace std::chrono_literals;

PushToControlTower::PushToControlTower(rclcpp::NodeOptions options)
    : Node("nturt_push_to_control_tower_node", options),
      can_sub_(this->create_subscription<can_msgs::msg::Frame>(
          "/from_can_bus", 50,
          std::bind(&PushToControlTower::onCan, this, std::placeholders::_1))),
      gps_fix_sub_(this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "/fix", 10,
          std::bind(&PushToControlTower::onGpsFix, this,
                    std::placeholders::_1))),
      gps_vel_sub_(this->create_subscription<geometry_msgs::msg::TwistStamped>(
          "/vel", 10,
          std::bind(&PushToControlTower::onGpsVel, this,
                    std::placeholders::_1))),
      system_stats_sub_(
          this->create_subscription<nturt_ros_interface::msg::SystemStats>(
              "system_stats", 10,
              std::bind(&PushToControlTower::onSystemStats, this,
                        std::placeholders::_1))),
      send_fast_data_timer_(this->create_wall_timer(
          200ms,
          std::bind(&PushToControlTower::send_fast_data_timer_callback, this))),
      send_slow_data_timer_(this->create_wall_timer(
          1s,
          std::bind(&PushToControlTower::send_slow_data_timer_callback, this))),
      check_ws_connection_timer_(this->create_wall_timer(
          3s, std::bind(&PushToControlTower::check_ws_connection_timer_callback,
                        this))),
      ws_ip_(this->declare_parameter("ip", "")),
      ws_port_(this->declare_parameter("port", "")) {
  // init can_rx_
  memset(&can_rx_, 0, sizeof(can_rx_));

  if (connect_to_ws() != 0) {
    RCLCPP_ERROR(
        get_logger(),
        "Failed to connect to websocket at %s:%s, retrying in 3 seconds.",
        ws_ip_.c_str(), ws_port_.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "Successfully connect to websocket at %s:%s",
                ws_ip_.c_str(), ws_port_.c_str());
  }
};

void PushToControlTower::onCan(
    const std::shared_ptr<can_msgs::msg::Frame> msg) {
  uint32_t id = nturt_can_config_logger_Receive(&can_rx_, msg->data.data(),
                                                msg->id, msg->dlc);

  if (id == BMS_Cell_Stats_CANID) {
    BMS_Cell_Stats_t* bms_cell_stats = &can_rx_.BMS_Cell_Stats;
    int segment_index = bms_cell_stats->BMS_Segment_Index,
        cell_index =
            NUM_BATTERY_CELL_PER_FRAME * bms_cell_stats->BMS_Cell_Index;

    battery_cell_voltage_[segment_index][cell_index] =
        bms_cell_stats->BMS_Cell_Voltage_1_phys;
    battery_cell_voltage_[segment_index][cell_index + 1] =
        bms_cell_stats->BMS_Cell_Voltage_2_phys;
    battery_cell_voltage_[segment_index][cell_index + 2] =
        bms_cell_stats->BMS_Cell_Voltage_3_phys;

    battery_cell_temperature_[segment_index][cell_index] =
        bms_cell_stats->BMS_Cell_Temperature_1_phys;
    battery_cell_temperature_[segment_index][cell_index + 1] =
        bms_cell_stats->BMS_Cell_Temperature_2_phys;
    battery_cell_temperature_[segment_index][cell_index + 2] =
        bms_cell_stats->BMS_Cell_Temperature_3_phys;
  }
}

void PushToControlTower::onGpsFix(
    const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg) {
  gps_fix_ = *msg;
}

void PushToControlTower::onGpsVel(
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
  gps_vel_ = *msg;
}

void PushToControlTower::onSystemStats(
    const std::shared_ptr<nturt_ros_interface::msg::SystemStats> msg) {
  system_stats_ = *msg;
}

void PushToControlTower::send_fast_data_timer_callback() {
  ss_ << "{\"batch\":{";

  // vcu_status
  VCU_Status_t* vcu_status = &can_rx_.VCU_Status;
  ss_ << "\"vcu_status\":" << static_cast<int>(vcu_status->VCU_Status)
      << ",\"vcu_error_code\":" << vcu_status->VCU_Error_Code;

  // rear_sensor_status
  REAR_SENSOR_Status_t* rear_sensor_status = &can_rx_.REAR_SENSOR_Status;
  ss_ << ",\"rear_sensor_status\":"
      << static_cast<int>(rear_sensor_status->REAR_SENSOR_Status)
      << ",\"rear_sensor_error_code\":"
      << rear_sensor_status->REAR_SENSOR_Error_Code;

  // front_sensor_1
  FRONT_SENSOR_1_t* front_sensor_1 = &can_rx_.FRONT_SENSOR_1;
  ss_ << ",\"brake\":" << front_sensor_1->FRONT_SENSOR_Brake_phys
      << ",\"accelerator_1\":"
      << front_sensor_1->FRONT_SENSOR_Accelerator_1_phys
      << ",\"accelerator_2\":"
      << front_sensor_1->FRONT_SENSOR_Accelerator_2_phys
      << ",\"steer_angle\":" << front_sensor_1->FRONT_SENSOR_Steer_Angle
      << ",\"brake_micro\":"
      << static_cast<int>(front_sensor_1->FRONT_SENSOR_Accelerator_Micro)
      << ",\"accelerator_micro\":"
      << static_cast<int>(front_sensor_1->FRONT_SENSOR_Brake_Micro);

  // front_sensor_2
  FRONT_SENSOR_2_t* front_sensor_2 = &can_rx_.FRONT_SENSOR_2;
  ss_ << ",\"front_left_wheel_speed\":"
      << front_sensor_2->FRONT_SENSOR_Front_Left_Wheel_Speed_phys
      << ",\"front_right_wheel_speed\":"
      << front_sensor_2->FRONT_SENSOR_Front_Right_Wheel_Speed_phys
      << ",\"front_brake_pressure\":"
      << front_sensor_2->FRONT_SENSOR_Front_Brake_Pressure_phys
      << ",\"rear_brake_pressure\":"
      << front_sensor_2->FRONT_SENSOR_Rear_Brake_Pressure_phys
      << ",\"front_left_suspension\":"
      << front_sensor_2->FRONT_SENSOR_Front_Left_Suspension_phys
      << ",\"front_right_suspension\":"
      << front_sensor_2->FRONT_SENSOR_Front_Right_Suspension_phys;

  // front_sensor_3
  FRONT_SENSOR_3_t* front_sensor_3 = &can_rx_.FRONT_SENSOR_3;
  ss_ << ",\"front_left_tire_temperature_1\":"
      << front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_1_phys
      << ",\"front_left_tire_temperature_2\":"
      << front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_2_phys
      << ",\"front_left_tire_temperature_3\":"
      << front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_3_phys
      << ",\"front_left_tire_temperature_4\":"
      << front_sensor_3->FRONT_SENSOR_Front_Left_Tire_Temperature_4_phys
      << ",\"front_right_tire_temperature_1\":"
      << front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_1_phys
      << ",\"front_right_tire_temperature_2\":"
      << front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_2_phys
      << ",\"front_right_tire_temperature_3\":"
      << front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_3_phys
      << ",\"front_right_tire_temperature_4\":"
      << front_sensor_3->FRONT_SENSOR_Front_Right_Tire_Temperature_4_phys;

  // rear_sensor_1
  REAR_SENSOR_1_t* rear_sensor_1 = &can_rx_.REAR_SENSOR_1;
  ss_ << ",\"rear_left_wheel_speed\":"
      << rear_sensor_1->REAR_SENSOR_Rear_Left_Wheel_Speed_phys
      << ",\"rear_right_wheel_speed\":"
      << rear_sensor_1->REAR_SENSOR_Rear_Right_Wheel_Speed_phys
      << ",\"rear_left_suspension\":"
      << rear_sensor_1->REAR_SENSOR_Rear_Left_Suspension_phys
      << ",\"rear_right_suspension\":"
      << rear_sensor_1->REAR_SENSOR_Rear_Right_Suspension_phys;

  // rear_sensor_2
  REAR_SENSOR_2_t* rear_sensor_2 = &can_rx_.REAR_SENSOR_2;
  ss_ << ",\"rear_left_tire_temperature_1\":"
      << rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_1_phys
      << ",\"rear_left_tire_temperature_2\":"
      << rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_2_phys
      << ",\"rear_left_tire_temperature_3\":"
      << rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_3_phys
      << ",\"rear_left_tire_temperature_4\":"
      << rear_sensor_2->REAR_SENSOR_Rear_Left_Tire_Temperature_4_phys
      << ",\"rear_right_tire_temperature_1\":"
      << rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_1_phys
      << ",\"rear_right_tire_temperature_2\":"
      << rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_2_phys
      << ",\"rear_right_tire_temperature_3\":"
      << rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_3_phys
      << ",\"rear_right_tire_temperature_4\":"
      << rear_sensor_2->REAR_SENSOR_Rear_Right_Tire_Temperature_4_phys;

  // inverter_fault_codes
  INV_Fault_Codes_t* inverter_fault_codes = &can_rx_.INV_Fault_Codes;
  ss_ << ",\"inverter_post_fault_lo\":"
      << inverter_fault_codes->INV_Post_Fault_Lo
      << ",\"inverter_post_fault_hi\":"
      << inverter_fault_codes->INV_Post_Fault_Hi
      << ",\"inverter_run_fault_lo\":" << inverter_fault_codes->INV_Run_Fault_Lo
      << ",\"inverter_run_fault_hi\":"
      << inverter_fault_codes->INV_Run_Fault_Hi;

  // inverter_fast_info
  INV_Fast_Info_t* inverter_fast_info = &can_rx_.INV_Fast_Info;
  ss_ << ",\"torque_command\":"
      << inverter_fast_info->INV_Fast_Torque_Command_phys
      << ",\"torque_feedback\":"
      << inverter_fast_info->INV_Fast_Torque_Feedback_phys
      << ",\"motor_speed\":" << inverter_fast_info->INV_Fast_Motor_Speed
      << ",\"inverter_dc_bus_voltage\":"
      << inverter_fast_info->INV_Fast_DC_Bus_Voltage_phys;

  // inverter other
  ss_ << ",\"inverter_control_board_temperature\":"
      << can_rx_.INV_Temperature_Set_2.INV_Control_Board_Temp_phys
      << ",\"inverter_hot_spot_temperature\":"
      << can_rx_.INV_Temperature_Set_3.INV_Hot_Spot_Temp_phys
      << ",\"motor_temperature\":"
      << can_rx_.INV_Temperature_Set_3.INV_Motor_Temp_phys
      << ",\"inverter_dc_bus_current\":"
      << can_rx_.INV_Current_Info.INV_DC_Bus_Current_phys;

  // imu_acceleration
  IMU_Acceleration_t* imu_acceleration = &can_rx_.IMU_Acceleration;
  ss_ << ",\"imu_acceleration_x\":" << imu_acceleration->IMU_Acceleration_X_phys
      << ",\"imu_acceleration_y\":" << imu_acceleration->IMU_Acceleration_Y_phys
      << ",\"imu_acceleration_z\":"
      << imu_acceleration->IMU_Acceleration_Z_phys;

  // imu_angular_velocity
  IMU_Angular_Velocity_t* imu_angular_velocity = &can_rx_.IMU_Angular_Velocity;
  ss_ << ",\"imu_angular_velocity_x\":"
      << imu_angular_velocity->IMU_Angular_Velocity_X_phys
      << ",\"imu_angular_velocity_y\":"
      << imu_angular_velocity->IMU_Angular_Velocity_Y_phys
      << ",\"imu_angular_velocity_z\":"
      << imu_angular_velocity->IMU_Angular_Velocity_Z_phys;

  // imu_quaternion
  IMU_Quaternion_t* imu_quaternion = &can_rx_.IMU_Quaternion;
  ss_ << ",\"imu_quaternion_w\":" << imu_quaternion->IMU_Quaternion_W_phys
      << ",\"imu_quaternion_x\":" << imu_quaternion->IMU_Quaternion_X_phys
      << ",\"imu_quaternion_y\":" << imu_quaternion->IMU_Quaternion_Y_phys
      << ",\"imu_quaternion_z\":" << imu_quaternion->IMU_Quaternion_Z_phys;

  // gps_fix
  if (gps_fix_.status.status == 0) {
    ss_ << ",\"gps_fix_status\":null"
        << ",\"gps_fix_longitude\":null"
        << ",\"gps_fix_altitude\":null";
  } else {
    ss_ << ",\"gps_fix_longitude\":" << gps_fix_.longitude
        << ",\"gps_fix_latitude\":" << gps_fix_.latitude
        << ",\"gps_fix_altitude\":" << gps_fix_.altitude;
  }

  // gps_vel
  ss_ << ",\"gps_vel_linear_x\":" << gps_vel_.twist.linear.x
      << ",\"gps_vel_linear_y\":" << gps_vel_.twist.linear.y;

  // system stats
  ss_ << ",\"cpu_usage\":" << system_stats_.cpu_usage
      << ",\"memory_usage\":" << system_stats_.memory_usage
      << ",\"swap_usage\":" << system_stats_.swap_usage
      << ",\"disk_usage\":" << system_stats_.disk_usage
      << ",\"cpu_temperature\":" << system_stats_.cpu_temperature;

  ss_ << "}}";

  RCLCPP_DEBUG(get_logger(), "Sending data to control tower: %s",
               ss_.str().c_str());

  // send to control tower
  try {
    ws_.write(net::buffer(ss_.str()));
  } catch (std::exception& error) {
  };

  // clear stringstream
  ss_.clear();
  ss_.str("");
};

void PushToControlTower::send_slow_data_timer_callback() {
  ss_ << "{\"batch\":{";

  // battery cell voltage
  ss_ << "\"accumulator_voltage\":[";
  for (int i = 0; i < NUM_BATTERY_SEGMENT; i++) {
    ss_ << "[";
    for (int j = 0; j < NUM_BATTERY_CELL_PER_SEGMENT; j++) {
      ss_ << battery_cell_voltage_[i][j];
      if (j != NUM_BATTERY_CELL_PER_SEGMENT - 1) {
        ss_ << ",";
      }
    }
    ss_ << "]";
    if (i != NUM_BATTERY_SEGMENT - 1) {
      ss_ << ",";
    }
  }

  // battery cell temperature
  ss_ << "],\"accumulator_temperature\":[";
  for (int i = 0; i < NUM_BATTERY_SEGMENT; i++) {
    ss_ << "[";
    for (int j = 0; j < NUM_BATTERY_CELL_PER_SEGMENT; j++) {
      ss_ << battery_cell_temperature_[i][j];
      if (j != NUM_BATTERY_CELL_PER_SEGMENT - 1) {
        ss_ << ",";
      }
    }
    ss_ << "]";
    if (i != NUM_BATTERY_SEGMENT - 1) {
      ss_ << ",";
    }
  }

  ss_ << "]}}";

  RCLCPP_DEBUG(get_logger(), "Sending data to control tower: %s",
               ss_.str().c_str());

  // send to control tower
  try {
    ws_.write(net::buffer(ss_.str()));
  } catch (std::exception& error) {
  };

  // clear stringstream
  ss_.clear();
  ss_.str("");
}

void PushToControlTower::check_ws_connection_timer_callback() {
  if (!(ws_.is_open())) {
    if (connect_to_ws()) {
      RCLCPP_ERROR(
          get_logger(),
          "Failed to connect to websocket at %s:%s, retrying in 3 seconds.",
          ws_ip_.c_str(), ws_port_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Successfully connect to websocket at %s:%s",
                  ws_ip_.c_str(), ws_port_.c_str());
    }
  }
};

int PushToControlTower::connect_to_ws() {
  try {
    auto const results = resolver_.resolve(ws_ip_, ws_port_);
    ep_ = net::connect(ws_.next_layer(), results);
    std::string ws_hostIP_PORT = ws_ip_ + ':' + std::to_string(ep_.port());
    ws_.set_option(
        websocket::stream_base::decorator([](websocket::request_type& req) {
          req.set(http::field::user_agent,
                  std::string(BOOST_BEAST_VERSION_STRING) +
                      " websocket-client-coro");
        }));
    ws_.handshake(ws_hostIP_PORT, "/");
    ws_.write(net::buffer(std::string("Successfully init webocket")));
    return 0;
  } catch (std::exception& error) {
    return 1;
  }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PushToControlTower)
