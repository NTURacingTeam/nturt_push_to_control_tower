#include "nturt_push_to_control_tower/push_to_control_tower.hpp"

PushToControlTower::PushToControlTower(rclcpp::NodeOptions options)
    : Node("nturt_push_to_control_tower_node", options),
      can_sub_(this->create_subscription<can_msgs::msg::Frame>(
          "/from_can_bus", 10,
          std::bind(&PushToControlTower::onCan, this, std::placeholders::_1))),
      gps_fix_sub_(this->create_subscription<sensor_msgs::msg::NavSatFix>(
          "/fix", 10,
          std::bind(&PushToControlTower::onGpsFix, this,
                    std::placeholders::_1))),
      gps_vel_sub_(this->create_subscription<geometry_msgs::msg::TwistStamped>(
          "/vel", 10,
          std::bind(&PushToControlTower::onGpsVel, this,
                    std::placeholders::_1))),
      send_data_timer_(this->create_wall_timer(
          100ms,
          std::bind(&PushToControlTower::send_data_timer_callback, this))),
      check_ws_connection_timer_(this->create_wall_timer(
          3s, std::bind(&PushToControlTower::check_ws_connection_timer_callback,
                        this))),
      ws_ip_(this->declare_parameter("ip", "")),
      ws_port_(this->declare_parameter("port", "")) {
  // init can_rx_
  memset(&can_rx_, 0, sizeof(can_rx_));

  if (connect_to_ws() != 0) {
    RCLCPP_ERROR(
        this->get_logger(),
        "Failed to connect to websocket at %s:%s, retrying in 3 seconds.",
        ws_ip_.c_str(), ws_port_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Successfully connect to websocket at %s:%s", ws_ip_.c_str(),
                ws_port_.c_str());
  }
};

void PushToControlTower::onCan(
    const std::shared_ptr<can_msgs::msg::Frame> msg) {
  nturt_can_config_logger_Receive(&can_rx_, msg->data.data(), msg->id,
                                  msg->dlc);
}

void PushToControlTower::onGpsFix(
    const std::shared_ptr<sensor_msgs::msg::NavSatFix> msg) {
  gps_fix_ = *msg;
}

void PushToControlTower::onGpsVel(
    const std::shared_ptr<geometry_msgs::msg::TwistStamped> msg) {
  gps_vel_ = *msg;
}

void PushToControlTower::send_data_timer_callback() {
  ss_ << "{\"batch\":{";

  // front_sensor_1
  FRONT_SENSOR_1_t* front_sensor_1 = &can_rx_.FRONT_SENSOR_1;
  ss_ << "\"front_left_wheel_speed\":"
      << front_sensor_1->FRONT_SENSOR_Front_Left_Wheel_Speed_phys
      << ",\"front_right_wheel_speed\":"
      << front_sensor_1->FRONT_SENSOR_Front_Right_Wheel_Speed_phys
      << ",\"front_left_tire_temperature\":"
      << front_sensor_1->FRONT_SENSOR_Front_Left_Tire_Temperature_phys
      << ",\"front_right_tire_temperature\":"
      << front_sensor_1->FRONT_SENSOR_Front_Right_Tire_Temperature_phys;

  // front_sensor_2
  FRONT_SENSOR_2_t* front_sensor_2 = &can_rx_.FRONT_SENSOR_2;
  ss_ << ",\"brake\":" << front_sensor_2->FRONT_SENSOR_Brake_phys
      << ",\"accelerator_1\":"
      << front_sensor_2->FRONT_SENSOR_Accelerator_1_phys
      << ",\"accelerator_2\":"
      << front_sensor_2->FRONT_SENSOR_Accelerator_2_phys
      << ",\"front_brake_pressure\":"
      << front_sensor_2->FRONT_SENSOR_Front_Brake_Pressure_phys
      << ",\"accelerator_micro\":"
      << static_cast<int>(front_sensor_2->FRONT_SENSOR_Accelerator_Micro)
      << ",\"brake_micro\":"
      << static_cast<int>(front_sensor_2->FRONT_SENSOR_Brake_Micro)
      << ",\"steering_angle\":" << front_sensor_2->FRONT_SENSOR_Steer_Angle;

  // rear_sensor_1
  REAR_SENSOR_1_t* rear_sensor_1 = &can_rx_.REAR_SENSOR_1;
  ss_ << ",\"rear_left_wheel_speed\":"
      << rear_sensor_1->REAR_SENSOR_Rear_Left_Wheel_Speed_phys
      << ",\"rear_right_wheel_speed\":"
      << rear_sensor_1->REAR_SENSOR_Rear_Right_Wheel_Speed_phys
      << ",\"rear_left_tire_temperature\":"
      << rear_sensor_1->REAR_SENSOR_Rear_Left_Tire_Temperature_phys
      << ",\"rear_right_tire_temperature\":"
      << rear_sensor_1->REAR_SENSOR_Rear_Right_Tire_Temperature_phys;

  // rear_sensor_2
  REAR_SENSOR_2_t* rear_sensor_2 = &can_rx_.REAR_SENSOR_2;
  ss_ << ",\"rear_brake_pressure\":"
      << rear_sensor_2->REAR_SENSOR_Rear_Brake_Pressure_phys;

  // inverter
  ss_ << ",\"control_board_temperature\":"
      << can_rx_.INV_Temperature_Set_2.INV_Control_Board_Temp_phys
      << ",\"motor_temperature\":"
      << can_rx_.INV_Temperature_Set_3.INV_Motor_Temp_phys
      << ",\"motor_speed\":" << can_rx_.INV_Fast_Info.INV_Fast_Motor_Speed
      << ",\"input_voltage\":"
      << can_rx_.INV_Fast_Info.INV_Fast_DC_Bus_Voltage_phys;

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

  ss_ << "}}";

  RCLCPP_DEBUG(this->get_logger(), "Sending data to control tower: %s",
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

void PushToControlTower::check_ws_connection_timer_callback() {
  if (!(ws_.is_open())) {
    if (connect_to_ws()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Failed to connect to websocket at %s:%s, retrying in 3 seconds.",
          ws_ip_.c_str(), ws_port_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Successfully connect to websocket at %s:%s", ws_ip_.c_str(),
                  ws_port_.c_str());
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
