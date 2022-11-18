#include "nturt_push_to_control_tower_core.hpp"

int P2ctower_core::push2_ctower(std::string name, double value, double time) {
    /* std::cout << name << ": " << value << ", "<< time << std::endl ; */
    std::string message = "{\"name\":\"" + name + "\",\"value\":" + std::to_string(value) + "}";
    std::cout << message << std::endl;
    /* std_msgs::String send_msg ; */
    /* send_msg.data = message ; */
    /* bridge_pub_.publish(send_msg); */
    ws_.write(net::buffer(message));
    // ws_.write(net::buffer(std::string("test")));
    /* publisher(message); */
    /* {name:["FWS","L"],value:1.1,time:123.4} */
    return OK;
};

int P2ctower_core::init_websocket(std::string host, std::string port){
    try {
        ws_hostIP_ = host;
        ws_port_ = port;
        // tcp::resolver resolvger_{ioc_};
        std::cout << "[debug]:tcp::resolver resolver_{ioc_};" << std::endl;
        auto const results = resolver_.resolve(ws_hostIP_, ws_port_);
        std::cout << "[debug]:auto const results = resolver_.resolve(ws_hostIP_, ws_port_);" << std::endl ;
        ep_ = net::connect(ws_.next_layer(), results);
        std::cout << "[debug]:ep_ = net::connect(ws_.next_layer(), results);" << std::endl ;
        ws_hostIP_ += ':' + std::to_string(ep_.port());
        ws_.set_option(websocket::stream_base::decorator(
            [](websocket::request_type& req)
            {
                req.set(http::field::user_agent,
                    std::string(BOOST_BEAST_VERSION_STRING) +
                        " websocket-client-coro");
            }));
        std::cout << "[debug]:set_option" << std::endl ;
        ws_.handshake(ws_hostIP_, "/");
        std::cout << "[debug]:ws_.handshake(ws_hostIP_, '');" << std::endl ;
        ws_.write(net::buffer(std::string("Successfully init webocket")));
        ws_connected_ = true;
        return OK;
    } catch (int error) {
        std::cout << "init websocket fail, error code: " << error << std::endl;
        ws_connected_ = false;
        return ERR;
    }
};

void P2ctower_core::timer_check_and_retry_websocket_connection(const ros::TimerEvent& event){
    if ( !(ws_connected_) ) {
        std::cout << "ws not connected" << std::endl;
    }
};

P2ctower_core::P2ctower_core(std::shared_ptr<ros::NodeHandle> &_nh) : 
    nh_(_nh), timestemp_last_(ros::Time::now().toSec()),
    state_sub_(_nh->subscribe("/node_state", 10, &P2ctower_core::onState, this)),
    register_clt_(_nh->serviceClient<nturt_ros_interface::RegisterCanNotification>("/register_can_notification")) {

    std::cout << "node init" << std::endl ;

    // register to can parser
    // wait until "/register_can_notification" service is avalible
    if(!ros::service::waitForService("/register_can_notification", 10000)) {
        ROS_FATAL("register to can parser timeout after 10 seconds");
        ros::shutdown();
    }

    // construct register call
    nturt_ros_interface::RegisterCanNotification register_srv;
    register_srv.request.node_name = ros::this_node::getName();

    register_srv.request.data_name = {
        // front_box_1
        "front_left_wheel_speed",
        "front_right_wheel_speed",
        "front_left_tyre_temperature_1",
        "front_left_tyre_temperature_2",
        "front_right_tyre_temperature_1",
        "front_right_tyre_temperature_2",
        // front_box_2
        "brake",
        "accelerator_1",
        "accelerator_2",
        "steer_angle",
        "oil_pressure",
        "accelerator_micro",
        "brake_micro",
        // imu_acceleration
        "imu_acceleration_x",
        "imu_acceleration_y",
        "imu_acceleration_z",
        // imu_gyro
        "imu_gyro_x",
        "imu_gyro_y",
        "imu_gyro_z",
        // imu_quaternion
        "imu_quaternion_w",
        "imu_quaternion_x",
        "imu_quaternion_y",
        "imu_quaternion_z",
        // muc_data
        "control_board_temperature",
        "motor_temperature",
        "motor_speed",
        "input_voltage",
        // rear_box_1
        "rear_left_wheel_speed",
        "rear_right_wheel_speed",
        "rear_left_tyre_temperature_1",
        "rear_left_tyre_temperature_2",
        "rear_right_tyre_temperature_1",
        "rear_right_tyre_temperature_2"
    };

    // call service
    if(!register_clt_.call(register_srv)) {
        ROS_FATAL("register to can parser failed");
        ros::shutdown();
    }

    // subscribe to the register topic
    notification_sub_ = nh_->subscribe(register_srv.response.topic, 10, &P2ctower_core::onNotification, this);

    // need to get can data using service "/get_can_data" for frame "mcu_command" of data "torque_command", "inverter_enable"

    check_ws_connection_timer_ = nh_->createTimer(ros::Duration(3), timer_check_and_retry_websocket_connection);

    gps_sub_ = nh_->subscribe("GPS", 10, &P2ctower_core::GPS_Callback, this);
};

void P2ctower_core::onNotification(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg) {
    std::cout << "notification called" << std::endl ;
    double time = 0;
    push2_ctower(_msg->name, _msg->data, time);
}

void P2ctower_core::onState(const std_msgs::Bool::ConstPtr &_msg) {
    if (is_activated_ != _msg->data) {
        is_activated_ = _msg->data;
    }
    /* std::cout << "is_activated: " << is_activated_ << std::endl; */
}

void P2ctower_core::CAN_Callback(const can_msgs::Frame::ConstPtr &msg){
}

void P2ctower_core::GPS_Callback(const gps_common::GPSFix::ConstPtr &msg){
    double lon = 0;
    double lat = 0;
    double time = 0.0 ;
    std::cout << "get nav msgs!!!" << std::endl ;
    push2_ctower(
        "GPS_lon", lon,
        time);
        /* msg->header.stamp); */
    push2_ctower(
        "GPS_lat", lat,
        time);
        /* msg->header.stamp); */
}
