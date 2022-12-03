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
    ws_hostIP_ = host;
    ws_port_ = port;
    return connect_to_ws();
};

int P2ctower_core::connect_to_ws() {
    try {
        auto const results = resolver_.resolve(ws_hostIP_, ws_port_);
        /* ROS_INFO("[debug]:auto const results = resolver_.resolve(ws_hostIP_, ws_port_);"); */
        ep_ = net::connect(ws_.next_layer(), results);
        /* ROS_INFO("[debug]:ep_ = net::connect(ws_.next_layer(), results);"); */
        std::string ws_hostIP_PORT = ws_hostIP_ + ':' + std::to_string(ep_.port());
        ws_.set_option(websocket::stream_base::decorator(
            [](websocket::request_type& req)
            {
                req.set(http::field::user_agent,
                    std::string(BOOST_BEAST_VERSION_STRING) +
                        " websocket-client-coro");
            }));
        /* ROS_INFO("[debug]:set_option"); */
        ws_.handshake(ws_hostIP_PORT, "/");
        /* ROS_INFO("[debug]:ws_.handshake(ws_hostIP_PORT, '');"); */
        ws_.write(net::buffer(std::string("Successfully init webocket")));
        ROS_INFO("connect to ws succeeded");
        return OK;
    } catch (std::exception&  error) {
        ROS_WARN("connect to ws failed");
        /* std::string error_str(error.what()); */
        /* ROS_INFO(error_str); */
        return ERR;
    }
}

void P2ctower_core::timer_check_and_retry_websocket_connection_(const ros::TimerEvent& event){
    /* ROS_INFO("retry ws function called"); */
    if ( !(ws_.is_open()) ) {
        /* ROS_FATAL("ws not connected, trying to reconnect"); */
        connect_to_ws();
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

    std::vector<std::string > frames = {
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

    register_srv.request.data_name = frames;
    /* ROS_FATAL("attention"); */
    /* std::cout << ((register_srv.request.data_name)) << std::endl; */

    // call service
    if(!register_clt_.call(register_srv)) {
        ROS_FATAL("register to can parser failed");
        ros::shutdown();
    }

    // subscribe to the register topic
    notification_sub_ = nh_->subscribe(register_srv.response.topic, 100, &P2ctower_core::onNotification, this);

    // need to get can data using service "/get_can_data" for frame "mcu_command" of data "torque_command", "inverter_enable"
    double retry_ws_duration ;
    double  push_buffer_duration ;
    ros::param::get("~retry_ws_duration", retry_ws_duration) ;
    ros::param::get("~push_buffer_duration", push_buffer_duration) ;

    check_ws_connection_timer_ = nh_->createTimer(ros::Duration(3), &P2ctower_core::timer_check_and_retry_websocket_connection_, this);

    gps_sub_ = nh_->subscribe("GPS", 10, &P2ctower_core::GPS_Callback, this);

    // for sending message with timer

    push_buffer_timer_ = nh_->createTimer(ros::Duration(0.1), &P2ctower_core::push_buffer_timer_callback_, this); // called every duration(n) second

    /* std::vector<std::string>::iterator ptr = frames.begin(); */
    /* for (ptr = frames.begin(); ptr < frames.end(); ptr++) frame_buffer_[*ptr] = 0; */

};

void P2ctower_core::push_buffer_timer_callback_(const ros::TimerEvent& event) {
    /* ROS_INFO("timer triggered"); */
    push_buffer_to_ctower();
};

void P2ctower_core::push_buffer_to_ctower() {
    std::string message = "{\"batch\":{" ;
    std::map<std::string, double>::iterator i;

    for (std::map<std::string, double>::iterator i = frame_buffer_.begin() ; i != frame_buffer_.end() ; i++) {
        /* cout << i->first << " => " << i->second << '\n'; */
        if (i != frame_buffer_.begin()) message += "," ;
        message += "\"" + i->first + "\""  + ":" + std::to_string(i->second) ;
    };

    message += "}}" ;

    /* ROS_FATAL("message below"); */
    /* std::cout << message << std::endl; */

    frame_buffer_.clear();
    try {
        ws_.write(net::buffer(message));
    } catch (std::exception& error) {
      /* ROS_FATAL(error.what()) ; */
    };
    /* {name:["FWS","L"],value:1.1,time:123.4} */
}


void P2ctower_core::onNotification(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg) {
    /* std::cout << "notification called" << std::endl ; */
    double time = 0;
    /* push2_ctower(_msg->name, _msg->data, time); */
    Store_to_frame_buffer_(_msg->name, _msg->data, time);
}

int P2ctower_core::Store_to_frame_buffer_(std::string name, double value, double time) {
    frame_buffer_[name] = value ;
    return OK;
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
    Store_to_frame_buffer_( "GPS_lon", lon, time);
    Store_to_frame_buffer_( "GPS_lat", lat, time);
    /* push2_ctower( "GPS_lon", lon, time); */
        /* msg->header.stamp); */
    /* push2_ctower( "GPS_lat", lat, time); */
        /* msg->header.stamp); */
}
