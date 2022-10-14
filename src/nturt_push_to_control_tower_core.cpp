#include "nturt_push_to_control_tower_core.hpp"

int P2ctower_core::push2_ctower(std::string type, std::string sub_type, double value, double time ) {
    std::cout << type << sub_type << value << time << std::endl ;
    std::string message = "{\"name\":[\"" + type + "\",\"" + sub_type + "\"],\"value\":" + std::to_string(value) + "}";
    std::cout << message << std::endl;
    // std_msgs::String send_msg ;
    // send_msg.data = message ;
    // bridge_pub_.publish(send_msg);
    ws_.write(net::buffer(message));
    // ws_.write(net::buffer(std::string("test")));
    /* publisher(message); */
    /* {name:["FWS","L"],value:1.1,time:123.4} */
    return OK;
};

int P2ctower_core::init_websocket(std::string host, std::string port){
    host_ = host;
    port_ = port;
    // tcp::resolver resolvger_{ioc_};
    std::cout << "[debug]:tcp::resolver resolver_{ioc_};" << std::endl;
    auto const results = resolver_.resolve(host_, port_);
    std::cout << "[debug]:auto const results = resolver_.resolve(host_, port_);" << std::endl ;
    ep_ = net::connect(ws_.next_layer(), results);
    std::cout << "[debug]:ep_ = net::connect(ws_.next_layer(), results);" << std::endl ;
    host_ += ':' + std::to_string(ep_.port());
    ws_.set_option(websocket::stream_base::decorator(
        [](websocket::request_type& req)
        {
            req.set(http::field::user_agent,
                std::string(BOOST_BEAST_VERSION_STRING) +
                    " websocket-client-coro");
        }));
    std::cout << "[debug]:set_option" << std::endl ;
    ws_.handshake(host_, "/");
    std::cout << "[debug]:ws_.handshake(host_, '');" << std::endl ;
    ws_.write(net::buffer(std::string("Successfully init webocket")));
    return OK;
};

P2ctower_core::P2ctower_core(std::shared_ptr<ros::NodeHandle> &nh) : nh_(nh) {
    std::cout << "node init" << std::endl ;
    /* bridge_pub_ = nh->advertise<std_msgs::String>("send_to_ctower_data", 50); */
    can_sub_ = nh->subscribe("received_messages", 10, &P2ctower_core::CAN_Callback, this);
    gps_sub_ = nh->subscribe("GPS", 10, &P2ctower_core::GPS_Callback, this);
};

void P2ctower_core::CAN_Callback(const can_msgs::Frame::ConstPtr &msg){
}

void P2ctower_core::GPS_Callback(const gps_common::GPSFix::ConstPtr &msg){
    double time = 0.0 ;
    std::cout << "get nav msgs!!!" << std::endl ;
    push2_ctower(
        "GPS", "x",
        0,
        time);
        /* msg->header.stamp); */
    push2_ctower(
        "GPS", "y",
        0,
        time);
        /* msg->header.stamp); */
}
