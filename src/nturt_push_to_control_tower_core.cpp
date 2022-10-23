#include "nturt_push_to_control_tower_core.hpp"

int P2ctower_core::push2_ctower(std::string name, double value, double time) {
    std::cout << name << value << time << std::endl ;
    /* std::string message = "{\"name\":[\"" + type + "\",\"" + sub_type + "\"],\"value\":" + std::to_string(value) + "}"; */
    /* std::cout << message << std::endl; */
    // std_msgs::String send_msg ;
    // send_msg.data = message ;
    // bridge_pub_.publish(send_msg);
    /* ws_.write(net::buffer(message)); */
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

    /*
        data name registering to be notified
        brake -> brake level (front box 2)
        accelerator_1 -> accelerator level 1 (front box 2)
        accelerator_2 -> accelerator level 2 (front box 2)
        brake_micro -> brake trigger (front box 2)
    */
    register_srv.request.data_name = {"brake", "accelerator_1", "accelerator_2", "accelerator_micro"};

    // call service
    if(!register_clt_.call(register_srv)) {
        ROS_FATAL("register to can parser failed");
        ros::shutdown();
    }

    // subscribe to the register topic
    notification_sub_ = nh_->subscribe(register_srv.response.topic, 10, &P2ctower_core::onNotification, this);

    gps_sub_ = nh_->subscribe("GPS", 10, &P2ctower_core::GPS_Callback, this);
};

void P2ctower_core::onNotification(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg) {
    std::cout << "notification called" << std::endl ;
    double time = 0;
    push2_ctower(_msg->name, _msg->data, time);
}

void P2ctower_core::onState(const std_msgs::Bool::ConstPtr &_msg) {
    is_activated_ = _msg->data;
    std::cout << "is_activated: " << is_activated_ ;
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
