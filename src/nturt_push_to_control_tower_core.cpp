#include <nturt_push_to_control_tower_core.hpp>
                                        //
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

int P2ctower_core::init_websocket(){
    host_ = "localhost";
    port_ = "8080";
    text_ = "Constructed";
    tcp::resolver resolver{ioc_};
    auto const results = resolver.resolve(host_, port_);
    ep_ = net::connect(ws_.next_layer(), results);
    host_ += ':' + std::to_string(ep_.port());
    ws_.set_option(websocket::stream_base::decorator(
        [](websocket::request_type& req)
        {
            req.set(http::field::user_agent,
                std::string(BOOST_BEAST_VERSION_STRING) +
                    " websocket-client-coro");
        }));
    ws_.handshake(host_, "/");
    ws_.write(net::buffer(std::string("Successfully init webocket")));
    return OK;
};


P2ctower_core::P2ctower_core(std::shared_ptr<ros::NodeHandle> &nh) : nh_(nh) {
    std::cout << "node init" << std::endl ;
    myparser_.init_parser();
    bridge_pub_ = nh->advertise<std_msgs::String>("send_to_ctower_data", 50);
    can_sub_ = nh->subscribe("received_messages", 10, &P2ctower_core::CAN_Callback, this);
}

void P2ctower_core::CAN_Callback(const can_msgs::Frame::ConstPtr &msg){
    int data[8];
    double time = 0.0 ;

    std::cout << msg->id << std::endl;
    std::vector<std::pair<std::string, std::string> > can_data = myparser_.get_key(msg->id);
    if (myparser_.check_key(msg->id, can_data[0].first) == OK) {
        if (myparser_.decode(msg->id, data) == OK) {
            for (int i=0; i<can_data.size(); i++) {
                push2_ctower(can_data[i].first, can_data[i].second, myparser_.get_afd(can_data[i].first, can_data[i].second), time);
            }
        } 
    } 
    myparser_.print_err_log();
}
