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

int P2ctower_core::log_to_csv_(std::string file_name){
    std::ofstream file_out;
    std::cout << "Logging ..." << std::endl ;
    std::cout << "path :" << csv_log_db_path_ << file_name << std::endl;
    file_out.open(csv_log_db_path_ + file_name + ".csv", std::ios_base::app);
    file_out << csv_log_buf_ << "\n" ;
    csv_log_buf_ = "";
    file_out.close();
    return OK;
};

int P2ctower_core::csv_log_buf_append_(double one_data){
    csv_log_buf_ += std::to_string(one_data) + ",";
    return OK;
};

P2ctower_core::P2ctower_core(std::shared_ptr<ros::NodeHandle> &nh) : nh_(nh) {
    std::cout << "node init" << std::endl ;
    myparser_.init_parser();
    bridge_pub_ = nh->advertise<std_msgs::String>("send_to_ctower_data", 50);
    can_sub_ = nh->subscribe("received_messages", 10, &P2ctower_core::CAN_Callback, this);
    can_sub_ = nh->subscribe("GPS", 10, &P2ctower_core::GPS_Callback, this);
    csv_log_db_path_ = "/home/ros/nturt_ws/output_files/"; // need to be edited
};

void P2ctower_core::CAN_Callback(const can_msgs::Frame::ConstPtr &msg){
    int data[8];
    double time = 0.0 ;
    can_data_ = myparser_.get_key(msg->id);
    std::cout << msg->id << std::endl;
    csv_log_buf_append_(time);

    if (myparser_.check_key(msg->id, can_data_[0].first) == OK) {
        if (myparser_.decode(msg->id, data) == OK) {
            for (int i=0; i<can_data_.size(); i++) {
                get_afd_value = myparser_.get_afd(can_data_[i].first, can_data_[i].second);
                push2_ctower(
                        can_data_[i].first,
                        can_data_[i].second,
                        get_afd_value,
                        time);
                csv_log_buf_append_(get_afd_value);
            }
        }
    }

    log_to_csv_(std::to_string(msg->id));
    myparser_.print_err_log();
}

void P2ctower_core::GPS_Callback(const nav_msgs::Odometry::ConstPtr &msg){
    double time = 0.0 ;
    std::cout << "get nav msgs!!!" << std::endl ;
    std::cout << msg->header.stamp << std::endl ;
    std::cout << msg->pose.pose.position.x << std::endl ;
    std::cout << msg->pose.pose.position.y << std::endl ;
    std::cout << msg->pose.pose.orientation.x << std::endl ;
    std::cout << msg->pose.pose.orientation.y << std::endl ;
}
