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

    std::string filename("/home/ros/nturt_ws/src/nturt_push_to_control_tower/cp_can_id.csv");
    std::vector<std::string> lines;
    std::string line;
    std::vector<int> can_ids;
    std::ifstream input_file(filename);
    if (!input_file.is_open()) {
        std::cout << "Could not open the file - '"
             << filename << "'" << std::endl;
    }
    else {
        std::cout << "file open successfully" << std::endl; 
    }

    while (getline(input_file, line)){
        lines.push_back(line);
    }
    
    for (int i=0; i<lines.size(); i++) {
        can_ids.push_back(std::stoi(lines[i], 0, 16));
    }
    input_file.close();
}

void P2ctower_core::CAN_Callback(const can_msgs::Frame::ConstPtr &msg){
    int data[8];
    double time = 0.0 ;

    //std::cout << msg << std::endl; 
     
    //std::vector<std::pair<std::string, std::string> > can_data = myparser_.get_key(msg);
    //for (int i=0; i<can_data.size(); i++) {
    //    push2_ctower(can_data[i].first, can_data[i].second, myparser_.get_afd(can_data[i].first, can_data[i].second), time);
    //} 

    if (myparser_.check_key(_CAN_FB1, "FWS") == OK) {
        if (myparser_.decode(_CAN_FB1, data) == OK) {
            push2_ctower("FWS", "L", myparser_.get_afd("FWS", "L"), time );
            push2_ctower("FWS", "R", myparser_.get_afd("FWS", "L"), time );
            push2_ctower("FWS", "L1", myparser_.get_afd("FWS", "L"), time );
            push2_ctower("FWS", "L2", myparser_.get_afd("FWS", "L"), time );
            push2_ctower("FWS", "R1", myparser_.get_afd("FWS", "L"), time );
            push2_ctower("FWS", "R2", myparser_.get_afd("FWS", "L"), time );
        };
    };
    if (myparser_.check_key(_CAN_FB2, "THR") == OK) {
        if (myparser_.decode(_CAN_FB2, data) == OK) {
            push2_ctower("THR", "A", myparser_.get_afd("THR", "A"), time );
            push2_ctower("THR", "B", myparser_.get_afd("THR", "B"), time );
        };
    };
    if (myparser_.check_key(_CAN_FB2, "STR") == OK) {
        if (myparser_.decode(_CAN_FB2, data) == OK) {
            push2_ctower("STR", "N", myparser_.get_afd("STR", "N"), time );
        };
    };
    if (myparser_.check_key(_CAN_FB2, "FSS") == OK) {
        if (myparser_.decode(_CAN_FB2, data) == OK) {
            push2_ctower("FSS", "L", myparser_.get_afd("FSS", "L"), time );
            push2_ctower("FSS", "R", myparser_.get_afd("FSS", "R"), time );
        };
    };
    if (myparser_.check_key(_CAN_FB2, "OPR") == OK) {
        if (myparser_.decode(_CAN_FB2, data) == OK) {
            push2_ctower("OPR", "N", myparser_.get_afd("OPR", "N"), time );
        };
    };
    if (myparser_.check_key(_CAN_RB1, "RWS") == OK) {
        if (myparser_.decode(_CAN_RB1, data) == OK) {
            push2_ctower("RWS", "L", myparser_.get_afd("RWS", "L"), time );
            push2_ctower("RWS", "R", myparser_.get_afd("RWS", "R"), time );
        };
    };
    if (myparser_.check_key(_CAN_RB1, "RWT") == OK) {
        if (myparser_.decode(_CAN_RB1, data) == OK) {
            push2_ctower("RWT", "L1", myparser_.get_afd("RWT", "L1"), time );
            push2_ctower("RWT", "L2", myparser_.get_afd("RWT", "L2"), time );
            push2_ctower("RWT", "R1", myparser_.get_afd("RWT", "R1"), time );
            push2_ctower("RWT", "R2", myparser_.get_afd("RWT", "R2"), time );
        };
    };
    if (myparser_.check_key(_CAN_RB2, "RSS") == OK) {
        if (myparser_.decode(_CAN_RB2, data) == OK) {
            push2_ctower("RSS", "L", myparser_.get_afd("RSS", "L"), time );
            push2_ctower("RSS", "R", myparser_.get_afd("RSS", "R"), time );
        };
    };
    if (myparser_.check_key(_CAN_HIS, "HIS") == OK) {
        if (myparser_.decode(_CAN_HIS, data) == OK) {
            push2_ctower("HIS", "P", myparser_.get_afd("HIS", "P"), time );
            push2_ctower("HIS", "R", myparser_.get_afd("HIS", "R"), time );
        };
    };
    if (myparser_.check_key(_CAN_HIA, "HIA") == OK) {
        if (myparser_.decode(_CAN_HIA, data) == OK) {
            push2_ctower("HIA", "X", myparser_.get_afd("HIA", "X"), time );
            push2_ctower("HIA", "Y", myparser_.get_afd("HIA", "Y"), time );
            push2_ctower("HIA", "Z", myparser_.get_afd("HIA", "Z"), time );
        };
    };
    if (myparser_.check_key(_CAN_HIG, "HIG") == OK) {
        if (myparser_.decode(_CAN_HIG, data) == OK) {
            push2_ctower("HIG", "X", myparser_.get_afd("HIG", "X"), time );
            push2_ctower("HIG", "Y", myparser_.get_afd("HIG", "Y"), time );
            push2_ctower("HIG", "Z", myparser_.get_afd("HIG", "Z"), time );
        };
    };
    if (myparser_.check_key(_CAN_OIS, "OIS") == OK) {
        if (myparser_.decode(_CAN_OIS, data) == OK) {
            push2_ctower("OIS", "P", myparser_.get_afd("OIS", "P"), time );
            push2_ctower("OIS", "R", myparser_.get_afd("OIS", "R"), time );
        };
    };
    if (myparser_.check_key(_CAN_OIA, "OIA") == OK) {
        if (myparser_.decode(_CAN_OIA, data) == OK) {
            push2_ctower("OIA", "X", myparser_.get_afd("OIA", "X"), time );
            push2_ctower("OIA", "Y", myparser_.get_afd("OIA", "Y"), time );
            push2_ctower("OIA", "Z", myparser_.get_afd("OIA", "Z"), time );
        };
    };
    if (myparser_.check_key(_CAN_OIG, "OIG") == OK) {
        if (myparser_.decode(_CAN_OIG, data) == OK) {
            push2_ctower("OIG", "X", myparser_.get_afd("OIG", "X"), time );
            push2_ctower("OIG", "Y", myparser_.get_afd("OIG", "Y"), time );
            push2_ctower("OIG", "Z", myparser_.get_afd("OIG", "Z"), time );
        };
    };
    if (myparser_.check_key(_CAN_OIC, "OIC") == OK) {
        if (myparser_.decode(_CAN_OIC, data) == OK) {
            push2_ctower("OIC", "X", myparser_.get_afd("OIC", "X"), time );
            push2_ctower("OIC", "Y", myparser_.get_afd("OIC", "Y"), time );
            push2_ctower("OIC", "Z", myparser_.get_afd("OIC", "Z"), time );
        };
    };
    if (myparser_.check_key(_CAN_MCM, "MTC") == OK) {
        if (myparser_.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MTC", "N", myparser_.get_afd("MTC", "N"), time );
        };
    };
    if (myparser_.check_key(_CAN_MCM, "MSC") == OK) {
        if (myparser_.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MSC", "N", myparser_.get_afd("MSC", "N"), time );
        };
    };
    if (myparser_.check_key(_CAN_MCM, "MDC") == OK) {
        if (myparser_.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MDC", "N", myparser_.get_afd("MDC", "N"), time );
        };
    };
    if (myparser_.check_key(_CAN_MCM, "MIE") == OK) {
        if (myparser_.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MIE", "N", myparser_.get_afd("MIE", "N"), time );
        };
    };
    if (myparser_.check_key(_CAN_MCM, "MID") == OK) {
        if (myparser_.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MID", "N", myparser_.get_afd("MID", "N"), time );
        };
    };
    if (myparser_.check_key(_CAN_MCM, "MSM") == OK) {
        if (myparser_.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MSM", "N", myparser_.get_afd("MSM", "N"), time );
        };
    };
    if (myparser_.check_key(_CAN_MCM, "MTL") == OK) {
        if (myparser_.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MTL", "N", myparser_.get_afd("MTL", "N"), time );
        };
    };
    myparser_.print_err_log();
}
