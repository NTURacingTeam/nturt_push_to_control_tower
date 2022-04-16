#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdlib>
#include <iostream>
#include <string>

#include <nturt_push_to_control_tower_core.hpp>

namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

/* -------------------------------------------------------- */

int P2ctower_core::close_connection() {
    // Close the WebSocket connection
    ws_.close(websocket::close_code::normal);
    return OK;
}

int P2ctower_core::push2_ctower(std::string type, std::string sub_type, double value, double time ) {
    std::string message = "{'name':[";
    message += type + "','" + sub_type + "'],'value':";
    message += std::to_string(value);
    message += "}";
    std::cout << message << std::endl;
    /* std_msgs::String send_msg ; */
    /* send_msg.data = message ; */
    /* bridge_pub.publish(send_msg); */
    ws_.write(net::buffer(std::string(message)));
    return OK;
};

int P2ctower_core::init_websocket(){
    // Update the host_ string. This will provide the value of the
    // Host HTTP header during the WebSocket handshake.
    // See https://tools.ietf.org/html/rfc7230#section-5.4
    host_ += ':' + std::to_string(ep.port());

    // Set a decorator to change the User-Agent of the handshake
    ws_.set_option(websocket::stream_base::decorator(
        [](websocket::request_type& req)
        {
            req.set(http::field::user_agent,
                std::string(BOOST_BEAST_VERSION_STRING) +
                    " websocket-client-coro");
        }));

    // Perform the websocket handshake
    ws_.handshake(host, "/");
    return OK;
};


P2ctower_core::P2ctower_core(std::shared_ptr<ros::NodeHandle> &nh) : nh_(nh) {
    std::cout << "node init" << std::endl ;
    myparser.init_parser();
    bridge_pub = nh->advertise<std_msgs::String>("send_to_ctower_data", 50);
    can_sub = nh->subscribe("received_messages", 10, &P2ctower_core::CAN_Callback, this);

    // for websocket
    host_ = "localhost";
    port_ = "8080";
    results = resolver.resolve(host_, port_);
    ep = net::connect(ws_.next_layer(), results);
}

void P2ctower_core::CAN_Callback(const can_msgs::Frame::ConstPtr &msg){
    int data[8];
    double time = 0.0 ;
    if (myparser.check_key(_CAN_FB1, "FWS") == OK) {
        if (myparser.decode(_CAN_FB1, data) == OK) {
            push2_ctower("FWS", "L", myparser.get_afd("FWS", "L"), time );
            push2_ctower("FWS", "R", myparser.get_afd("FWS", "L"), time );
            push2_ctower("FWS", "L1", myparser.get_afd("FWS", "L"), time );
            push2_ctower("FWS", "L2", myparser.get_afd("FWS", "L"), time );
            push2_ctower("FWS", "R1", myparser.get_afd("FWS", "L"), time );
            push2_ctower("FWS", "R2", myparser.get_afd("FWS", "L"), time );
        };
    };
    if (myparser.check_key(_CAN_FB2, "THR") == OK) {
        if (myparser.decode(_CAN_FB2, data) == OK) {
            push2_ctower("THR", "A", myparser.get_afd("THR", "A"), time );
            push2_ctower("THR", "B", myparser.get_afd("THR", "B"), time );
        };
    };
    if (myparser.check_key(_CAN_FB2, "STR") == OK) {
        if (myparser.decode(_CAN_FB2, data) == OK) {
            push2_ctower("STR", "N", myparser.get_afd("STR", "N"), time );
        };
    };
    if (myparser.check_key(_CAN_FB2, "FSS") == OK) {
        if (myparser.decode(_CAN_FB2, data) == OK) {
            push2_ctower("FSS", "L", myparser.get_afd("FSS", "L"), time );
            push2_ctower("FSS", "R", myparser.get_afd("FSS", "R"), time );
        };
    };
    if (myparser.check_key(_CAN_FB2, "OPR") == OK) {
        if (myparser.decode(_CAN_FB2, data) == OK) {
            push2_ctower("OPR", "N", myparser.get_afd("OPR", "N"), time );
        };
    };
    if (myparser.check_key(_CAN_RB1, "RWS") == OK) {
        if (myparser.decode(_CAN_RB1, data) == OK) {
            push2_ctower("RWS", "L", myparser.get_afd("RWS", "L"), time );
            push2_ctower("RWS", "R", myparser.get_afd("RWS", "R"), time );
        };
    };
    if (myparser.check_key(_CAN_RB1, "RWT") == OK) {
        if (myparser.decode(_CAN_RB1, data) == OK) {
            push2_ctower("RWT", "L1", myparser.get_afd("RWT", "L1"), time );
            push2_ctower("RWT", "L2", myparser.get_afd("RWT", "L2"), time );
            push2_ctower("RWT", "R1", myparser.get_afd("RWT", "R1"), time );
            push2_ctower("RWT", "R2", myparser.get_afd("RWT", "R2"), time );
        };
    };
    if (myparser.check_key(_CAN_RB2, "RSS") == OK) {
        if (myparser.decode(_CAN_RB2, data) == OK) {
            push2_ctower("RSS", "L", myparser.get_afd("RSS", "L"), time );
            push2_ctower("RSS", "R", myparser.get_afd("RSS", "R"), time );
        };
    };
    if (myparser.check_key(_CAN_HIS, "HIS") == OK) {
        if (myparser.decode(_CAN_HIS, data) == OK) {
            push2_ctower("HIS", "P", myparser.get_afd("HIS", "P"), time );
            push2_ctower("HIS", "R", myparser.get_afd("HIS", "R"), time );
        };
    };
    if (myparser.check_key(_CAN_HIA, "HIA") == OK) {
        if (myparser.decode(_CAN_HIA, data) == OK) {
            push2_ctower("HIA", "X", myparser.get_afd("HIA", "X"), time );
            push2_ctower("HIA", "Y", myparser.get_afd("HIA", "Y"), time );
            push2_ctower("HIA", "Z", myparser.get_afd("HIA", "Z"), time );
        };
    };
    if (myparser.check_key(_CAN_HIG, "HIG") == OK) {
        if (myparser.decode(_CAN_HIG, data) == OK) {
            push2_ctower("HIG", "X", myparser.get_afd("HIG", "X"), time );
            push2_ctower("HIG", "Y", myparser.get_afd("HIG", "Y"), time );
            push2_ctower("HIG", "Z", myparser.get_afd("HIG", "Z"), time );
        };
    };
    if (myparser.check_key(_CAN_OIS, "OIS") == OK) {
        if (myparser.decode(_CAN_OIS, data) == OK) {
            push2_ctower("OIS", "P", myparser.get_afd("OIS", "P"), time );
            push2_ctower("OIS", "R", myparser.get_afd("OIS", "R"), time );
        };
    };
    if (myparser.check_key(_CAN_OIA, "OIA") == OK) {
        if (myparser.decode(_CAN_OIA, data) == OK) {
            push2_ctower("OIA", "X", myparser.get_afd("OIA", "X"), time );
            push2_ctower("OIA", "Y", myparser.get_afd("OIA", "Y"), time );
            push2_ctower("OIA", "Z", myparser.get_afd("OIA", "Z"), time );
        };
    };
    if (myparser.check_key(_CAN_OIG, "OIG") == OK) {
        if (myparser.decode(_CAN_OIG, data) == OK) {
            push2_ctower("OIG", "X", myparser.get_afd("OIG", "X"), time );
            push2_ctower("OIG", "Y", myparser.get_afd("OIG", "Y"), time );
            push2_ctower("OIG", "Z", myparser.get_afd("OIG", "Z"), time );
        };
    };
    if (myparser.check_key(_CAN_OIC, "OIC") == OK) {
        if (myparser.decode(_CAN_OIC, data) == OK) {
            push2_ctower("OIC", "X", myparser.get_afd("OIC", "X"), time );
            push2_ctower("OIC", "Y", myparser.get_afd("OIC", "Y"), time );
            push2_ctower("OIC", "Z", myparser.get_afd("OIC", "Z"), time );
        };
    };
    if (myparser.check_key(_CAN_MCM, "MTC") == OK) {
        if (myparser.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MTC", "N", myparser.get_afd("MTC", "N"), time );
        };
    };
    if (myparser.check_key(_CAN_MCM, "MSC") == OK) {
        if (myparser.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MSC", "N", myparser.get_afd("MSC", "N"), time );
        };
    };
    if (myparser.check_key(_CAN_MCM, "MDC") == OK) {
        if (myparser.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MDC", "N", myparser.get_afd("MDC", "N"), time );
        };
    };
    if (myparser.check_key(_CAN_MCM, "MIE") == OK) {
        if (myparser.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MIE", "N", myparser.get_afd("MIE", "N"), time );
        };
    };
    if (myparser.check_key(_CAN_MCM, "MID") == OK) {
        if (myparser.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MID", "N", myparser.get_afd("MID", "N"), time );
        };
    };
    if (myparser.check_key(_CAN_MCM, "MSM") == OK) {
        if (myparser.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MSM", "N", myparser.get_afd("MSM", "N"), time );
        };
    };
    if (myparser.check_key(_CAN_MCM, "MTL") == OK) {
        if (myparser.decode(_CAN_MCM, data) == OK) {
            push2_ctower("MTL", "N", myparser.get_afd("MTL", "N"), time );
        };
    };
    myparser.print_err_log();
}
