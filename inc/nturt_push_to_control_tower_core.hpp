#ifndef NTURT_PUSH_TO_CONTROL_TOWER_CORE__H
#define NTURT_PUSH_TO_CONTROL_TOWER_CORE__H

#include <cp_can_id.hpp>
#include <NTURT_CAN_Parser.hpp>
#include <string>
#include <memory>
#include "ros/ros.h"
#include <cp_can_id.hpp>
#include <NTURT_CAN_Parser.hpp>
#include "can_msgs/Frame.h"
#include "std_msgs/String.h"

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <cstdlib>
#include <string>

#include <iostream>
namespace beast = boost::beast;         // from <boost/beast.hpp>
namespace http = beast::http;           // from <boost/beast/http.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio;            // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp;       // from <boost/asio/ip/tcp.hpp>

// define result state here
#ifndef OK_ERR
#define OK_ERR

#define OK -1
#define ERR 0

#endif

class P2ctower_core {
    public :
        P2ctower_core(std::shared_ptr<ros::NodeHandle> &nh);
        int push2_ctower(std::string type, std::string sub_type, double value, double time );
        int init_websocket();
        void CAN_Callback(const can_msgs::Frame::ConstPtr &msg);

    private :
        Parser myparser;
        std::shared_ptr<ros::NodeHandle> nh_;
        ros::Publisher bridge_pub ;
        ros::Subscriber can_sub ;

        // for websocket start
        std::string host;
        std::string port;
        std::string text;

        net::io_context ioc;
        websocket::stream<tcp::socket> ws{ioc};
        boost::asio::ip::tcp::endpoint ep;
        // for websocket end
};

#endif