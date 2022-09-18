#ifndef NTURT_PUSH_TO_CONTROL_TOWER_CORE_HPP
#define NTURT_PUSH_TO_CONTROL_TOWER_CORE_HPP

// std include
#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

// boost include
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

// ros include
#include "ros/ros.h"

// ros mags include
#include "can_msgs/Frame.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"

// nturt include
#include "can_parser.hpp"

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

#endif // OK_ERR

class P2ctower_core {
    public :
        P2ctower_core(std::shared_ptr<ros::NodeHandle> &nh);
        int push2_ctower(std::string type, std::string sub_type, double value, double time );
        int init_websocket();
        void CAN_Callback(const can_msgs::Frame::ConstPtr &msg);
        void GPS_Callback(const nav_msgs::Odometry::ConstPtr &msg);


    private :
        CanParser myparser_;
        std::shared_ptr<ros::NodeHandle> nh_;
        ros::Publisher bridge_pub_ ;
        ros::Subscriber can_sub_ ;
        ros::Subscriber gps_sub_ ;
        std::vector<std::pair<std::string,std::string>> can_data_;

        // for csv logger
        std::string csv_log_db_path_;
        std::string csv_log_buf_;

        // for websocket start
        std::string host_;
        std::string port_;
        std::string text_;

        net::io_context ioc_;
        websocket::stream<tcp::socket> ws_{ioc_};
        boost::asio::ip::tcp::endpoint ep_;
        // for websocket end

        // for csv logger start
        int log_to_csv_(std::string file_name);
        int csv_log_buf_append_(double one_data);
        double get_afd_value;
        // for csv logger end
};

#endif // NTURT_PUSH_TO_CONTROL_TOWER_CORE_HPP
