/**
 * @file nturt_push_to_control_tower_core.hpp
 * @author Jack (b10502016@ntu.edu.tw)
 * @brief a ROS package to get data from CAN sensor and GPS sensor, then push them to the sever (124.218.222.22). The server package repo is [here](https://github.com/NTURacingTeam/ROS_server)
 */

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
// #include "nav_msgs/Odometry.h"
#include "gps_common/GPSFix"
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

/**
 * @brief Brief explanation of the class.
 * @author Your name
 */
class P2ctower_core {
    public :
        P2ctower_core(std::shared_ptr<ros::NodeHandle> &nh);
        
        /**
         * @brief Brief explanation of the function.
         * @param[in] _argument Brief explanation of the argument.
         * @return Brief explanation of the return.
         */
        int push2_ctower(std::string type, std::string sub_type, double value, double time );
        
        /**
         * @brief Initialize websocket daemon.
         * @param[in] host the IP of the host
         * @param port the port of the websocket tunnel
         */
        int init_websocket(std::string host, std::string port);

        /// @brief A callback function every time 
        void CAN_Callback(const can_msgs::Frame::ConstPtr &msg);
        
        // void GPS_Callback(const nav_msgs::Odometry::ConstPtr &msg);
        
        /// @brief 
        void GPS_Callback(const gps_common::GPSFix::ConstPtr &msg);


    private :

        /// @brief Containes can parser.
        CanParser myparser_;

        /// @brief Node handler.
        std::shared_ptr<ros::NodeHandle> nh_;

        // ros::Publisher bridge_pub_ ;

        /// @brief Subscriber for the topic about CAN data.
        ros::Subscriber can_sub_ ;

        /// @brief Subscirber fot the topic about GPS data
        ros::Subscriber gps_sub_ ;

        /// @brief Vector storing CAN data, used by can parser.
        std::vector<std::pair<std::string,std::string>> can_data_;

        // for csv logger

        // std::string csv_log_db_path_;
        // std::string csv_log_buf_;

        // for websocket start
        
        /// @brief The IP of the host server.
        std::string host_;

        /// @brief The specified port to use to connect to eh server. 
        std::string port_;

        /// @brief Io_context for resolver_ and ws_. See detail in [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
        net::io_context ioc_;

        /// @brief Resolver. See detail in [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
        tcp::resolver resolver_{ioc_};

        /// @brief Websocket holder. See detail in [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
        websocket::stream<tcp::socket> ws_{ioc_};

        /// @brief I don't know what this is. See detail in [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
        boost::asio::ip::tcp::endpoint ep_;

        // for csv logger start
        // int log_to_csv_(std::string file_name);
        // int csv_log_buf_append_(double one_data);
        // double get_afd_value;
        // for csv logger end
};

#endif // NTURT_PUSH_TO_CONTROL_TOWER_CORE_HPP
