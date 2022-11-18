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
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

// ros mags include
#include "can_msgs/Frame.h"
// #include "nav_msgs/Odometry.h"
#include "gps_common/GPSFix.h"
#include "std_msgs/String.h"

// nturt include
#include "nturt_ros_interface/GetCanData.h"
#include "nturt_ros_interface/RegisterCanNotification.h"
#include "nturt_ros_interface/UpdateCanData.h"

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
         * @brief Initialize websocket daemon.
         * @param[in] host the IP of the host
         * @param port the port of the websocket tunnel
         */
        int init_websocket(std::string host, std::string port);

    private :

        /**
         * @brief Brief explanation of the function.
         * @param[in] _argument Brief explanation of the argument.
         * @return Brief explanation of the return.
         */
        int push2_ctower(std::string name, double value, double time );

        /// @brief A callback function every time 
        void CAN_Callback(const can_msgs::Frame::ConstPtr &msg);

        // void GPS_Callback(const nav_msgs::Odometry::ConstPtr &msg);

        /// @brief
        void GPS_Callback(const gps_common::GPSFix::ConstPtr &msg);

        /// @brief Callback function when receiving message form can data notification.
        void onNotification(const nturt_ros_interface::UpdateCanData::ConstPtr &_msg);

        /// @brief Callback function when receiving message from topic "/node_state".
        void onState(const std_msgs::Bool::ConstPtr &_msg);

        /// @brief Pointer to ros node handle.
        std::shared_ptr<ros::NodeHandle> nh_;

        /// @brief Publisher to "/publish_can_frame", for publishing can frames.
        ros::Publisher publish_frame_pub_;

        /// @brief Subscriber to can data notification topic, for getting can data when they got updated.
        ros::Subscriber notification_sub_;

        /// @brief Subscriber to "/node_state", for being controlled by nturt_state_controller.
        ros::Subscriber state_sub_;

        /// @brief Service client to "/register_can_notification", for registering to notification.
        ros::ServiceClient register_clt_;

        /// @brief Subscriber for the topic about CAN data.
        ros::Subscriber can_sub_ ;

        /// @brief Subscirber fot the topic about GPS data
        ros::Subscriber gps_sub_ ;

        /// @brief Vector storing CAN data, used by can parser.
        std::vector<std::pair<std::string,std::string>> can_data_;


        // for websocket start

        /// @brief timer to reconnect to ws if disconnected
        ros::Timer check_ws_connection_timer_ ;

        /// @brief check if websockt connection is on. If not, try to reconnect
        void timer_check_and_retry_websocket_connection(const ros::TimerEvent&);

        // @brief if websocket is connected, set true
        bool ws_connected_ ;

        /// @brief The IP of the host server.
        std::string ws_hostIP_;

        /// @brief The specified port to use to connect to eh server. 
        std::string ws_port_;

        /// @brief Io_context for resolver_ and ws_. See detail in [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
        net::io_context ioc_;

        /// @brief Resolver. See detail in [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
        tcp::resolver resolver_{ioc_};

        /// @brief Websocket holder. See detail in [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
        websocket::stream<tcp::socket> ws_{ioc_};

        /// @brief I don't know what this is. See detail in [here](https://www.boost.org/doc/libs/master/libs/beast/example/websocket/client/async/websocket_client_async.cpp).
        boost::asio::ip::tcp::endpoint ep_;

        // data input
        /// @brief Signal to activate this node controlled by topic "node_state".
        bool is_activated_ = false;

        // internal control parameters
        /// @brief Last timestemp when "update" function is called [s].
        double timestemp_last_;

        // accelerator pedal position sensor (apps)
        /// @brief Time duration to trigger accelerator pedal position sensor error.
        double apps_duration_ = 0;

};

#endif // NTURT_PUSH_TO_CONTROL_TOWER_CORE_HPP

