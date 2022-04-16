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

#include <iostream>

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
        int close_connection();

    private :
        Parser myparser;
        std::shared_ptr<ros::NodeHandle> nh_;
        ros::Publisher bridge_pub ;
        ros::Subscriber can_sub ;

        std::string host_;
        std::string const port_;
        // The io_context is required for all I/O
        net::io_context ioc;
        // These objects perform our I/O
        tcp::resolver resolver_{ioc};
        websocket::stream<tcp::socket> ws_{ioc};
        // Look up the domain name
        boost::asio::ip::basic_resolver_results<boost::asio::ip::tcp> const results;
        // Make the connection on the IP address we get from a lookup
        tcp::endpoint ep;

};

#endif
