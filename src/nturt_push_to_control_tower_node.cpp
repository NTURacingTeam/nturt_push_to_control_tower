#include "ros/ros.h"
#include "can_msgs/Frame.h"

#include <iostream>
#include <sstream>
#include <nturt_push_to_control_tower_core.hpp>
#include <cp_can_id.hpp>
#include <NTURT_CAN_Parser.hpp>


void CAN_Callback(const can_msgs::Frame::ConstPtr &msg);

NTURT_CAN_parser* myparser;
P2ctower_core* p2ctower_core;

int main(int argc, char **argv){
    myparser = new NTURT_CAN_parser();
    p2ctower_core = new P2ctower_core();
    myparser->init_parser();
    p2ctower_core->init_websocket();

    // start up topic publisher (publish to nturt bridge to control tower)
    ros::init(argc, argv, "send_to_ctower_data");
    ros::NodeHandle send_to_ctower_data_nodehandler;
    ros::Publisher chatter_pub = send_to_ctower_data_nodehandler.advertise<std_msgs::String>("send_to_ctower_data", 50);

    // start up topic subsciber
    ros::init(argc, argv, "nturt_push_to_control_tower_core");

    ros::NodeHandle nodehandle;

    ros::Subscriber subscriber = nodehandle.subscribe("received_messages", 10, CAN_Callback);

    std::cout<<"Start Spinning"<<std::endl;

    ros::spin();
    return OK ;
};


void CAN_Callback(const can_msgs::Frame::ConstPtr &msg){
    int data[8];
    double time = 0.0 ;
    if (myparser->check_key(_CAN_FB1, "FWS") == OK) {
        if (myparser->decode(_CAN_FB1, data) == OK) {
            p2ctower_core->push2_ctower("FWS", "L", myparser->get_afd("FWS", "L"), time );
            p2ctower_core->push2_ctower("FWS", "R", myparser->get_afd("FWS", "L"), time );
            p2ctower_core->push2_ctower("FWS", "L1", myparser->get_afd("FWS", "L"), time );
            p2ctower_core->push2_ctower("FWS", "L2", myparser->get_afd("FWS", "L"), time );
            p2ctower_core->push2_ctower("FWS", "R1", myparser->get_afd("FWS", "L"), time );
            p2ctower_core->push2_ctower("FWS", "R2", myparser->get_afd("FWS", "L"), time );
        };
    };
    if (myparser->check_key(_CAN_FB2, "THR") == OK) {
        if (myparser->decode(_CAN_FB2, data) == OK) {
            p2ctower_core->push2_ctower("THR", "A", myparser->get_afd("THR", "A"), time );
            p2ctower_core->push2_ctower("THR", "B", myparser->get_afd("THR", "B"), time );
        };
    };
    if (myparser->check_key(_CAN_FB2, "STR") == OK) {
        if (myparser->decode(_CAN_FB2, data) == OK) {
            p2ctower_core->push2_ctower("STR", "N", myparser->get_afd("STR", "N"), time );
        };
    };
    if (myparser->check_key(_CAN_FB2, "FSS") == OK) {
        if (myparser->decode(_CAN_FB2, data) == OK) {
            p2ctower_core->push2_ctower("FSS", "L", myparser->get_afd("FSS", "L"), time );
            p2ctower_core->push2_ctower("FSS", "R", myparser->get_afd("FSS", "R"), time );
        };
    };
    if (myparser->check_key(_CAN_FB2, "OPR") == OK) {
        if (myparser->decode(_CAN_FB2, data) == OK) {
            p2ctower_core->push2_ctower("OPR", "N", myparser->get_afd("OPR", "N"), time );
        };
    };
    if (myparser->check_key(_CAN_RB1, "RWS") == OK) {
        if (myparser->decode(_CAN_RB1, data) == OK) {
            p2ctower_core->push2_ctower("RWS", "L", myparser->get_afd("RWS", "L"), time );
            p2ctower_core->push2_ctower("RWS", "R", myparser->get_afd("RWS", "R"), time );
        };
    };
    if (myparser->check_key(_CAN_RB1, "RWT") == OK) {
        if (myparser->decode(_CAN_RB1, data) == OK) {
            p2ctower_core->push2_ctower("RWT", "L1", myparser->get_afd("RWT", "L1"), time );
            p2ctower_core->push2_ctower("RWT", "L2", myparser->get_afd("RWT", "L2"), time );
            p2ctower_core->push2_ctower("RWT", "R1", myparser->get_afd("RWT", "R1"), time );
            p2ctower_core->push2_ctower("RWT", "R2", myparser->get_afd("RWT", "R2"), time );
        };
    };
    if (myparser->check_key(_CAN_RB2, "RSS") == OK) {
        if (myparser->decode(_CAN_RB2, data) == OK) {
            p2ctower_core->push2_ctower("RSS", "L", myparser->get_afd("RSS", "L"), time );
            p2ctower_core->push2_ctower("RSS", "R", myparser->get_afd("RSS", "R"), time );
        };
    };
    if (myparser->check_key(_CAN_HIS, "HIS") == OK) {
        if (myparser->decode(_CAN_HIS, data) == OK) {
            p2ctower_core->push2_ctower("HIS", "P", myparser->get_afd("HIS", "P"), time );
            p2ctower_core->push2_ctower("HIS", "R", myparser->get_afd("HIS", "R"), time );
        };
    };
    if (myparser->check_key(_CAN_HIA, "HIA") == OK) {
        if (myparser->decode(_CAN_HIA, data) == OK) {
            p2ctower_core->push2_ctower("HIA", "X", myparser->get_afd("HIA", "X"), time );
            p2ctower_core->push2_ctower("HIA", "Y", myparser->get_afd("HIA", "Y"), time );
            p2ctower_core->push2_ctower("HIA", "Z", myparser->get_afd("HIA", "Z"), time );
        };
    };
    if (myparser->check_key(_CAN_HIG, "HIG") == OK) {
        if (myparser->decode(_CAN_HIG, data) == OK) {
            p2ctower_core->push2_ctower("HIG", "X", myparser->get_afd("HIG", "X"), time );
            p2ctower_core->push2_ctower("HIG", "Y", myparser->get_afd("HIG", "Y"), time );
            p2ctower_core->push2_ctower("HIG", "Z", myparser->get_afd("HIG", "Z"), time );
        };
    };
    if (myparser->check_key(_CAN_OIS, "OIS") == OK) {
        if (myparser->decode(_CAN_OIS, data) == OK) {
            p2ctower_core->push2_ctower("OIS", "P", myparser->get_afd("OIS", "P"), time );
            p2ctower_core->push2_ctower("OIS", "R", myparser->get_afd("OIS", "R"), time );
        };
    };
    if (myparser->check_key(_CAN_OIA, "OIA") == OK) {
        if (myparser->decode(_CAN_OIA, data) == OK) {
            p2ctower_core->push2_ctower("OIA", "X", myparser->get_afd("OIA", "X"), time );
            p2ctower_core->push2_ctower("OIA", "Y", myparser->get_afd("OIA", "Y"), time );
            p2ctower_core->push2_ctower("OIA", "Z", myparser->get_afd("OIA", "Z"), time );
        };
    };
    if (myparser->check_key(_CAN_OIG, "OIG") == OK) {
        if (myparser->decode(_CAN_OIG, data) == OK) {
            p2ctower_core->push2_ctower("OIG", "X", myparser->get_afd("OIG", "X"), time );
            p2ctower_core->push2_ctower("OIG", "Y", myparser->get_afd("OIG", "Y"), time );
            p2ctower_core->push2_ctower("OIG", "Z", myparser->get_afd("OIG", "Z"), time );
        };
    };
    if (myparser->check_key(_CAN_OIC, "OIC") == OK) {
        if (myparser->decode(_CAN_OIC, data) == OK) {
            p2ctower_core->push2_ctower("OIC", "X", myparser->get_afd("OIC", "X"), time );
            p2ctower_core->push2_ctower("OIC", "Y", myparser->get_afd("OIC", "Y"), time );
            p2ctower_core->push2_ctower("OIC", "Z", myparser->get_afd("OIC", "Z"), time );
        };
    };
    if (myparser->check_key(_CAN_MCM, "MTC") == OK) {
        if (myparser->decode(_CAN_MCM, data) == OK) {
            p2ctower_core->push2_ctower("MTC", "N", myparser->get_afd("MTC", "N"), time );
        };
    };
    if (myparser->check_key(_CAN_MCM, "MSC") == OK) {
        if (myparser->decode(_CAN_MCM, data) == OK) {
            p2ctower_core->push2_ctower("MSC", "N", myparser->get_afd("MSC", "N"), time );
        };
    };
    if (myparser->check_key(_CAN_MCM, "MDC") == OK) {
        if (myparser->decode(_CAN_MCM, data) == OK) {
            p2ctower_core->push2_ctower("MDC", "N", myparser->get_afd("MDC", "N"), time );
        };
    };
    if (myparser->check_key(_CAN_MCM, "MIE") == OK) {
        if (myparser->decode(_CAN_MCM, data) == OK) {
            p2ctower_core->push2_ctower("MIE", "N", myparser->get_afd("MIE", "N"), time );
        };
    };
    if (myparser->check_key(_CAN_MCM, "MID") == OK) {
        if (myparser->decode(_CAN_MCM, data) == OK) {
            p2ctower_core->push2_ctower("MID", "N", myparser->get_afd("MID", "N"), time );
        };
    };
    if (myparser->check_key(_CAN_MCM, "MSM") == OK) {
        if (myparser->decode(_CAN_MCM, data) == OK) {
            p2ctower_core->push2_ctower("MSM", "N", myparser->get_afd("MSM", "N"), time );
        };
    };
    if (myparser->check_key(_CAN_MCM, "MTL") == OK) {
        if (myparser->decode(_CAN_MCM, data) == OK) {
            p2ctower_core->push2_ctower("MTL", "N", myparser->get_afd("MTL", "N"), time );
        };
    };
    myparser->print_err_log();

    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    send_to_ctower_data_nodehandler(msg);
}
