#include "ros/ros.h"
#include "can_msgs/Frame.h"

#include <iostream>
#include <sstream>
#include <nturt_push_to_control_tower_core.hpp>
#include <NTURT_CAN_Parser.hpp>


int CAN_Callback(const can_msgs::Frame::ConstPtr &msg);

NTURT_CAN_parser* myparser;
P2ctower_core* p2ctower_core;

int main(int argc, char **argv){
    myparser = new NTURT_CAN_parser();
    p2ctower_core = new P2ctower_core();
    myparser.init_parser();

    // start up topic subsciber
    logger_core->setup_all_loggers();

    ros::init(argc, argv, "nturt_push_to_control_tower_core");

    ros::NodeHandle nodehandle;

    ros::Subscriber subscriber = nodehandle.subscribe("received_messages", 10, CAN_Callback);

    std::cout<<"Start Spinning"<<std::endl;

    ros::spin();
    return OK ;
};


int CAN_Callback(const can_msgs::Frame::ConstPtr &msg) {
    int data[8];
    time = 0.0 ;
    if (myparser.check_key(_CAN_FB1, "FWS") == OK) {
        if (myparser.decode(_CAN_FB1, data) == OK) {
            p2ctower_core.push2_ctower("FWS", "L", myparser.get_afd("FWS", "L"), time );
            p2ctower_core.push2_ctower("FWS", "R", myparser.get_afd("FWS", "L"), time );
            p2ctower_core.push2_ctower("FWS", "L1", myparser.get_afd("FWS", "L"), time );
            p2ctower_core.push2_ctower("FWS", "L2", myparser.get_afd("FWS", "L"), time );
            p2ctower_core.push2_ctower("FWS", "R1", myparser.get_afd("FWS", "L"), time );
            p2ctower_core.push2_ctower("FWS", "R2", myparser.get_afd("FWS", "L"), time );
        }
    }
/* #define  0x040AD092 // front box 2 */
    if (myparser.check_key(_CAN_FB2, "THR") == OK) {
        if (myparser.decode(_CAN_FB2, data) == OK) {
            p2ctower_core.push2_ctower("THR", "A", myparser.get_afd("THR", "A"), time );
            p2ctower_core.push2_ctower("THR", "B", myparser.get_afd("THR", "B"), time );
            p2ctower_core.push2_ctower("STR", "N", myparser.get_afd("STR", "N"), time );
            p2ctower_core.push2_ctower("FSS", "L", myparser.get_afd("FSS", "L"), time );
            p2ctower_core.push2_ctower("FSS", "R", myparser.get_afd("FSS", "R"), time );
            p2ctower_core.push2_ctower("OPR", "N", myparser.get_afd("OPR", "N"), time );

/* #define _CAN_RB1 0x040AD093 // rear box 1 */
            p2ctower_core.push2_ctower("RWS", "L", myparser.get_afd("RWS", "L"), time );
            p2ctower_core.push2_ctower("RWS", "R", myparser.get_afd("RWS", "R"), time );
            p2ctower_core.push2_ctower("RWT", "L1", myparser.get_afd("RWT", "L1"), time );
            p2ctower_core.push2_ctower("RWT", "L2", myparser.get_afd("RWT", "L2"), time );
            p2ctower_core.push2_ctower("RWT", "R1", myparser.get_afd("RWT", "R1"), time );
            p2ctower_core.push2_ctower("RWT", "R2", myparser.get_afd("RWT", "R2"), time );

/* #define _CAN_RB2 0x040AD094 // rear box 2 */
            p2ctower_core.push2_ctower("RSS", "L", myparser.get_afd("RSS", "L"), time );
            p2ctower_core.push2_ctower("RSS", "R", myparser.get_afd("RSS", "R"), time );

/* #define _CAN_HIS 0x0CF029E2 // slope frame from Honeywell IMU */
            p2ctower_core.push2_ctower("HIS", "P", myparser.get_afd("HIS", "P"), time );
            p2ctower_core.push2_ctower("HIS", "R", myparser.get_afd("HIS", "R"), time );

/* #define _CAN_HIA 0x08F02DE2 // accelerometer frame from Honeywell IMU */
            p2ctower_core.push2_ctower("HIA", "X", myparser.get_afd("HIA", "X"), time );
            p2ctower_core.push2_ctower("HIA", "Y", myparser.get_afd("HIA", "Y"), time );
            p2ctower_core.push2_ctower("HIA", "Z", myparser.get_afd("HIA", "Z"), time );

/* #define _CAN_HIG 0x0CF02AE2 // gyroscope frame from Honeywell IMU */
            p2ctower_core.push2_ctower("HIG", "X", myparser.get_afd("HIG", "X"), time );
            p2ctower_core.push2_ctower("HIG", "Y", myparser.get_afd("HIG", "Y"), time );
            p2ctower_core.push2_ctower("HIG", "Z", myparser.get_afd("HIG", "Z"), time );

/* HIC,X,0x18FF6AE2,2,4,0,2,0,0,0.00025,-8 */
/* HIC,Y,0x18FF6AE2,2,4,2,4,0,0,0.00025,-8 */
/* HIC,Z,0x18FF6AE2,2,4,4,6,0,0,0.00025,-8 */

/* #define _CAN_OIS 0x0CF029E2 // slope frame from OpenIMU */
            p2ctower_core.push2_ctower("OIS", "P", myparser.get_afd("OIS", "P"), time );
            p2ctower_core.push2_ctower("OIS", "R", myparser.get_afd("OIS", "R"), time );

/* #define _CAN_OIA 0x08F02DE2 // accelerometer frame from OpenIMU */
            p2ctower_core.push2_ctower("OIA", "X", myparser.get_afd("OIA", "X"), time );
            p2ctower_core.push2_ctower("OIA", "Y", myparser.get_afd("OIA", "Y"), time );
            p2ctower_core.push2_ctower("OIA", "Z", myparser.get_afd("OIA", "Z"), time );

/* #define _CAN_OIG 0x0CF02AE2 // gyroscope frame from OpenIMU */
            p2ctower_core.push2_ctower("OIG", "X", myparser.get_afd("OIG", "X"), time );
            p2ctower_core.push2_ctower("OIG", "Y", myparser.get_afd("OIG", "Y"), time );
            p2ctower_core.push2_ctower("OIG", "Z", myparser.get_afd("OIG", "Z"), time );

/* #define _CAN_OIC 0x18FF6AE2 // pose frame from OpenIMU */
            p2ctower_core.push2_ctower("OIC", "X", myparser.get_afd("OIC", "X"), time );
            p2ctower_core.push2_ctower("OIC", "Y", myparser.get_afd("OIC", "Y"), time );
            p2ctower_core.push2_ctower("OIC", "Z", myparser.get_afd("OIC", "Z"), time );

/* #define _CAN_MCM 0x0C0      // inverter */
            p2ctower_core.push2_ctower("MTC", "N", myparser.get_afd("MTC", "N"), time );
            p2ctower_core.push2_ctower("MSC", "N", myparser.get_afd("MSC", "N"), time );
            p2ctower_core.push2_ctower("MDC", "N", myparser.get_afd("MDC", "N"), time );
            p2ctower_core.push2_ctower("MIE", "N", myparser.get_afd("MIE", "N"), time );
            p2ctower_core.push2_ctower("MID", "N", myparser.get_afd("MID", "N"), time );
            p2ctower_core.push2_ctower("MSM", "N", myparser.get_afd("MSM", "N"), time );
            p2ctower_core.push2_ctower("MTL", "N", myparser.get_afd("MTL", "N"), time );
    }



    std::cout << "Front wheel speed left: " << afdfwsl << ", right: " << afdfwsr << std::endl;
    myparser.print_err_log();
    // std::cout<<"can message"<< msg->data[0] <<std::endl;
    return OK;
}
