#include "nturt_push_to_control_tower_core.hpp"

// start up topic publisher (publish to nturt bridge to control tower)
int main(int argc, char **argv){
    const std::string ws_host_ip = "124.218.222.22";
    const std::string ws_host_port =  "8080";

	// register as a ros node
    ros::init(argc, argv, "nturt_push_to_control_tower_node");

    // create a node handle
    auto nodehandle_ptr = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

    // initialize torque controller
    P2ctower_core p2ctower_core(nodehandle_ptr);
    p2ctower_core.init_websocket(ws_host_ip ,ws_host_port);

    // frequancy 1000 Hz
    ros::Rate loop_rate(1000);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    };

    std::cout << "end succed" << std::endl;
    return OK ;
};
