#include "nturt_push_to_control_tower_core.hpp"

// start up topic publisher (publish to nturt bridge to control tower)
int main(int argc, char **argv){

	// register as a ros node
    ros::init(argc, argv, "nturt_push_to_control_tower_node");

    // create a node handle
    auto nodehandle_ptr = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());

    // initialize torque controller
    P2ctower_core p2ctower_core(nodehandle_ptr);
    /* p2ctower_core.init_websocket("124.218.222.22", "8080"); */

    // frequancy 1000 Hz
    ros::Rate loop_rate(1000);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    };

    std::cout << "end succed" << std::endl;
    return OK ;
};
