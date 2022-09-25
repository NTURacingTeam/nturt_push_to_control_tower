#include "nturt_push_to_control_tower_core.hpp"

// start up topic publisher (publish to nturt bridge to control tower)
int main(int argc, char **argv){
    ros::init(argc, argv, "nturt_push_to_control_tower_node");
    auto nodehandle_ptr = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
    P2ctower_core p2ctower_core(nodehandle_ptr);
    p2ctower_core.init_websocket("124.218.222.22", "8080");
    std::cout << "init succed" << std::endl;
    ros::Rate loop_rate(1000);

    std::cout << "loop rate succed" << std::endl;
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    };

    std::cout << "end succed" << std::endl;
    return OK ;
};
