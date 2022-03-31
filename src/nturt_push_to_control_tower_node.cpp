#include <sstream>
#include <nturt_push_to_control_tower_core.hpp>

int push2_ctower(std::string type, std::string sub_type, double value, double time );

void CAN_Callback(const can_msgs::Frame::ConstPtr &msg);


// start up topic publisher (publish to nturt bridge to control tower)
int main(int argc, char **argv){
    ros::init(argc, argv, "nturt_push_to_control_tower_node");
    auto nodehandle_ptr = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
    P2ctower_core p2ctower_core(nodehandle_ptr);

    ros::Rate loop_rate(1000);

    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    };

    return OK ;
};


