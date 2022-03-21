#include <nturt_push_to_control_tower_core.hpp>
#include <cp_can_id.hpp>
#include <NTURT_CAN_Parser.hpp>

int P2ctower_core::push2_ctower(std::string type, std::string sub_type, double value, double time ) {
    std::cout << "type: " << type << "  subtype: " << sub_type << "  value: " << value << "  time: " << time << std::endl ;
    return OK;
};

int P2ctower_core::init_websocket(){
    return OK;
};
