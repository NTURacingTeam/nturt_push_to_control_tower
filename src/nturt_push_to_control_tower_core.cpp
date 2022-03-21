#include <nturt_push_to_control_tower_core.hpp>
#include <cp_can_id.hpp>
#include <NTURT_CAN_Parser.hpp>

int P2ctower_core::push2_ctower(string type, string sub_type, double value, double time );
    s{td::cout << "type: " << type << "  subtype: " << sub_type << "  value: " << value << "  time: " << time << std::endl ;
    return OK;
};

int P2ctower_core::init_websocket(){
    return OK;
};
