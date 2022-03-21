#ifndef NTURT_PUSH_TO_CONTROL_TOWER_CORE__H
#define NTURT_PUSH_TO_CONTROL_TOWER_CORE__H

#include <string>

// define result state here
#ifndef OK_ERR
#define OK_ERR

#define OK -1
#define ERR 0

#endif

class P2ctower_core {
    public :
        int push2_ctower(std::string type, std::string sub_type, double value, double time );
        int init_websocket();
};

#endif
