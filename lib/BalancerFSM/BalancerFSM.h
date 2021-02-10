#ifndef BALANCER_FSM_H
#define BALANCER_FSM_H
#include <Arduino.h>
#include "DataInterface.h"

enum e_BalncerFSM
{
    INITIAL,
    RESET,
    CHECK_TEMPERATURE,
    FIND_LOWESTCELL,
    BALANCE_GROUP_A,
    BALANCE_GROUP_B

};

class BalancerFSM
{
private:
    /* data */
public:
    BalancerFSM(uint totalIC);
    ~BalancerFSM();

    bool run(str_SlaveBMUData *slaveBMUData);
};

#endif