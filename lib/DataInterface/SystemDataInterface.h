
#pragma once

const int DISPLAY_PIN_ENABLE = 2;
const int TOTAL_NUMBER_OF_IC = 1;

enum balancingStates
{
    SETUP,
    IDLE,
    READ_SLAVES,
    CHECK_DATA,
    ERROR
}; // STATES of Balancing

struct str_SystemData
{

    bool balancingEnabled = false;

    uint32_t timerSleep;
    uint32_t timerSleep_Remaining;
    uint32_t timerMemory;
    uint32_t timerMemory_Remaining;
    uint32_t timerDisplayUpdate;
    uint32_t timerDisplayUpdate_Remaining;
    uint32_t timerLTC68;
    uint32_t timerLTC68_Remaining;

    uint16_t maxAllowedVoltage10000;
    uint16_t minAllowedVoltage10000;
    uint16_t balancingVoltageThreshold10000;

    int32_t maxAlowedTemperature10;
    int32_t minAllowedTemperature10;

    balancingStates activeBalancingState;
};
