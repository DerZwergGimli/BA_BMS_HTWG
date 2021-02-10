#pragma once

#include <Automaton.h>
#include <DataInterface.h>
#include <LTC6810_Interface.h>
#include <SystemDataInterface.h>

class FSM_BatteryMonitor : public Machine
{

public:
    enum
    {
        SETUP,
        IDLE,
        READ_SLAVES,
        CHECK_DATA,
        ERROR
    }; // STATES
    enum
    {
        EVT_T_READOUT,
        EVT_E_ERROR,
        EVT_E_NOERROR,
        ELSE
    }; // EVENTS
    FSM_BatteryMonitor(void) : Machine(){};
    FSM_BatteryMonitor &begin(str_SystemData *pSystemData, LTC6810_Interface *pLTC6810_Interface, str_SlaveBMUData *pSlaveBMUData);
    FSM_BatteryMonitor &trace(Stream &stream);
    FSM_BatteryMonitor &trigger(int event);
    int state(void);
    FSM_BatteryMonitor &t_readout(void);
    FSM_BatteryMonitor &e_error(void);
    FSM_BatteryMonitor &e_noerror(void);

    FSM_BatteryMonitor &automatic(int timerreadout);

private:
    atm_timer_millis timer_ReadOut;

    str_SystemData *ptr_systemData;
    LTC6810_Interface *ptr_ltc6810Interface;
    str_SlaveBMUData *ptr_slaveBMUData;

    enum
    {
        ENT_READ_SLAVES,
        ENT_CHECK_DATA
    }; // ACTIONS
    int event(int id);
    void action(int id);
};