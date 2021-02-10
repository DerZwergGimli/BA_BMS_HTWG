#include "FSM_BatteryMonitor.h"
#include "FSM_BatteryMonitorHelper.h"
/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

FSM_BatteryMonitor &FSM_BatteryMonitor::begin(str_SystemData *pSystemData, LTC6810_Interface *pLTC6810_Interface, str_SlaveBMUData *pSlaveBMUData)
{
    ptr_systemData = ptr_systemData;
    ptr_ltc6810Interface = pLTC6810_Interface;
    ptr_slaveBMUData = pSlaveBMUData;

    // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                ON_ENTER  ON_LOOP  ON_EXIT  EVT_T_READOUT  EVT_E_ERROR  EVT_E_NOERROR        ELSE */
    /*       SETUP */       -1,      -1,      -1,            -1,          -1,            -1,       IDLE,
    /*        IDLE */       -1,      -1,      -1,   READ_SLAVES,          -1,            -1,         -1,
    /* READ_SLAVES */       ENT_READ_SLAVES,      -1,      -1,            -1,          -1,            -1, CHECK_DATA,
    /*  CHECK_DATA */       ENT_CHECK_DATA,      -1,      -1,            -1,       ERROR,          IDLE,         -1,
    /*       ERROR */       -1,      -1,      -1,            -1,          -1,            -1,       IDLE,
  };
    // clang-format on
    Machine::begin(state_table, ELSE);
    return *this;
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int FSM_BatteryMonitor::event(int id)
{
    switch (id)
    {
    case EVT_T_READOUT:
        return timer_ReadOut.expired(this);
    case EVT_E_ERROR:
        return !fsm_batteryMonitor_CheckData(ptr_slaveBMUData);
    case EVT_E_NOERROR:
        return fsm_batteryMonitor_CheckData(ptr_slaveBMUData);
    }
    return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 */

void FSM_BatteryMonitor::action(int id)
{
    switch (id)
    {
    case ENT_READ_SLAVES:
        fsm_batteryMonitor_ReadData(ptr_ltc6810Interface, ptr_slaveBMUData);
        return;
    case ENT_CHECK_DATA:
        return;
    }
}

FSM_BatteryMonitor &FSM_BatteryMonitor::automatic(int timerreadout)
{
    timer_ReadOut.set(timerreadout);
    return *this;
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

FSM_BatteryMonitor &FSM_BatteryMonitor::trigger(int event)
{
    Machine::trigger(event);
    return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int FSM_BatteryMonitor::state(void)
{
    return Machine::state();
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

FSM_BatteryMonitor &FSM_BatteryMonitor::t_readout()
{
    trigger(EVT_T_READOUT);
    return *this;
}

FSM_BatteryMonitor &FSM_BatteryMonitor::e_error()
{
    trigger(EVT_E_ERROR);
    return *this;
}

FSM_BatteryMonitor &FSM_BatteryMonitor::e_noerror()
{
    trigger(EVT_E_NOERROR);
    return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

FSM_BatteryMonitor &FSM_BatteryMonitor::trace(Stream &stream)
{
    Machine::setTrace(&stream, atm_serial_debug::trace,
                      "FSM_BATTERYMONITOR\0EVT_T_READOUT\0EVT_E_ERROR\0EVT_E_NOERROR\0ELSE\0SETUP\0IDLE\0READ_SLAVES\0CHECK_DATA\0ERROR");
    return *this;
}