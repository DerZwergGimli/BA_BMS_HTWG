#include "FSM_SYSTEM.h"
#include "ReportRam.h"
#include "SystemDataInterface.h"

extern const int DISPLAY_PIN_ENABLE;

bool nextState = false;

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

FSM_SYSTEM &FSM_SYSTEM::begin(str_SystemData *systemData)
{
    // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                   ON_ENTER  ON_LOOP    ON_EXIT  EVT_E_NEXT  EVT_E_RESET  EVT_E_ALARM  EVT_E_SLEEP  EVT_E_CHECKRAM  ELSE */
    /*     SETUP */     ENT_SETUP,      -1,        -1,       IDLE,          -1,          -1,          -1,             -1,   -1,
    /*      IDLE */            -1, LP_IDLE,        -1,         -1,          -1,          -1,       SLEEP,      CHECK_RAM,   -1,
    /* CHECK_RAM */ ENT_CHECK_RAM,      -1,        -1,       -1,          -1,          -1,          -1,             -1,   IDLE,
    /*     SLEEP */     ENT_SLEEP,      -1, EXT_SLEEP,      SETUP,          -1,          -1,          -1,             -1,   -1,
    /*     ALARM */            -1,      -1,        -1,         -1,          -1,          -1,          -1,             -1,   -1,
  };
    // clang-format on
    Machine::begin(state_table, ELSE);

    ptr_systemData = systemData;
    ram.initialize();

    return *this;
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int FSM_SYSTEM::event(int id)
{

    switch (id)
    {
    case EVT_E_NEXT:
        if (nextState)
        {
            nextState = false;
            return 1;
        }
        else
            return 0;
    case EVT_E_RESET:
        return 0;
    case EVT_E_ALARM:
        return 0;
    case EVT_E_SLEEP:

        if (millis() - timerSleep_last > ptr_systemData->timerSleep)
        {
            timerSleep_last = millis();
            return 1;
        }
        ptr_systemData->timerSleep_Remaining = ptr_systemData->timerSleep - (millis() - timerSleep_last);
        return 0;
    case EVT_E_CHECKRAM:
        if (millis() - timerCheckRam_last > ptr_systemData->timerMemory)
        {
            timerCheckRam_last = millis();
            return 1;
        }
        ptr_systemData->timerMemory_Remaining = ptr_systemData->timerMemory - (millis() - timerCheckRam_last);
        return 0;
    }
    return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 *
 * Available connectors:
 *   push( connectors, ON_PUSHRESET, 0, <v>, <up> );
 */

void FSM_SYSTEM::action(int id)
{
    switch (id)
    {
    case ENT_SETUP:
        nextState = true;
        return;
    case LP_IDLE:
        ram.run();
        return;
    case ENT_CHECK_RAM:
        report_ram();
        return;
    case ENT_SLEEP:
        digitalWriteFast(DISPLAY_PIN_ENABLE, LOW);
        //Do some sleep stuff
        delay(5000);
        nextState = true;
        timerSleep_last = millis();
        return;
    case EXT_SLEEP:
        push(connectors, ON_PUSHRESET, 0, 0, 0);
        return;
    }
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

FSM_SYSTEM &FSM_SYSTEM::trigger(int event)
{
    Machine::trigger(event);
    return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int FSM_SYSTEM::state(void)
{
    return Machine::state();
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

FSM_SYSTEM &FSM_SYSTEM::e_next()
{
    trigger(EVT_E_NEXT);
    return *this;
}

FSM_SYSTEM &FSM_SYSTEM::e_reset()
{
    trigger(EVT_E_RESET);
    return *this;
}

FSM_SYSTEM &FSM_SYSTEM::e_alarm()
{
    trigger(EVT_E_ALARM);
    return *this;
}

FSM_SYSTEM &FSM_SYSTEM::e_sleep()
{
    trigger(EVT_E_SLEEP);
    return *this;
}

FSM_SYSTEM &FSM_SYSTEM::e_checkram()
{
    trigger(EVT_E_CHECKRAM);
    return *this;
}

/*
 * onPushreset() push connector variants ( slots 1, autostore 0, broadcast 0 )
 */

FSM_SYSTEM &FSM_SYSTEM::onPushreset(Machine &machine, int event)
{
    onPush(connectors, ON_PUSHRESET, 0, 1, 1, machine, event);
    return *this;
}

FSM_SYSTEM &FSM_SYSTEM::onPushreset(atm_cb_push_t callback, int idx)
{
    onPush(connectors, ON_PUSHRESET, 0, 1, 1, callback, idx);
    return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

FSM_SYSTEM &FSM_SYSTEM::trace(Stream &stream)
{
    Machine::setTrace(&stream, atm_serial_debug::trace,
                      "FSM_SYSTEM\0EVT_E_NEXT\0EVT_E_RESET\0EVT_E_ALARM\0EVT_E_SLEEP\0EVT_E_CHECKRAM\0ELSE\0SETUP\0IDLE\0CHECK_RAM\0SLEEP\0ALARM");
    return *this;
}
