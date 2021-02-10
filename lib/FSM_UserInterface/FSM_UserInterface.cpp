
#include "FSM_UserInterface.h"
#include "projects.h"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

FSM_UserInterface &FSM_UserInterface::begin(str_SystemData *psystemData, LTC6810_Interface *pltc6810Interface, str_SlaveBMUData *pslaveBMUData, uint32_t timerUpdateInterval)
{
    // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*             ON_ENTER  ON_LOOP  ON_EXIT  EVT_C_RESET  EVT_E_UPDATE    ELSE */
    /*  SETUP */  ENT_SETUP,      -1,      -1,          -1,           -1, UPDATE,
    /* UPDATE */ ENT_UPDATE,      -1,      -1,       RESET,       UPDATE,     -1,
    /*  RESET */  ENT_RESET,      -1,      -1,       -1,           SETUP,     -1,
  };
    // clang-format on
    Machine::begin(state_table, ELSE);

    ptr_systemData = psystemData;
    ptr_ltc6810Interface = pltc6810Interface;
    ptr_slaveBMUData = pslaveBMUData;
    //timerUpdate_interval = timerUpdateInterval;

    pinMode(DISPLAY_PIN_ENABLE, OUTPUT);

    return *this;
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int FSM_UserInterface::event(int id)
{
    switch (id)
    {
    case EVT_C_RESET:
        return 0;
    case EVT_E_UPDATE:
        if (millis() - timerUpdate_last > ptr_systemData->timerDisplayUpdate)
        {
            timerUpdate_last = millis();
            return 1;
        }
        ptr_systemData->timerDisplayUpdate_Remaining = ptr_systemData->timerDisplayUpdate - (millis() - timerUpdate_last);
        return 0;
    }
    return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 */

void FSM_UserInterface::action(int id)
{
    switch (id)
    {
    case ENT_SETUP:
        digitalWriteFast(DISPLAY_PIN_ENABLE, HIGH);
        delay(100);
        InitGUIslice_gen(ptr_systemData, ptr_ltc6810Interface, ptr_slaveBMUData);
        gslc_SetPageCur(&m_gui, E_PG_BOOT);
        gslc_Update(&m_gui);
        gslc_SetPageCur(&m_gui, E_PG_HOME);
        gslc_Update(&m_gui);
        return;
    case ENT_UPDATE:
        switch (gslc_GetPageCur(&m_gui))
        {
        case E_PG_HOME:
            drawHome_PAGE(ptr_slaveBMUData, TOTAL_NUMBER_OF_IC);
            break;
        case E_PG_MONITOR:
            drawCellMonitor_PAGE(ptr_slaveBMUData, 0);
            break;
        case E_PG_SETTINGS_BMS:;
            drawSettingsBMS_PAGE(ptr_systemData);
            break;
        case E_PG_SETTINGS_BALANCING:
            drawBalancing_PAGE(ptr_systemData);
            drawStatusBalancingStatus_BOX("TEST");
            break;
        default:
            break;
        }
        gslc_Update(&m_gui);
        return;
    case ENT_RESET:
        return;
    }
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

FSM_UserInterface &FSM_UserInterface::trigger(int event)
{
    Machine::trigger(event);
    return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int FSM_UserInterface::state(void)
{
    return Machine::state();
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

FSM_UserInterface &FSM_UserInterface::c_reset()
{
    trigger(EVT_C_RESET);
    return *this;
}

FSM_UserInterface &FSM_UserInterface::e_update()
{
    trigger(EVT_E_UPDATE);
    return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

FSM_UserInterface &FSM_UserInterface::trace(Stream &stream)
{
    Machine::setTrace(&stream, atm_serial_debug::trace,
                      "FSM_USERINTERFACE\0EVT_C_RESET\0EVT_E_UPDATE\0ELSE\0SETUP\0UPDATE\0RESET");
    return *this;
}
