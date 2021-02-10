#include "Atm_BMS.h"

/* Add optional parameters for the state machine to begin()
 * Add extra initialization code
 */

Atm_BMS &Atm_BMS::begin()
{
  // clang-format off
  const static state_t state_table[] PROGMEM = {
    /*                   ON_ENTER  ON_LOOP  ON_EXIT  EVT_T_SLEEP      EVT_E_IDLE  EVT_E_CHARGE  EVT_E_DISCHARGE  EVT_E_ERROR  ELSE */
    /*          SETUP */       -1,      -1,      -1,          -1,             -1,           -1,              -1,       ERROR, IDLE,
    /*           IDLE */       -1,      -1,      -1,       SLEEP,      BALANCING,     CHARGING,     DISCHARGING,       ERROR,   -1,
    /*       CHARGING */       -1,      -1,      -1,          -1,           IDLE,           -1,            IDLE,       ERROR,   -1,
    /*    DISCHARGING */       -1,      -1,      -1,          -1,           IDLE,         IDLE,              -1,       ERROR,   -1,
    /*      BALANCING */       -1,      -1,      -1,          -1, BALANCE_GROUPA,           -1,              -1,       ERROR,   -1,
    /* BALANCE_GROUPA */       -1,      -1,      -1,          -1, BALANCE_GROUPB,           -1,              -1,       ERROR,   -1,
    /* BALANCE_GROUPB */       -1,      -1,      -1,          -1,           IDLE,           -1,              -1,       ERROR,   -1,
    /*          SLEEP */       -1,      -1,      -1,          -1,             -1,           -1,              -1,       ERROR,   -1,
    /*          ERROR */       -1,      -1,      -1,          -1,             -1,           -1,              -1,       ERROR,   -1,
  };
  // clang-format on
  Machine::begin(state_table, ELSE);
  return *this;
}

/* Add C++ code for each internally handled event (input) 
 * The code must return 1 to trigger the event
 */

int Atm_BMS::event(int id)
{
  switch (id)
  {
  case EVT_T_SLEEP:
    return 0;
  case EVT_E_IDLE:
    return 0;
  case EVT_E_CHARGE:
    return 0;
  case EVT_E_DISCHARGE:
    return 0;
  case EVT_E_ERROR:
    return 0;
  }
  return 0;
}

/* Add C++ code for each action
 * This generates the 'output' for the state machine
 */

void Atm_BMS::action(int id)
{
  switch (id)
  {
  }
}

/* Optionally override the default trigger() method
 * Control how your machine processes triggers
 */

Atm_BMS &Atm_BMS::trigger(int event)
{
  Machine::trigger(event);
  return *this;
}

/* Optionally override the default state() method
 * Control what the machine returns when another process requests its state
 */

int Atm_BMS::state(void)
{
  return Machine::state();
}

/* Nothing customizable below this line                          
 ************************************************************************************************
*/

/* Public event methods
 *
 */

Atm_BMS &Atm_BMS::t_sleep()
{
  trigger(EVT_T_SLEEP);
  return *this;
}

Atm_BMS &Atm_BMS::e_idle()
{
  trigger(EVT_E_IDLE);
  return *this;
}

Atm_BMS &Atm_BMS::e_charge()
{
  trigger(EVT_E_CHARGE);
  return *this;
}

Atm_BMS &Atm_BMS::e_discharge()
{
  trigger(EVT_E_DISCHARGE);
  return *this;
}

Atm_BMS &Atm_BMS::e_error()
{
  trigger(EVT_E_ERROR);
  return *this;
}

/* State trace method
 * Sets the symbol table and the default logging method for serial monitoring
 */

Atm_BMS &Atm_BMS::trace(Stream &stream)
{
  Machine::setTrace(&stream, atm_serial_debug::trace,
                    "BMS\0EVT_T_SLEEP\0EVT_E_IDLE\0EVT_E_CHARGE\0EVT_E_DISCHARGE\0EVT_E_ERROR\0ELSE\0SETUP\0IDLE\0CHARGING\0DISCHARGING\0BALANCING\0BALANCE_GROUPA\0BALANCE_GROUPB\0SLEEP\0ERROR");
  return *this;
}