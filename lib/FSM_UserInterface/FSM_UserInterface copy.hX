#pragma once

#include <Automaton.h>
#include "LTC6810_Interface.h"
#include "DataInterface.h"
#include "SystemDataInterface.h"

extern const int DISPLAY_PIN_ENABLE;
extern const int TOTAL_NUMBER_OF_IC;

class FSM_UserInterface : public Machine
{

public:
  enum
  {
    SETUP,
    RESET,
    UPDATE
  }; // STATES
  enum
  {
    EVT_E_UPDATE,
    EVT_C_RESET,
    ELSE
  }; // EVENTS
  FSM_UserInterface(void) : Machine(){};
  FSM_UserInterface &begin(LTC6810_Interface *pltc6810Interface, str_SlaveBMUData *pslaveBMUData, uint32_t timerUpdateInterval);
  FSM_UserInterface &trace(Stream &stream);
  FSM_UserInterface &trigger(int event);
  int state(void);
  FSM_UserInterface &e_update(void);
  FSM_UserInterface &c_reset(void);

private:
  enum
  {
    ENT_SETUP,
    ENT_RESET,
    ENT_UPDATE
  }; // ACTIONS
  int event(int id);
  void action(int id);

  uint32_t timerUpdate_interval;
  uint32_t timerUpdate_last;
  LTC6810_Interface *ptr_ltc6810Interface;
  str_SlaveBMUData *ptr_slaveBMUData;
};

/*
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="FSM_UserInterface">
    <states>
      <SETUP index="0" on_enter="ENT_SETUP">
        <ELSE>UPDATE</ELSE>
      </SETUP>
      <RESET index="1" on_enter="ENT_RESET">
        <EVT_E_UPDATE>UPDATE</EVT_E_UPDATE>
      </RESET>
      <UPDATE index="2" on_enter="ENT_UPDATE">
        <EVT_C_RESET>RESET</EVT_C_RESET>
        <ELSE>SETUP</ELSE>
      </UPDATE>
    </states>
    <events>
      <EVT_E_UPDATE index="0" access="MIXED"/>
      <EVT_C_RESET index="1" access="MIXED"/>
    </events>
    <connectors>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end
*/
