#pragma once

#include <Automaton.h>
#include "SystemDataInterface.h"

class FSM_SYSTEM : public Machine
{

public:
    enum
    {
        SETUP,
        IDLE,
        CHECK_RAM,
        SLEEP,
        ALARM
    }; // STATES
    enum
    {
        EVT_E_NEXT,
        EVT_E_RESET,
        EVT_E_ALARM,
        EVT_E_SLEEP,
        EVT_E_CHECKRAM,
        ELSE
    }; // EVENTS
    FSM_SYSTEM(void) : Machine(){};
    FSM_SYSTEM &begin(str_SystemData *systemData);
    FSM_SYSTEM &trace(Stream &stream);
    FSM_SYSTEM &trigger(int event);
    int state(void);
    FSM_SYSTEM &onPushreset(Machine &machine, int event = 0);
    FSM_SYSTEM &onPushreset(atm_cb_push_t callback, int idx = 0);
    FSM_SYSTEM &e_next(void);
    FSM_SYSTEM &e_reset(void);
    FSM_SYSTEM &e_alarm(void);
    FSM_SYSTEM &e_sleep(void);
    FSM_SYSTEM &e_checkram(void);

private:
    enum
    {
        ENT_SETUP,
        LP_IDLE,
        ENT_CHECK_RAM,
        ENT_SLEEP,
        EXT_SLEEP
    }; // ACTIONS
    enum
    {
        ON_PUSHRESET,
        CONN_MAX
    }; // CONNECTORS
    atm_connector connectors[CONN_MAX];
    int event(int id);
    void action(int id);

    str_SystemData *ptr_systemData;

    uint32_t timerCheckRam_last, timerSleep_last;
};

/* 
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="FSM_SYSTEM">
    <states>
      <SETUP index="0" on_enter="ENT_SETUP">
        <ELSE>IDLE</ELSE>
      </SETUP>
      <IDLE index="1" on_loop="LP_IDLE">
        <EVT_E_SLEEP>SLEEP</EVT_E_SLEEP>
        <EVT_E_CHECKRAM>CHECK_RAM</EVT_E_CHECKRAM>
      </IDLE>
      <CHECK_RAM index="2" on_enter="ENT_CHECK_RAM">
        <ELSE>IDLE</ELSE>
      </CHECK_RAM>
      <SLEEP index="3" on_enter="ENT_SLEEP" on_exit="EXT_SLEEP">
        <ELSE>SETUP</ELSE>
      </SLEEP>
      <ALARM index="4">
      </ALARM>
    </states>
    <events>
      <EVT_E_RESET index="0" access="MIXED"/>
      <EVT_E_ALARM index="1" access="MIXED"/>
      <EVT_E_SLEEP index="2" access="MIXED"/>
      <EVT_E_CHECKRAM index="3" access="MIXED"/>
    </events>
    <connectors>
      <ONPUSHRESET autostore="0" broadcast="0" dir="PUSH" slots="1"/>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end 
*/
/* 
Automaton::ATML::begin - Automaton Markup Language

<?xml version="1.0" encoding="UTF-8"?>
<machines>
  <machine name="FSM_SYSTEM">
    <states>
      <SETUP index="0" on_enter="ENT_SETUP">
        <EVT_E_NEXT>IDLE</EVT_E_NEXT>
      </SETUP>
      <IDLE index="1" on_loop="LP_IDLE">
        <EVT_E_SLEEP>SLEEP</EVT_E_SLEEP>
        <EVT_E_CHECKRAM>CHECK_RAM</EVT_E_CHECKRAM>
      </IDLE>
      <CHECK_RAM index="2" on_enter="ENT_CHECK_RAM">
        <EVT_E_NEXT>IDLE</EVT_E_NEXT>
      </CHECK_RAM>
      <SLEEP index="3" on_enter="ENT_SLEEP" on_exit="EXT_SLEEP">
        <EVT_E_NEXT>SETUP</EVT_E_NEXT>
      </SLEEP>
      <ALARM index="4">
      </ALARM>
    </states>
    <events>
      <EVT_E_NEXT index="0" access="MIXED"/>
      <EVT_E_RESET index="1" access="MIXED"/>
      <EVT_E_ALARM index="2" access="MIXED"/>
      <EVT_E_SLEEP index="3" access="MIXED"/>
      <EVT_E_CHECKRAM index="4" access="MIXED"/>
    </events>
    <connectors>
      <PUSHRESET autostore="0" broadcast="0" dir="PUSH" slots="1"/>
    </connectors>
    <methods>
    </methods>
  </machine>
</machines>

Automaton::ATML::end 
*/