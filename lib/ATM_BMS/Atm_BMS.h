#pragma once

#include <Automaton.h>


class Atm_BMS: public Machine {

 public:
  enum { SETUP, IDLE, CHARGING, DISCHARGING, BALANCING, BALANCE_GROUPA, BALANCE_GROUPB, SLEEP, ERROR }; // STATES
  enum { EVT_T_SLEEP, EVT_E_IDLE, EVT_E_CHARGE, EVT_E_DISCHARGE, EVT_E_ERROR, ELSE }; // EVENTS
  Atm_BMS( void ) : Machine() {};
  Atm_BMS& begin( void );
  Atm_BMS& trace( Stream & stream );
  Atm_BMS& trigger( int event );
  int state( void );
  Atm_BMS& t_sleep( void );
  Atm_BMS& e_idle( void );
  Atm_BMS& e_charge( void );
  Atm_BMS& e_discharge( void );
  Atm_BMS& e_error( void );

 private:
  enum {  }; // ACTIONS
  int event( int id ); 
  void action( int id ); 

};