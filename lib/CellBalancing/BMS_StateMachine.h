#ifndef CELL_BALANCING_H
#define CELL_BALANCING_H
#include "StateMachine.h"
#include "Arduino.h"
#include "DataInterface.h"
#include "LTC6810_Interface.h"
#include "ArduinoLog.h"
#include "CheckingFunctions.h"
#include "Chrono.h"
#include "helper_deepSleep.h"

#include "projects.h"

//extern int TOTAL_IC;
// uint TOTAL_CELLS = 7;
// uint TOTAL_TEMPSENSORS = 8;
#define TIMEOUTBALANCING 10000
#define TIMEOUTDEEPSLEEP 100000
#define DISPLAY_POWER_PIN 2

enum SubState
{
    NONE,
    CHARGEABLE,
    DISCHAGEABLE,
    BALANCEABLE
};

StateMachine machine = StateMachine();
str_SlaveBMUData *ptr_bmuDataStateMachine;
LTC6810_Interface *ptr_ltc6810StateMachine;
Chrono timeoutBalancing;
Chrono timeoutDeepSleep;

bool errorFound = false;
SubState subState = NONE;
uint8_t highestCellIndex[1];
uint8_t lowestCellIndex[1];

void state_Initial();
void state_Idle();
void state_ReadData();
void state_CheckData();
void state_Error();
void state_Work();
void state_CheckBalance();
void state_BalanceGroupA();
void state_BalanceGroupB();
void state_DeepSleep();

bool t_initalized();
bool t_tick();
bool t_errorAcknowledged();
bool t_errorFound();
bool t_noErrorFound();
bool t_balanceable();
bool t_noBalanceNeeded();
bool t_initBalanceGroupA();
bool t_balanceGroupB();
bool t_balanceTimerPassed();
bool t_charging();
bool t_discharging();
bool t_deepSleepTimerPassed();

State *S0_Initial = machine.addState(&state_Initial);
State *S1_Idle = machine.addState(&state_Idle);
State *S2_ReadData = machine.addState(&state_ReadData);
State *S3_CheckData = machine.addState(&state_CheckData);
State *S100_Error = machine.addState(&state_Error);
State *S4_Work = machine.addState(&state_Work);
State *S50_CheckBalance = machine.addState(&state_CheckBalance);

State *S51_BalanceGroupA = machine.addState(&state_BalanceGroupA);
State *S52_BalanceGroupB = machine.addState(&state_BalanceGroupB);

State *S90_DeepSleep = machine.addState(&state_DeepSleep);

void setupCellBalancing(LTC6810_Interface *ltc6810Interface, str_SlaveBMUData *slaveBMUData)
{
    ptr_bmuDataStateMachine = slaveBMUData;
    ptr_ltc6810StateMachine = ltc6810Interface;

    S0_Initial->addTransition(&t_initalized, S1_Idle);
    S1_Idle->addTransition(&t_tick, S2_ReadData);
    S100_Error->addTransition(&t_errorAcknowledged, S1_Idle);
    S2_ReadData->addTransition(&t_tick, S3_CheckData);
    S3_CheckData->addTransition(&t_errorFound, S100_Error);
    S3_CheckData->addTransition(&t_noErrorFound, S4_Work);

    //S2_ReadData->addTransition(&t_noErrorFound, S4_Work);
    S4_Work->addTransition(t_deepSleepTimerPassed, S90_DeepSleep);
    S4_Work->addTransition(&t_balanceable, S50_CheckBalance);
    S50_CheckBalance->addTransition(&t_initBalanceGroupA, S51_BalanceGroupA);
    S51_BalanceGroupA->addTransition(&t_balanceTimerPassed, S52_BalanceGroupB);
    S50_CheckBalance->addTransition(&t_noBalanceNeeded, S1_Idle);
    S52_BalanceGroupB->addTransition(&t_balanceTimerPassed, S1_Idle);

    S90_DeepSleep->addTransition(t_tick, S0_Initial);
}
void runStateMachine()
{
    machine.run();
}

//State Functions
void state_Initial()
{

    if (machine.executeOnce)
    {
        while (!Serial)
        {
            ;
        }
        Log.notice("--> STATE: INITIAL" CR);
        timeoutDeepSleep.restart();
        snoozeSetup();

        digitalWriteFast(DISPLAY_POWER_PIN, HIGH);
        delay(1);
        InitGUIslice_gen(ptr_ltc6810StateMachine, ptr_bmuDataStateMachine);
        // gslc_SetPageCur(&m_gui, E_PG_BOOT);
        // gslc_Update(&m_gui);
        delay(1);
        gslc_SetPageCur(&m_gui, E_PG_HOME);
        gslc_Update(&m_gui);

        drawStatusMessage_BOX("STATE: Initial");
    }
    subState = NONE;
}
void state_Idle()
{
    if (machine.executeOnce)
    {
        Log.notice("--> STATE: IDLE" CR);
        drawStatusMessage_BOX("STATE: Idle");
    }
}
void state_ReadData()
{
    if (machine.executeOnce)
    {
        Log.notice("--> STATE: ReadData" CR);
        drawStatusMessage_BOX("STATE: ReadData");
        ptr_ltc6810StateMachine->ic_spi_speed();
        ptr_ltc6810StateMachine->ltc6810_pullSerialID(ptr_bmuDataStateMachine);
        ptr_ltc6810StateMachine->ltc6810_pullVoltage(ptr_bmuDataStateMachine);
        ptr_ltc6810StateMachine->ltc6810_pullTemperatureData(ptr_bmuDataStateMachine);
        ptr_ltc6810StateMachine->ltc6810_pullStatusData(ptr_bmuDataStateMachine);
    }
}
void state_CheckData()
{
    errorFound = false;
    if (machine.executeOnce)
    {
        Log.notice("--> STATE: CheckData" CR);
        drawStatusMessage_BOX("STATE: CheckData");

        //Check for OverVoltage
        if (checkForErrorOverVoltage(ptr_bmuDataStateMachine) == true)
        {
            errorFound = true;
        }

        //Check for UnderVoltage
        if (checkForErrorUnderVoltage(ptr_bmuDataStateMachine) == true)
        {
            errorFound = true;
        }

        //Check for OverTemperature
        if (checkForErrorOverTemperature(ptr_bmuDataStateMachine) == true)
        {
            errorFound = true;
        }

        //Check for UnderTemperature
        if (checkForErrorUnderTemperature(ptr_bmuDataStateMachine) == true)
        {
            errorFound = true;
        }

        //Check for ICTemperature
        if (checkForErrorICTemperature(ptr_bmuDataStateMachine) == true)
        {
            errorFound = true;
        }
    }
}
void state_Error()
{
    if (machine.executeOnce)
    {
        Log.notice("--> STATE: ERROR" CR);
        drawStatusMessage_BOX("STATE: Error");
    }
}
void state_Work()
{
    if (machine.executeOnce)
    {
        Log.notice("--> STATE: WORK" CR);
        drawStatusMessage_BOX("STATE: Work");
    }

    //If there is no current flow than balancing can be done
    subState = BALANCEABLE;
}
void state_CheckBalance()
{
    if (machine.executeOnce)
    {
        Log.notice("--> STATE: CheckBalance" CR);
        //timeoutBalancing.restart();
    }

    //Find highest and lowest Cell
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++)
    {
        for (uint8_t cell = 0; cell < TOTAL_CELLS; cell++)
        {
            if (ptr_bmuDataStateMachine[ic].cells[cell].voltage10000 > ptr_bmuDataStateMachine[ic].cells[highestCellIndex[ic]].voltage10000)
            {
                highestCellIndex[ic] = cell;
            }
            if (ptr_bmuDataStateMachine[ic].cells[cell].voltage10000 < ptr_bmuDataStateMachine[ic].cells[lowestCellIndex[ic]].voltage10000)
            {
                lowestCellIndex[ic] = cell;
            }
        }
        ptr_bmuDataStateMachine[ic].config.makredForBalacing = false;
    }

    //Check for Thresholds
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++)
    {
        uint16_t thresholdVoltage10000 = ptr_bmuDataStateMachine[ic].config.thresholdVoltage10000;
        uint16_t highestCellVoltage10000 = ptr_bmuDataStateMachine[ic].cells[highestCellIndex[ic]].voltage10000;
        uint16_t lowestCellVoltage10000 = ptr_bmuDataStateMachine[ic].cells[lowestCellIndex[ic]].voltage10000;

        if ((highestCellVoltage10000 - lowestCellVoltage10000 > thresholdVoltage10000))
        {
            Serial.printf("Highest=%i \t Lowest=%i \n", highestCellIndex[0] + 1, lowestCellIndex[0] + 1);
            Log.notice("Balancing Allowed" CR);
            ptr_bmuDataStateMachine[ic].config.makredForBalacing = true;
        }
    }
}
void state_BalanceGroupA()
{
    if (machine.executeOnce)
    {
        Log.notice("--> STATE: Balance Group A" CR);
        timeoutBalancing.restart();
        //Build balance Group A
        for (uint8_t ic = 0; ic < TOTAL_IC; ic++)
        {
            for (uint8_t cell = 0; cell < TOTAL_CELLS; cell++)
            {
                uint16_t currentCellVoltage10000 = ptr_bmuDataStateMachine[ic].cells[cell].voltage10000;
                uint16_t balanceCellVoltageThesholdTotal10000 = ptr_bmuDataStateMachine[ic].cells[lowestCellIndex[ic]].voltage10000 + ptr_bmuDataStateMachine[ic].config.thresholdVoltage10000;
                if (currentCellVoltage10000 > balanceCellVoltageThesholdTotal10000)
                {
                    if (cell % 2 == 0)
                    {
                        ptr_bmuDataStateMachine[ic].config.dischargeCellBits[cell] = true;
                    }
                    else
                    {
                        ptr_bmuDataStateMachine[ic].config.dischargeCellBits[cell] = false;
                    }
                }
            }
        }

        Serial.print("DCTO: ");
        for (uint8_t ic = 0; ic < TOTAL_IC; ic++)
        {
            for (uint8_t cell = 0; cell < TOTAL_CELLS; cell++)
            {
                Serial.printf("%i", ptr_bmuDataStateMachine[ic].config.dischargeCellBits[cell]);
            }
        }
        Serial.println();
    }
}
void state_BalanceGroupB()
{
    if (machine.executeOnce)
    {
        Log.notice("--> STATE: Balance Group B" CR);
        timeoutBalancing.restart();

        //Build balance Group B
        for (uint8_t ic = 0; ic < TOTAL_IC; ic++)
        {
            for (uint8_t cell = 0; cell < TOTAL_CELLS; cell++)
            {
                uint16_t currentCellVoltage10000 = ptr_bmuDataStateMachine[ic].cells[cell].voltage10000;
                uint16_t balanceCellVoltageThesholdTotal10000 = ptr_bmuDataStateMachine[ic].cells[lowestCellIndex[ic]].voltage10000 + ptr_bmuDataStateMachine[ic].config.thresholdVoltage10000;

                if (currentCellVoltage10000 > balanceCellVoltageThesholdTotal10000)
                {
                    if (cell % 2 == 1)
                    {
                        ptr_bmuDataStateMachine[ic].config.dischargeCellBits[cell] = true;
                    }
                    else
                    {
                        ptr_bmuDataStateMachine[ic].config.dischargeCellBits[cell] = false;
                    }
                }
            }
        }

        Serial.print("DCTO: ");
        for (uint8_t ic = 0; ic < TOTAL_IC; ic++)
        {
            for (uint8_t cell = 0; cell < TOTAL_CELLS; cell++)
            {
                Serial.printf("%i", ptr_bmuDataStateMachine[ic].config.dischargeCellBits[cell]);
            }
        }
        Serial.println();
    }
}

void state_DeepSleep()
{

    if (machine.executeOnce)
    {
        Log.notice("... Entering Deep Sleep");
        digitalWriteFast(DISPLAY_POWER_PIN, LOW);
        delay(1000);
        snoozeDeepSleep();
    }
}
//State Transition Functions
bool t_initalized()
{
    return true;
}
bool t_tick()
{
    return true;
}
bool t_errorAcknowledged()
{
    return true;
}
bool t_errorFound()
{
    if (errorFound == true)
    {
        return true;
    }
    return false;
}
bool t_noErrorFound()
{
    if (errorFound == false)
    {
        return true;
    }
    return false;
}
bool t_balanceable()
{
    //TODO: make a current detection
    if (subState == BALANCEABLE)
    {
        return true;
    }
    return false;
}
bool t_noBalanceNeeded()
{
    return true;
}
bool t_initBalanceGroupA()
{
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++)
    {
        if (ptr_bmuDataStateMachine[ic].config.makredForBalacing == true)
        {
            timeoutDeepSleep.restart();
            return true;
        }
    }
    return false;
}
bool t_balanceGroupB()
{
    if (timeoutBalancing.hasPassed(TIMEOUTBALANCING))
    {
        return true;
    }
    return false;
}

bool t_balanceTimerPassed()
{
    if (timeoutBalancing.hasPassed(TIMEOUTBALANCING))
    {
        return true;
    }

    return false;
}

bool t_deepSleepTimerPassed()
{
    if (timeoutDeepSleep.hasPassed(TIMEOUTDEEPSLEEP))
    {
        timeoutDeepSleep.restart();
        return true;
    }
    return false;
}
#endif