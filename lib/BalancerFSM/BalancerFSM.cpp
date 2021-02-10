#include "BalancerFSM.h"
#include "ArduinoLog.h"

uint TOTAL_IC;

e_BalncerFSM state;

#define NUMBER_OF_CELLS 7
#define NUMBER_OF_EXTTEMPERATREPROBES 8
#define MIN_BALANCIING_TEMPERATURE 0 //DegreeCelcius
#define MAX_BALANCING_TEMPERATURE 30 //DegreeCelcius

#define MIN_BALANCING_VOLTAGE 20000
#define MAX_BALANCING_VOLTAGE 40000

uint16_t lowestCellVoltage = 1000000;

bool cellsMarkedForBalance[NUMBER_OF_CELLS];
int cellsGroupA[(NUMBER_OF_CELLS + 1) / 2];
int cellsGroupB[(NUMBER_OF_CELLS + 1) / 2];

BalancerFSM::BalancerFSM(uint totalIC)
{
    TOTAL_IC = totalIC;
}

BalancerFSM::~BalancerFSM()
{
}

bool BalancerFSM::run(str_SlaveBMUData *slaveBMUData)
{

    switch (state)
    {
    case INITIAL:
    {
        state = CHECK_TEMPERATURE;
        Log.notice("BFSM: INITIAL -> CHECK_TEMPERATURE" CR);
    }
    break;

    case CHECK_TEMPERATURE:
    {
        //Check for Temperatures
        uint tempCheckCount = 0;
        for (int ic = 0; ic < TOTAL_IC; ic++)
        {
            for (size_t temp = 0; temp < NUMBER_OF_EXTTEMPERATREPROBES; temp++)
            {
                int tempeature = slaveBMUData[ic].temperatureExternal[temp].temperature10 / 10;
                if ((tempeature > MIN_BALANCIING_TEMPERATURE) && (tempeature < MAX_BALANCING_TEMPERATURE))
                {
                    tempCheckCount++;
                }
            }
        }
        if (tempCheckCount == NUMBER_OF_EXTTEMPERATREPROBES)
        {
            state = FIND_LOWESTCELL;
            lowestCellVoltage = 100000;
            Log.notice("BFSM: CHECK_TEMPERATURE -> FIND_LOWESTCELL" CR);
        }
        else
        {
            Log.warning("BFSM: Temperature check failed!" CR);
            Log.notice("BFSM: CHECK_TEMPERATURE -> INITIAL" CR);
        }
    }
    break;
    case FIND_LOWESTCELL:
    {
        for (size_t ic = 0; ic < TOTAL_IC; ic++)
        {
            for (size_t cell = 0; cell < NUMBER_OF_CELLS; cell++)
            {
                if (slaveBMUData[ic].cells[cell].voltage10000 < lowestCellVoltage)
                {
                    lowestCellVoltage = slaveBMUData[ic].cells[cell].voltage10000;
                }
            }
        }
        lowestCellVoltage += 1000;
        Serial.printf("lowestCellVoltage=%i \n", lowestCellVoltage);
        if (lowestCellVoltage > MIN_BALANCING_VOLTAGE && lowestCellVoltage < MAX_BALANCING_VOLTAGE)
        {
            state = BALANCE_GROUP_A;
            Log.notice("BFSM: FIND_LOWESTCELL -> BALANCE_GROUP_A" CR);
        }
        else
        {
            state = INITIAL;
            Log.notice("Cell voltage is to low please ChargePack!" CR);
            Log.notice("BFSM: FIND_LOWESTCELL -> INITIAL" CR);
        }
    }
    break;
    case BALANCE_GROUP_A:
    {
        for (size_t ic = 0; ic < TOTAL_IC; ic++)
        {
            for (size_t cell = 0; cell < NUMBER_OF_CELLS; cell++)
            {
                if (cell % 2 == 0)
                {
                    if (slaveBMUData[ic].cells[cell].voltage10000 > lowestCellVoltage)
                    {
                        slaveBMUData[ic].config.dischargeCellBits[cell] = true;
                    }
                    else
                    {
                        slaveBMUData[ic].config.dischargeCellBits[cell] = false;
                    }
                }
                else
                {
                    slaveBMUData[ic].config.dischargeCellBits[cell] = false;
                }
            }
        }

        Serial.print("DCCBITS: ");
        for (size_t ic = 0; ic < TOTAL_IC; ic++)
        {
            for (size_t cell = 0; cell < NUMBER_OF_CELLS; cell++)
            {
                Serial.printf("%i, ", slaveBMUData[ic].config.dischargeCellBits[cell]);
            }
        }
        Serial.println();

        state = BALANCE_GROUP_B;
        Log.notice("BFSM: BALANCE_GROUP_A -> BALANCE_GROUP_B" CR);
    }
    break;
    case BALANCE_GROUP_B:
    {
        for (size_t ic = 0; ic < TOTAL_IC; ic++)
        {
            for (size_t cell = 0; cell < NUMBER_OF_CELLS; cell++)
            {
                if (cell % 2 == 1)
                {
                    if (slaveBMUData[ic].cells[cell].voltage10000 > lowestCellVoltage)
                    {
                        slaveBMUData[ic].config.dischargeCellBits[cell] = true;
                    }
                    else
                    {
                        slaveBMUData[ic].config.dischargeCellBits[cell] = false;
                    }
                }
                else
                {
                    slaveBMUData[ic].config.dischargeCellBits[cell] = false;
                }
            }
        }

        Serial.print("DCCBITS: ");
        for (size_t ic = 0; ic < TOTAL_IC; ic++)
        {
            for (size_t cell = 0; cell < NUMBER_OF_CELLS; cell++)
            {
                Serial.printf("%i, ", slaveBMUData[ic].config.dischargeCellBits[cell]);
            }
        }
        Serial.println();

        state = RESET;
        Log.notice("BFSM: BALANCE_GROUP_B -> RESET" CR);
    }
    break;

    case RESET:
    {
        for (size_t ic = 0; ic < TOTAL_IC; ic++)
        {
            for (size_t cell = 0; cell < NUMBER_OF_CELLS; cell++)
            {
                slaveBMUData[ic].config.dischargeCellBits[cell] = false;
            }
        }
        state = INITIAL;
        Log.notice("BFSM: RESET -> INITIAL" CR);
    }
    break;
    default:
    {
        state = INITIAL;
        Log.notice("BFSM: DEFAULT -> INITIAL" CR);
    }
    break;
    }

    for (size_t ic = 0; ic < TOTAL_IC; ic++)
    {
        for (size_t cell = 0; cell < NUMBER_OF_CELLS; cell++)
        {
            if (slaveBMUData[ic].config.dischargeCellBits[cell] == true)
            {
                return true;
            }
        }
    }
    return false;
}