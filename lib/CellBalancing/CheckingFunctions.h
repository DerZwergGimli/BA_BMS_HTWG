#ifndef CHECKING_FUNCTIONS_H
#define CHECKING_FUNCTIONS_H
#include "Arduino.h"
#include "ArduinoLog.h"
#include "DataInterface.h"
#include "LTC6810_Interface.h"

extern int TOTAL_IC;
uint TOTAL_CELLS = 7;
uint TOTAL_TEMPSENSORS = 8;

bool checkForErrorOverVoltage(str_SlaveBMUData *bmuData)
{
    for (uint8_t ic = 0; ic < TOTAL_IC; ic++)
    {
        uint16_t overVoltage10000 = bmuData[0].config.overVoltage10000;
        for (uint8_t cell = 0; cell < TOTAL_CELLS; cell++)
        {
            if (overVoltage10000 < bmuData[ic].cells[cell].voltage10000)
            {
                Log.fatal("\t OverVoltage detected! [IC=%i; Cell=%i]" CR, ic, cell + 1);
                return true;
            }
        }
    }
    return false;
}

bool checkForErrorUnderVoltage(str_SlaveBMUData *bmuData)
{
    for (size_t ic = 0; ic < TOTAL_IC; ic++)
    {
        uint16_t underVoltage10000 = bmuData[0].config.underVoltage10000;
        for (size_t cell = 0; cell < TOTAL_CELLS; cell++)
        {
            if (underVoltage10000 > bmuData[ic].cells[cell].voltage10000)
            {
                Log.fatal("\t UnderVoltage detected! [IC=%i; Cell=%i]" CR, ic, cell + 1);
                return true;
            }
        }
    }
    return false;
}

bool checkForErrorOverTemperature(str_SlaveBMUData *bmuData)
{
    for (size_t ic = 0; ic < TOTAL_IC; ic++)
    {
        uint16_t overTemperature10 = bmuData[0].config.overTemperature10;
        for (size_t sensor = 0; sensor < TOTAL_TEMPSENSORS; sensor++)
        {
            if (overTemperature10 < bmuData[ic].temperatureExternal[sensor].temperature10)
            {
                Log.fatal("\t OverTemperature detected! [IC=%i; Sensor=%i]" CR, ic, sensor + 1);
                return true;
            }
        }
    }
    return false;
}

bool checkForErrorUnderTemperature(str_SlaveBMUData *bmuData)
{
    for (size_t ic = 0; ic < TOTAL_IC; ic++)
    {
        uint16_t underTemperature10 = bmuData[0].config.underTemperature10;
        for (size_t sensor = 0; sensor < TOTAL_TEMPSENSORS; sensor++)
        {
            if (underTemperature10 > bmuData[ic].temperatureExternal[sensor].temperature10)
            {
                Log.fatal("\t UnderTemperature detected! [IC=%i; Sensor=%i]" CR, ic, sensor + 1);
                return true;
            }
        }
    }
    return false;
}

bool checkForErrorICTemperature(str_SlaveBMUData *bmuData)
{
    for (size_t ic = 0; ic < TOTAL_IC; ic++)
    {
        uint16_t overTemperature10 = bmuData[0].config.overTemperature10;
        if (overTemperature10 < bmuData[ic].temperatureIC.temperature10)
        {
            Log.fatal("\t OverTemperature of IC detected! [IC=%i]" CR, ic);
            return true;
        }
    }
    return false;
}
#endif