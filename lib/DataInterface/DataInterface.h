#ifndef DATA_INTERFACE_H
#define DATA_INTERFACE_H
#include <Arduino.h>

#define NUMBEROFCELLS 7
#define NUMBEROFTEMPERATURESENSORS 8

struct str_CellData
{
    uint16_t voltage10000;
};

struct str_TemperatureData
{
    int32_t temperature10;
    int32_t temperatureMAX10;
    int32_t temperatureMIN10;
};

struct str_Config
{
    uint16_t overVoltage10000 = 40000;
    uint16_t underVoltage10000 = 20000;
    uint16_t thresholdVoltage10000 = 5000;
    int32_t overTemperature10 = 300;
    int32_t underTemperature10 = 0;
    bool makredForBalacing;

    bool gpio[4];
    bool dischargeCellBits[7];
    bool referenceOn;
    bool dischargeTimerEnable;
    bool adcOPTMode;
    bool multiCalibration;
    uint8_t dischargeTimeOutValue;
    bool dischargeTimeOutValueBits[4];
    bool enableCellMeasurementRedundancySPIn;
    bool forceDigitalRedundancyFailure;
    bool disableDigitalRedundancyCheck;
    bool enableDischargeTimerMonitor;
};

struct str_SlaveBMUData
{
    str_Config config;

    uint8_t *serialID;
    uint16_t errorCommunicationCount;
    uint32_t totalVolatge10000;
    uint16_t totalStateOfCharge;

    uint32_t VregA10000;
    uint32_t VregB10000;

    str_CellData cells[NUMBEROFCELLS];
    str_CellData sPins[NUMBEROFCELLS];
    str_TemperatureData temperatureIC;
    str_TemperatureData temperatureExternal[NUMBEROFTEMPERATURESENSORS];
    //bool dischargeBits[6];
};

#endif