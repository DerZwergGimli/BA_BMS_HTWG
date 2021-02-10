
#ifndef LTC6810_INTERFACE_H
#define LTC6810_INTERFACE_H

#include <Arduino.h>
#include "ArduinoLog.h"
#include "LT_SPI.h"
#include "LTC681x.h"
#include "LTC6810.h"
#include "DataInterface.h"

class LTC6810_Interface
{
private:
    //Check
    void check_error(int8_t error);

    //Print message via Serial
    void print_sid(void);
    void print_conv_time(uint32_t conv_time);
    void print_cells(uint8_t datalog_en);
    void print_aux(uint8_t datalog_en);
    void print_wrcomm(void);
    void serial_print_hex(uint8_t data);
    void print_rxcomm(void);
    void print_rxconfig(void);
    void print_wrconfig(void);
    void print_sumofcells(void);
    void print_svolt(uint8_t datalog_en);
    void print_stat(void);

public:
    LTC6810_Interface(str_SlaveBMUData *str_SlaveBMUData);
    ~LTC6810_Interface();

    //Commands
    void ic_spi_speed();
    void ic_wakeup();
    void ic_writeConfig(bool debug);
    void ic_resetGPIO(bool debug);

    void ic_read_SerialID(bool debug);

    void ic_IC_read_ConfigRegister(bool debug);
    void ic_IC_start_Status(bool debug);
    void ic_IC_read_Status(bool debug);

    void ic_CELL_start_ADCMeasurement(bool debug);
    void ic_CELL_read_VoltageRegister(bool debug);
    void ic_CELL_read_Combined(bool debug);
    void ic_CELL_read_CombinedSum(bool debug);

    void ic_SPIN_read_SVoltageRegister(bool debug);

    void ic_GPIO_start_ADCMeasurement(bool debug);
    void ic_GPIO_read_AuxVoltageRegister(bool debug);

    uint8_t i2c_mux_write(bool debug, uint16_t muxPin);
    uint16_t i2c_mux_read(bool debug);

    //Getter
    uint16_t getGPIOVoltage10000(uint8_t ic, uint8_t gpio);
    uint16_t getCELLVoltage10000(uint8_t ic, int cell);

    //Setter

    //Public Interface
    void ltc6810_pullConfig(str_SlaveBMUData *str_SlaveBMUData, bool debug = false);
    void ltc6810_pushConfig(str_SlaveBMUData *str_SlaveBMUData, bool reset = false, bool debug = false);
    void ltc6810_pullSerialID(str_SlaveBMUData *slaveData, bool debug = false);
    void ltc6810_pullStatusData(str_SlaveBMUData *slaveData, bool debug = false);
    void ltc6810_pullVoltage(str_SlaveBMUData *slaveData, bool debug = false);
    void ltc6810_pullSPinVoltageRegister(str_SlaveBMUData *slaveData, bool debug = false);
    void ltc6810_pullTemperatureData(str_SlaveBMUData *slaveData, bool debug = false);
};
#endif
