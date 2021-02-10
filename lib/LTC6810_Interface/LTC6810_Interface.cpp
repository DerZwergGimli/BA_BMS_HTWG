#include "LTC6810_Interface.h"
#include "Thermistor.h"

#define ISOSPI_CS1 10

/************************* Defines *****************************/
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define SPI_CLK_DEVIDER SPI_CLOCK_DIV64 //SPI_CLOCK_DIV16

/********************************************************************
  Setup Variables
  The following variables can be modified to configure the software.
*********************************************************************/
const uint8_t TOTAL_IC = 1;    //!< Number of ICs in the daisy chain
const uint8_t TOTAL_CELLS = 7; //Number of Cells per IC

//ADC Command Configurations. See LTC681x.h for options.
const uint8_t ADC_OPT = ADC_OPT_DISABLED;         //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED;             //!< Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL;   //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL;     //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL;   //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL;              //!< Register Selection
const uint8_t SEL_REG_A = REG_1;                  //!< Register Selection
const uint8_t SEL_REG_B = REG_2;                  //!< Register Selection

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000;        //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 20000;        //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(1V)
const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)
//Loop Measurement Setup. These Variables are ENABLED or DISABLED Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = DISABLED;  //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED;  //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = DISABLED;  //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = DISABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = DISABLED;    //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop
                                       /************************************
  END SETUP
*************************************/

/******************************************************
 Global Battery Variables received from 681x commands
 These variables store the results from the LTC6810
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable

bool ISOSPI_DIR = BMS_IC[0].isospi_reverse;

/*************************************************************************
 Set configuration register. Refer to the data sheet
**************************************************************************/
bool ADCOPT = false;                                          //!< ADC Mode option bit
bool REFON = true;                                            //!< Reference Powered Up Bit
bool GPIOBITS[4] = {false, true, true, true};                 //!< GPIO Pin Control // Gpio 1,2,3,4
uint16_t UV = UV_THRESHOLD;                                   //!< Under voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD;                                   //!< Over voltage Comparison Voltage
bool DCCBITS[6] = {false, false, false, false, false, false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6
bool DCCBIT_0 = false;                                        //!< Discharge cell switch //Dcc 0 // For discharging optional 7th cell
bool MCAL = false;                                            //!< Enable Multi-Calibration
bool EN_DTMEN = true;                                         //!< Enable Discharge timer monitor
bool DIS_RED = false;                                         //!< Disable Digital Redundancy Check
bool FDRF = false;                                            //!< Force digital Redundancy Failure
bool SCONV = true;                                            //!< Enable Cell Measurement Redundancy using S Pin
bool DCTOBITS[4] = {true, false, true, false};                //!<  Discharge Time Out Value //Dcto 0,1,2,3 // Programed for 4 min
                                                              /*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */

int8_t error = 0;
uint32_t conv_time = 0;

// Constructor
LTC6810_Interface::LTC6810_Interface(str_SlaveBMUData *str_SlaveBMUData)
{
    //Set-Up SPI Connection to ISO-SPI_Interface
    pinMode(ISOSPI_CS1, OUTPUT);
    digitalWriteFast(ISOSPI_CS1, LOW);

    SPI.begin();
    SPI.setClockDivider(SPI_CLK_DEVIDER);

    wakeup_sleep(TOTAL_IC);
    LTC6810_init_cfg(TOTAL_IC, BMS_IC);

    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        LTC6810_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS, DCCBITS, DCCBIT_0, MCAL, EN_DTMEN, DIS_RED, FDRF, SCONV, DCTOBITS, UV, OV);
        str_SlaveBMUData[current_ic].config.referenceOn = REFON;
        str_SlaveBMUData[current_ic].config.adcOPTMode = ADCOPT;
        for (int gpio = 0; gpio < 4; gpio++)
        {
            str_SlaveBMUData[current_ic].config.gpio[gpio] = GPIOBITS[gpio];
        }
        str_SlaveBMUData[current_ic].config.dischargeCellBits[0] = DCCBIT_0;
        for (int dccPIN = 1; dccPIN < 7; dccPIN++)
        {
            str_SlaveBMUData[current_ic].config.dischargeCellBits[dccPIN] = DCCBITS[dccPIN - 1];
        }
        str_SlaveBMUData[current_ic].config.multiCalibration = MCAL;
        str_SlaveBMUData[current_ic].config.enableDischargeTimerMonitor = EN_DTMEN;
        str_SlaveBMUData[current_ic].config.disableDigitalRedundancyCheck = DIS_RED;
        str_SlaveBMUData[current_ic].config.forceDigitalRedundancyFailure = FDRF;
        str_SlaveBMUData[current_ic].config.enableCellMeasurementRedundancySPIn = SCONV;
        for (int dctoBits = 0; dctoBits < 4; dctoBits++)
        {
            str_SlaveBMUData[current_ic].config.dischargeTimeOutValueBits[dctoBits] = DCTOBITS[dctoBits];
        }
        str_SlaveBMUData[current_ic].config.underVoltage10000 = UV;
        str_SlaveBMUData[current_ic].config.overVoltage10000 = OV;
    }
    LTC6810_reset_crc_count(TOTAL_IC, BMS_IC);
    LTC6810_init_reg_limits(TOTAL_IC, BMS_IC);
}

// Remover
LTC6810_Interface::~LTC6810_Interface()
{
}

void LTC6810_Interface::ltc6810_pullConfig(str_SlaveBMUData *str_SlaveBMUData, bool debug = false)
{
    ic_spi_speed();
    ic_IC_read_ConfigRegister(true);

    str_SlaveBMUData[0].config.gpio[3] = (bool)(BMS_IC[0].config.rx_data[0] & 0x40);
    str_SlaveBMUData[0].config.gpio[2] = (bool)(BMS_IC[0].config.rx_data[0] & 0x20);
    str_SlaveBMUData[0].config.gpio[1] = (bool)(BMS_IC[0].config.rx_data[0] & 0x10);
    str_SlaveBMUData[0].config.gpio[0] = (bool)(BMS_IC[0].config.rx_data[0] & 0x08);
    str_SlaveBMUData[0].config.referenceOn = (bool)(BMS_IC[0].config.rx_data[0] & 0x04);
    str_SlaveBMUData[0].config.dischargeTimerEnable = (bool)(BMS_IC[0].config.rx_data[0] & 0x02);
    str_SlaveBMUData[0].config.adcOPTMode = (bool)(BMS_IC[0].config.rx_data[0] & 0x01);

    str_SlaveBMUData[0].config.underVoltage10000 = 0;

    str_SlaveBMUData[0].config.underVoltage10000 = BMS_IC[0].config.rx_data[1] & 0x00FF;
    str_SlaveBMUData[0].config.underVoltage10000 = str_SlaveBMUData[0].config.underVoltage10000 | ((BMS_IC[0].config.rx_data[2] & 0x0F) << 8);
    str_SlaveBMUData[0].config.underVoltage10000 *= 16;

    str_SlaveBMUData[0].config.overVoltage10000 = 0;
    str_SlaveBMUData[0].config.overVoltage10000 = (BMS_IC[0].config.rx_data[2] & 0xF0) >> 4;
    str_SlaveBMUData[0].config.overVoltage10000 = str_SlaveBMUData[0].config.overVoltage10000 | ((BMS_IC[0].config.rx_data[3] & 0xFF) << 4);
    str_SlaveBMUData[0].config.overVoltage10000 *= 16;

    str_SlaveBMUData[0].config.dischargeCellBits[0] = (bool)(BMS_IC[0].config.rx_data[4] & 0x80);
    str_SlaveBMUData[0].config.multiCalibration = (bool)(BMS_IC[0].config.rx_data[4] & 0x40);
    str_SlaveBMUData[0].config.dischargeCellBits[6] = (bool)(BMS_IC[0].config.rx_data[4] & 0x20);
    str_SlaveBMUData[0].config.dischargeCellBits[5] = (bool)(BMS_IC[0].config.rx_data[4] & 0x10);
    str_SlaveBMUData[0].config.dischargeCellBits[4] = (bool)(BMS_IC[0].config.rx_data[4] & 0x08);
    str_SlaveBMUData[0].config.dischargeCellBits[3] = (bool)(BMS_IC[0].config.rx_data[4] & 0x04);
    str_SlaveBMUData[0].config.dischargeCellBits[2] = (bool)(BMS_IC[0].config.rx_data[4] & 0x02);
    str_SlaveBMUData[0].config.dischargeCellBits[1] = (bool)(BMS_IC[0].config.rx_data[4] & 0x01);

    str_SlaveBMUData[0].config.dischargeTimeOutValue = (uint8_t)BMS_IC[0].config.rx_data[5] & 0xF0;
    str_SlaveBMUData[0].config.enableCellMeasurementRedundancySPIn = (bool)(BMS_IC[0].config.rx_data[5] & 0x08);
    str_SlaveBMUData[0].config.forceDigitalRedundancyFailure = (bool)(BMS_IC[0].config.rx_data[5] & 0x04);
    str_SlaveBMUData[0].config.disableDigitalRedundancyCheck = (bool)(BMS_IC[0].config.rx_data[5] & 0x02);
    str_SlaveBMUData[0].config.enableDischargeTimerMonitor = (bool)(BMS_IC[0].config.rx_data[5] & 0x01);
}

void LTC6810_Interface::ltc6810_pushConfig(str_SlaveBMUData *str_SlaveBMUData, bool reset, bool debug)
{
    ic_spi_speed();
    if (reset)
    {
        for (int ic = 0; ic < TOTAL_IC; ic++)
        {
            LTC6810_set_cfgr(ic, BMS_IC, str_SlaveBMUData[ic].config.referenceOn, ADCOPT, GPIOBITS, DCCBITS, DCCBIT_0, MCAL, EN_DTMEN, DIS_RED, FDRF, SCONV, DCTOBITS, UV, OV);
        }
    }
    else
    {
        for (int ic = 0; ic < TOTAL_IC; ic++)
        {
            Serial.print("DCCBITS: ");
            for (size_t ic = 0; ic < TOTAL_IC; ic++)
            {
                for (size_t cell = 0; cell < 8; cell++)
                {
                    Serial.printf("%i, ", str_SlaveBMUData[ic].config.dischargeCellBits[cell]);
                }
            }

            bool dccBits[6];
            for (int dccBIT = 0; dccBIT < 6; dccBIT++)
            {
                dccBits[dccBIT] = str_SlaveBMUData[ic].config.dischargeCellBits[dccBIT + 1];
            }
            bool dccBit0 = str_SlaveBMUData[ic].config.dischargeCellBits[0];

            LTC6810_set_cfgr(ic, BMS_IC, str_SlaveBMUData[ic].config.referenceOn, str_SlaveBMUData[ic].config.adcOPTMode, str_SlaveBMUData[ic].config.gpio, dccBits, dccBit0, str_SlaveBMUData[ic].config.multiCalibration, str_SlaveBMUData[ic].config.enableDischargeTimerMonitor, str_SlaveBMUData[ic].config.disableDigitalRedundancyCheck, str_SlaveBMUData[ic].config.forceDigitalRedundancyFailure, str_SlaveBMUData[ic].config.enableCellMeasurementRedundancySPIn, str_SlaveBMUData[ic].config.dischargeTimeOutValueBits, str_SlaveBMUData[ic].config.underVoltage10000, str_SlaveBMUData[ic].config.overVoltage10000);
            //Serial.printf("RefON = %d \n", str_SlaveBMUData[ic].config.referenceOn);

            //LTC6810_set_cfgr(ic, BMS_IC, str_SlaveBMUData[ic].config.referenceOn, ADCOPT, GPIOBITS, DCCBITS, DCCBIT_0, MCAL, EN_DTMEN, DIS_RED, FDRF, SCONV, DCTOBITS, UV, OV);
        }
    }

    ic_writeConfig(debug);
}

void LTC6810_Interface::ltc6810_pullSerialID(str_SlaveBMUData *slaveData, bool debug)
{
    //Comm to IC
    ic_read_SerialID(debug);
    //Data feedback
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        slaveData[ic].serialID = BMS_IC[ic].sid;
    }
}

void LTC6810_Interface::ltc6810_pullStatusData(str_SlaveBMUData *slaveData, bool debug)
{
    ic_spi_speed();
    //Comm to IC
    ic_IC_start_Status(false);
    ic_IC_read_Status(false);

    //Data feedback
    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        //slaveData[ic].totalVolatge
        slaveData[ic].totalVolatge10000 = (BMS_IC[ic].stat.stat_codes[0]) * 10;

        double temp = (double)((BMS_IC[ic].stat.stat_codes[1] * (0.0001 / 0.0075)) - 273); //Internal Die Temperature(°C) = ITMP • (100 µV / 7.5mV)°C - 273°C
        slaveData[ic].temperatureIC.temperature10 = (int32_t)(temp * 10);

        slaveData[ic].VregA10000 = BMS_IC[ic].stat.stat_codes[2];
        slaveData[ic].VregB10000 = BMS_IC[ic].stat.stat_codes[3];

        float part = ((float)slaveData[ic].totalVolatge10000 / 10000 - ((float)slaveData[ic].config.underVoltage10000 / 10000 * 7));

        float whole = ((float)slaveData[ic].config.overVoltage10000 / 10000 * 7 - ((float)slaveData[ic].config.underVoltage10000 / 10000 * 7));

        slaveData[ic].totalStateOfCharge = part / whole * 100;
    }
}

void LTC6810_Interface::ltc6810_pullVoltage(str_SlaveBMUData *slaveData, bool debug)
{
    ic_spi_speed();
    ic_CELL_read_Combined(debug);

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        for (int cell = -1; cell < 6; cell++)
        {
            slaveData[ic].cells[cell + 1].voltage10000 = getCELLVoltage10000(ic, cell);
        }
    }

    //Caluclate total Voltage and estimate SOC

    // for (int ic = 0; ic < TOTAL_IC; ic++)
    // {
    //     int totalVoltage = 0;
    //     for (int cell = -1; cell < 6; cell++)
    //     {
    //         totalVoltage += slaveData[ic].cells[cell + 1].voltage10000;
    //     }
    //     slaveData[ic].totalVolatge10000 = totalVoltage;
    //     Serial.printf("TOTALVOLATGE=%i \n", totalVoltage);
    //     /// slaveData[ic].totalStateOfCharge = TODO
    // }
}

void LTC6810_Interface::ltc6810_pullSPinVoltageRegister(str_SlaveBMUData *slaveData, bool debug)
{
    ic_spi_speed();
    ic_SPIN_read_SVoltageRegister(debug);

    for (int ic = 0; ic < TOTAL_IC; ic++)
    {
        for (int sPIN = 6; sPIN < 12; sPIN++)
        {
            slaveData[ic].sPins[sPIN - 5].voltage10000 = BMS_IC[ic].cells.c_codes[sPIN];
        }
    }
}

void LTC6810_Interface::ltc6810_pullTemperatureData(str_SlaveBMUData *slaveData, bool debug)
{
    ic_spi_speed();
    //ic_resetGPIO(debug);
    //delay(10);
    for (int i = 1; i < 9; i++)
    {

        if (i2c_mux_write(debug, i) == 0)
        {
            delay(10);
            if (i2c_mux_read(debug))
                delay(10);
            delay(10);
            ic_GPIO_start_ADCMeasurement(debug); //Conversion time of ADC should be done in this command so it will take some time to execute?
            ic_GPIO_read_AuxVoltageRegister(true);

            for (int ic = 0; ic < TOTAL_IC; ic++)
            {
                slaveData[ic].temperatureExternal[i - 1].temperature10 = convertVoltageToTemperature10(getGPIOVoltage10000(ic, 2));
            }
        }
        //ic_resetGPIO(debug);
        delay(10);
    }
}

void LTC6810_Interface::ic_spi_speed()
{
    SPI.setClockDivider(SPI_CLK_DEVIDER);
}

void LTC6810_Interface::ic_wakeup()
{
    wakeup_sleep(TOTAL_IC);
}

//Commands
void LTC6810_Interface::ic_writeConfig(bool debug)
{
    //SPI.setClockDivider(SPI_CLK_DEVIDER);
    wakeup_idle(TOTAL_IC);
    LTC6810_wrcfg(TOTAL_IC, BMS_IC); // Write into Configuration Register
    if (debug)
        print_wrconfig();
    wakeup_idle(TOTAL_IC);
    error = LTC6810_rdcfg(TOTAL_IC, BMS_IC); // Read Configuration Register
    check_error(error);
    if (debug)
        print_rxconfig();
}

void LTC6810_Interface::ic_resetGPIO(bool debug)
{
    wakeup_sleep(TOTAL_IC);
    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        LTC6810_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS, DCCBITS, DCCBIT_0, MCAL, EN_DTMEN, DIS_RED, FDRF, SCONV, DCTOBITS, UV, OV);
    }
    wakeup_idle(TOTAL_IC);
    LTC6810_wrcfg(TOTAL_IC, BMS_IC);
    if (debug)
        print_wrconfig();

    delay(10);
}

void LTC6810_Interface::ic_read_SerialID(bool debug)
{
    ic_spi_speed();
    wakeup_sleep(TOTAL_IC);
    error = LTC6810_rdsid(TOTAL_IC, BMS_IC);
    check_error(error);
    if (debug)
        print_sid();
}

void LTC6810_Interface::ic_IC_read_ConfigRegister(bool debug)
{
    ic_spi_speed();
    wakeup_sleep(TOTAL_IC);
    error = LTC6810_rdcfg(TOTAL_IC, BMS_IC);
    check_error(error);
    if (debug)
        print_rxconfig();
}

void LTC6810_Interface::ic_IC_start_Status(bool debug)
{
    wakeup_sleep(TOTAL_IC);
    LTC6810_adstat(ADC_CONVERSION_MODE, STAT_CH_TO_CONVERT);
    conv_time = LTC6810_pollAdc();
    if (debug)
        print_conv_time(conv_time);
}

void LTC6810_Interface::ic_IC_read_Status(bool debug)
{
    wakeup_sleep(TOTAL_IC);
    error = LTC6810_rdstat(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all Status registers

    check_error(error);
    if (debug)
        print_stat();
}

void LTC6810_Interface::ic_CELL_start_ADCMeasurement(bool debug)
{
    wakeup_sleep(TOTAL_IC);
    LTC6810_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    conv_time = LTC6810_pollAdc();
    if (debug)
        print_conv_time(conv_time);
}

void LTC6810_Interface::ic_CELL_read_VoltageRegister(bool debug)
{
    wakeup_sleep(TOTAL_IC);
    error = LTC6810_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    if (debug)
        print_cells(DATALOG_DISABLED);
}

void LTC6810_Interface::ic_CELL_read_Combined(bool debug)
{
    wakeup_sleep(TOTAL_IC);
    LTC6810_adcvax(ADC_CONVERSION_MODE, ADC_DCP);
    conv_time = LTC6810_pollAdc();
    if (debug)
        print_conv_time(conv_time);
    wakeup_idle(TOTAL_IC);
    error = LTC6810_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    if (debug)
        print_cells(DATALOG_DISABLED);
    wakeup_idle(TOTAL_IC);
    error = LTC6810_rdaux(SEL_REG_A, TOTAL_IC, BMS_IC); // Set to read back aux register A
    check_error(error);
    if (debug)
        print_aux(DATALOG_DISABLED);
}

void LTC6810_Interface::ic_CELL_read_CombinedSum(bool debug)
{
    wakeup_sleep(TOTAL_IC);
    LTC6810_adcvsc(ADC_CONVERSION_MODE, ADC_DCP);

    conv_time = LTC6810_pollAdc();
    if (debug)
        print_conv_time(conv_time);
    wakeup_idle(TOTAL_IC);
    error = LTC6810_rdcv(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all cell voltage registers
    check_error(error);
    if (debug)
        print_cells(DATALOG_DISABLED);
    wakeup_idle(TOTAL_IC);
    error = LTC6810_rdstat(SEL_REG_A, TOTAL_IC, BMS_IC); // Set to read back stat register A
    check_error(error);
    if (debug)
        print_sumofcells();
}

void LTC6810_Interface::ic_SPIN_read_SVoltageRegister(bool debug)
{
    /****************************************************************
     Ensure that the SCONV bit in the configuration register is set. 
     ****************************************************************/
    wakeup_sleep(TOTAL_IC);
    LTC6810_wrcfg(TOTAL_IC, BMS_IC); // Write into Configuration Register
    LTC6810_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
    conv_time = LTC6810_pollAdc();
    if (debug)
        print_conv_time(conv_time);
    wakeup_idle(TOTAL_IC);
    error = LTC6810_rds(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all S voltage registers
    check_error(error);
    if (debug)
        print_svolt(DATALOG_DISABLED);
}

void LTC6810_Interface::ic_GPIO_start_ADCMeasurement(bool debug)
{
    wakeup_sleep(TOTAL_IC);
    LTC6810_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
    conv_time = LTC6810_pollAdc();
    if (debug)
        print_conv_time(conv_time);
}

void LTC6810_Interface::ic_GPIO_read_AuxVoltageRegister(bool debug)
{
    wakeup_sleep(TOTAL_IC);
    error = LTC6810_rdaux(SEL_ALL_REG, TOTAL_IC, BMS_IC); // Set to read back all aux registers
    check_error(error);
    if (debug)
        print_aux(DATALOG_DISABLED);
}

uint8_t LTC6810_Interface::i2c_mux_write(bool debug, uint16_t muxPin)
{
    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        //Communication control bits and communication data bytes. Refer to the data sheet.
        BMS_IC[current_ic].com.tx_data[0] = 0x69;
        BMS_IC[current_ic].com.tx_data[1] = 0xE0;
        if (muxPin > 4 && muxPin < 9)
            BMS_IC[current_ic].com.tx_data[2] = (1 << (muxPin - 1 - 4));
        else
            BMS_IC[current_ic].com.tx_data[2] = 0x00;
        if (muxPin > 0 && muxPin < 5)
            BMS_IC[current_ic].com.tx_data[3] = (1 << (muxPin - 1 + 4)) | 0x01;
        else
            BMS_IC[current_ic].com.tx_data[3] = 0x01;
        BMS_IC[current_ic].com.tx_data[4] = 0x7F;
        BMS_IC[current_ic].com.tx_data[5] = 0xF9;
    }
    wakeup_sleep(TOTAL_IC);
    LTC6810_wrcomm(TOTAL_IC, BMS_IC); // write to comm register
    if (debug)
    {
        print_wrcomm(); // print data from the comm register
    }
    wakeup_idle(TOTAL_IC);
    LTC6810_stcomm(2);

    wakeup_idle(TOTAL_IC);
    error = LTC6810_rdcomm(TOTAL_IC, BMS_IC); // read from comm register
    check_error(error);
    if (debug)
    {
        print_rxcomm(); // print received data into the comm register
    }
    uint8_t errorCount = 0;
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {

        for (int i = 0; i < 6; i++)
        {
            if (BMS_IC[current_ic].com.tx_data[i] != BMS_IC[current_ic].com.rx_data[i])

            {
                errorCount++;
            }
        }
    }
    return errorCount;
}

uint16_t LTC6810_Interface::i2c_mux_read(bool debug)
{
    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        //Communication control bits and communication data bytes. Refer to the data sheet.
        BMS_IC[current_ic].com.tx_data[0] = 0x69; // Icom Start(6)
        BMS_IC[current_ic].com.tx_data[1] = 0xF7;
        BMS_IC[current_ic].com.tx_data[2] = 0x0F;
        BMS_IC[current_ic].com.tx_data[3] = 0xF9;
        BMS_IC[current_ic].com.tx_data[4] = 0x7F;
        BMS_IC[current_ic].com.tx_data[5] = 0xFF;
    }
    wakeup_sleep(TOTAL_IC);
    LTC6810_wrcomm(TOTAL_IC, BMS_IC); // write to comm register
    if (debug)
    {
        print_wrcomm(); // print data from the comm register
    }

    wakeup_idle(TOTAL_IC);
    LTC6810_stcomm(2); // data length=3 // initiates communication between master and the I2C slave

    wakeup_idle(TOTAL_IC);
    error = LTC6810_rdcomm(TOTAL_IC, BMS_IC); // read from comm register
    check_error(error);
    if (debug)
    {
        print_rxcomm(); // print received data from the comm register
    }

    uint8_t errorCount = 0;
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {

        for (int i = 0; i < 6; i++)
        {
            if (BMS_IC[current_ic].com.tx_data[i] != BMS_IC[current_ic].com.rx_data[i])

            {
                errorCount++;
            }
        }
    }
    return errorCount;
}

uint16_t LTC6810_Interface::getGPIOVoltage10000(uint8_t ic, uint8_t gpio)
{
    return BMS_IC[ic].aux.a_codes[gpio];
}

// Private Functions
void LTC6810_Interface::check_error(int8_t error)
{
    if (error == -1)
    {
        Log.error("A PEC error was detected in the received data" CR);
        Log.error("Check communction between isoSPI-Board and BMU-Board" CR);
    }
}

void LTC6810_Interface::print_sid(void)
{
    int sid_pec;

    //Log.notice(F("Serial ID: " CR));
    for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        char buf[128] = "\0"; // IMPORTANT
        sprintf(buf, "%s", "Serial ID:");
        sprintf(buf, "%s IC %d", buf, current_ic + 1);

        for (int i = 0; i < 6; i++)
        {
            sprintf(buf, "%s 0x%02X", buf, BMS_IC[current_ic].sid[i]);
        }

        sprintf(buf, "%s %s", buf, ", Calculated PEC:");
        sid_pec = pec15_calc(6, &BMS_IC[current_ic].sid[0]);
        sprintf(buf, "%s 0x%02X", buf, (uint8_t)(sid_pec >> 8));
        sprintf(buf, "%s 0x%02X", buf, (uint8_t)(sid_pec));

        Log.notice("%s" CR, buf);
    }
}

uint16_t LTC6810_Interface::getCELLVoltage10000(uint8_t ic, int cell)
{
    if (cell == -1)
    {
        return BMS_IC[ic].aux.a_codes[1];
    }
    else
    {
        return BMS_IC[ic].cells.c_codes[cell];
    }
}

// void LTC6810_Interface::serial_print_hex(uint8_t data)
// {
//     if (data < 16)
//     {
//         Serial.print("0");
//         Serial.print((byte)data, HEX);
//     }
//     else
//         Serial.print((byte)data, HEX);
// }

void LTC6810_Interface::print_conv_time(uint32_t conv_time)
{
    uint16_t m_factor = 1000; // to print in ms

    char buf[128] = "\0"; // IMPORTANT
    sprintf(buf, "%s", "Conversion completed in:");
    sprintf(buf, "%s %d", buf, (int)(conv_time / m_factor));
    sprintf(buf, "%s %s", buf, "ms");
    Log.notice("%s" CR, buf);
}

void LTC6810_Interface::print_cells(uint8_t datalog_en)
{
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        char buf[128] = "\0"; // IMPORTANT

        if (datalog_en == 0)
        {
            sprintf(buf, "IC %d:", current_ic + 1);

            for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
            {
                sprintf(buf, "%s C%d:%.4f", buf, i + 1, (BMS_IC[current_ic].cells.c_codes[i] * 0.0001));
            }
            Log.notice("%s" CR, buf);
        }
        else
        {
            sprintf(buf, "Cells: ");
            //Serial.print(" Cells :");
            for (int i = 0; i < BMS_IC[0].ic_reg.cell_channels; i++)
            {
                sprintf(buf, "%s %.4f, ", buf, BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
                //Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
                //Serial.print(",");
            }
        }
    }
}

void LTC6810_Interface::print_aux(uint8_t datalog_en)
{
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        char buf[128] = "\0"; // IMPORTANT
        if (datalog_en == 0)
        {
            for (int i = 1; i < 5; i++)
            {
                sprintf(buf, "%s GPIO-%i:%.4f,", buf, i, BMS_IC[current_ic].aux.a_codes[i] * 0.0001);
            }
            sprintf(buf, "%s Vref2:%.4f", buf, BMS_IC[current_ic].aux.a_codes[5] * 0.0001);
        }
        else
        {
            sprintf(buf, "%s AUX:", buf);

            for (int i = 0; i < 6; i++)
            {
                sprintf(buf, "%s%.4f,", buf, BMS_IC[current_ic].aux.a_codes[i] * 0.0001);
            }
        }
        Log.notice("%s" CR, buf);
    }
}

void LTC6810_Interface::print_wrcomm(void)
{
    int comm_pec;

    Serial.println(F("Written Data in COMM Register: "));
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        Serial.print(F(" IC- "));
        Serial.print(current_ic + 1, DEC);

        for (int i = 0; i < 6; i++)
        {
            Serial.print(F(", 0x"));
            serial_print_hex(BMS_IC[current_ic].com.tx_data[i]);
        }
        Serial.print(F(", Calculated PEC: 0x"));
        comm_pec = pec15_calc(6, &BMS_IC[current_ic].com.tx_data[0]);
        serial_print_hex((uint8_t)(comm_pec >> 8));
        Serial.print(F(", 0x"));
        serial_print_hex((uint8_t)(comm_pec));
        Serial.println("\n");
    }
}

void LTC6810_Interface::serial_print_hex(uint8_t data)
{
    if (data < 16)
    {
        Serial.print("0");
        Serial.print((byte)data, HEX);
    }
    else
        Serial.print((byte)data, HEX);
}

void LTC6810_Interface::print_rxcomm(void)
{
    Serial.println(F("Received Data in COMM register:"));
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        Serial.print(F(" IC- "));
        Serial.print(current_ic + 1, DEC);

        for (int i = 0; i < 6; i++)
        {
            Serial.print(F(", 0x"));
            serial_print_hex(BMS_IC[current_ic].com.rx_data[i]);
        }
        Serial.print(F(", Received PEC: 0x"));
        serial_print_hex(BMS_IC[current_ic].com.rx_data[6]);
        Serial.print(F(", 0x"));
        serial_print_hex(BMS_IC[current_ic].com.rx_data[7]);
        Serial.println("\n");
    }
}

void LTC6810_Interface::print_rxconfig(void)
{
    Log.notice(F("Received Configuration "));
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        Serial.print(F("CFGA IC "));
        Serial.print(current_ic + 1, DEC);
        for (int i = 0; i < 6; i++)
        {
            Serial.print(F(", 0x"));
            serial_print_hex(BMS_IC[current_ic].config.rx_data[i]);
        }
        Serial.print(F(", Received PEC: 0x"));
        serial_print_hex(BMS_IC[current_ic].config.rx_data[6]);
        Serial.print(F(", 0x"));
        serial_print_hex(BMS_IC[current_ic].config.rx_data[7]);
        Serial.println("\n");
    }
}

void LTC6810_Interface::print_wrconfig(void)
{
    int cfg_pec;

    Serial.println(F("Written Configuration: "));
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        Serial.print(F("CFGA IC "));
        Serial.print(current_ic + 1, DEC);
        for (int i = 0; i < 6; i++)
        {
            Serial.print(F(", 0x"));
            serial_print_hex(BMS_IC[current_ic].config.tx_data[i]);
        }
        Serial.print(F(", Calculated PEC: 0x"));
        cfg_pec = pec15_calc(6, &BMS_IC[current_ic].config.tx_data[0]);
        serial_print_hex((uint8_t)(cfg_pec >> 8));
        Serial.print(F(", 0x"));
        serial_print_hex((uint8_t)(cfg_pec));
        Serial.println("\n");
    }
}

void LTC6810_Interface::print_sumofcells(void)
{
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        Serial.print(F(" IC "));
        Serial.print(current_ic + 1, DEC);
        Serial.print(F(": SOC:"));
        Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 10, 4);
        Serial.print(F(","));
    }
    Serial.println("\n");
}

void LTC6810_Interface::print_svolt(uint8_t datalog_en)
{
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        if (datalog_en == 0)
        {
            Serial.print(" IC ");
            Serial.print(current_ic + 1, DEC);
            Serial.print(": ");
            int j = 1;
            for (int i = 6; i < 12; i++)
            {
                Serial.print(" S");
                Serial.print(j, DEC);
                Serial.print(":");
                Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
                Serial.print(",");
                j++;
            }
            Serial.println();
        }
        else
        {
            Serial.print("S pin:, ");
            for (int i = 6; i < 12; i++)
            {
                Serial.print(BMS_IC[current_ic].cells.c_codes[i] * 0.0001, 4);
                Serial.print(",");
            }
        }
    }
    Serial.println("\n");
}

void LTC6810_Interface::print_stat(void)
{
    double itmp;
    for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++)
    {
        Serial.print(F(" IC "));
        Serial.print(current_ic + 1, DEC);
        Serial.print(F(" SOC:"));
        Serial.print(BMS_IC[current_ic].stat.stat_codes[0] * 0.0001 * 10, 4);
        Serial.print(F(", Itemp:"));
        itmp = (double)((BMS_IC[current_ic].stat.stat_codes[1] * (0.0001 / 0.0075)) - 273); //Internal Die Temperature(°C) = ITMP • (100 µV / 7.5mV)°C - 273°C
        Serial.print(itmp, 4);
        Serial.print(F(", VregA:"));
        Serial.print(BMS_IC[current_ic].stat.stat_codes[2] * 0.0001, 4);
        Serial.print(F(", VregD:"));
        Serial.println(BMS_IC[current_ic].stat.stat_codes[3] * 0.0001, 4);
        Serial.print(F(" Flag bits: 0x"));
        serial_print_hex(BMS_IC[current_ic].stat.flags[0]);
        Serial.print(F(",\t Flag bits and Mute bit:"));
        Serial.print(F(" 0x"));
        serial_print_hex((BMS_IC[current_ic].stat.flags[1]) & (0x1F));
        Serial.print(F(",\t Mux fail flag:"));
        Serial.print(F(" 0x"));
        serial_print_hex(BMS_IC[current_ic].stat.mux_fail[0]);
        Serial.print(F(",\t THSD:"));
        Serial.print(F(" 0x"));
        serial_print_hex(BMS_IC[current_ic].stat.thsd[0]);
        Serial.println("\n");
    }
}