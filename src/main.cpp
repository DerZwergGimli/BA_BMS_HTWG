#include <Arduino.h>
#include "LTC6810_Interface.h"
#include "ArduinoLog.h"
#include <cstdint>
#include "DataInterface.h"
//#include "Thermistor.h"
//#include "ReportRam.h"
//#include "Metro.h"
//#include "../lib/UI/projects_GSLC.h"
//#include "projects_GSLC.h"
//#include "projects.h"

//#include "uiDraw.h"
//#include "ui.h"
//#include "UIhelper.h"
#include "Chrono.h"
//#include "BalancerFSM.h"
//#include "BMS_StateMachine.h"
//#include "Atm_bms.h"
//#include "Atm_BMS.h"

#include "GlobalDefines.h"
#include "EEPROMAnything.h"

#include "SystemDataInterface.h"
#include "FSM_SYSTEM.h"
#include "FSM_BatteryMonitor.h"
#include "FSM_UserInterface.h"

#include "HeartbeatLed.h"
//#include "TeensyWDT.h"

//#define RAMREPORT_TIMER 10000
//#define LTCPOLLING_TIMER 5000
//#define DISPLAYUPDATE_TIMER 500

//#define TOTAL_IC 1
//#define TOTAL_CELLS 7

void printTimestamp(Print *_logOutput);
void toStringCellVoltages();
void toStringTemperatureValues();
void toStringStatus();
void toDisplayOnce();
void toDisplay();

LTC6810_Interface *ltc6810;
str_SlaveBMUData bmuData[TOTAL_IC];
str_SystemData systemData;
//BalancerFSM *balancerFSM;
//e_BalncerFSM eBalancerFSM;

//extern EN_DisplayConextUpdate enDisplayConextUpdate;

//Metro ramReportTIMER = Metro(RAMREPORT_TIMER);
//Metro ltcPollingTIMER = Metro(LTCPOLLING_TIMER);
//Metro displayUpdateTIMER = Metro(DISPLAYUPDATE_TIMER);

//Chrono ramReportTIMER;
//Chrono ltcPollingTIMER;
//Chrono displayUpdateTIMER;

//Atm_BMS atmBms;

FSM_SYSTEM fsmSystem;
FSM_BatteryMonitor fsmBatteryMonitor;
FSM_UserInterface fsmUserInterface;

unsigned long heartbeatSequence[] = {1000, 1};
HeartbeatLed heartbeat(6, heartbeatSequence, 2);

void setup()
{
  //ram.initialize();
  Serial.begin(115200);
  if (SERIAL_WAIT)
  {
    while (!Serial)
    {
      ; // wait for serial port to connect. Needed for native USB
    }
  }
  else
  {
    delay(2000); // Do a simple delay to wait for user to connect serial - for proper data transmit this is needed.
  }
  Log.begin(LOG_LEVEL_VERBOSE, &Serial, true);
  Log.setPrefix(printTimestamp); // Uncomment to get timestamps as prefix

  //gslc_InitDebug(&DebugOut);

  // ------------------------------------------------
  // Create graphic elements
  // ------------------------------------------------

  ltc6810 = new LTC6810_Interface(bmuData);
  //balancerFSM = new BalancerFSM(TOTAL_IC);

  Log.notice("Booted..." CR);
  // pinMode(DISPLAY_POWER_PIN, OUTPUT);
  // setupCellBalancing(ltc6810, bmuData);

  // pinMode(DISPLAY_POWER_PIN, OUTPUT);
  // digitalWriteFast(DISPLAY_POWER_PIN, HIGH);
  // delay(10);
  // InitGUIslice_gen(ltc6810, bmuData);
  // gslc_SetPageCur(&m_gui, E_PG_BOOT);
  // gslc_Update(&m_gui);
  // delay(1000);
  // gslc_SetPageCur(&m_gui, E_PG_HOME);
  // gslc_Update(&m_gui);

  // //gslc_setContextPointer(&enDisplayConextUpdate);

  // // toDisplayOnce();
  // drawSettingsBMS_PAGE(bmuData, TOTAL_IC);
  // drawBalancing_PAGE(bmuData);

  //atmBms.begin();
  //atmBms.trace(Serial);

  //-----------------------------------------------------------------------------------
  //systemData.timerSleep = -1;
  //systemData.timerMemory = 2000;
  //systemData.timerDisplayUpdate = 10;
  //systemData.timerLTC68 = 1000;

  //systemData.maxAllowedVoltage10000 = 40000;
  //systemData.minAllowedVoltage10000 = 30000;
  //systemData.balancingVoltageThreshold10000 = 10000;
  //systemData.maxAlowedTemperature10 = 300;
  //systemData.minAllowedTemperature10 = 0;

  //EEPROM_writeAnything(0, systemData);
  Serial.println(EEPROM_readAnything(0, systemData));
  Serial.print(sizeof(systemData));

  fsmSystem.begin(&systemData).onPushreset(fsmUserInterface, fsmUserInterface.EVT_C_RESET);
  //fsmSystem.trace(Serial);

  fsmBatteryMonitor.begin(&systemData, ltc6810, bmuData).automatic(systemData.timerLTC68);
  //fsmBatteryMonitor.trace(Serial);

  fsmUserInterface.begin(&systemData, ltc6810, bmuData, 10);
  //fsmUserInterface.trace(Serial);
}

void loop()
{
  //fsmSystem.cycle();

  //fsmUserInterface.cycle();
  //automaton.run();

  heartbeat.update();

  //runWDT();

  // if (ltcPollingTIMER.hasPassed(LTCPOLLING_TIMER))
  // {
  //   ltcPollingTIMER.restart();
  //   //runStateMachine();

  //   Serial.print(".");

  //   //  ltc6810->ic_spi_speed();
  //   //  ltc6810->ltc6810_pullSerialID(bmuData);
  //   //    delay(10);
  //   //ltc6810->ltc6810_pullVoltage(bmuData);
  //   //toStringCellVoltages();
  //   //    delay(10);
  //   //    ltc6810->ltc6810_pullTemperatureData(bmuData);
  //   //toStringTemperatureValues();
  //   //   ltc6810->ltc6810_pullStatusData(bmuData);
  //   //toStringStatus();
  //   //    ltc6810->ltc6810_pullSPinVoltageRegister(bmuData);

  //   switch (gslc_GetPageCur(&m_gui))
  //   {
  //   case E_PG_HOME:
  //     drawHome_PAGE(bmuData, TOTAL_IC);
  //     break;
  //   case E_PG_MONITOR:
  //     drawCellMonitor_PAGE(bmuData, 0);
  //     break;
  //   case E_PG_SETTINGS_BMS:

  //     break;
  //   default:
  //     break;
  //   }

  //   //   balancerFSM->run(bmuData);

  //   //ltc6810->ltc6810_pushConfig(bmuData, false, true);
  //   //delay(10000);
  // }

  // // if (displayUpdateTIMER.hasPassed(DISPLAYUPDATE_TIMER))
  // // {
  // //   displayUpdateTIMER.restart();
  // //   toDisplay();
  // // }

  // // if (ramReportTIMER.hasPassed(RAMREPORT_TIMER))
  // // {
  // //   ramReportTIMER.restart();
  // //   report_ram();
  // //   gslc_ElemXGraphAdd(&m_gui, m_pElemGraph1, 5);
  // //   gslc_ElemXGraphAdd(&m_gui, m_pElemGraph1, 10);
  // //   gslc_ElemXGraphScrollSet(&m_gui, m_pElemGraph1, 2, 4);
  // //   gslc_ElemXGraphAdd(&m_gui, m_pElemGraph1, 100);
  // // }

  // drawBase_PAGE();
  // gslc_Update(&m_gui);

  // if (ramReportTIMER.hasPassed(RAMREPORT_TIMER))
  // {
  //   ramReportTIMER.restart();
  //   report_ram();
  // }
  // ram.run();
}

void printTimestamp(Print *_logOutput)
{
  char c[12];
  int m = sprintf(c, "%10lu ", millis());
  _logOutput->print(c);
}

void toStringCellVoltages()
{
  for (int i = 0; i < TOTAL_IC; i++)
  {
    for (int cell = 0; cell < 7; cell++)
    {
      Serial.printf("CELL%i=%.3f \t", cell, (float)bmuData[i].cells[cell].voltage10000 / 10000);
    }
    Serial.println();
  }
}

void toStringTemperatureValues()
{
  for (int i = 0; i < TOTAL_IC; i++)
  {
    for (uint8_t temp = 0; temp < 8; temp++)
    {
      Serial.printf("Temp%i=%.1f \t", temp + 1, (float)bmuData[i].temperatureExternal[temp].temperature10 / 10);
    }
    Serial.println();
  }
}

void toStringStatus()
{
  for (int ic = 0; ic < TOTAL_IC; ic++)
  {
    Serial.printf("TotalVoltage=%.3f \t ITemp=%.1f \t VregA=%.2f \t VregB=%.2f", (float)bmuData[ic].totalVolatge10000 / 10000, (float)bmuData[ic].temperatureIC.temperature10 / 10, (float)bmuData[ic].VregA10000 / 10000, (float)bmuData[ic].VregB10000 / 10000);
  }
  Serial.println();
}

void toDisplayOnce()
{
  uint sizeOfChars = 5;
  char *textDisplay;
  textDisplay = new char[sizeOfChars];
  for (int i = 0; i < 2; i++)
  {
    snprintf(textDisplay, sizeOfChars, "IC_%i", i + 1);
    //gslc_ElemXListboxAddItem(&m_gui, m_pElemListboxSelectIC, textDisplay);
  }
  delete[] textDisplay;

  //sizeOfChars = 6;
  //textDisplay = new char[sizeOfChars];
  //snprintf(textDisplay, sizeOfChars, "%.4f", (float)bmuData[0].config.overVoltage10000 / 10000);
  //gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBalancing_overVoltage, textDisplay);
  //delete[] textDisplay;
}

void toDisplay()
{

  // //HomeScreen
  // uint sizeOfChars = 6;
  // char *textDisplay;
  // textDisplay = new char[sizeOfChars];

  // //TotalVoltage
  // uint totalVoltage10000 = 0;
  // for (int ic = 0; ic < TOTAL_IC; ic++)
  // {
  //   totalVoltage10000 += bmuData[ic].totalVolatge10000;
  // }
  // snprintf(textDisplay, sizeOfChars, "%.2f", (float)totalVoltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_home_TotalVoltage, textDisplay);
  // delete[] textDisplay;
  // //TotalCurrent

  // //Total SOC
  // uint totalSOC = 0;
  // sizeOfChars = 6;
  // textDisplay = new char[sizeOfChars];
  // for (int ic = 0; ic < TOTAL_IC; ic++)
  // {
  //   totalSOC += bmuData[ic].totalStateOfCharge;
  // }
  // totalSOC = totalSOC / TOTAL_IC;
  // // Serial.printf("TOTAL SOC=%i", totalSOC);
  // gslc_ElemXProgressSetVal(&m_gui, m_pElemProgress_home_StateOfCharge, totalSOC);
  // snprintf(textDisplay, sizeOfChars, "%03.0f %c", (float)totalSOC, char(37));
  // gslc_ElemSetTxtStr(&m_gui, ptr_home_StateOfChargeText, textDisplay);
  // delete[] textDisplay;

  // //MonitorScreen
  // //--->CellVoltages
  // sizeOfChars = 8;
  // textDisplay = new char[sizeOfChars];
  // snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[0].cells[0].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell1, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[0].cells[1].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell2, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[0].cells[2].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell3, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[0].cells[3].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell4, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[0].cells[4].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell5, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[0].cells[5].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell6, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[0].cells[6].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell7, textDisplay);
  // delete[] textDisplay;

  // //--->CellVoltageBars
  // uint avarageVoltage = (uint)(((float)bmuData[0].totalVolatge10000 / 7));
  // uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL1P, m_pElemProgressCELL1N, avarageVoltage, bmuData[0].cells[0].voltage10000);
  // uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL2P, m_pElemProgressCELL2N, avarageVoltage, bmuData[0].cells[1].voltage10000);
  // uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL3P, m_pElemProgressCELL3N, avarageVoltage, bmuData[0].cells[2].voltage10000);
  // uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL4P, m_pElemProgressCELL4N, avarageVoltage, bmuData[0].cells[3].voltage10000);
  // uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL5P, m_pElemProgressCELL5N, avarageVoltage, bmuData[0].cells[4].voltage10000);
  // uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL6P, m_pElemProgressCELL6N, avarageVoltage, bmuData[0].cells[5].voltage10000);
  // uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL7P, m_pElemProgressCELL7N, avarageVoltage, bmuData[0].cells[6].voltage10000);

  // //-->SPinVoltages
  // sizeOfChars = 6;
  // textDisplay = new char[sizeOfChars];
  // snprintf(textDisplay, sizeOfChars, "%.2fV", (float)bmuData[0].sPins[0].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell1_SVoltage, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.2fV", (float)bmuData[0].sPins[1].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell2_SVoltage, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.2fV", (float)bmuData[0].sPins[2].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell3_SVoltage, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.2fV", (float)bmuData[0].sPins[3].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell4_SVoltage, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.2fV", (float)bmuData[0].sPins[4].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell5_SVoltage, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.2fV", (float)bmuData[0].sPins[5].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell6_SVoltage, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.2fV", (float)bmuData[0].sPins[6].voltage10000 / 10000);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell7_SVoltage, textDisplay);
  // delete[] textDisplay;

  // //--->TemperatureValues
  // sizeOfChars = 6;
  // textDisplay = new char[sizeOfChars];
  // snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[0].temperatureExternal[0].temperature10 / 10, (char)247);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp1, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[0].temperatureExternal[1].temperature10 / 10, (char)247);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp2, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[0].temperatureExternal[2].temperature10 / 10, (char)247);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp3, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[0].temperatureExternal[3].temperature10 / 10, (char)247);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp4, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[0].temperatureExternal[4].temperature10 / 10, (char)247);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp5, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[0].temperatureExternal[5].temperature10 / 10, (char)247);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp6, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[0].temperatureExternal[6].temperature10 / 10, (char)247);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp7, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[0].temperatureExternal[7].temperature10 / 10, (char)247);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp8, textDisplay);
  // snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[0].temperatureIC.temperature10 / 10, (char)247);
  // gslc_ElemSetTxtStr(&m_gui, ptr_monitor_tempIC, textDisplay);
  // delete[] textDisplay;

  // // sizeOfChars = 5;
  // // textDisplay = new char[sizeOfChars];
  // // snprintf(textDisplay, sizeOfChars, "IC_1");
  // // gslc_ElemXListboxAddItem(&m_gui, m_pElemListboxSelectIC, textDisplay);
  // // snprintf(textDisplay, sizeOfChars, "IC_2");
  // // gslc_ElemXListboxAddItem(&m_gui, m_pElemListboxSelectIC, textDisplay);
  // // delete[] textDisplay;

  // //GRAPH

  // // SETTINGS
  // // ---> SETTINGS_BMU LTC68**
  // switch (enDisplayConextUpdate)
  // {
  // case SETTINGS_BTN_LT_READ:
  // {
  //   enDisplayConextUpdate = SETTINGS_NONE;
  //   ltc6810->ltc6810_pullConfig(bmuData, false);

  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_GPIO1, bmuData[0].config.gpio[0]);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_GPIO2, bmuData[0].config.gpio[1]);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_GPIO3, bmuData[0].config.gpio[2]);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_GPIO4, bmuData[0].config.gpio[3]);

  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC0, bmuData[0].config.dischargeCellBits[0]);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC1, bmuData[0].config.dischargeCellBits[1]);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC2, bmuData[0].config.dischargeCellBits[2]);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC3, bmuData[0].config.dischargeCellBits[3]);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC4, bmuData[0].config.dischargeCellBits[4]);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC5, bmuData[0].config.dischargeCellBits[5]);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC6, bmuData[0].config.dischargeCellBits[6]);

  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_REFON, bmuData[0].config.referenceOn);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DTEN, bmuData[0].config.dischargeTimerEnable);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_ADCOPT, bmuData[0].config.adcOPTMode);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_MCAL, bmuData[0].config.multiCalibration);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_SCONV, bmuData[0].config.enableCellMeasurementRedundancySPIn);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_FDRF, bmuData[0].config.forceDigitalRedundancyFailure);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DIS_RED, bmuData[0].config.disableDigitalRedundancyCheck);
  //   gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DTMEN, bmuData[0].config.enableDischargeTimerMonitor);

  //   sizeOfChars = 7;
  //   textDisplay = new char[sizeOfChars];
  //   snprintf(textDisplay, sizeOfChars, "%imin", bmuData[0].config.dischargeTimeOutValueBits);
  //   gslc_ElemSetTxtStr(&m_gui, ptr_Text_dcto, textDisplay);
  //   delete[] textDisplay;

  //   sizeOfChars = 7;
  //   textDisplay = new char[sizeOfChars];
  //   snprintf(textDisplay, sizeOfChars, "%.4f", (float)bmuData[0].config.overVoltage10000 / 10000);
  //   gslc_ElemSetTxtStr(&m_gui, m_pElemVal_overVoltage, textDisplay);
  //   snprintf(textDisplay, sizeOfChars, "%.4f", (float)bmuData[0].config.underVoltage10000 / 10000);
  //   gslc_ElemSetTxtStr(&m_gui, m_pElemVal_underVoltage, textDisplay);
  //   delete[] textDisplay;

  //   gslc_SetPageCur(&m_gui, E_PG_SETTINGS_BMU);
  // }
  // break;
  // case SETTINGS_BTN_LT_WRITE:
  // {
  //   Serial.println("--------WRITE!");
  //   enDisplayConextUpdate = SETTINGS_BTN_LT_READ; // Just make sure to read after a write of config

  //   bmuData[0].config.gpio[0] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_GPIO1);
  //   bmuData[0].config.gpio[1] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_GPIO2);
  //   bmuData[0].config.gpio[2] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_GPIO3);
  //   bmuData[0].config.gpio[3] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_GPIO4);

  //   bmuData[0].config.dischargeCellBits[0] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC0);
  //   bmuData[0].config.dischargeCellBits[1] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC1);
  //   bmuData[0].config.dischargeCellBits[2] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC2);
  //   bmuData[0].config.dischargeCellBits[3] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC3);
  //   bmuData[0].config.dischargeCellBits[4] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC4);
  //   bmuData[0].config.dischargeCellBits[5] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC5);
  //   bmuData[0].config.dischargeCellBits[6] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC6);

  //   bmuData[0].config.referenceOn = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_REFON);
  //   bmuData[0].config.dischargeTimerEnable = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DTEN);
  //   bmuData[0].config.adcOPTMode = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_ADCOPT);
  //   bmuData[0].config.multiCalibration = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_MCAL);
  //   bmuData[0].config.enableCellMeasurementRedundancySPIn = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_SCONV);
  //   bmuData[0].config.forceDigitalRedundancyFailure = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_FDRF);
  //   bmuData[0].config.disableDigitalRedundancyCheck = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DIS_RED);
  //   bmuData[0].config.enableDischargeTimerMonitor = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DTMEN);

  //   String textTemp = gslc_ElemGetTxtStr(&m_gui, ptr_Text_dcto);
  //   bmuData[0].config.dischargeTimeOutValue = textTemp.toInt();

  //   textTemp = gslc_ElemGetTxtStr(&m_gui, m_pElemVal_overVoltage);
  //   bmuData[0].config.overVoltage10000 = (uint16_t)textTemp.toFloat() * 10000;

  //   textTemp = gslc_ElemGetTxtStr(&m_gui, m_pElemVal_underVoltage);
  //   bmuData[0].config.underVoltage10000 = (uint16_t)textTemp.toFloat() * 10000;

  //   ltc6810->ltc6810_pushConfig(bmuData);
  // }
  // break;
  // case SETTINGS_BTN_LT_RESET:
  // {
  //   Serial.println("--------RESET!!");
  //   enDisplayConextUpdate = SETTINGS_BTN_LT_READ;
  //   ltc6810->ltc6810_pushConfig(bmuData, true, true);
  // }
  // break;
  // default:
  //   break;
  // }
}