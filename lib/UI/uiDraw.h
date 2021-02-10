#pragma once
#include <Arduino.h>
#include <projects_GSLC.h>
#include "DataInterface.h"
#include "UIhelper.h"

void drawBase_PAGE()
{
    //---UpTime
    char *textDisplay;
    uint sizeOfChars = 10;
    textDisplay = new char[sizeOfChars];
    snprintf(textDisplay, sizeOfChars, "%is", millis() / 1000);
    gslc_ElemSetTxtStr(&m_gui, ptr_home_uptime, textDisplay);
    delete[] textDisplay;
}

void drawHome_PAGE(str_SlaveBMUData *bmuData, uint16_t total_IC)
{
    //HomeScreen
    uint sizeOfChars = 6;
    char *textDisplay;
    textDisplay = new char[sizeOfChars];

    //TotalVoltage
    uint totalVoltage10000 = 0;
    for (int ic = 0; ic < total_IC; ic++)
    {
        totalVoltage10000 += bmuData[ic].totalVolatge10000;
    }
    snprintf(textDisplay, sizeOfChars, "%.2f", (float)totalVoltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, ptr_home_TotalVoltage, textDisplay);
    delete[] textDisplay;
    //TotalCurrent

    //Total SOC
    uint totalSOC = 0;
    sizeOfChars = 6;
    textDisplay = new char[sizeOfChars];
    for (int ic = 0; ic < total_IC; ic++)
    {
        totalSOC += bmuData[ic].totalStateOfCharge;
    }
    totalSOC = totalSOC / total_IC;
    // Serial.printf("TOTAL SOC=%i", totalSOC);
    gslc_ElemXProgressSetVal(&m_gui, m_pElemProgress_home_StateOfCharge, totalSOC);
    snprintf(textDisplay, sizeOfChars, "%03.0f %c", (float)totalSOC, char(37));
    gslc_ElemSetTxtStr(&m_gui, ptr_home_StateOfChargeText, textDisplay);
    delete[] textDisplay;
}

void drawCellMonitor_PAGE(str_SlaveBMUData *bmuData, uint16_t selected_IC)
{
    //MonitorScreen
    //--->CellVoltages
    uint sizeOfChars = 8;
    char *textDisplay;
    textDisplay = new char[sizeOfChars];
    snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[selected_IC].cells[0].voltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell1, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[selected_IC].cells[1].voltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell2, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[selected_IC].cells[2].voltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell3, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[selected_IC].cells[3].voltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell4, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[selected_IC].cells[4].voltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell5, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[selected_IC].cells[5].voltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell6, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.4fV", (float)bmuData[selected_IC].cells[6].voltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_cell7, textDisplay);
    delete[] textDisplay;

    //--->CellVoltageBars
    uint avarageVoltage = (uint)(((float)bmuData[0].totalVolatge10000 / 7));
    uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL1P, m_pElemProgressCELL1N, avarageVoltage, bmuData[selected_IC].cells[0].voltage10000);
    uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL2P, m_pElemProgressCELL2N, avarageVoltage, bmuData[selected_IC].cells[1].voltage10000);
    uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL3P, m_pElemProgressCELL3N, avarageVoltage, bmuData[selected_IC].cells[2].voltage10000);
    uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL4P, m_pElemProgressCELL4N, avarageVoltage, bmuData[selected_IC].cells[3].voltage10000);
    uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL5P, m_pElemProgressCELL5N, avarageVoltage, bmuData[selected_IC].cells[4].voltage10000);
    uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL6P, m_pElemProgressCELL6N, avarageVoltage, bmuData[selected_IC].cells[5].voltage10000);
    uiHelper_UICellBarMaker(&m_gui, m_pElemProgressCELL7P, m_pElemProgressCELL7N, avarageVoltage, bmuData[selected_IC].cells[6].voltage10000);

    //--->TemperatureValues
    sizeOfChars = 6;
    textDisplay = new char[sizeOfChars];
    snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[selected_IC].temperatureExternal[0].temperature10 / 10, (char)247);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp1, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[selected_IC].temperatureExternal[1].temperature10 / 10, (char)247);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp2, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[selected_IC].temperatureExternal[2].temperature10 / 10, (char)247);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp3, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[selected_IC].temperatureExternal[3].temperature10 / 10, (char)247);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp4, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[selected_IC].temperatureExternal[4].temperature10 / 10, (char)247);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp5, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[selected_IC].temperatureExternal[5].temperature10 / 10, (char)247);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp6, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[selected_IC].temperatureExternal[6].temperature10 / 10, (char)247);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp7, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[selected_IC].temperatureExternal[7].temperature10 / 10, (char)247);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_temp8, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.1f%cC", (float)bmuData[selected_IC].temperatureIC.temperature10 / 10, (char)247);
    gslc_ElemSetTxtStr(&m_gui, ptr_monitor_tempIC, textDisplay);
    delete[] textDisplay;
}

void drawSettingsBMU_PAGE(str_SlaveBMUData *bmuData)
{
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_GPIO1, bmuData[0].config.gpio[0]);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_GPIO2, bmuData[0].config.gpio[1]);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_GPIO3, bmuData[0].config.gpio[2]);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_GPIO4, bmuData[0].config.gpio[3]);

    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC0, bmuData[0].config.dischargeCellBits[0]);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC1, bmuData[0].config.dischargeCellBits[1]);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC2, bmuData[0].config.dischargeCellBits[2]);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC3, bmuData[0].config.dischargeCellBits[3]);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC4, bmuData[0].config.dischargeCellBits[4]);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC5, bmuData[0].config.dischargeCellBits[5]);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DCC6, bmuData[0].config.dischargeCellBits[6]);

    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_REFON, bmuData[0].config.referenceOn);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DTEN, bmuData[0].config.dischargeTimerEnable);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_ADCOPT, bmuData[0].config.adcOPTMode);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_MCAL, bmuData[0].config.multiCalibration);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_SCONV, bmuData[0].config.enableCellMeasurementRedundancySPIn);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_FDRF, bmuData[0].config.forceDigitalRedundancyFailure);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DIS_RED, bmuData[0].config.disableDigitalRedundancyCheck);
    gslc_ElemXCheckboxSetState(&m_gui, ptr_CheckBox_DTMEN, bmuData[0].config.enableDischargeTimerMonitor);

    uint sizeOfChars = 7;
    char *textDisplay;
    textDisplay = new char[sizeOfChars];
    snprintf(textDisplay, sizeOfChars, "%imin", bmuData[0].config.dischargeTimeOutValueBits);
    gslc_ElemSetTxtStr(&m_gui, ptr_Text_dcto, textDisplay);
    delete[] textDisplay;

    sizeOfChars = 7;
    textDisplay = new char[sizeOfChars];
    snprintf(textDisplay, sizeOfChars, "%.4f", (float)bmuData[0].config.overVoltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_overVoltage, textDisplay);
    snprintf(textDisplay, sizeOfChars, "%.4f", (float)bmuData[0].config.underVoltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_underVoltage, textDisplay);
    delete[] textDisplay;
}

void readSettingsBMU_PAGE(str_SlaveBMUData *bmuData)
{
    bmuData[0].config.gpio[0] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_GPIO1);
    bmuData[0].config.gpio[1] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_GPIO2);
    bmuData[0].config.gpio[2] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_GPIO3);
    bmuData[0].config.gpio[3] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_GPIO4);

    bmuData[0].config.dischargeCellBits[0] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC0);
    bmuData[0].config.dischargeCellBits[1] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC1);
    bmuData[0].config.dischargeCellBits[2] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC2);
    bmuData[0].config.dischargeCellBits[3] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC3);
    bmuData[0].config.dischargeCellBits[4] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC4);
    bmuData[0].config.dischargeCellBits[5] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC5);
    bmuData[0].config.dischargeCellBits[6] = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DCC6);

    bmuData[0].config.referenceOn = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_REFON);
    bmuData[0].config.dischargeTimerEnable = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DTEN);
    bmuData[0].config.adcOPTMode = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_ADCOPT);
    bmuData[0].config.multiCalibration = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_MCAL);
    bmuData[0].config.enableCellMeasurementRedundancySPIn = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_SCONV);
    bmuData[0].config.forceDigitalRedundancyFailure = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_FDRF);
    bmuData[0].config.disableDigitalRedundancyCheck = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DIS_RED);
    bmuData[0].config.enableDischargeTimerMonitor = gslc_ElemXCheckboxGetState(&m_gui, ptr_CheckBox_DTMEN);

    String textTemp = gslc_ElemGetTxtStr(&m_gui, ptr_Text_dcto);
    bmuData[0].config.dischargeTimeOutValue = textTemp.toInt();

    textTemp = gslc_ElemGetTxtStr(&m_gui, m_pElemVal_overVoltage);
    bmuData[0].config.overVoltage10000 = (uint16_t)textTemp.toFloat() * 10000;

    textTemp = gslc_ElemGetTxtStr(&m_gui, m_pElemVal_underVoltage);
    bmuData[0].config.underVoltage10000 = (uint16_t)textTemp.toFloat() * 10000;
}

void drawSettingsBMS_PAGE(str_SystemData *systemData)
{
    uint8_t sizeOfChars = 8;

    char *textDisplay;
    textDisplay = new char[sizeOfChars];
    snprintf(textDisplay, sizeOfChars, "%lu", systemData->timerSleep);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBMS_TimerSleep, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%lu", systemData->timerMemory);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBMS_TimerMemory, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%lu", systemData->timerDisplayUpdate);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBMS_TimerDisplayUpdate, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%lu", systemData->timerLTC68);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBMS_TimerBMUPolling, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%lu", systemData->timerSleep_Remaining);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBMS_TimerSleepRemaining, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%lu", systemData->timerMemory_Remaining);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBMS_TimerMemoryRemaining, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%lu", systemData->timerDisplayUpdate_Remaining);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBMS_TimerDisplayUpdateRemaining, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%lu", systemData->timerLTC68_Remaining);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBMS_TimerBMUPollingRemaining, textDisplay);

    delete[] textDisplay;
}

void drawBalancing_PAGE(str_SystemData *systemData)
{
    uint8_t sizeOfChars = 7;
    char *textDisplay;
    textDisplay = new char[sizeOfChars];
    snprintf(textDisplay, sizeOfChars, "%.4f", (float)systemData->maxAllowedVoltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBalancing_overVoltage, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%.4f", (float)systemData->minAllowedVoltage10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBalancing_underVoltage, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%.4f", (float)systemData->balancingVoltageThreshold10000 / 10000);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBalancing_thresholdVoltage, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%.4f", (float)systemData->maxAlowedTemperature10 / 10);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBalancing_overTemperature, textDisplay);

    snprintf(textDisplay, sizeOfChars, "%.4f", (float)systemData->minAllowedTemperature10 / 10);
    gslc_ElemSetTxtStr(&m_gui, m_pElemVal_SettingsBalancing_underTemperature, textDisplay);

    gslc_ElemXTogglebtnSetState(&m_gui, m_pElemToggle_SettingsBalancing_EnableBalancing, ptr_systemData->balancingEnabled);

    delete[] textDisplay;
}

void drawStatusMessage_BOX(char *message)
{
    uint16_t sizeOfChars = 43;
    char *textDisplay;
    textDisplay = new char[sizeOfChars];
    snprintf(textDisplay, sizeOfChars, "%s", message);
    gslc_ElemSetTxtStr(&m_gui, m_pElemOutTxt_MessageBox, textDisplay);
    gslc_Update(&m_gui);
}

void drawStatusBalancingStatus_BOX(char *message)
{
    uint16_t sizeOfChars = 30;
    char *textDisplay;
    textDisplay = new char[sizeOfChars];
    snprintf(textDisplay, sizeOfChars, "%s", message);
    gslc_ElemSetTxtStr(&m_gui, m_pElemOutTxt_BalancingStatus, textDisplay);
    gslc_Update(&m_gui);
}