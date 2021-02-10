#pragma once
//<App !Start!>
// FILE: [projects.ino]
// Created by GUIslice Builder version: [0.16.b004]
//
// GUIslice Builder Generated File
//
// For the latest guides, updates and support view:
// https://github.com/ImpulseAdventure/GUIslice
//
//<App !End!>

// ------------------------------------------------
// Headers to include
// ------------------------------------------------
#include "projects_GSLC.h"
#include "uiDraw.h"
#include "EEPROMAnything.h"

// ------------------------------------------------
// Program Globals
// ------------------------------------------------

// Save some element references for direct access
//<Save_References !Start!>
gslc_tsElemRef *m_pElemGraph1 = NULL;
gslc_tsElemRef *m_pElemListboxSelectIC = NULL;
gslc_tsElemRef *m_pElemOutTxt_BalancingStatus = NULL;
gslc_tsElemRef *m_pElemOutTxt_MessageBox = NULL;
gslc_tsElemRef *m_pElemProgressCELL1N = NULL;
gslc_tsElemRef *m_pElemProgressCELL1P = NULL;
gslc_tsElemRef *m_pElemProgressCELL2N = NULL;
gslc_tsElemRef *m_pElemProgressCELL2P = NULL;
gslc_tsElemRef *m_pElemProgressCELL3N = NULL;
gslc_tsElemRef *m_pElemProgressCELL3P = NULL;
gslc_tsElemRef *m_pElemProgressCELL4N = NULL;
gslc_tsElemRef *m_pElemProgressCELL4P = NULL;
gslc_tsElemRef *m_pElemProgressCELL5N = NULL;
gslc_tsElemRef *m_pElemProgressCELL5P = NULL;
gslc_tsElemRef *m_pElemProgressCELL6N = NULL;
gslc_tsElemRef *m_pElemProgressCELL6P = NULL;
gslc_tsElemRef *m_pElemProgressCELL7N = NULL;
gslc_tsElemRef *m_pElemProgressCELL7P = NULL;
gslc_tsElemRef *m_pElemProgress_home_StateOfCharge = NULL;
gslc_tsElemRef *m_pElemSpinner1 = NULL;
gslc_tsElemRef *m_pElemToggle_SettingsBalancing_EnableBalancing = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBMS_TimerBMUPolling = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBMS_TimerBMUPollingRemaining = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBMS_TimerDisplayUpdate = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBMS_TimerDisplayUpdateRemaining = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBMS_TimerMemory = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBMS_TimerMemoryRemaining = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBMS_TimerSleep = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBMS_TimerSleepRemaining = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBalancing_overTemperature = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBalancing_overVoltage = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBalancing_thresholdVoltage = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBalancing_underTemperature = NULL;
gslc_tsElemRef *m_pElemVal_SettingsBalancing_underVoltage = NULL;
gslc_tsElemRef *m_pElemVal_overVoltage = NULL;
gslc_tsElemRef *m_pElemVal_underVoltage = NULL;
gslc_tsElemRef *m_pListSlider1 = NULL;
gslc_tsElemRef *pE_ELEM_BTN_HOME = NULL;
gslc_tsElemRef *ptr_CheckBox_ADCOPT = NULL;
gslc_tsElemRef *ptr_CheckBox_DCC0 = NULL;
gslc_tsElemRef *ptr_CheckBox_DCC1 = NULL;
gslc_tsElemRef *ptr_CheckBox_DCC2 = NULL;
gslc_tsElemRef *ptr_CheckBox_DCC3 = NULL;
gslc_tsElemRef *ptr_CheckBox_DCC4 = NULL;
gslc_tsElemRef *ptr_CheckBox_DCC5 = NULL;
gslc_tsElemRef *ptr_CheckBox_DCC6 = NULL;
gslc_tsElemRef *ptr_CheckBox_DIS_RED = NULL;
gslc_tsElemRef *ptr_CheckBox_DTEN = NULL;
gslc_tsElemRef *ptr_CheckBox_DTMEN = NULL;
gslc_tsElemRef *ptr_CheckBox_FDRF = NULL;
gslc_tsElemRef *ptr_CheckBox_GPIO1 = NULL;
gslc_tsElemRef *ptr_CheckBox_GPIO2 = NULL;
gslc_tsElemRef *ptr_CheckBox_GPIO3 = NULL;
gslc_tsElemRef *ptr_CheckBox_GPIO4 = NULL;
gslc_tsElemRef *ptr_CheckBox_MCAL = NULL;
gslc_tsElemRef *ptr_CheckBox_REFON = NULL;
gslc_tsElemRef *ptr_CheckBox_SCONV = NULL;
gslc_tsElemRef *ptr_Text_dcto = NULL;
gslc_tsElemRef *ptr_home_Current = NULL;
gslc_tsElemRef *ptr_home_StateOfChargeText = NULL;
gslc_tsElemRef *ptr_home_TotalVoltage = NULL;
gslc_tsElemRef *ptr_home_uptime = NULL;
gslc_tsElemRef *ptr_monitor_cell1 = NULL;
gslc_tsElemRef *ptr_monitor_cell2 = NULL;
gslc_tsElemRef *ptr_monitor_cell3 = NULL;
gslc_tsElemRef *ptr_monitor_cell4 = NULL;
gslc_tsElemRef *ptr_monitor_cell5 = NULL;
gslc_tsElemRef *ptr_monitor_cell6 = NULL;
gslc_tsElemRef *ptr_monitor_cell7 = NULL;
gslc_tsElemRef *ptr_monitor_temp1 = NULL;
gslc_tsElemRef *ptr_monitor_temp2 = NULL;
gslc_tsElemRef *ptr_monitor_temp3 = NULL;
gslc_tsElemRef *ptr_monitor_temp4 = NULL;
gslc_tsElemRef *ptr_monitor_temp5 = NULL;
gslc_tsElemRef *ptr_monitor_temp6 = NULL;
gslc_tsElemRef *ptr_monitor_temp7 = NULL;
gslc_tsElemRef *ptr_monitor_temp8 = NULL;
gslc_tsElemRef *ptr_monitor_tempIC = NULL;
gslc_tsElemRef *m_pElemKeyPadNum = NULL;
//<Save_References !End!>

// Define debug message function
static int16_t DebugOut(char ch)
{
  if (ch == (char)'\n')
    Serial.println("");
  else
    Serial.write(ch);
  return 0;
}

// ------------------------------------------------
// Callback Methods
// ------------------------------------------------
// Common Button callback
bool CbBtnCommon(void *pvGui, void *pvElemRef, gslc_teTouch eTouch, int16_t nX, int16_t nY)
{
  // Typecast the parameters to match the GUI and element types
  gslc_tsGui *pGui = (gslc_tsGui *)(pvGui);
  gslc_tsElemRef *pElemRef = (gslc_tsElemRef *)(pvElemRef);
  gslc_tsElem *pElem = gslc_GetElemFromRef(pGui, pElemRef);

  if (eTouch == GSLC_TOUCH_UP_IN)
  {
    // From the element's ID we can determine which button was pressed.
    switch (pElem->nId)
    {
      //<Button Enums !Start!>
    case E_ELEM_BTN_HOME:
      gslc_SetPageCur(&m_gui, E_PG_HOME);
      break;
    case E_ELEM_BTN_SETTINGS:
      gslc_SetPageCur(&m_gui, E_PG_SETTINGS_MAIN);
      break;
    case E_ELEM_BTN_MONITOR:
      gslc_SetPageCur(&m_gui, E_PG_MONITOR);
      break;
    case E_ELEM_BTN_BALANCING:
      gslc_SetPageCur(&m_gui, E_PG_SETTINGS_BALANCING);
      break;
    case E_ELEM_BTN_ICSELECT:
      break;
    case E_ELEM_BTN5:
      break;
    case E_ELEM_BTN_BMUSETTINGS:
      gslc_SetPageCur(&m_gui, E_PG_SETTINGS_BMU);
      break;
    case E_ELEM_BTN_BMSSETTINGS:
      gslc_SetPageCur(&m_gui, E_PG_SETTINGS_BMS);
      break;
    case E_ELEM_BTN8:
      break;
    case E_ELEM_BTN11:
      break;
    case E_ELEM_BTN12:
      break;
    case E_ELEM_BTN_SETTINGS_BMU_RESET:
      ptr_ltc6810->ltc6810_pushConfig(ptr_bmuData, true, false);
      drawSettingsBMU_PAGE(ptr_bmuData);
      break;
    case E_ELEM_BTN_SETTINGS_BMU_WRITE:
      readSettingsBMU_PAGE(ptr_bmuData);
      ptr_ltc6810->ltc6810_pushConfig(ptr_bmuData, false, false);
      break;
    case E_ELEM_BTN_SETTINGS_BMU_READ:
      ptr_ltc6810->ltc6810_pullConfig(ptr_bmuData, false);
      drawSettingsBMU_PAGE(ptr_bmuData);
      break;
    case E_ELEM_NUMINPUT1:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_overVoltage);
      break;
    case E_ELEM_NUMINPUT2:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_underVoltage);
      break;
    case E_ELEM_NUMINPUT_TIMERSLEEP:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_SettingsBMS_TimerSleep);
      break;
    case E_ELEM_NUMINPUT_TIMERMEMORY:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_SettingsBMS_TimerMemory);
      break;
    case E_ELEM_NUMINPUT_TIMERDISPLAYUPDATE:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_SettingsBMS_TimerDisplayUpdate);
      break;
    case E_ELEM_NUMINPUT_TIMERBMUPOLLING:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_SettingsBMS_TimerBMUPolling);
      break;
    case E_ELEM_TOGGLE2:
      // TODO Add code for Toggle button ON/OFF state
      if (gslc_ElemXTogglebtnGetState(&m_gui, m_pElemToggle_SettingsBalancing_EnableBalancing))
      {
        ptr_systemData->balancingEnabled = true;
      }
      else
      {
        ptr_systemData->balancingEnabled = false;
      }
      break;
    case E_ELEM_NUMINPUT_OVERVOLTAGE:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_SettingsBalancing_overVoltage);
      break;
    case E_ELEM_NUMINPUT_UNDERVOLTAGE:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_SettingsBalancing_underVoltage);
      break;
    case E_ELEM_NUMINPUT5:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_SettingsBalancing_thresholdVoltage);
      break;
    case E_ELEM_NUMINPUT6:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_SettingsBalancing_overTemperature);
      break;
    case E_ELEM_NUMINPUT7:
      // Clicked on edit field, so show popup box and associate with this text field
      gslc_ElemXKeyPadInputAsk(&m_gui, m_pElemKeyPadNum, E_POP_KEYPAD_NUM, m_pElemVal_SettingsBalancing_underTemperature);
      break;
    case E_ELEM_BTN20:
      break;
    case E_ELEM_BTN_BALANCING_SAVE:
      int savedSize = EEPROM_writeAnything(0, *ptr_systemData);
      if (savedSize == sizeof(*ptr_systemData))
      {
        drawStatusMessage_BOX("Saved Config!");
      }
      break;
    case E_ELEM_BTN_BALANCING_RESET:
      break;
      //<Button Enums !End!>
    default:
      break;
    }
  }
  return true;
}
//<Checkbox Callback !Start!>
//<Checkbox Callback !End!>
// KeyPad Input Ready callback
bool CbKeypad(void *pvGui, void *pvElemRef, int16_t nState, void *pvData)
{
  gslc_tsGui *pGui = (gslc_tsGui *)pvGui;
  gslc_tsElemRef *pElemRef = (gslc_tsElemRef *)(pvElemRef);
  gslc_tsElem *pElem = gslc_GetElemFromRef(pGui, pElemRef);

  // From the pvData we can get the ID element that is ready.
  int16_t nTargetElemId = gslc_ElemXKeyPadDataTargetIdGet(pGui, pvData);
  if (nState == XKEYPAD_CB_STATE_DONE)
  {
    // User clicked on Enter to leave popup
    // - If we have a popup active, pass the return value directly to
    //   the corresponding value field
    switch (nTargetElemId)
    {
      //<Keypad Enums !Start!>
    case E_ELEM_NUMINPUT1:
      gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_overVoltage, pvData);
      gslc_PopupHide(&m_gui);
      break;
    case E_ELEM_NUMINPUT2:
      gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_underVoltage, pvData);
      gslc_PopupHide(&m_gui);
      break;
    case E_ELEM_NUMINPUT5:
      ptr_systemData->balancingVoltageThreshold10000 = (uint16_t)(atof(gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_SettingsBalancing_thresholdVoltage, pvData)) * 10000);
      gslc_PopupHide(&m_gui);
      break;
    case E_ELEM_NUMINPUT6:
      ptr_systemData->maxAlowedTemperature10 = (uint16_t)(atof(gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_SettingsBalancing_overTemperature, pvData)) * 10);
      gslc_PopupHide(&m_gui);
      break;
    case E_ELEM_NUMINPUT7:
      ptr_systemData->minAllowedTemperature10 = (uint16_t)(atof(gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_SettingsBalancing_underTemperature, pvData)) * 10);
      gslc_PopupHide(&m_gui);
      break;
    case E_ELEM_NUMINPUT_OVERVOLTAGE:
      ptr_systemData->maxAllowedVoltage10000 = (uint16_t)(atof(gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_SettingsBalancing_overVoltage, pvData)) * 10000);
      gslc_PopupHide(&m_gui);
      break;
    case E_ELEM_NUMINPUT_UNDERVOLTAGE:
      ptr_systemData->minAllowedVoltage10000 = (uint16_t)(atof(gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_SettingsBalancing_underVoltage, pvData)) * 10000);
      gslc_PopupHide(&m_gui);
      break;
    case E_ELEM_NUMINPUT_TIMERSLEEP:
      gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_SettingsBMS_TimerSleep, pvData);
      gslc_PopupHide(&m_gui);
      break;
    case E_ELEM_NUMINPUT_TIMERMEMORY:
      gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_SettingsBMS_TimerMemory, pvData);
      gslc_PopupHide(&m_gui);
      break;
    case E_ELEM_NUMINPUT_TIMERDISPLAYUPDATE:
      gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_SettingsBMS_TimerDisplayUpdate, pvData);
      gslc_PopupHide(&m_gui);
      break;
    case E_ELEM_NUMINPUT_TIMERBMUPOLLING:
      gslc_ElemXKeyPadInputGet(pGui, m_pElemVal_SettingsBMS_TimerBMUPolling, pvData);
      gslc_PopupHide(&m_gui);
      break;
      //<Keypad Enums !End!>
    default:
      break;
    }
  }
  else if (nState == XKEYPAD_CB_STATE_CANCEL)
  {
    // User escaped from popup, so don't update values
    gslc_PopupHide(&m_gui);
  }
  return true;
}
// Spinner Input Ready callback
bool CbSpinner(void *pvGui, void *pvElemRef, int16_t nState, void *pvData)
{
  gslc_tsGui *pGui = (gslc_tsGui *)pvGui;
  gslc_tsElemRef *pElemRef = (gslc_tsElemRef *)(pvElemRef);
  gslc_tsElem *pElem = gslc_GetElemFromRef(pGui, pElemRef);

  // NOTE: pvData is NULL
  if (nState == XSPINNER_CB_STATE_UPDATE)
  {
    // From the element's ID we can determine which input field is ready.
    switch (pElem->nId)
    {
      //<Spinner Enums !Start!>
    case E_ELEM_SPINNER1:
      //TODO- Add Spinner handling code
      // using gslc_ElemXSpinnerGetCounter(&m_gui, &m_sXSpinner1);
      break;

      //<Spinner Enums !End!>
    default:
      break;
    }
  }
}
bool CbListbox(void *pvGui, void *pvElemRef, int16_t nSelId)
{
  gslc_tsGui *pGui = (gslc_tsGui *)(pvGui);
  gslc_tsElemRef *pElemRef = (gslc_tsElemRef *)(pvElemRef);
  gslc_tsElem *pElem = gslc_GetElemFromRef(pGui, pElemRef);
  char acTxt[MAX_STR + 1];

  if (pElemRef == NULL)
  {
    return false;
  }

  // From the element's ID we can determine which listbox was active.
  switch (pElem->nId)
  {
    //<Listbox Enums !Start!>
  case E_ELEM_LISTBOX1:
    if (nSelId != XLISTBOX_SEL_NONE)
    {
      gslc_ElemXListboxGetItem(&m_gui, pElemRef, nSelId, acTxt, MAX_STR);
    }
    break;

    //<Listbox Enums !End!>
  default:
    break;
  }
  return true;
}
//<Draw Callback !Start!>
//<Draw Callback !End!>

// Callback function for when a slider's position has been updated
bool CbSlidePos(void *pvGui, void *pvElemRef, int16_t nPos)
{
  gslc_tsGui *pGui = (gslc_tsGui *)(pvGui);
  gslc_tsElemRef *pElemRef = (gslc_tsElemRef *)(pvElemRef);
  gslc_tsElem *pElem = gslc_GetElemFromRef(pGui, pElemRef);
  int16_t nVal;

  // From the element's ID we can determine which slider was updated.
  switch (pElem->nId)
  {
    //<Slider Enums !Start!>
  case E_LISTSCROLL1:
    // Fetch the slider position
    nVal = gslc_ElemXSliderGetPos(pGui, m_pListSlider1);
    break;

    //<Slider Enums !End!>
  default:
    break;
  }

  return true;
}
//<Tick Callback !Start!>
//<Tick Callback !End!>
