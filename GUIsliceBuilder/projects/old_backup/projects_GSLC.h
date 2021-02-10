//<File !Start!>
// FILE: [projects_GSLC.h]
// Created by GUIslice Builder version: [0.16.b004]
//
// GUIslice Builder Generated GUI Framework File
//
// For the latest guides, updates and support view:
// https://github.com/ImpulseAdventure/GUIslice
//
//<File !End!>

#ifndef _GUISLICE_GEN_H
#define _GUISLICE_GEN_H
#define FONTREF_MODE_1 GSLC_FONTREF_MODE_1

// ------------------------------------------------
// Headers to include
// ------------------------------------------------
#include "GUIslice.h"
#include "GUIslice_drv.h"

// Include any extended elements
//<Includes !Start!>
// Include extended elements
#include "elem/XCheckbox.h"
#include "elem/XGraph.h"
#include "elem/XKeyPad_Num.h"
#include "elem/XListbox.h"
#include "elem/XProgress.h"
#include "elem/XSlider.h"
#include "elem/XSpinner.h"
#include "elem/XTogglebtn.h"

// Ensure optional features are enabled in the configuration
#if !(GSLC_FEATURE_COMPOUND)
  #error "Config: GSLC_FEATURE_COMPOUND required for this program but not enabled. Please see: https://github.com/ImpulseAdventure/GUIslice/wiki/Configuring-GUIslice"
#endif
//<Includes !End!>

// ------------------------------------------------
// Headers and Defines for fonts
// Note that font files are located within the Adafruit-GFX library folder:
// ------------------------------------------------
//<Fonts !Start!>
#include <SPI.h>
#include "font_AwesomeF000.h"
//<Fonts !End!>

// ------------------------------------------------
// Defines for resources
// ------------------------------------------------
//<Resources !Start!>
//<Resources !End!>

// ------------------------------------------------
// Enumerations for pages, elements, fonts, images
// ------------------------------------------------
//<Enum !Start!>
enum {E_PG_BASE,E_PG_BOOT,E_PG_HOME,E_PG_Monitor,E_PG_SELECT_IC
      ,E_PG_SETTINGS_MAIN,E_PG_SETTINGS_BMU,E_PG_SETTINGS_BMS
      ,E_PG_SETTINGS_BALANCING,E_POP_KEYPAD_NUM};
enum {E_DRAW_LINE1,E_DRAW_LINE2,E_DRAW_LINE3,E_DRAW_LINE4,E_DRAW_LINE5
      ,E_DRAW_LINE6,E_ELEM_BOX1,E_ELEM_BOX2,E_ELEM_BTN11,E_ELEM_BTN12
      ,E_ELEM_BTN5,E_ELEM_BTN7,E_ELEM_BTN8,E_ELEM_BTN_BALANCING
      ,E_ELEM_BTN_BMUSETTINGS,E_ELEM_BTN_HOME,E_ELEM_BTN_ICSELECT
      ,E_ELEM_BTN_MONITOR,E_ELEM_BTN_SETTINGS
      ,E_ELEM_BTN_SETTINGS_BMU_READ,E_ELEM_BTN_SETTINGS_BMU_RESET
      ,E_ELEM_BTN_SETTINGS_BMU_WRITE,E_ELEM_CHECK13,E_ELEM_CHECK14
      ,E_ELEM_CHECK15,E_ELEM_CHECK16,E_ELEM_CHECK17,E_ELEM_CHECK18
      ,E_ELEM_CHECK19,E_ELEM_CHECK20,E_ELEM_CHECK21,E_ELEM_CHECK22
      ,E_ELEM_CHECK23,E_ELEM_CHECK24,E_ELEM_CHECK25,E_ELEM_CHECK26
      ,E_ELEM_CHECK27,E_ELEM_CHECK_GPIO1,E_ELEM_CHECK_GPIO2
      ,E_ELEM_CHECK_GPIO3,E_ELEM_CHECK_GPIO4,E_ELEM_GRAPH1
      ,E_ELEM_LISTBOX1,E_ELEM_NUMINPUT1,E_ELEM_NUMINPUT2
      ,E_ELEM_NUMINPUT3,E_ELEM_NUMINPUT4,E_ELEM_NUMINPUT5
      ,E_ELEM_NUMINPUT6,E_ELEM_NUMINPUT7,E_ELEM_PROGRESS1
      ,E_ELEM_PROGRESS15,E_ELEM_PROGRESS16,E_ELEM_PROGRESS17
      ,E_ELEM_PROGRESS18,E_ELEM_PROGRESS19,E_ELEM_PROGRESS20
      ,E_ELEM_PROGRESS21,E_ELEM_PROGRESS22,E_ELEM_PROGRESS23
      ,E_ELEM_PROGRESS24,E_ELEM_PROGRESS25,E_ELEM_PROGRESS26
      ,E_ELEM_PROGRESS27,E_ELEM_PROGRESS28,E_ELEM_RADIO1,E_ELEM_RADIO2
      ,E_ELEM_RADIO3,E_ELEM_SPINNER1,E_ELEM_TEXT100,E_ELEM_TEXT101
      ,E_ELEM_TEXT102,E_ELEM_TEXT104,E_ELEM_TEXT105,E_ELEM_TEXT106
      ,E_ELEM_TEXT107,E_ELEM_TEXT108,E_ELEM_TEXT109,E_ELEM_TEXT11
      ,E_ELEM_TEXT114,E_ELEM_TEXT115,E_ELEM_TEXT116,E_ELEM_TEXT117
      ,E_ELEM_TEXT119,E_ELEM_TEXT12,E_ELEM_TEXT121,E_ELEM_TEXT13
      ,E_ELEM_TEXT14,E_ELEM_TEXT15,E_ELEM_TEXT16,E_ELEM_TEXT17
      ,E_ELEM_TEXT18,E_ELEM_TEXT2,E_ELEM_TEXT20,E_ELEM_TEXT21
      ,E_ELEM_TEXT22,E_ELEM_TEXT23,E_ELEM_TEXT24,E_ELEM_TEXT25
      ,E_ELEM_TEXT26,E_ELEM_TEXT27,E_ELEM_TEXT28,E_ELEM_TEXT29
      ,E_ELEM_TEXT3,E_ELEM_TEXT30,E_ELEM_TEXT31,E_ELEM_TEXT32
      ,E_ELEM_TEXT33,E_ELEM_TEXT34,E_ELEM_TEXT35,E_ELEM_TEXT36
      ,E_ELEM_TEXT37,E_ELEM_TEXT38,E_ELEM_TEXT39,E_ELEM_TEXT4
      ,E_ELEM_TEXT40,E_ELEM_TEXT41,E_ELEM_TEXT42,E_ELEM_TEXT43
      ,E_ELEM_TEXT44,E_ELEM_TEXT45,E_ELEM_TEXT46,E_ELEM_TEXT47
      ,E_ELEM_TEXT5,E_ELEM_TEXT54,E_ELEM_TEXT55,E_ELEM_TEXT56
      ,E_ELEM_TEXT58,E_ELEM_TEXT59,E_ELEM_TEXT6,E_ELEM_TEXT60
      ,E_ELEM_TEXT61,E_ELEM_TEXT63,E_ELEM_TEXT64,E_ELEM_TEXT65
      ,E_ELEM_TEXT66,E_ELEM_TEXT67,E_ELEM_TEXT68,E_ELEM_TEXT69
      ,E_ELEM_TEXT70,E_ELEM_TEXT71,E_ELEM_TEXT72,E_ELEM_TEXT73
      ,E_ELEM_TEXT74,E_ELEM_TEXT75,E_ELEM_TEXT76,E_ELEM_TEXT77
      ,E_ELEM_TEXT78,E_ELEM_TEXT79,E_ELEM_TEXT80,E_ELEM_TEXT81
      ,E_ELEM_TEXT82,E_ELEM_TEXT83,E_ELEM_TEXT84,E_ELEM_TEXT85
      ,E_ELEM_TEXT86,E_ELEM_TEXT87,E_ELEM_TEXT88,E_ELEM_TEXT89
      ,E_ELEM_TEXT90,E_ELEM_TEXT91,E_ELEM_TEXT92,E_ELEM_TEXT93
      ,E_ELEM_TEXT94,E_ELEM_TEXT95,E_ELEM_TEXT96,E_ELEM_TEXT97
      ,E_ELEM_TEXT98,E_ELEM_TEXT99,E_ELEM_TOGGLE1,E_ELEM_TOGGLE2
      ,E_LISTSCROLL1,E_ELEM_KEYPAD_NUM};
enum {E_GROUP1};
// Must use separate enum for fonts with MAX_FONT at end to use gslc_FontSet.
enum {E_AWESOMEF000_12,E_AWESOMEF000_8,E_BUILTIN10X16,E_BUILTIN20X32
      ,E_BUILTIN5X8,MAX_FONT};
//<Enum !End!>

// ------------------------------------------------
// Instantiate the GUI
// ------------------------------------------------

// ------------------------------------------------
// Define the maximum number of elements and pages
// ------------------------------------------------
//<ElementDefines !Start!>
#define MAX_PAGE                10

#define MAX_ELEM_PG_BASE 9 // # Elems total on page
#define MAX_ELEM_PG_BASE_RAM MAX_ELEM_PG_BASE // # Elems in RAM

#define MAX_ELEM_PG_BOOT 1 // # Elems total on page
#define MAX_ELEM_PG_BOOT_RAM MAX_ELEM_PG_BOOT // # Elems in RAM

#define MAX_ELEM_PG_HOME 14 // # Elems total on page
#define MAX_ELEM_PG_HOME_RAM MAX_ELEM_PG_HOME // # Elems in RAM

#define MAX_ELEM_PG_Monitor 68 // # Elems total on page
#define MAX_ELEM_PG_Monitor_RAM MAX_ELEM_PG_Monitor // # Elems in RAM

#define MAX_ELEM_PG_SELECT_IC 4 // # Elems total on page
#define MAX_ELEM_PG_SELECT_IC_RAM MAX_ELEM_PG_SELECT_IC // # Elems in RAM

#define MAX_ELEM_PG_SETTINGS_MAIN 3 // # Elems total on page
#define MAX_ELEM_PG_SETTINGS_MAIN_RAM MAX_ELEM_PG_SETTINGS_MAIN // # Elems in RAM

#define MAX_ELEM_PG_SETTINGS_BMU 53 // # Elems total on page
#define MAX_ELEM_PG_SETTINGS_BMU_RAM MAX_ELEM_PG_SETTINGS_BMU // # Elems in RAM

#define MAX_ELEM_PG_SETTINGS_BMS 2 // # Elems total on page
#define MAX_ELEM_PG_SETTINGS_BMS_RAM MAX_ELEM_PG_SETTINGS_BMS // # Elems in RAM

#define MAX_ELEM_PG_SETTINGS_BALANCING 19 // # Elems total on page
#define MAX_ELEM_PG_SETTINGS_BALANCING_RAM MAX_ELEM_PG_SETTINGS_BALANCING // # Elems in RAM
//<ElementDefines !End!>

// ------------------------------------------------
// Create element storage
// ------------------------------------------------
gslc_tsGui m_gui;
gslc_tsDriver m_drv;
gslc_tsFont m_asFont[MAX_FONT];
gslc_tsPage m_asPage[MAX_PAGE];

//<GUI_Extra_Elements !Start!>
gslc_tsElem                     m_asBasePage1Elem[MAX_ELEM_PG_BASE_RAM];
gslc_tsElemRef                  m_asBasePage1ElemRef[MAX_ELEM_PG_BASE];
gslc_tsElem                     m_asPage1Elem[MAX_ELEM_PG_BOOT_RAM];
gslc_tsElemRef                  m_asPage1ElemRef[MAX_ELEM_PG_BOOT];
gslc_tsElem                     m_asPage2Elem[MAX_ELEM_PG_HOME_RAM];
gslc_tsElemRef                  m_asPage2ElemRef[MAX_ELEM_PG_HOME];
gslc_tsElem                     m_asPage3Elem[MAX_ELEM_PG_Monitor_RAM];
gslc_tsElemRef                  m_asPage3ElemRef[MAX_ELEM_PG_Monitor];
gslc_tsElem                     m_asPage4Elem[MAX_ELEM_PG_SELECT_IC_RAM];
gslc_tsElemRef                  m_asPage4ElemRef[MAX_ELEM_PG_SELECT_IC];
gslc_tsElem                     m_asPage5Elem[MAX_ELEM_PG_SETTINGS_MAIN_RAM];
gslc_tsElemRef                  m_asPage5ElemRef[MAX_ELEM_PG_SETTINGS_MAIN];
gslc_tsElem                     m_asPage6Elem[MAX_ELEM_PG_SETTINGS_BMU_RAM];
gslc_tsElemRef                  m_asPage6ElemRef[MAX_ELEM_PG_SETTINGS_BMU];
gslc_tsElem                     m_asPage7Elem[MAX_ELEM_PG_SETTINGS_BMS_RAM];
gslc_tsElemRef                  m_asPage7ElemRef[MAX_ELEM_PG_SETTINGS_BMS];
gslc_tsElem                     m_asPage8Elem[MAX_ELEM_PG_SETTINGS_BALANCING_RAM];
gslc_tsElemRef                  m_asPage8ElemRef[MAX_ELEM_PG_SETTINGS_BALANCING];
gslc_tsElem                     m_asKeypadNumElem[1];
gslc_tsElemRef                  m_asKeypadNumElemRef[1];
gslc_tsXKeyPad                  m_sKeyPadNum;
gslc_tsXCheckbox                m_asXRadio1;
gslc_tsXCheckbox                m_asXRadio2;
gslc_tsXCheckbox                m_asXRadio3;
gslc_tsXProgress                m_sXBarGauge28;
gslc_tsXProgress                m_sXBarGauge1;
gslc_tsXProgress                m_sXBarGauge15;
gslc_tsXProgress                m_sXBarGauge16;
gslc_tsXProgress                m_sXBarGauge17;
gslc_tsXProgress                m_sXBarGauge18;
gslc_tsXProgress                m_sXBarGauge19;
gslc_tsXProgress                m_sXBarGauge20;
gslc_tsXProgress                m_sXBarGauge21;
gslc_tsXProgress                m_sXBarGauge22;
gslc_tsXProgress                m_sXBarGauge23;
gslc_tsXProgress                m_sXBarGauge24;
gslc_tsXProgress                m_sXBarGauge25;
gslc_tsXProgress                m_sXBarGauge26;
gslc_tsXProgress                m_sXBarGauge27;
gslc_tsXListbox                 m_sListbox1;
// - Note that XLISTBOX_BUF_OH_R is extra required per item
char                            m_acListboxBuf1[50 + XLISTBOX_BUF_OH_R];
gslc_tsXSlider                  m_sListScroll1;
gslc_tsXGraph                   m_sGraph1;
int16_t                         m_anGraphBuf1[10]; // NRows=10
gslc_tsXCheckbox                m_asXCheck9;
gslc_tsXCheckbox                m_asXCheck10;
gslc_tsXCheckbox                m_asXCheck11;
gslc_tsXCheckbox                m_asXCheck12;
gslc_tsXCheckbox                m_asXCheck13;
gslc_tsXCheckbox                m_asXCheck14;
gslc_tsXCheckbox                m_asXCheck15;
gslc_tsXCheckbox                m_asXCheck16;
gslc_tsXCheckbox                m_asXCheck17;
gslc_tsXCheckbox                m_asXCheck18;
gslc_tsXCheckbox                m_asXCheck19;
gslc_tsXCheckbox                m_asXCheck20;
gslc_tsXCheckbox                m_asXCheck21;
gslc_tsXCheckbox                m_asXCheck22;
gslc_tsXCheckbox                m_asXCheck23;
gslc_tsXCheckbox                m_asXCheck24;
gslc_tsXCheckbox                m_asXCheck25;
gslc_tsXCheckbox                m_asXCheck26;
gslc_tsXCheckbox                m_asXCheck27;
gslc_tsXSpinner                 m_sXSpinner1;
gslc_tsXTogglebtn               m_asXToggle1;
gslc_tsXTogglebtn               m_asXToggle2;

#define MAX_STR                 100

//<GUI_Extra_Elements !End!>

// ------------------------------------------------
// Program Globals
// ------------------------------------------------

// Element References for direct access
//<Extern_References !Start!>
extern gslc_tsElemRef* m_pElemGraph1;
extern gslc_tsElemRef* m_pElemListboxSelectIC;
extern gslc_tsElemRef* m_pElemProgressCELL1N;
extern gslc_tsElemRef* m_pElemProgressCELL1P;
extern gslc_tsElemRef* m_pElemProgressCELL2N;
extern gslc_tsElemRef* m_pElemProgressCELL2P;
extern gslc_tsElemRef* m_pElemProgressCELL3N;
extern gslc_tsElemRef* m_pElemProgressCELL3P;
extern gslc_tsElemRef* m_pElemProgressCELL4N;
extern gslc_tsElemRef* m_pElemProgressCELL4P;
extern gslc_tsElemRef* m_pElemProgressCELL5N;
extern gslc_tsElemRef* m_pElemProgressCELL5P;
extern gslc_tsElemRef* m_pElemProgressCELL6N;
extern gslc_tsElemRef* m_pElemProgressCELL6P;
extern gslc_tsElemRef* m_pElemProgressCELL7N;
extern gslc_tsElemRef* m_pElemProgressCELL7P;
extern gslc_tsElemRef* m_pElemProgress_home_StateOfCharge;
extern gslc_tsElemRef* m_pElemSpinner1;
extern gslc_tsElemRef* m_pElemToggle1;
extern gslc_tsElemRef* m_pElemToggle_SettingsBalancing_EnableBalancing;
extern gslc_tsElemRef* m_pElemVal_SettingsBalancing_overTemperature;
extern gslc_tsElemRef* m_pElemVal_SettingsBalancing_overVoltage;
extern gslc_tsElemRef* m_pElemVal_SettingsBalancing_thresholdVoltage;
extern gslc_tsElemRef* m_pElemVal_SettingsBalancing_underTemperature;
extern gslc_tsElemRef* m_pElemVal_SettingsBalancing_underVoltage;
extern gslc_tsElemRef* m_pElemVal_overVoltage;
extern gslc_tsElemRef* m_pElemVal_underVoltage;
extern gslc_tsElemRef* m_pListSlider1;
extern gslc_tsElemRef* ptr_CheckBox_ADCOPT;
extern gslc_tsElemRef* ptr_CheckBox_DCC0;
extern gslc_tsElemRef* ptr_CheckBox_DCC1;
extern gslc_tsElemRef* ptr_CheckBox_DCC2;
extern gslc_tsElemRef* ptr_CheckBox_DCC3;
extern gslc_tsElemRef* ptr_CheckBox_DCC4;
extern gslc_tsElemRef* ptr_CheckBox_DCC5;
extern gslc_tsElemRef* ptr_CheckBox_DCC6;
extern gslc_tsElemRef* ptr_CheckBox_DIS_RED;
extern gslc_tsElemRef* ptr_CheckBox_DTEN;
extern gslc_tsElemRef* ptr_CheckBox_DTMEN;
extern gslc_tsElemRef* ptr_CheckBox_FDRF;
extern gslc_tsElemRef* ptr_CheckBox_GPIO1;
extern gslc_tsElemRef* ptr_CheckBox_GPIO2;
extern gslc_tsElemRef* ptr_CheckBox_GPIO3;
extern gslc_tsElemRef* ptr_CheckBox_GPIO4;
extern gslc_tsElemRef* ptr_CheckBox_MCAL;
extern gslc_tsElemRef* ptr_CheckBox_REFON;
extern gslc_tsElemRef* ptr_CheckBox_SCONV;
extern gslc_tsElemRef* ptr_Text_dcto;
extern gslc_tsElemRef* ptr_home_Current;
extern gslc_tsElemRef* ptr_home_StateOfChargeText;
extern gslc_tsElemRef* ptr_home_TotalVoltage;
extern gslc_tsElemRef* ptr_home_uptime;
extern gslc_tsElemRef* ptr_monitor_cell1;
extern gslc_tsElemRef* ptr_monitor_cell1_SVoltage;
extern gslc_tsElemRef* ptr_monitor_cell2;
extern gslc_tsElemRef* ptr_monitor_cell2_SVoltage;
extern gslc_tsElemRef* ptr_monitor_cell3;
extern gslc_tsElemRef* ptr_monitor_cell3_SVoltage;
extern gslc_tsElemRef* ptr_monitor_cell4;
extern gslc_tsElemRef* ptr_monitor_cell4_SVoltage;
extern gslc_tsElemRef* ptr_monitor_cell5;
extern gslc_tsElemRef* ptr_monitor_cell5_SVoltage;
extern gslc_tsElemRef* ptr_monitor_cell6;
extern gslc_tsElemRef* ptr_monitor_cell6_SVoltage;
extern gslc_tsElemRef* ptr_monitor_cell7;
extern gslc_tsElemRef* ptr_monitor_cell7_SVoltage;
extern gslc_tsElemRef* ptr_monitor_temp1;
extern gslc_tsElemRef* ptr_monitor_temp2;
extern gslc_tsElemRef* ptr_monitor_temp3;
extern gslc_tsElemRef* ptr_monitor_temp4;
extern gslc_tsElemRef* ptr_monitor_temp5;
extern gslc_tsElemRef* ptr_monitor_temp6;
extern gslc_tsElemRef* ptr_monitor_temp7;
extern gslc_tsElemRef* ptr_monitor_temp8;
extern gslc_tsElemRef* ptr_monitor_tempIC;
extern gslc_tsElemRef* m_pElemKeyPadNum;
//<Extern_References !End!>

// Define debug message function
static int16_t DebugOut(char ch);

// ------------------------------------------------
// Callback Methods
// ------------------------------------------------
bool CbBtnCommon(void *pvGui, void *pvElemRef, gslc_teTouch eTouch, int16_t nX, int16_t nY);
bool CbCheckbox(void *pvGui, void *pvElemRef, int16_t nSelId, bool bState);
bool CbDrawScanner(void *pvGui, void *pvElemRef, gslc_teRedrawType eRedraw);
bool CbKeypad(void *pvGui, void *pvElemRef, int16_t nState, void *pvData);
bool CbListbox(void *pvGui, void *pvElemRef, int16_t nSelId);
bool CbSlidePos(void *pvGui, void *pvElemRef, int16_t nPos);
bool CbSpinner(void *pvGui, void *pvElemRef, int16_t nState, void *pvData);
bool CbTickScanner(void *pvGui, void *pvScope);

// ------------------------------------------------
// Create page elements
// ------------------------------------------------
void InitGUIslice_gen()
{
  gslc_tsElemRef *pElemRef = NULL;

  if (!gslc_Init(&m_gui, &m_drv, m_asPage, MAX_PAGE, m_asFont, MAX_FONT))
  {
    return;
  }

  // ------------------------------------------------
  // Load Fonts
  // ------------------------------------------------
  //<Load_Fonts !Start!>
    if (!gslc_FontSet(&m_gui,E_AWESOMEF000_12,GSLC_FONTREF_PTR,&AwesomeF000_12,1)) { return; }
    gslc_FontSetMode(&m_gui, E_AWESOMEF000_12, FONTREF_MODE_1);	
    if (!gslc_FontSet(&m_gui,E_AWESOMEF000_8,GSLC_FONTREF_PTR,&AwesomeF000_8,1)) { return; }
    gslc_FontSetMode(&m_gui, E_AWESOMEF000_8, FONTREF_MODE_1);	
    if (!gslc_FontSet(&m_gui,E_BUILTIN10X16,GSLC_FONTREF_PTR,NULL,2)) { return; }
    if (!gslc_FontSet(&m_gui,E_BUILTIN20X32,GSLC_FONTREF_PTR,NULL,4)) { return; }
    if (!gslc_FontSet(&m_gui,E_BUILTIN5X8,GSLC_FONTREF_PTR,NULL,1)) { return; }
  //<Load_Fonts !End!>

  //<InitGUI !Start!>
  gslc_PageAdd(&m_gui,E_PG_BASE,m_asBasePage1Elem,MAX_ELEM_PG_BASE_RAM,m_asBasePage1ElemRef,MAX_ELEM_PG_BASE);
  gslc_PageAdd(&m_gui,E_PG_BOOT,m_asPage1Elem,MAX_ELEM_PG_BOOT_RAM,m_asPage1ElemRef,MAX_ELEM_PG_BOOT);
  gslc_PageAdd(&m_gui,E_PG_HOME,m_asPage2Elem,MAX_ELEM_PG_HOME_RAM,m_asPage2ElemRef,MAX_ELEM_PG_HOME);
  gslc_PageAdd(&m_gui,E_PG_Monitor,m_asPage3Elem,MAX_ELEM_PG_Monitor_RAM,m_asPage3ElemRef,MAX_ELEM_PG_Monitor);
  gslc_PageAdd(&m_gui,E_PG_SELECT_IC,m_asPage4Elem,MAX_ELEM_PG_SELECT_IC_RAM,m_asPage4ElemRef,MAX_ELEM_PG_SELECT_IC);
  gslc_PageAdd(&m_gui,E_PG_SETTINGS_MAIN,m_asPage5Elem,MAX_ELEM_PG_SETTINGS_MAIN_RAM,m_asPage5ElemRef,MAX_ELEM_PG_SETTINGS_MAIN);
  gslc_PageAdd(&m_gui,E_PG_SETTINGS_BMU,m_asPage6Elem,MAX_ELEM_PG_SETTINGS_BMU_RAM,m_asPage6ElemRef,MAX_ELEM_PG_SETTINGS_BMU);
  gslc_PageAdd(&m_gui,E_PG_SETTINGS_BMS,m_asPage7Elem,MAX_ELEM_PG_SETTINGS_BMS_RAM,m_asPage7ElemRef,MAX_ELEM_PG_SETTINGS_BMS);
  gslc_PageAdd(&m_gui,E_PG_SETTINGS_BALANCING,m_asPage8Elem,MAX_ELEM_PG_SETTINGS_BALANCING_RAM,m_asPage8ElemRef,MAX_ELEM_PG_SETTINGS_BALANCING);
  gslc_PageAdd(&m_gui,E_POP_KEYPAD_NUM,m_asKeypadNumElem,1,m_asKeypadNumElemRef,1);  // KeyPad

  // Now mark E_PG_BASE as a "base" page which means that it's elements
  // are always visible. This is useful for common page elements.
  gslc_SetPageBase(&m_gui, E_PG_BASE);


  // NOTE: The current page defaults to the first page added. Here we explicitly
  //       ensure that the main page is the correct page no matter the add order.
  gslc_SetPageCur(&m_gui,E_PG_BOOT);
  
  // Set Background to a flat color
  gslc_SetBkgndColor(&m_gui,GSLC_COL_BLACK);

  // -----------------------------------
  // PAGE: E_PG_BASE
  

  // Create E_DRAW_LINE1 line 
  pElemRef = gslc_ElemCreateLine(&m_gui,E_DRAW_LINE1,E_PG_BASE,0,21,320,21);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_GRAY_LT2,GSLC_COL_GRAY_LT2);
  
  // create E_ELEM_BTN_HOME button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN_HOME,E_PG_BASE,
    (gslc_tsRect){0,0,30,20},(char*)"\x15",0,E_AWESOMEF000_12,&CbBtnCommon);
  
  // create E_ELEM_BTN_SETTINGS button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN_SETTINGS,E_PG_BASE,
    (gslc_tsRect){290,0,30,20},(char*)"\x13",0,E_AWESOMEF000_12,&CbBtnCommon);
  
  // create E_ELEM_BTN_MONITOR button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN_MONITOR,E_PG_BASE,
    (gslc_tsRect){260,0,30,20},(char*)"n",0,E_AWESOMEF000_12,&CbBtnCommon);

  // Create E_DRAW_LINE2 line 
  pElemRef = gslc_ElemCreateLine(&m_gui,E_DRAW_LINE2,E_PG_BASE,0,219,320,219);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_GRAY_LT2,GSLC_COL_GRAY_LT2);
  
  // Create E_ELEM_TEXT54 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT54,E_PG_BASE,(gslc_tsRect){0,220,49,10},
    (char*)"UP-Time:",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_GRAY);
  
  // Create E_ELEM_TEXT55 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT55,E_PG_BASE,(gslc_tsRect){0,230,49,10},
    (char*)"Message:",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_GRAY);
  
  // Create E_ELEM_TEXT56 runtime modifiable text
  static char m_sDisplayText56[11] = "000000000s";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT56,E_PG_BASE,(gslc_tsRect){55,220,61,10},
    (char*)m_sDisplayText56,11,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_GRAY);
  ptr_home_uptime = pElemRef;
  
  // create E_ELEM_BTN_BALANCING button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN_BALANCING,E_PG_BASE,
    (gslc_tsRect){230,0,30,20},(char*)"m",0,E_AWESOMEF000_8,&CbBtnCommon);

  // -----------------------------------
  // PAGE: E_PG_BOOT
  
  
  // Create E_ELEM_TEXT2 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT2,E_PG_BOOT,(gslc_tsRect){10,170,241,34},
    (char*)"Booting...",0,E_BUILTIN20X32);

  // -----------------------------------
  // PAGE: E_PG_HOME
  
  
  // Create E_ELEM_TEXT3 runtime modifiable text
  static char m_sDisplayText3[6] = "00.00";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT3,E_PG_HOME,(gslc_tsRect){90,45,121,34},
    (char*)m_sDisplayText3,6,E_BUILTIN20X32);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_WHITE);
  ptr_home_TotalVoltage = pElemRef;
  
  // Create E_ELEM_TEXT4 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT4,E_PG_HOME,(gslc_tsRect){225,55,13,18},
    (char*)"V",0,E_BUILTIN10X16);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_WHITE);
  gslc_ElemSetTxtEnc(&m_gui,pElemRef,GSLC_TXT_ENC_UTF8);
  
  // Create E_ELEM_TEXT5 runtime modifiable text
  static char m_sDisplayText5[6] = "00.00";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT5,E_PG_HOME,(gslc_tsRect){90,90,121,34},
    (char*)m_sDisplayText5,6,E_BUILTIN20X32);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_WHITE);
  ptr_home_Current = pElemRef;
  
  // Create E_ELEM_TEXT6 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT6,E_PG_HOME,(gslc_tsRect){225,105,13,18},
    (char*)"A",0,E_BUILTIN10X16);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_WHITE);
  
  // Create radio button E_ELEM_RADIO1
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_RADIO1,E_PG_HOME,&m_asXRadio1,
    (gslc_tsRect){295,165,20,20},true,GSLCX_CHECKBOX_STYLE_ROUND,GSLC_COL_ORANGE,false);
  gslc_ElemSetGroup(&m_gui,pElemRef,E_GROUP1);
  
  // Create radio button E_ELEM_RADIO2
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_RADIO2,E_PG_HOME,&m_asXRadio2,
    (gslc_tsRect){295,190,20,20},true,GSLCX_CHECKBOX_STYLE_ROUND,GSLC_COL_ORANGE,false);
  gslc_ElemSetGroup(&m_gui,pElemRef,E_GROUP1);
  
  // Create radio button E_ELEM_RADIO3
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_RADIO3,E_PG_HOME,&m_asXRadio3,
    (gslc_tsRect){295,140,20,20},true,GSLCX_CHECKBOX_STYLE_ROUND,GSLC_COL_ORANGE,false);
  gslc_ElemSetGroup(&m_gui,pElemRef,E_GROUP1);
   
  // Create E_ELEM_BOX1 box
  pElemRef = gslc_ElemCreateBox(&m_gui,E_ELEM_BOX1,E_PG_HOME,(gslc_tsRect){15,53,50,130});

  // Create progress bar E_ELEM_PROGRESS28 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS28,E_PG_HOME,&m_sXBarGauge28,
    (gslc_tsRect){20,58,40,105},0,100,0,GSLC_COL_GREEN,true);
  m_pElemProgress_home_StateOfCharge = pElemRef;
   
  // Create E_ELEM_BOX2 box
  pElemRef = gslc_ElemCreateBox(&m_gui,E_ELEM_BOX2,E_PG_HOME,(gslc_tsRect){25,43,30,10});
  
  // Create E_ELEM_TEXT97 runtime modifiable text
  static char m_sDisplayText97[7] = "000 %";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT97,E_PG_HOME,(gslc_tsRect){25,168,37,10},
    (char*)m_sDisplayText97,7,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  ptr_home_StateOfChargeText = pElemRef;
  
  // Create E_ELEM_TEXT98 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT98,E_PG_HOME,(gslc_tsRect){155,145,133,10},
    (char*)"Balance while Charging",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_ORANGE);
  
  // Create E_ELEM_TEXT99 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT99,E_PG_HOME,(gslc_tsRect){135,170,151,10},
    (char*)"Balance While Discharging",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_ORANGE);
  
  // Create E_ELEM_TEXT100 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT100,E_PG_HOME,(gslc_tsRect){175,195,109,10},
    (char*)"Balance while Idle",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_ORANGE);

  // -----------------------------------
  // PAGE: E_PG_Monitor
  
  
  // create E_ELEM_BTN_ICSELECT button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN_ICSELECT,E_PG_Monitor,
    (gslc_tsRect){80,0,30,20},(char*)">1",0,E_BUILTIN5X8,&CbBtnCommon);
  
  // create E_ELEM_BTN5 button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN5,E_PG_Monitor,
    (gslc_tsRect){30,0,50,20},(char*)">Monitor",0,E_BUILTIN5X8,&CbBtnCommon);
  
  // Create E_ELEM_TEXT11 runtime modifiable text
  static char m_sDisplayText11[9] = "00.0000V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT11,E_PG_Monitor,(gslc_tsRect){40,165,49,10},
    (char*)m_sDisplayText11,9,E_BUILTIN5X8);
  gslc_ElemSetFillEn(&m_gui,pElemRef,false);
  ptr_monitor_cell1 = pElemRef;
  
  // Create E_ELEM_TEXT12 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT12,E_PG_Monitor,(gslc_tsRect){10,165,19,10},
    (char*)"[1]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT13 runtime modifiable text
  static char m_sDisplayText13[9] = "00.0000V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT13,E_PG_Monitor,(gslc_tsRect){40,85,49,10},
    (char*)m_sDisplayText13,9,E_BUILTIN5X8);
  ptr_monitor_cell5 = pElemRef;
  
  // Create E_ELEM_TEXT14 runtime modifiable text
  static char m_sDisplayText14[9] = "00.0000V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT14,E_PG_Monitor,(gslc_tsRect){40,145,49,10},
    (char*)m_sDisplayText14,9,E_BUILTIN5X8);
  ptr_monitor_cell2 = pElemRef;
  
  // Create E_ELEM_TEXT15 runtime modifiable text
  static char m_sDisplayText15[9] = "00.0000V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT15,E_PG_Monitor,(gslc_tsRect){40,65,49,10},
    (char*)m_sDisplayText15,9,E_BUILTIN5X8);
  ptr_monitor_cell6 = pElemRef;
  
  // Create E_ELEM_TEXT16 runtime modifiable text
  static char m_sDisplayText16[9] = "00.0000V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT16,E_PG_Monitor,(gslc_tsRect){40,125,49,10},
    (char*)m_sDisplayText16,9,E_BUILTIN5X8);
  ptr_monitor_cell3 = pElemRef;
  
  // Create E_ELEM_TEXT17 runtime modifiable text
  static char m_sDisplayText17[9] = "00.0000V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT17,E_PG_Monitor,(gslc_tsRect){40,45,49,10},
    (char*)m_sDisplayText17,9,E_BUILTIN5X8);
  ptr_monitor_cell7 = pElemRef;
  
  // Create E_ELEM_TEXT18 runtime modifiable text
  static char m_sDisplayText18[9] = "00.0000V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT18,E_PG_Monitor,(gslc_tsRect){40,105,49,10},
    (char*)m_sDisplayText18,9,E_BUILTIN5X8);
  ptr_monitor_cell4 = pElemRef;
  
  // Create E_ELEM_TEXT20 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT20,E_PG_Monitor,(gslc_tsRect){10,145,19,10},
    (char*)"[2]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT21 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT21,E_PG_Monitor,(gslc_tsRect){235,165,19,10},
    (char*)"[1]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT22 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT22,E_PG_Monitor,(gslc_tsRect){235,150,19,10},
    (char*)"[2]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT23 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT23,E_PG_Monitor,(gslc_tsRect){235,135,19,10},
    (char*)"[3]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT24 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT24,E_PG_Monitor,(gslc_tsRect){235,120,19,10},
    (char*)"[4]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT25 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT25,E_PG_Monitor,(gslc_tsRect){235,105,19,10},
    (char*)"[5]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT26 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT26,E_PG_Monitor,(gslc_tsRect){235,90,19,10},
    (char*)"[6]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT27 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT27,E_PG_Monitor,(gslc_tsRect){235,75,19,10},
    (char*)"[7]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT28 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT28,E_PG_Monitor,(gslc_tsRect){10,125,19,10},
    (char*)"[3]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT29 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT29,E_PG_Monitor,(gslc_tsRect){10,105,19,10},
    (char*)"[4]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT30 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT30,E_PG_Monitor,(gslc_tsRect){10,85,19,10},
    (char*)"[5]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT31 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT31,E_PG_Monitor,(gslc_tsRect){10,65,19,10},
    (char*)"[6]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT32 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT32,E_PG_Monitor,(gslc_tsRect){10,45,19,10},
    (char*)"[7]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT33 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT33,E_PG_Monitor,(gslc_tsRect){45,30,43,10},
    (char*)"Voltage",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT34 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT34,E_PG_Monitor,(gslc_tsRect){235,60,19,10},
    (char*)"[8]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);

  // Create progress bar E_ELEM_PROGRESS1 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS1,E_PG_Monitor,&m_sXBarGauge1,
    (gslc_tsRect){105,165,25,8},100,0,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL1N = pElemRef;
  
  // Create E_ELEM_TEXT35 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT35,E_PG_Monitor,(gslc_tsRect){120,30,25,10},
    (char*)"Diff",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT36 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT36,E_PG_Monitor,(gslc_tsRect){180,30,25,10},
    (char*)"Dis.",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);

  // Create E_DRAW_LINE3 line 
  pElemRef = gslc_ElemCreateLine(&m_gui,E_DRAW_LINE3,E_PG_Monitor,220,21,220,219);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_GRAY_LT2,GSLC_COL_GRAY_LT2);
  
  // Create E_ELEM_TEXT37 runtime modifiable text
  static char m_sDisplayText37[7] = "00.0\xf8C";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT37,E_PG_Monitor,(gslc_tsRect){275,165,37,10},
    (char*)m_sDisplayText37,7,E_BUILTIN5X8);
  ptr_monitor_temp1 = pElemRef;
  
  // Create E_ELEM_TEXT38 runtime modifiable text
  static char m_sDisplayText38[7] = "00.0\xf8C";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT38,E_PG_Monitor,(gslc_tsRect){275,150,37,10},
    (char*)m_sDisplayText38,7,E_BUILTIN5X8);
  ptr_monitor_temp2 = pElemRef;
  
  // Create E_ELEM_TEXT39 runtime modifiable text
  static char m_sDisplayText39[7] = "00.0\xf8C";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT39,E_PG_Monitor,(gslc_tsRect){275,135,37,10},
    (char*)m_sDisplayText39,7,E_BUILTIN5X8);
  ptr_monitor_temp3 = pElemRef;
  
  // Create E_ELEM_TEXT40 runtime modifiable text
  static char m_sDisplayText40[7] = "00.0\xf8C";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT40,E_PG_Monitor,(gslc_tsRect){275,120,37,10},
    (char*)m_sDisplayText40,7,E_BUILTIN5X8);
  ptr_monitor_temp4 = pElemRef;
  
  // Create E_ELEM_TEXT41 runtime modifiable text
  static char m_sDisplayText41[7] = "00.0\xf8C";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT41,E_PG_Monitor,(gslc_tsRect){275,105,37,10},
    (char*)m_sDisplayText41,7,E_BUILTIN5X8);
  ptr_monitor_temp5 = pElemRef;
  
  // Create E_ELEM_TEXT42 runtime modifiable text
  static char m_sDisplayText42[7] = "00.0\xf8C";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT42,E_PG_Monitor,(gslc_tsRect){275,90,37,10},
    (char*)m_sDisplayText42,7,E_BUILTIN5X8);
  ptr_monitor_temp6 = pElemRef;
  
  // Create E_ELEM_TEXT43 runtime modifiable text
  static char m_sDisplayText43[7] = "00.0\xf8C";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT43,E_PG_Monitor,(gslc_tsRect){275,75,37,10},
    (char*)m_sDisplayText43,7,E_BUILTIN5X8);
  ptr_monitor_temp7 = pElemRef;
  
  // Create E_ELEM_TEXT44 runtime modifiable text
  static char m_sDisplayText44[7] = "00.0\xf8C";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT44,E_PG_Monitor,(gslc_tsRect){275,60,37,10},
    (char*)m_sDisplayText44,7,E_BUILTIN5X8);
  ptr_monitor_temp8 = pElemRef;
  
  // Create E_ELEM_TEXT45 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT45,E_PG_Monitor,(gslc_tsRect){235,30,67,10},
    (char*)"Temperature",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT46 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT46,E_PG_Monitor,(gslc_tsRect){235,45,25,10},
    (char*)"[ic]",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);
  
  // Create E_ELEM_TEXT47 runtime modifiable text
  static char m_sDisplayText47[7] = "00.0\xf8C";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT47,E_PG_Monitor,(gslc_tsRect){275,45,37,10},
    (char*)m_sDisplayText47,7,E_BUILTIN5X8);
  ptr_monitor_tempIC = pElemRef;
  
  // Create E_ELEM_TEXT72 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT72,E_PG_Monitor,(gslc_tsRect){200,45,10,11},
    (char*)"X",0,E_AWESOMEF000_8);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,true);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_ORANGE);
  
  // Create E_ELEM_TEXT86 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT86,E_PG_Monitor,(gslc_tsRect){10,30,25,10},
    (char*)"Cell",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_TEAL);

  // Create progress bar E_ELEM_PROGRESS15 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS15,E_PG_Monitor,&m_sXBarGauge15,
    (gslc_tsRect){130,165,25,8},0,100,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL1P = pElemRef;

  // Create progress bar E_ELEM_PROGRESS16 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS16,E_PG_Monitor,&m_sXBarGauge16,
    (gslc_tsRect){105,145,25,8},100,0,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL2N = pElemRef;

  // Create progress bar E_ELEM_PROGRESS17 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS17,E_PG_Monitor,&m_sXBarGauge17,
    (gslc_tsRect){130,145,25,8},0,100,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL2P = pElemRef;

  // Create progress bar E_ELEM_PROGRESS18 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS18,E_PG_Monitor,&m_sXBarGauge18,
    (gslc_tsRect){105,125,25,8},100,0,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL3N = pElemRef;

  // Create progress bar E_ELEM_PROGRESS19 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS19,E_PG_Monitor,&m_sXBarGauge19,
    (gslc_tsRect){130,125,25,8},0,100,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL3P = pElemRef;

  // Create progress bar E_ELEM_PROGRESS20 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS20,E_PG_Monitor,&m_sXBarGauge20,
    (gslc_tsRect){105,105,25,8},100,0,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL4N = pElemRef;

  // Create progress bar E_ELEM_PROGRESS21 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS21,E_PG_Monitor,&m_sXBarGauge21,
    (gslc_tsRect){130,105,25,8},0,100,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL4P = pElemRef;

  // Create progress bar E_ELEM_PROGRESS22 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS22,E_PG_Monitor,&m_sXBarGauge22,
    (gslc_tsRect){105,85,25,8},100,0,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL5N = pElemRef;

  // Create progress bar E_ELEM_PROGRESS23 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS23,E_PG_Monitor,&m_sXBarGauge23,
    (gslc_tsRect){130,85,25,8},0,100,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL5P = pElemRef;

  // Create progress bar E_ELEM_PROGRESS24 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS24,E_PG_Monitor,&m_sXBarGauge24,
    (gslc_tsRect){105,65,25,8},100,0,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL6N = pElemRef;

  // Create progress bar E_ELEM_PROGRESS25 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS25,E_PG_Monitor,&m_sXBarGauge25,
    (gslc_tsRect){130,65,25,8},0,100,0,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL6P = pElemRef;

  // Create progress bar E_ELEM_PROGRESS26 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS26,E_PG_Monitor,&m_sXBarGauge26,
    (gslc_tsRect){105,45,25,8},100,0,50,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL7N = pElemRef;

  // Create progress bar E_ELEM_PROGRESS27 
  pElemRef = gslc_ElemXProgressCreate(&m_gui,E_ELEM_PROGRESS27,E_PG_Monitor,&m_sXBarGauge27,
    (gslc_tsRect){130,45,25,8},0,100,50,GSLC_COL_RED,false);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_BLACK,GSLC_COL_BLACK);
  m_pElemProgressCELL7P = pElemRef;

  // Create E_DRAW_LINE4 line 
  pElemRef = gslc_ElemCreateLine(&m_gui,E_DRAW_LINE4,E_PG_Monitor,130,45,130,175);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_GRAY_LT2,GSLC_COL_GRAY_LT2);

  // Create E_DRAW_LINE5 line 
  pElemRef = gslc_ElemCreateLine(&m_gui,E_DRAW_LINE5,E_PG_Monitor,105,45,105,175);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_GRAY_LT2,GSLC_COL_GRAY_LT2);

  // Create E_DRAW_LINE6 line 
  pElemRef = gslc_ElemCreateLine(&m_gui,E_DRAW_LINE6,E_PG_Monitor,155,45,155,175);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLACK,GSLC_COL_GRAY_LT2,GSLC_COL_GRAY_LT2);
  
  // Create E_ELEM_TEXT87 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT87,E_PG_Monitor,(gslc_tsRect){127,175,7,10},
    (char*)"0",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_GRAY);
  
  // Create E_ELEM_TEXT88 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT88,E_PG_Monitor,(gslc_tsRect){155,175,19,10},
    (char*)"-1V",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_GRAY);
  
  // Create E_ELEM_TEXT89 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT89,E_PG_Monitor,(gslc_tsRect){90,175,19,10},
    (char*)"+1V",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_GRAY);
  
  // Create E_ELEM_TEXT90 runtime modifiable text
  static char m_sDisplayText90[7] = "00.00V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT90,E_PG_Monitor,(gslc_tsRect){160,45,37,10},
    (char*)m_sDisplayText90,7,E_BUILTIN5X8);
  ptr_monitor_cell7_SVoltage = pElemRef;
  
  // Create E_ELEM_TEXT91 runtime modifiable text
  static char m_sDisplayText91[7] = "00.00V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT91,E_PG_Monitor,(gslc_tsRect){160,65,37,10},
    (char*)m_sDisplayText91,7,E_BUILTIN5X8);
  ptr_monitor_cell6_SVoltage = pElemRef;
  
  // Create E_ELEM_TEXT92 runtime modifiable text
  static char m_sDisplayText92[7] = "00.00V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT92,E_PG_Monitor,(gslc_tsRect){160,85,37,10},
    (char*)m_sDisplayText92,7,E_BUILTIN5X8);
  ptr_monitor_cell5_SVoltage = pElemRef;
  
  // Create E_ELEM_TEXT93 runtime modifiable text
  static char m_sDisplayText93[7] = "00.00V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT93,E_PG_Monitor,(gslc_tsRect){160,105,37,10},
    (char*)m_sDisplayText93,7,E_BUILTIN5X8);
  ptr_monitor_cell4_SVoltage = pElemRef;
  
  // Create E_ELEM_TEXT94 runtime modifiable text
  static char m_sDisplayText94[7] = "00.00V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT94,E_PG_Monitor,(gslc_tsRect){160,125,37,10},
    (char*)m_sDisplayText94,7,E_BUILTIN5X8);
  ptr_monitor_cell3_SVoltage = pElemRef;
  
  // Create E_ELEM_TEXT95 runtime modifiable text
  static char m_sDisplayText95[7] = "00.00V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT95,E_PG_Monitor,(gslc_tsRect){160,145,37,10},
    (char*)m_sDisplayText95,7,E_BUILTIN5X8);
  ptr_monitor_cell2_SVoltage = pElemRef;
  
  // Create E_ELEM_TEXT96 runtime modifiable text
  static char m_sDisplayText96[7] = "00.00V";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT96,E_PG_Monitor,(gslc_tsRect){160,165,37,10},
    (char*)m_sDisplayText96,7,E_BUILTIN5X8);
  ptr_monitor_cell1_SVoltage = pElemRef;

  // -----------------------------------
  // PAGE: E_PG_SELECT_IC
  
   
  // Create wrapping box for listbox E_ELEM_LISTBOX1 and scrollbar
  pElemRef = gslc_ElemCreateBox(&m_gui,GSLC_ID_AUTO,E_PG_SELECT_IC,(gslc_tsRect){50,35,200,60});
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLUE,GSLC_COL_BLACK,GSLC_COL_BLACK);
  
  // Create listbox
  pElemRef = gslc_ElemXListboxCreate(&m_gui,E_ELEM_LISTBOX1,E_PG_SELECT_IC,&m_sListbox1,
    (gslc_tsRect){50+2,35+4,200-23,60-7},E_BUILTIN5X8,
    (uint8_t*)&m_acListboxBuf1,sizeof(m_acListboxBuf1),0);
  gslc_ElemXListboxSetSize(&m_gui, pElemRef, 5, 1); // 5 rows, 1 columns
  gslc_ElemXListboxItemsSetSize(&m_gui, pElemRef, XLISTBOX_SIZE_AUTO, XLISTBOX_SIZE_AUTO);
  gslc_ElemSetTxtMarginXY(&m_gui, pElemRef, 5, 0);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_WHITE);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLUE,GSLC_COL_BLACK,GSLC_COL_BLACK);
  gslc_ElemXListboxSetSelFunc(&m_gui, pElemRef, &CbListbox);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,true);
  m_pElemListboxSelectIC = pElemRef;

  // Create vertical scrollbar for listbox
  pElemRef = gslc_ElemXSliderCreate(&m_gui,E_LISTSCROLL1,E_PG_SELECT_IC,&m_sListScroll1,
          (gslc_tsRect){50+200-21,35+4,20,60-8},0,100,0,5,true);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLUE,GSLC_COL_BLACK,GSLC_COL_BLUE);
  gslc_ElemXSliderSetPosFunc(&m_gui,pElemRef,&CbSlidePos);
  m_pListSlider1 = pElemRef;

  // Create graph E_ELEM_GRAPH1
  pElemRef = gslc_ElemXGraphCreate(&m_gui,E_ELEM_GRAPH1,E_PG_SELECT_IC,
    &m_sGraph1,(gslc_tsRect){60,75,180,120},E_BUILTIN5X8,(int16_t*)&m_anGraphBuf1,
        10,((gslc_tsColor){255,200,0}));
  gslc_ElemXGraphSetStyle(&m_gui,pElemRef, GSLCX_GRAPH_STYLE_DOT, 5);
  m_pElemGraph1 = pElemRef;

  // -----------------------------------
  // PAGE: E_PG_SETTINGS_MAIN
  
  
  // create E_ELEM_BTN_BMUSETTINGS button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN_BMUSETTINGS,E_PG_SETTINGS_MAIN,
    (gslc_tsRect){180,40,120,40},(char*)"Edit BMU Settings",0,E_BUILTIN5X8,&CbBtnCommon);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_BLACK);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLUE_DK2,GSLC_COL_ORANGE,GSLC_COL_BLUE_DK1);
  gslc_ElemSetTxtEnc(&m_gui,pElemRef,GSLC_TXT_ENC_UTF8);
  
  // create E_ELEM_BTN7 button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN7,E_PG_SETTINGS_MAIN,
    (gslc_tsRect){25,40,120,40},(char*)"Edit BMS Settings",0,E_BUILTIN5X8,&CbBtnCommon);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_BLACK);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_BLUE_DK2,GSLC_COL_ORANGE,GSLC_COL_BLUE_DK1);
  gslc_ElemSetTxtEnc(&m_gui,pElemRef,GSLC_TXT_ENC_UTF8);
  
  // create E_ELEM_BTN8 button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN8,E_PG_SETTINGS_MAIN,
    (gslc_tsRect){30,0,80,20},(char*)">Settings",0,E_BUILTIN5X8,&CbBtnCommon);

  // -----------------------------------
  // PAGE: E_PG_SETTINGS_BMU
  
  
  // create E_ELEM_BTN11 button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN11,E_PG_SETTINGS_BMU,
    (gslc_tsRect){30,0,80,20},(char*)">Settings",0,E_BUILTIN5X8,&CbBtnCommon);
  
  // create E_ELEM_BTN12 button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN12,E_PG_SETTINGS_BMU,
    (gslc_tsRect){110,0,80,20},(char*)">BMU(chain)",0,E_BUILTIN5X8,&CbBtnCommon);
  
  // Create E_ELEM_TEXT58 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT58,E_PG_SETTINGS_BMU,(gslc_tsRect){10,135,31,10},
    (char*)"REFON",0,E_BUILTIN5X8);
  
  // create E_ELEM_BTN_SETTINGS_BMU_RESET button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN_SETTINGS_BMU_RESET,E_PG_SETTINGS_BMU,
    (gslc_tsRect){120,185,60,30},(char*)"RESET",0,E_BUILTIN5X8,&CbBtnCommon);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_BLACK);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_RED,GSLC_COL_RED,GSLC_COL_BLUE_DK1);
  gslc_ElemSetRoundEn(&m_gui, pElemRef, true);
  
  // create E_ELEM_BTN_SETTINGS_BMU_WRITE button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN_SETTINGS_BMU_WRITE,E_PG_SETTINGS_BMU,
    (gslc_tsRect){185,185,60,30},(char*)"WRITE",0,E_BUILTIN5X8,&CbBtnCommon);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_BLACK);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_ORANGE,GSLC_COL_ORANGE,GSLC_COL_BLUE_DK1);
  gslc_ElemSetRoundEn(&m_gui, pElemRef, true);
  
  // create E_ELEM_BTN_SETTINGS_BMU_READ button with text label
  pElemRef = gslc_ElemCreateBtnTxt(&m_gui,E_ELEM_BTN_SETTINGS_BMU_READ,E_PG_SETTINGS_BMU,
    (gslc_tsRect){250,185,60,30},(char*)"READ",0,E_BUILTIN5X8,&CbBtnCommon);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_BLACK);
  gslc_ElemSetCol(&m_gui,pElemRef,GSLC_COL_GREEN,GSLC_COL_GREEN,GSLC_COL_BLUE_DK1);
  gslc_ElemSetRoundEn(&m_gui, pElemRef, true);
  
  // Create E_ELEM_TEXT59 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT59,E_PG_SETTINGS_BMU,(gslc_tsRect){90,135,37,10},
    (char*)"ADCOPT",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT60 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT60,E_PG_SETTINGS_BMU,(gslc_tsRect){10,45,61,10},
    (char*)"GPIO BITS:",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT61 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT61,E_PG_SETTINGS_BMU,(gslc_tsRect){10,85,55,10},
    (char*)"DCC BITS:",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT63 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT63,E_PG_SETTINGS_BMU,(gslc_tsRect){90,160,25,10},
    (char*)"MCAL",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT64 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT64,E_PG_SETTINGS_BMU,(gslc_tsRect){10,160,25,10},
    (char*)"DTEN",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT65 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT65,E_PG_SETTINGS_BMU,(gslc_tsRect){240,135,43,10},
    (char*)"DIS_RED",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT66 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT66,E_PG_SETTINGS_BMU,(gslc_tsRect){170,160,25,10},
    (char*)"FDRF",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT67 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT67,E_PG_SETTINGS_BMU,(gslc_tsRect){170,135,31,10},
    (char*)"SCONV",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT68 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT68,E_PG_SETTINGS_BMU,(gslc_tsRect){245,160,31,10},
    (char*)"DTMEN",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT69 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT69,E_PG_SETTINGS_BMU,(gslc_tsRect){160,65,79,10},
    (char*)"Under Voltage",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT70 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT70,E_PG_SETTINGS_BMU,(gslc_tsRect){160,50,73,10},
    (char*)"Over Voltage",0,E_BUILTIN5X8);
   
  // create checkbox E_ELEM_CHECK_GPIO4
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK_GPIO4,E_PG_SETTINGS_BMU,&m_asXCheck9,
    (gslc_tsRect){70,60,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_GPIO4 = pElemRef;
   
  // create checkbox E_ELEM_CHECK_GPIO1
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK_GPIO1,E_PG_SETTINGS_BMU,&m_asXCheck10,
    (gslc_tsRect){10,60,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_GPIO1 = pElemRef;
   
  // create checkbox E_ELEM_CHECK_GPIO2
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK_GPIO2,E_PG_SETTINGS_BMU,&m_asXCheck11,
    (gslc_tsRect){30,60,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_GPIO2 = pElemRef;
   
  // create checkbox E_ELEM_CHECK_GPIO3
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK_GPIO3,E_PG_SETTINGS_BMU,&m_asXCheck12,
    (gslc_tsRect){50,60,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_GPIO3 = pElemRef;
  
  // Create E_ELEM_NUMINPUT1 numeric input field
  static char m_sInputNumber1[7] = "";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_NUMINPUT1,E_PG_SETTINGS_BMU,(gslc_tsRect){250,50,50,10},
    (char*)m_sInputNumber1,7,E_BUILTIN5X8);
  gslc_ElemSetTxtMargin(&m_gui,pElemRef,5);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,true);
  gslc_ElemSetClickEn(&m_gui, pElemRef, true);
  gslc_ElemSetTouchFunc(&m_gui, pElemRef, &CbBtnCommon);
  m_pElemVal_overVoltage = pElemRef;
  
  // Create E_ELEM_NUMINPUT2 numeric input field
  static char m_sInputNumber2[7] = "";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_NUMINPUT2,E_PG_SETTINGS_BMU,(gslc_tsRect){250,65,50,10},
    (char*)m_sInputNumber2,7,E_BUILTIN5X8);
  gslc_ElemSetTxtMargin(&m_gui,pElemRef,5);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,true);
  gslc_ElemSetClickEn(&m_gui, pElemRef, true);
  gslc_ElemSetTouchFunc(&m_gui, pElemRef, &CbBtnCommon);
  m_pElemVal_underVoltage = pElemRef;
   
  // create checkbox E_ELEM_CHECK13
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK13,E_PG_SETTINGS_BMU,&m_asXCheck13,
    (gslc_tsRect){10,100,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_DCC0 = pElemRef;
   
  // create checkbox E_ELEM_CHECK14
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK14,E_PG_SETTINGS_BMU,&m_asXCheck14,
    (gslc_tsRect){30,100,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_DCC1 = pElemRef;
   
  // create checkbox E_ELEM_CHECK15
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK15,E_PG_SETTINGS_BMU,&m_asXCheck15,
    (gslc_tsRect){50,100,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_DCC2 = pElemRef;
   
  // create checkbox E_ELEM_CHECK16
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK16,E_PG_SETTINGS_BMU,&m_asXCheck16,
    (gslc_tsRect){70,100,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_DCC3 = pElemRef;
   
  // create checkbox E_ELEM_CHECK17
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK17,E_PG_SETTINGS_BMU,&m_asXCheck17,
    (gslc_tsRect){90,100,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_DCC4 = pElemRef;
   
  // create checkbox E_ELEM_CHECK18
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK18,E_PG_SETTINGS_BMU,&m_asXCheck18,
    (gslc_tsRect){110,100,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_DCC5 = pElemRef;
   
  // create checkbox E_ELEM_CHECK19
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK19,E_PG_SETTINGS_BMU,&m_asXCheck19,
    (gslc_tsRect){50,130,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_REFON = pElemRef;
   
  // create checkbox E_ELEM_CHECK20
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK20,E_PG_SETTINGS_BMU,&m_asXCheck20,
    (gslc_tsRect){50,155,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_DTEN = pElemRef;
   
  // create checkbox E_ELEM_CHECK21
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK21,E_PG_SETTINGS_BMU,&m_asXCheck21,
    (gslc_tsRect){135,130,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_ADCOPT = pElemRef;
   
  // create checkbox E_ELEM_CHECK22
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK22,E_PG_SETTINGS_BMU,&m_asXCheck22,
    (gslc_tsRect){135,155,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_MCAL = pElemRef;
   
  // create checkbox E_ELEM_CHECK23
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK23,E_PG_SETTINGS_BMU,&m_asXCheck23,
    (gslc_tsRect){205,130,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_SCONV = pElemRef;
   
  // create checkbox E_ELEM_CHECK24
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK24,E_PG_SETTINGS_BMU,&m_asXCheck24,
    (gslc_tsRect){205,155,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_FDRF = pElemRef;
   
  // create checkbox E_ELEM_CHECK25
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK25,E_PG_SETTINGS_BMU,&m_asXCheck25,
    (gslc_tsRect){290,130,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_DIS_RED = pElemRef;
   
  // create checkbox E_ELEM_CHECK26
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK26,E_PG_SETTINGS_BMU,&m_asXCheck26,
    (gslc_tsRect){290,155,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_DTMEN = pElemRef;
  
  // Create E_ELEM_TEXT71 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT71,E_PG_SETTINGS_BMU,(gslc_tsRect){0,25,223,10},
    (char*)"!Settings will be written to all BMUs",0,E_BUILTIN5X8);
  gslc_ElemSetTxtCol(&m_gui,pElemRef,GSLC_COL_CYAN);
  
  // Create E_ELEM_TEXT73 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT73,E_PG_SETTINGS_BMU,(gslc_tsRect){20,70,7,10},
    (char*)"1",0,E_BUILTIN5X8);
  gslc_ElemSetFillEn(&m_gui,pElemRef,false);
  
  // Create E_ELEM_TEXT74 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT74,E_PG_SETTINGS_BMU,(gslc_tsRect){40,70,7,10},
    (char*)"2",0,E_BUILTIN5X8);
  gslc_ElemSetFillEn(&m_gui,pElemRef,false);
  
  // Create E_ELEM_TEXT75 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT75,E_PG_SETTINGS_BMU,(gslc_tsRect){60,70,7,10},
    (char*)"3",0,E_BUILTIN5X8);
  gslc_ElemSetFillEn(&m_gui,pElemRef,false);
  
  // Create E_ELEM_TEXT76 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT76,E_PG_SETTINGS_BMU,(gslc_tsRect){80,70,7,10},
    (char*)"4",0,E_BUILTIN5X8);
  gslc_ElemSetFillEn(&m_gui,pElemRef,false);
  
  // Create E_ELEM_TEXT77 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT77,E_PG_SETTINGS_BMU,(gslc_tsRect){40,110,7,10},
    (char*)"1",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT78 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT78,E_PG_SETTINGS_BMU,(gslc_tsRect){20,110,7,10},
    (char*)"0",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT79 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT79,E_PG_SETTINGS_BMU,(gslc_tsRect){60,110,7,10},
    (char*)"2",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT80 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT80,E_PG_SETTINGS_BMU,(gslc_tsRect){80,110,7,10},
    (char*)"3",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT81 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT81,E_PG_SETTINGS_BMU,(gslc_tsRect){100,110,7,10},
    (char*)"4",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT82 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT82,E_PG_SETTINGS_BMU,(gslc_tsRect){120,110,7,10},
    (char*)"5",0,E_BUILTIN5X8);
   
  // create checkbox E_ELEM_CHECK27
  pElemRef = gslc_ElemXCheckboxCreate(&m_gui,E_ELEM_CHECK27,E_PG_SETTINGS_BMU,&m_asXCheck27,
    (gslc_tsRect){130,100,15,15},false,GSLCX_CHECKBOX_STYLE_X,GSLC_COL_ORANGE,false);
  ptr_CheckBox_DCC6 = pElemRef;
  
  // Create E_ELEM_TEXT83 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT83,E_PG_SETTINGS_BMU,(gslc_tsRect){140,110,7,10},
    (char*)"6",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT84 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT84,E_PG_SETTINGS_BMU,(gslc_tsRect){160,80,31,10},
    (char*)"DCTO:",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT85 runtime modifiable text
  static char m_sDisplayText85[8] = "120min";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT85,E_PG_SETTINGS_BMU,(gslc_tsRect){200,80,43,10},
    (char*)m_sDisplayText85,8,E_BUILTIN5X8);
  ptr_Text_dcto = pElemRef;

  // Add Spinner element
  pElemRef = gslc_ElemXSpinnerCreate(&m_gui,E_ELEM_SPINNER1,E_PG_SETTINGS_BMU,&m_sXSpinner1,
    (gslc_tsRect){250,80,63,20},0,99,0,1,E_BUILTIN5X8,20,&CbSpinner);
  m_pElemSpinner1 = pElemRef;

  // -----------------------------------
  // PAGE: E_PG_SETTINGS_BMS
  
  
  // Create toggle button E_ELEM_TOGGLE1
  pElemRef = gslc_ElemXTogglebtnCreate(&m_gui,E_ELEM_TOGGLE1,E_PG_SETTINGS_BMS,&m_asXToggle1,
    (gslc_tsRect){25,40,35,20},GSLC_COL_GRAY,GSLC_COL_BLUE_DK1,GSLC_COL_GRAY_LT3,
    true,false,&CbBtnCommon);
  m_pElemToggle1 = pElemRef;
  
  // Create E_ELEM_TEXT101 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT101,E_PG_SETTINGS_BMS,(gslc_tsRect){75,40,73,10},
    (char*)"ContanstPing",0,E_BUILTIN5X8);

  // -----------------------------------
  // PAGE: E_PG_SETTINGS_BALANCING
  
  
  // Create toggle button E_ELEM_TOGGLE2
  pElemRef = gslc_ElemXTogglebtnCreate(&m_gui,E_ELEM_TOGGLE2,E_PG_SETTINGS_BALANCING,&m_asXToggle2,
    (gslc_tsRect){135,145,47,10},GSLC_COL_GRAY,GSLC_COL_BLUE_DK1,GSLC_COL_GRAY_LT3,
    true,false,&CbBtnCommon);
  m_pElemToggle_SettingsBalancing_EnableBalancing = pElemRef;
  
  // Create E_ELEM_TEXT102 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT102,E_PG_SETTINGS_BALANCING,(gslc_tsRect){10,145,115,10},
    (char*)"ENABLE Balancing  :",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT104 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT104,E_PG_SETTINGS_BALANCING,(gslc_tsRect){10,50,115,10},
    (char*)"Over Voltage      :",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT105 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT105,E_PG_SETTINGS_BALANCING,(gslc_tsRect){10,65,115,10},
    (char*)"Under Voltage     :",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT106 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT106,E_PG_SETTINGS_BALANCING,(gslc_tsRect){10,80,115,10},
    (char*)"Threshold Voltage :",0,E_BUILTIN5X8);
  
  // Create E_ELEM_NUMINPUT3 numeric input field
  static char m_sInputNumber3[7] = "";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_NUMINPUT3,E_PG_SETTINGS_BALANCING,(gslc_tsRect){135,50,47,10},
    (char*)m_sInputNumber3,7,E_BUILTIN5X8);
  gslc_ElemSetTxtMargin(&m_gui,pElemRef,5);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,true);
  gslc_ElemSetClickEn(&m_gui, pElemRef, true);
  gslc_ElemSetTouchFunc(&m_gui, pElemRef, &CbBtnCommon);
  m_pElemVal_SettingsBalancing_overVoltage = pElemRef;
  
  // Create E_ELEM_NUMINPUT4 numeric input field
  static char m_sInputNumber4[7] = "";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_NUMINPUT4,E_PG_SETTINGS_BALANCING,(gslc_tsRect){135,65,47,10},
    (char*)m_sInputNumber4,7,E_BUILTIN5X8);
  gslc_ElemSetTxtMargin(&m_gui,pElemRef,5);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,true);
  gslc_ElemSetClickEn(&m_gui, pElemRef, true);
  gslc_ElemSetTouchFunc(&m_gui, pElemRef, &CbBtnCommon);
  m_pElemVal_SettingsBalancing_underVoltage = pElemRef;
  
  // Create E_ELEM_NUMINPUT5 numeric input field
  static char m_sInputNumber5[7] = "";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_NUMINPUT5,E_PG_SETTINGS_BALANCING,(gslc_tsRect){135,80,47,10},
    (char*)m_sInputNumber5,7,E_BUILTIN5X8);
  gslc_ElemSetTxtMargin(&m_gui,pElemRef,5);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,true);
  gslc_ElemSetClickEn(&m_gui, pElemRef, true);
  gslc_ElemSetTouchFunc(&m_gui, pElemRef, &CbBtnCommon);
  m_pElemVal_SettingsBalancing_thresholdVoltage = pElemRef;
  
  // Create E_ELEM_TEXT107 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT107,E_PG_SETTINGS_BALANCING,(gslc_tsRect){190,50,7,10},
    (char*)"V",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT108 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT108,E_PG_SETTINGS_BALANCING,(gslc_tsRect){190,65,7,10},
    (char*)"V",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT109 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT109,E_PG_SETTINGS_BALANCING,(gslc_tsRect){190,80,7,10},
    (char*)"V",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT114 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT114,E_PG_SETTINGS_BALANCING,(gslc_tsRect){10,170,115,10},
    (char*)"Balancing STATE   :",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT115 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT115,E_PG_SETTINGS_BALANCING,(gslc_tsRect){135,170,25,10},
    (char*)"",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT116 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT116,E_PG_SETTINGS_BALANCING,(gslc_tsRect){10,100,115,10},
    (char*)"Over Temperature  :",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT117 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT117,E_PG_SETTINGS_BALANCING,(gslc_tsRect){10,115,115,10},
    (char*)"Under Temperature :",0,E_BUILTIN5X8);
  
  // Create E_ELEM_NUMINPUT6 numeric input field
  static char m_sInputNumber6[7] = "";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_NUMINPUT6,E_PG_SETTINGS_BALANCING,(gslc_tsRect){135,100,47,10},
    (char*)m_sInputNumber6,7,E_BUILTIN5X8);
  gslc_ElemSetTxtMargin(&m_gui,pElemRef,5);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,true);
  gslc_ElemSetClickEn(&m_gui, pElemRef, true);
  gslc_ElemSetTouchFunc(&m_gui, pElemRef, &CbBtnCommon);
  m_pElemVal_SettingsBalancing_overTemperature = pElemRef;
  
  // Create E_ELEM_NUMINPUT7 numeric input field
  static char m_sInputNumber7[7] = "";
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_NUMINPUT7,E_PG_SETTINGS_BALANCING,(gslc_tsRect){135,115,47,10},
    (char*)m_sInputNumber7,7,E_BUILTIN5X8);
  gslc_ElemSetTxtMargin(&m_gui,pElemRef,5);
  gslc_ElemSetFrameEn(&m_gui,pElemRef,true);
  gslc_ElemSetClickEn(&m_gui, pElemRef, true);
  gslc_ElemSetTouchFunc(&m_gui, pElemRef, &CbBtnCommon);
  m_pElemVal_SettingsBalancing_underTemperature = pElemRef;
  
  // Create E_ELEM_TEXT119 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT119,E_PG_SETTINGS_BALANCING,(gslc_tsRect){190,100,13,10},
    (char*)"\xf8C",0,E_BUILTIN5X8);
  
  // Create E_ELEM_TEXT121 text label
  pElemRef = gslc_ElemCreateTxt(&m_gui,E_ELEM_TEXT121,E_PG_SETTINGS_BALANCING,(gslc_tsRect){190,115,13,10},
    (char*)"\xf8C",0,E_BUILTIN5X8);

  // -----------------------------------
  // PAGE: E_POP_KEYPAD_NUM
  
  static gslc_tsXKeyPadCfg_Num sCfg;
  sCfg = gslc_ElemXKeyPadCfgInit_Num();
  gslc_ElemXKeyPadCfgSetFloatEn_Num(&sCfg, true);
  gslc_ElemXKeyPadCfgSetSignEn_Num(&sCfg, true);
  m_pElemKeyPadNum = gslc_ElemXKeyPadCreate_Num(&m_gui, E_ELEM_KEYPAD_NUM, E_POP_KEYPAD_NUM,
    &m_sKeyPadNum, 65, 80, E_BUILTIN5X8, &sCfg);
  gslc_ElemXKeyPadValSetCb(&m_gui, m_pElemKeyPadNum, &CbKeypad);
  //<InitGUI !End!>

  //<Startup !Start!>
  //<Startup !End!>
}

#endif // end _GUISLICE_GEN_H
