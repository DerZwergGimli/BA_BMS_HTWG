#ifndef HELPER_GENERAL_H
#define HELPER_GENERAL_H

#include "Arduino.h"

void writeTextToDisplay()
{
    uint sizeOfChars = 43;
    char *textDisplay;
    textDisplay = new char[sizeOfChars];
    gslc_ElemSetTxtStr(&m_gui, m_pElemOutTxt_MessageBox, textDisplay);
}

#endif