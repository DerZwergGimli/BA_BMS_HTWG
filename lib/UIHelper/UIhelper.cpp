#include "UIhelper.h"
//#include "GUIslice.h"
#include "Arduino.h"

#include "elem/XProgress.h"

void uiHelper_UICellBarMaker(gslc_tsGui *pGui, gslc_tsElemRef *pElemRef_BarPOS, gslc_tsElemRef *pElemRef_BarNEG, uint averageVoltage, uint acutalVoltage)
{
    if (acutalVoltage > averageVoltage)
    {
        gslc_ElemXProgressSetVal(pGui, pElemRef_BarPOS, (uint)(acutalVoltage - averageVoltage) / 100);
        gslc_ElemXProgressSetVal(pGui, pElemRef_BarNEG, 0);
    }
    else
    {
        gslc_ElemXProgressSetVal(pGui, pElemRef_BarNEG, (uint)(averageVoltage - acutalVoltage) * 0.01);
        gslc_ElemXProgressSetVal(pGui, pElemRef_BarPOS, 0);
    }
}