#ifndef HELPER_DEEPSLEEP_H
#define HELPER_DEEPSLEEP_H
#include "Snooze.h"

SnoozeUSBSerial usbSerial;
SnoozeTimer snoozeTimer;

SnoozeBlock config_snooze(usbSerial, snoozeTimer);

void snoozeSetup()
{
    snoozeTimer.setTimer(5000);
}

void snoozeDeepSleep()
{
    Snooze.deepSleep(config_snooze);
}

void afterDeppSleep()
{
    Serial.write(0x00);
}

#endif HELPER_DEEPSLEEP_H