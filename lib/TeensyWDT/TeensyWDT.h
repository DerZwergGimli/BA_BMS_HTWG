#include "Arduino.h"

#if !defined(RCM_SRS0_WAKEUP)
#define RCM_SRS0_WAKEUP 0x01
#define RCM_SRS0_LVD 0x02
#define RCM_SRS0_LOC 0x04
#define RCM_SRS0_LOL 0x08
#define RCM_SRS0_WDOG 0x20
#define RCM_SRS0_PIN 0x40
#define RCM_SRS0_POR 0x80

#define RCM_SRS1_LOCKUP 0x02
#define RCM_SRS1_SW 0x04
#define RCM_SRS1_MDM_AP 0x08
#define RCM_SRS1_SACKERR 0x20
#endif

volatile uint32_t *VBAT_REGS = (uint32_t *)0x4003e000;

uint32_t kickCounter = 0;
uint32_t dogTimer = 0;

void kickDog()
{
    Serial.printf("Kicking the dog %lu\r\n", kickCounter);
    noInterrupts();
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;
    interrupts();
    kickCounter += 1;
}

void runWDT()
{
    if (millis() - dogTimer > 1000)
    {
        dogTimer = millis();
        kickDog();
    }
}

void printResetType()
{
    if (RCM_SRS1 & RCM_SRS1_SACKERR)
        Serial.println("[RCM_SRS1] - Stop Mode Acknowledge Error Reset");
    if (RCM_SRS1 & RCM_SRS1_MDM_AP)
        Serial.println("[RCM_SRS1] - MDM-AP Reset");
    if (RCM_SRS1 & RCM_SRS1_SW)
        Serial.println("[RCM_SRS1] - Software Reset");
    if (RCM_SRS1 & RCM_SRS1_LOCKUP)
        Serial.println("[RCM_SRS1] - Core Lockup Event Reset");
    if (RCM_SRS0 & RCM_SRS0_POR)
        Serial.println("[RCM_SRS0] - Power-on Reset");
    if (RCM_SRS0 & RCM_SRS0_PIN)
        Serial.println("[RCM_SRS0] - External Pin Reset");
    if (RCM_SRS0 & RCM_SRS0_WDOG)
        Serial.println("[RCM_SRS0] - Watchdog(COP) Reset");
    if (RCM_SRS0 & RCM_SRS0_LOC)
        Serial.println("[RCM_SRS0] - Loss of External Clock Reset");
    if (RCM_SRS0 & RCM_SRS0_LOL)
        Serial.println("[RCM_SRS0] - Loss of Lock in PLL Reset");
    if (RCM_SRS0 & RCM_SRS0_LVD)
        Serial.println("[RCM_SRS0] - Low-voltage Detect Reset");
}

void watchdog_isr()
{
    VBAT_REGS[0] = kickCounter;
}

#ifdef __cplusplus
extern "C"
{
#endif
    void startup_early_hook()
    {
        uint16_t toval = 1000;
        WDOG_TOVALL = 1000; //Abt a minute
        WDOG_TOVALH = 0;
        WDOG_STCTRLH = (WDOG_STCTRLH_ALLOWUPDATE | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_IRQRSTEN); // Enable WDG
        NVIC_ENABLE_IRQ(IRQ_WDOG);
        //WDOG_PRESC = 0; // prescaler
    }
#ifdef __cplusplus
}
#endif
