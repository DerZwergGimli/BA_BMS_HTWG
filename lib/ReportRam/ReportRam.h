#ifndef REPORTRAM_H
#define REPORTRAM_H
#include "ArduinoLog.h"
#include "RamMonitor.h"
RamMonitor ram;

void report_ram_stat(const char *aname, uint32_t avalue)
{
    Log.verbose("%s: %i Kb (%i%%)" CR, aname, ((avalue + 512) / 1024), (int)((float)(avalue * 100 / ram.total())));
};

void report_ram()
{
    bool lowmem;
    bool crash;
    Serial.println();
    Log.verbose(F("==== memory report ====" CR));
    report_ram_stat("|\tfree", ram.adj_free());
    report_ram_stat("|\tstack", ram.stack_total());
    report_ram_stat("|\theap", ram.heap_total());

    lowmem = ram.warning_lowmem();
    crash = ram.warning_crash();
    if (lowmem || crash)
    {
        if (crash)
            Log.warning(F("**warning: stack and heap crash possible" CR));

        else if (lowmem)
            Log.warning(F("**warning: unallocated memory running low" CR));
    };
    Log.verbose(F("==== memory report ====" CR));
    Serial.println();
};

#endif