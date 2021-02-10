#ifndef THERMISTOR_H
#define THERMISTOR_H
#include <Arduino.h>

#define TEMPERATURENOMINAL 25

uint16_t nominalResistance = 10000;
uint16_t serialResistance = 9000;
uint16_t bCoefficient = 3977;

int16_t convertVoltageToTemperature10(uint16_t adcVoltage10000)
{
    float steinhart;
    float adcVoltage = (float)adcVoltage10000 / 10000;
    //Serial.printf("%.2f \n", adcVoltage);
    float resistanceNTC = (adcVoltage / (2.9985 - adcVoltage)) * serialResistance;
    //Serial.printf("%.2f\n", resistanceNTC);
    steinhart = resistanceNTC / nominalResistance;
    steinhart = log(steinhart); // ln(R/Ro)

    steinhart /= bCoefficient;                        // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart;                      // Invert
    steinhart -= 273.15;                              // convert to C

    //Serial.printf("%.2f /n", steinhart);
    //TODO make this not returning resistance
    return (int16_t)(steinhart * 10);

    //return (int16_t)resistanceNTC * 10;
}

#endif
