#ifndef DEFINES_H
#define DEFINES_H

#ifdef DEBUG
  #define DEBUG_PRINT(x)       Serial.print(x)
  #define DEBUG_PRINTLN(x)     Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif
#if defined(ARDUINO_ARCH_SAM)
#define MAX_VOLTAGE 3300
#define WireMaster Wire1
#define SETRES(x) analogReadResolution(x)
// red board
#elif defined(ARDUINO_ARCH_AVR)
#define ADC_RESOLUTION 10
#define MAX_VOLTAGE 5000
#define WireMaster Wire
#define SETRES(x) while(false){}
// untested
#else
  #error "Wrong Hardware"
#endif
const unsigned long ADCMax = (1 << ADC_RESOLUTION) - 1;
#endif
