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
  #define ADC_RESOLUTION_BITS 12 
  #define SETRES(x) analogReadResolution(x) 
#elif defined(ARDUINO_ARCH_AVR)
  #define ADC_RESOLUTION_BITS 10 
  #define SETRES(x) while(false){} 
#else
  #error "Wrong Hardware"
#endif
const unsigned long ADCMax = (1UL << ADC_RESOLUTION_BITS) - 1;
#endif
