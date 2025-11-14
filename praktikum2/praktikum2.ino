#include <Arduino.h>
#include "defines.h"
#include <LiquidCrystal_I2C.h>

#include <Servo.h>
#include <math.h> 

LiquidCrystal_I2C lcd(0x27, 16, 2);
const int POTI_PIN = A1;
Servo meinServo;
const int LED_PIN = LED_BUILTIN; 
const long LED_INTERVAL = 3000;    
unsigned long previousLedMillis = 0;

const int TASTER_PIN = 2; 
volatile bool tasterGedrueckt = false; 

const long LOOP_CHECK_INTERVAL = 500; 
unsigned long previousLoopMillis = 0;
unsigned long loopStartMillis = 0;
const long ONE_MINUTE = 60000; 

const int pinTempSensor = A0;
const unsigned int B = 4275;      // b wert anpassen für ntc
const float T0_KELVIN = 298.15; 
float currentTempC = 0.0;

DEBUG

void handleTasterPress() {
  tasterGedrueckt = true;
}
void setup() {
  Serial.begin(9600); 
  DEBUG_PRINTLN("Praktikum 2");

  SETRES(ADC_RESOLUTION_BITS);

  pinMode(LED_PIN, OUTPUT);

  pinMode(TASTER_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(TASTER_PIN), handleTasterPress, FALLING); 
  meinServo.attach(4); 

  lcd.init();
  lcd.backlight();
}
void loop() {
  unsigned long currentMillis = millis();
  
  DEBUG_PRINT("Loopstart @ ");
  DEBUG_PRINTLN(currentMillis);
  
  if (loopStartMillis == 0) {
      loopStartMillis = currentMillis;
  }

  if (currentMillis - loopStartMillis < ONE_MINUTE) { 
      if (currentMillis - previousLoopMillis >= LOOP_CHECK_INTERVAL) {
          previousLoopMillis = currentMillis;
          
          DEBUG_PRINTLN("0.5s fertgi");
      }
  }

  if (currentMillis - previousLedMillis >= LED_INTERVAL) {
    previousLedMillis = currentMillis;
    int ledState = digitalRead(LED_PIN);
    digitalWrite(LED_PIN, !ledState);
  }

  if (tasterGedrueckt) {
    DEBUG_PRINTLN("Taster per Interrupt erkannt");
    tasterGedrueckt = false; 
  }

  DEBUG_PRINT("Loopende @ ");
  DEBUG_PRINTLN(millis());

  int potiValue = analogRead(POTI_PIN);
  int servoAngle = map(potiValue, 0, ADCMax, 5, 175);

  meinServo.write(servoAngle);

  DEBUG_PRINT("Servo Winkel: ");
  DEBUG_PRINTLN(servoAngle);
  DEBUG_PRINTLN(" Grad");

  unsigned int tempRawValue = analogRead(pinTempSensor);
  float resistance_ratio = ((float)ADCMax / tempRawValue) - 1.0;
  float currentTempK = 1.0 / ((log(resistance_ratio) / B) + (1.0 / T0_KELVIN));

  currentTempC = currentTempK - 273.15;
  DEBUG_PRINT("Aktuelle Temperatur: ");
  DEBUG_PRINT(currentTempC);
  DEBUG_PRINTLN(" °C");

  //2f

  lcd.setCursor(0, 0);
  lcd.print("T: ");
  lcd.print(currentTempC, 1);
  lcd.write(223); // grad code
  lcd.print("C   ");

  lcd.setCursor(0, 1);
  lcd.print("S: ");
  lcd.print(servoAngle);
  lcd.print("  deg");
}