#include <TaskScheduler.h>

#include <Arduino.h>

#define DEBUG
#include "defines.h"

#include <Servo.h>
#include <math.h> 

#include <Wire.h>
#include "rgb_lcd.h"

// Servo
Servo meinServo;
int servoAngle;

// LCD
rgb_lcd lcd;
const int LCD_MODE_ISTWERT = 0
const int LCD_MODE_SollWERT = 1
int lcdMode = LCD_MODE_ISTWERT;

const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

// Poti
const int POTI_PIN = A1;

// Taster
const int TASTER_PIN = 3;
void handleTasterPress();

// LED
const int LED_PIN = LED_BUILTIN;

// Taskscheduler
Scheduler runner;
void servoStellen();
void updateLCD();

Task taskServoStellen(1000, TASK_FOREVER, & servoStellen);
Task taskUpdateLCD(1000, TASK_FOREVER, &updateLCD);

// Temp
const int pinTempSensor = A0;
const unsigned int B = 4275;  // b wert anpassen für ntc
const float T0_KELVIN = 298.15; 
float currentTempC = 0.0;


void setup() {
  Serial.begin(9600); 
  DEBUG_PRINTLN("Praktikum 2");

  // ADC
  SETRES(12);

  // LED
  pinMode(LED_PIN, OUTPUT);

  // Taster Interupt
  pinMode(TASTER_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(TASTER_PIN), handleTasterPress, FALLING); 
  
  // Servo
  meinServo.attach(7);

  // Taskscheduler
  runner.addTask(taskServoStellen);
  runner.addTask(taskUpdateLCD);
  taskServoStellen.enable();
  taskUpdateLCD.enable();


  // LCD
  lcd.begin(16, 2, LCD_5x8DOTS, WireMaster);
  lcd.setRGB(colorR, colorG, colorB);
}
void loop() {
  unsigned long currentMillis = millis();
  unsigned long interval = turboMode ? TURBO_INTERVAL : NORMAL_INTERVAL;
    
  if (loopStartMillis == 0) {
      loopStartMillis = currentMillis;
  }


  // 3 Sekunden Durchlauf
  if (currentMillis - previousLedMillis >= interval) {
    previousLedMillis = currentMillis;
    logic();
  }

  if (turboMode && (millis() - turboStartMillis >= TURBO_DURATION)) {
      turboMode = false;
      DEBUG_PRINTLN("Turbo-Modus beendet");
  }
}

void logic() {
  // LED Updaten
  int ledState = digitalRead(LED_PIN);
  digitalWrite(LED_PIN, !ledState);
  
  // Servo
  int potiValue = analogRead(POTI_PIN);
  servoAngle = map(potiValue, 0, ADCMax, 5, 160);

  meinServo.write(servoAngle);

  DEBUG_PRINT("Servo Winkel: ");
  DEBUG_PRINT(servoAngle);
  DEBUG_PRINTLN("°");

  // Temp
  unsigned int tempRawValue = analogRead(pinTempSensor);
  float resistance_ratio = ((float)ADCMax / tempRawValue) - 1.0;
  float currentTempK = 1.0 / ((log(resistance_ratio) / B) + (1.0 / T0_KELVIN));

  currentTempC = currentTempK - 273.15;
  DEBUG_PRINT("Aktuelle Temperatur: ");
  DEBUG_PRINT(currentTempC);
  DEBUG_PRINTLN(" °C");

  updateLCD(currentTempC, servoAngle);
}

void updateLCD() {
  if (lcdMode == LCD_MODE_ISTWERT) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T: ");
    lcd.print(currentTempC, 1);
    lcd.write(223); // grad code
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("S: ");
    lcd.print(servoAngle);
    lcd.write(223); // grad code
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T: ");
    lcd.print("Sollwert:", 1);
    lcd.write(223); // grad code
    lcd.print("C");

    lcd.setCursor(0, 1);
    lcd.print("S: ");
    lcd.print(servoAngle);
    lcd.write(223); // grad code
  }
}