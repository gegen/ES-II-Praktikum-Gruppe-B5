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
const int LCD_MODE_ISTWERT = 0;
const int LCD_MODE_SollWERT = 1;
int lcdMode = LCD_MODE_ISTWERT;

const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

// Poti
const int POTI_PIN = A1;
int potiSollTemp;

// Taster
const int TASTER_PIN = 3;
void handleTasterPress();

// LED
const int LED_PIN = LED_BUILTIN;

// Taskscheduler
Scheduler runner;
void autoReturnToIstwert();
void sensorenLesen();
void servoStellen();
void updateLCD();

Task taskAutoReturn(20000, 1, &autoReturnToIstwert);
Task taskSensorenLesen(100, TASK_FOREVER, &sensorenLesen);
Task taskServoStellen(1000, TASK_FOREVER, & servoStellen);
Task taskUpdateLCD(100, TASK_FOREVER, &updateLCD);

// Temp
const int pinTempSensor = A0;
const unsigned int B = 4275;  // b wert anpassen für ntc
const float T0_KELVIN = 298.15; 
float currentTempC = 0.0;
int sollTempC = 0;


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
  runner.addTask(taskAutoReturn);
  runner.addTask(taskSensorenLesen);
  runner.addTask(taskServoStellen);
  runner.addTask(taskUpdateLCD);
  taskSensorenLesen.enable();
  taskServoStellen.enable();
  taskUpdateLCD.enable();


  // LCD
  lcd.begin(16, 2, LCD_5x8DOTS, WireMaster);
  lcd.setRGB(colorR, colorG, colorB);
}

void loop() {
  runner.execute();
}

void handleTasterPress() {
  if (lcdMode == LCD_MODE_ISTWERT) {
    // Sollwert Modus aktivieren
    lcdMode = LCD_MODE_SollWERT;

    taskAutoReturn.restartDelayed(20000);
  } else {
    // Temp übernehmen
    sollTempC = potiSollTemp;
    lcdMode = LCD_MODE_ISTWERT;

    taskAutoReturn.disable();
  }
}

void autoReturnToIstwert() {
  lcdMode = LCD_MODE_ISTWERT;
}

void sensorenLesen() {
  // LED Updaten
  int ledState = digitalRead(LED_PIN);
  digitalWrite(LED_PIN, !ledState);
  
  // Poti
  int potiValue = analogRead(POTI_PIN);
  potiSollTemp = map(potiValue, 0, ADCMax, 0, 30);
  // servoAngle = map(potiValue, 0, ADCMax, 5, 160);

  // meinServo.write(servoAngle);

  DEBUG_PRINT("Poti Soll Temp: ");
  DEBUG_PRINT(potiSollTemp);
  DEBUG_PRINTLN("° C");

  // Temp
  unsigned int tempRawValue = analogRead(pinTempSensor);
  float resistance_ratio = ((float)ADCMax / tempRawValue) - 1.0;
  float currentTempK = 1.0 / ((log(resistance_ratio) / B) + (1.0 / T0_KELVIN));

  currentTempC = currentTempK - 273.15;
  DEBUG_PRINT("Aktuelle Temperatur: ");
  DEBUG_PRINT(currentTempC);
  DEBUG_PRINTLN(" °C");
}

void servoStellen() {
  float diff = currentTempC - sollTempC;

  if (diff <= -5.0) {
    // 5K unter Soll 100%
    meinServo.write(160);
    servoAngle = 100;

  } else if (diff >= 5.0) {
    // 5K über Soll 0%
    meinServo.write(10);
    servoAngle = 0;

  } else {
    // Solltemperatur erreicht 50%
    meinServo.write(80);
    servoAngle = 50;
  }
}

void updateLCD() {
  if (lcdMode == LCD_MODE_ISTWERT) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T: ");
    lcd.print(currentTempC, 1);
    lcd.write(223); // grad code
    lcd.print(" C");

    lcd.setCursor(0, 1);
    lcd.print("Servo: ");
    lcd.print(servoAngle);
    lcd.print(" %");
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Sollwert:");

    lcd.setCursor(0, 1);
    lcd.print(potiSollTemp, 1);
    lcd.write(223); // grad code
    lcd.write(" C"); // grad code
  }
}