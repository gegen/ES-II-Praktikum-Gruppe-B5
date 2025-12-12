#include <Arduino.h>

#define DEBUG
#include "defines.h"

#include <Servo.h>
#include <math.h> 

#include <Wire.h>
#include "rgb_lcd.h"


Servo meinServo;
rgb_lcd lcd;

const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

const int POTI_PIN = A1;

const int TASTER_PIN = 3;
volatile bool tasterGedrueckt = false;

const int LED_PIN = LED_BUILTIN;

//Intervall
bool turboMode = false;
unsigned long turboStartMillis = 0;
unsigned long loopStartMillis = 0;
unsigned long previousLedMillis = 0;

const long NORMAL_INTERVAL = 3000;
const long TURBO_INTERVAL  = 500;    // 0,5 Sekunden
const long TURBO_DURATION  = 60000;  // 1 Minute

//Temp
const int pinTempSensor = A0;
const unsigned int B = 4275;  // b wert anpassen für ntc
const float T0_KELVIN = 298.15; 
float currentTempC = 0.0;

void handleTasterPress() {
  tasterGedrueckt = true;
}
void setup() {
  Serial.begin(9600); 
  DEBUG_PRINTLN("Praktikum 2");

  SETRES(12);

  pinMode(LED_PIN, OUTPUT);

  pinMode(TASTER_PIN, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(TASTER_PIN), handleTasterPress, FALLING); 
  meinServo.attach(7);


  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2, LCD_5x8DOTS, WireMaster);

  lcd.setRGB(colorR, colorG, colorB);

  lcd.print("hello, world!");
}
void loop() {
  unsigned long currentMillis = millis();
  unsigned long interval = turboMode ? TURBO_INTERVAL : NORMAL_INTERVAL;
  
  // DEBUG_PRINT("Loopstart @ ");
  // DEBUG_PRINTLN(currentMillis);
  
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

  if (tasterGedrueckt) {
    DEBUG_PRINTLN("Taster per Interrupt erkannt");
    tasterGedrueckt = false;

    turboMode = true;
    turboStartMillis = millis();
  }

  // DEBUG_PRINT("Loopende @ ");
  // DEBUG_PRINTLN(millis());
}

void logic() {
  // LED Updaten
  int ledState = digitalRead(LED_PIN);
  digitalWrite(LED_PIN, !ledState);
  
  // Servo
  int potiValue = analogRead(POTI_PIN);
  int servoAngle = map(potiValue, 0, ADCMax, 5, 160);

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

void updateLCD(float currentTempC, int servoAngle) {
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
}