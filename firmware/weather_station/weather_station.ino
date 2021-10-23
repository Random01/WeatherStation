/*
  Connecting the BME280 Sensor:
  Sensor              ->  Board
  -----------------------------
  Vin (Voltage In)    ->  3.3V
  Gnd (Ground)        ->  Gnd
  SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
  SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro
*/
#include <math.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>

#include "GyverTM1637.h"
#include "GyverEncoder.h"

#define DISPLAY_CLK   5 // DISPLAY pins definitions for TM1637 and can be changed to other ports
#define DISPLAY_DIO   4 // DISPLAY pins definitions for TM1637 and can be changed to other ports

#define ONE_WIRE_BUS  10 // DS1820 Pin

#define ENCODER_CLK   7 // Encoder S1 - CLK
#define ENCODER_DT    8 // Encoder S2 - DT
#define ENCODER_SW    9 // Encoder Key - SW

#define SEALEVELPRESSURE_HPA (1013.25)

#define TEMPERATURE_BME_MODE 0
#define TEMPERATURE_DS1_MODE 1
#define PRESSURE_MODE        2
#define HUMIDITY_MODE        3
#define DEW_POINT            4
#define TIME_MODE            5
#define SETTINGS_MODE        6

#define MENU_ITEMS_COUNT 7

// redefine BME280_ADDRESS to 0x76 in Adafruit_BME280.h
Adafruit_BME280 bme; // I2C

GyverTM1637 disp(DISPLAY_CLK, DISPLAY_DIO);

Encoder enc1(ENCODER_CLK, ENCODER_DT, ENCODER_SW);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);

RTC_DS3231 rtc;

int mode = TIME_MODE;

int previous_second = -1;
bool is_updating = false;
DateTime now;

bool curr_dots = POINT_OFF; // displays dots in clock

void setup() {
  Serial.begin(9600);

  initBmeSensor();
  initRTC();
  initDs1Sensor();
  initDisplay();
  initEncoder();
}

void initEncoder() {
  enc1.setType(TYPE2);
}

void initRTC() {
  Serial.println(F("RTC test"));

  if (!rtc.begin()) {
    Serial.println("Couldn't find a valid RTC, check wiring!");
    while (1);
  }

  Serial.println("RTC has been found");
  Serial.println();
}

void restoreTime() {
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, lets set the time!");
    // following line sets the RTC to the date & time this sketch was compiled
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void initBmeSensor() {
  Serial.println(F("BME280 test"));

  if (! bme.begin()) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("BME280 has been found");
  Serial.println();
}

void initDs1Sensor() {
  sensor.begin();
  sensor.setResolution(12);
}

void loop() {
  tickEncoder();

  now = rtc.now();
  uint8_t second =  now.second();
  if (previous_second != second) {
    previous_second = second;
    curr_dots = !curr_dots;
    // let's update UI each second
    is_updating = true;
  }

  tickTemperatureBme();
  tickTemperatureDs1();

  tickPressure();
  tickHumidity();
  tickDewPoint();

  tickTime();

  tickSettings();

  is_updating = false;
}

void initDisplay() {
  disp.clear();
  disp.brightness(BRIGHT_TYPICAL);
}

void tickEncoder() {
  enc1.tick();

  if (enc1.isRight()) mode++;
  if (enc1.isLeft()) mode--;

  if (mode < 0) {
    mode = MENU_ITEMS_COUNT - 1;
  } else if (mode >= MENU_ITEMS_COUNT) {
    mode = 0;
  }

  if (enc1.isTurn()) {
    is_updating = true;

    disp.point(POINT_OFF);
    disp.displayByte(0x40, 0x40, 0x40, 0x40);
  }
}

void tickTemperatureBme() {
  if (is_updating == true && TEMPERATURE_BME_MODE == mode) {
    float temperature = bme.readTemperature(); // *C
    printTemperature(temperature);
  }
}

/**
 * External temperature sensor number 1.
 */
void tickTemperatureDs1() {
  if (is_updating == true && TEMPERATURE_DS1_MODE == mode) {
    sensor.requestTemperatures();
    float temperature = sensor.getTempCByIndex(0); // *C
    printTemperature(temperature);
  }
}

void tickPressure() {
  if (is_updating == true && PRESSURE_MODE == mode) {
    float pressure = bme.readPressure() / 100.0F; // hPa
    printHumidity(pressure);
  }
}

void tickHumidity() {
  if (is_updating == true && HUMIDITY_MODE == mode) {
    float humidity = bme.readHumidity(); // "%"
    printHumidity(humidity);
  }
}

/**
 * Displays time.
 */
void tickTime() {
  if (is_updating && TIME_MODE == mode) {
    printTime(now.hour(), now.minute(), curr_dots);
  }
}

void tickSettings() {
  if (SETTINGS_MODE == mode) {
    // Proccess available settings
  }
}

/**
 * Displays Dew Point.
 */
void tickDewPoint() {
  if (is_updating && DEW_POINT == mode) {
    float dewPoint = getDewPoint();
    printTemperature(dewPoint);
  }
}

/**
   Calculates Dew Point.
   https://en.wikipedia.org/wiki/Dew_point#Calculating_the_dew_point
*/
float getDewPoint() {
  float b = 17.62;
  float c = 243.12;
  float temperature = bme.readTemperature();
  float humidity = bme.readHumidity();
  float gamma = (b * temperature / (c + temperature)) + log(humidity / 100.0);

  return (c * gamma) / (b - gamma);
}

/**
   "29:8_" <- 28.9C
   "_9:5_" <- 9.5C
   "_9:5-" <- -9.5C
*/
void printTemperature(float temperature) {
  bool belowZero = temperature < 0;

  uint8_t bit1 = digToHEX(1);
  uint8_t bit2 = digToHEX(5);
  uint8_t bit3 = digToHEX(6);
  uint8_t bit4 = belowZero ? 0x40 : 0x00;

  disp.point(POINT_ON);
  disp.display(bit1, bit2, bit3, bit4);
}

/**
   49.7p
*/
void printHumidity(float humidity) {

  
  disp.displayInt(int(humidity));
}

/**
   pressure in "hPa"
   Displays pressure in "mm Hg".
*/
void printPressure(float pressure) {
  pressure = pressure / 133.322;
  disp.displayInt(int(pressure));
}

void printTime(uint8_t hours, uint8_t minutes, bool dots) {
  disp.point(dots);
  disp.displayClock(hours, minutes);
}
