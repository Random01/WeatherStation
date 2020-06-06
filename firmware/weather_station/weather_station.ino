/*
  Connecting the BME280 Sensor:
  Sensor              ->  Board
  -----------------------------
  Vin (Voltage In)    ->  3.3V
  Gnd (Ground)        ->  Gnd
  SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
  SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro
*/

/*
  Encoder
  Key - SW
  S1 - CLK
  S2 - DT
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TM1637.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>

#include "GyverTM1637.h"
#include "GyverEncoder.h"

#define DISPLAY_CLK   5 // DISPLAY pins definitions for TM1637 and can be changed to other ports
#define DISPLAY_DIO   4 // DISPLAY pins definitions for TM1637 and can be changed to other ports

#define ONE_WIRE_BUS  10 // DS1820 Pin

#define ENCODER_CLK   7 // S1 - CLK
#define ENCODER_DT    8 // S2 - DT
#define ENCODER_SW    9 // Key - SW

#define SEALEVELPRESSURE_HPA (1013.25)

#define TEMPERATURE_BME_MODE 0
#define TEMPERATURE_DS1_MODE 1
#define PRESSURE_MODE        2
#define HUMIDITY_MODE        3
#define TIME_MODE            4
#define SETTINGS_MODE        5

#define MENU_ITEMS_COUNT 6

#define BME280_ADDRESS (0x76)

Adafruit_BME280 bme; // I2C

GyverTM1637 disp(DISPLAY_CLK, DISPLAY_DIO);

Encoder enc1(ENCODER_CLK, ENCODER_DT, ENCODER_SW);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);

RTC_DS3231 rtc;

unsigned long delayTime = 1000;
byte mode = TIME_MODE;
int8_t timeDisplay[] = {0x00, 0x00, 0x00, 0x00};
int previous_second = -1;
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

  tickTemperatureBme();
  tickTemperatureDs1();

  tickPressure();
  tickHumidity();
  tickTime();

  tickSettings();

  delay(delayTime);
}

void initDisplay() {
  disp.clear();
  disp.brightness(4);
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
}

void tickTemperatureBme() {
  if (TEMPERATURE_BME_MODE == mode) {
    float temperature = bme.readTemperature(); // *C
    printTemperature(temperature);
  }
}

void tickTemperatureDs1() {
  if (TEMPERATURE_DS1_MODE == mode) {
    sensor.requestTemperatures();
    float temperature = sensor.getTempCByIndex(0); // *C
    printTemperature(temperature);
  }
}

void tickPressure() {
  if (PRESSURE_MODE == mode) {
    float pressure = (bme.readPressure() / 100.0F); // hPa
    printHumidity(pressure);
  }
}

void tickHumidity() {
  if (HUMIDITY_MODE == mode) {
    float humidity = bme.readHumidity(); // "%"
    printHumidity(humidity);
  }
}

void tickTime() {
  DateTime now = rtc.now();
  if (previous_second != now.second()) {
    previous_second = now.second();
    curr_dots = !curr_dots;
  }

  if (TIME_MODE == mode) {
    printTime(now.hour(), now.minute(), curr_dots);
  }
}

void tickSettings() {
  if (SETTINGS_MODE == mode) {
    // Proccess available settings
  }
}

void printTemperature(float temperature) {
  disp.displayInt(-999);
}


void printHumidity(float humidity) {
  disp.displayInt(-999);
}

void printPressure(float pressure) {
  disp.displayInt(-999);
}

void printTime(int hours, int minutes, bool dots) {
  disp.point(flag);
  disp.displayClock(hours, minutes);
}

//void printValues() {
//  Serial.print("Temperature = ");
//  Serial.print(bme.readTemperature());
//  Serial.println(" *C");
//
//  Serial.print("Pressure = ");
//
//  Serial.print(bme.readPressure() / 100.0F);
//  Serial.println(" hPa");
//
//  Serial.print("Approx. Altitude = ");
//  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
//  Serial.println(" m");
//
//  Serial.print("Humidity = ");
//  Serial.print(bme.readHumidity());
//  Serial.println(" %");
//
//  Serial.println();
//}

void print(int firstSegment, int secondSegment, int thirdSegment, int fourthSegment) {
  print(firstSegment, secondSegment, thirdSegment, fourthSegment, POINT_OFF);
}

void print(int firstSegment, int secondSegment, int thirdSegment, int fourthSegment, bool dots) {
  timeDisplay[0] = firstSegment;
  timeDisplay[1] = secondSegment;
  timeDisplay[2] = thirdSegment;
  timeDisplay[3] = fourthSegment;

  tm1637.point(dots);
  tm1637.display(timeDisplay);
}
