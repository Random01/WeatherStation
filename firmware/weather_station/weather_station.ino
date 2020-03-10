/*
  Connecting the BME280 Sensor:
  Sensor              ->  Board
  -----------------------------
  Vin (Voltage In)    ->  3.3V
  Gnd (Ground)        ->  Gnd
  SDA (Serial Data)   ->  A4 on Uno/Pro-Mini, 20 on Mega2560/Due, 2 Leonardo/Pro-Micro
  SCK (Serial Clock)  ->  A5 on Uno/Pro-Mini, 21 on Mega2560/Due, 3 Leonardo/Pro-Micro
*/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TM1637.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#define CLK 5 // DISPLAY pins definitions for TM1637 and can be changed to other ports
#define DIO 4 // DISPLAY pins definitions for TM1637 and can be changed to other ports

#define ONE_WIRE_BUS 8 // DS1820 Pin

#define SEALEVELPRESSURE_HPA (1013.25)

#define TEMPERATURE_BME_MODE 0
#define TEMPERATURE_DS1_MODE 1
#define PRESSURE_MODE        2
#define HUMIDITY_MODE        3
#define TIME_MODE            4
#define SETTINGS_MODE        5

#define BME280_ADDRESS (0x76)

Adafruit_BME280 bme; // I2C
TM1637 tm1637(CLK, DIO);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);

unsigned long delayTime = 1000;
byte mode = TEMPERATURE_DS1_MODE;
int8_t timeDisplay[] = {0x00, 0x00, 0x00, 0x00};

void setup() {
  Serial.begin(9600);

  initBmeSensor();
  initDs1Sensor();
  initDisplay();
}

void initBmeSensor() {
  Serial.println(F("BME280 test"));

  bool status = bme.begin();
  if (!status) {
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
  tm1637.init();
  tm1637.set(BRIGHT_TYPICAL);
}

void tickEncoder() {

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
  if (TIME_MODE == mode) {

  }
}

void tickSettings() {
  if (SETTINGS_MODE == mode) {

  }
}

void printTemperature(float temperature) {

}


void printHumidity(float humidity) {

}

void printPressure(float pressure) {

}

void printTime(int hours, int minutes) {

}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");

  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}

void print(int firstSegment, int secondSegment, int thirdSegment, int fourthSegment) {
  print(firstSegment, secondSegment, thirdSegment, fourthSegment, POINT_OFF);
}

void print(int firstSegment, int secondSegment, int thirdSegment, int fourthSegment, bool point) {
  timeDisplay[0] = firstSegment;
  timeDisplay[1] = secondSegment;
  timeDisplay[2] = thirdSegment;
  timeDisplay[3] = fourthSegment;

  tm1637.point(point);
  tm1637.display(timeDisplay);
}
