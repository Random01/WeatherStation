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

#define CLK 5 // DISPLAY pins definitions for TM1637 and can be changed to other ports
#define DIO 4 // DISPLAY pins definitions for TM1637 and can be changed to other ports

#define SEALEVELPRESSURE_HPA (1013.25)

#define LOOP_DELAY 1000

Adafruit_BME280 bme; // I2C
TM1637 tm1637(CLK, DIO);

unsigned long delayTime;

void setup() {
  Serial.begin(9600);
  Serial.println(F("BME280 test"));

  bool status;

  // default settings
  // (you can also pass in a Wire library object like &Wire2)
  status = bme.begin();
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delayTime = 1000;

  Serial.println();

  initDisplay();
}


void loop() {
  printValues();
  displayTime(15, 20);

  delay(LOOP_DELAY);
}

void initDisplay() {
  tm1637.set();
  tm1637.init();
}

void printTemperature(float temperature) {

}

void printTime() {

}

void printHumidity() {

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

void displayTime(int hours, int minutes) {
  print(hours / 10, hours % 10,  minutes / 10, minutes % 10);
}

void print(int firstSegment, int secondSegment, int thirdSegment, int fourthSegment) {
  int8_t timeDisplay[] = {0x00, 0x00, 0x00, 0x00};

  timeDisplay[0] = firstSegment;
  timeDisplay[1] = secondSegment;
  timeDisplay[2] = thirdSegment;
  timeDisplay[3] = fourthSegment;

  tm1637.display(timeDisplay);
}
