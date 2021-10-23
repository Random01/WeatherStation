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

#include <TimerOne.h>
#include <TM74HC595Display.h>

#define SCLK  4 // TM74HC595 Pin
#define RCLK  3 // TM74HC595 Pin
#define DIO   2 // TM74HC595 Pin

#define FIRST_DIGIT   3
#define SECOND_DIGIT  2
#define THIRD_DIGIT   1
#define FOURTH_DIGIT  0
#define POINT_SHIFT   10

#define EMPTY_SYMBOL  0b11111111 // " "
#define DASH_SYMBOL   0b10111111 // "-"

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

#define MENU_ITEMS_COUNT 7

// If you see "Could not find a valid BME280 sensor, check wiring!"
// redefine BME280_ADDRESS to 0x76 in Adafruit_BME280.h
Adafruit_BME280 bme; // I2C

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor(&oneWire);

TM74HC595Display disp(SCLK, RCLK, DIO);

unsigned char SYMBOL[20];

int mode = TEMPERATURE_BME_MODE;

bool is_updating = false;

void setup() {
  Serial.begin(9600);

  initBmeSensor();
  initDs1Sensor();
  initDisplay();
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

void initDisplay() {
  symbols();                         // объявление пачки символов для работы дисплея

  Timer1.initialize(1500);           // установка таймера на каждые 1500 микросекунд
  Timer1.attachInterrupt(timerIsr);  // запуск таймера
}

void loop() {
  is_updating = true;

  mode = TEMPERATURE_BME_MODE;
  tickTemperatureBme();

  delay(2000);

  mode = TEMPERATURE_DS1_MODE;
  tickTemperatureDs1();

  delay(2000);

  mode = PRESSURE_MODE;
  tickPressure();

  delay(2000);

  mode = HUMIDITY_MODE;
  tickHumidity();

  delay(2000);

  mode = DEW_POINT;
  tickDewPoint();

  delay(2000);
}

void tickTemperatureBme() {
  if (is_updating == true && TEMPERATURE_BME_MODE == mode) {
    float temperature = bme.readTemperature(); // *C
    displayTemperature(temperature);
  }
}

void tickTemperatureDs1() {
  if (is_updating == true && TEMPERATURE_DS1_MODE == mode) {
    sensor.requestTemperatures();
    float temperature = sensor.getTempCByIndex(0); // *C
    displayTemperature(temperature);
  }
}

void tickPressure() {
  if (is_updating == true && PRESSURE_MODE == mode) {
    float pressure = bme.readPressure(); // hPa
    printPressure(pressure);
  }
}

void tickHumidity() {
  if (is_updating == true && HUMIDITY_MODE == mode) {
    float humidity = bme.readHumidity(); // "%"
    printHumidity(humidity);
  }
}

void tickDewPoint() {
  if (is_updating && DEW_POINT == mode) {
    float dewPoint = getDewPoint();
    displayTemperature(dewPoint);
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
   pressure in "hPa"
   Displays pressure in "mm Hg".
*/
void printPressure(float pressure) {
  pressure = pressure / 133.322;

  int firstDigit = (int) pressure / 100; // 768 -> 7
  int secondDigit =  (int)(pressure - firstDigit * 100) / 10; // 768 -> 6
  int thirdDigit = (int) pressure % 10;  // 768 -> 8

  disp.set(SYMBOL[firstDigit], FIRST_DIGIT);
  disp.set(SYMBOL[secondDigit], SECOND_DIGIT);
  disp.set(SYMBOL[thirdDigit], THIRD_DIGIT);
  disp.set(_P, FOURTH_DIGIT);
}

/**
   Humidity in %
   Displays humidity in %
   67.7 -> 67H
*/
void printHumidity(float humidity) {
  int firstDigit = (int) humidity / 100; // 100 -> 1
  int secondDigit = (int) humidity / 10; // 98 -> 9
  int thirdDigit = (int) humidity % 10;  // 98 -> 8

  disp.set((firstDigit > 0) ? SYMBOL[firstDigit] : EMPTY_SYMBOL, FIRST_DIGIT);
  disp.set((secondDigit > 0) ? SYMBOL[secondDigit] : EMPTY_SYMBOL, SECOND_DIGIT);
  disp.set(SYMBOL[thirdDigit], THIRD_DIGIT);
  disp.set(_H, FOURTH_DIGIT);
}

void displayTemperature(float temperature) {
  int sign = temperature < 0;
  temperature = abs(temperature);

  int firstDigit = (int) temperature / 10; // 25.8 -> 2
  int secondDigit = (int) temperature % 10;  // 25.8 -> 5
  int fractionDigit = (int) ((temperature  - (firstDigit * 10 + secondDigit)) * 10); // 25.8 -> 8

  disp.set((sign && firstDigit > 0) ? DASH_SYMBOL : EMPTY_SYMBOL, FIRST_DIGIT);

  if (firstDigit == 0) {
    disp.set(sign ? DASH_SYMBOL : EMPTY_SYMBOL, SECOND_DIGIT);
  } else {
    disp.set(SYMBOL[firstDigit], SECOND_DIGIT);
  }

  disp.set(SYMBOL[POINT_SHIFT + secondDigit], THIRD_DIGIT);
  disp.set(SYMBOL[fractionDigit], FOURTH_DIGIT);
}

void timerIsr() {   // прерывание таймера
  disp.timerIsr();  // пнуть дисплей
}

void symbols() {
  // обычные
  SYMBOL[0] = 0xC0; //0
  SYMBOL[1] = 0xF9; //1
  SYMBOL[2] = 0xA4; //2
  SYMBOL[3] = 0xB0; //3
  SYMBOL[4] = 0x99; //4
  SYMBOL[5] = 0x92; //5
  SYMBOL[6] = 0x82; //6
  SYMBOL[7] = 0xF8; //7
  SYMBOL[8] = 0x80; //8
  SYMBOL[9] = 0x90; //9

  // с точкой
  SYMBOL[10] = 0b01000000; //0.
  SYMBOL[11] = 0b01111001; //1.
  SYMBOL[12] = 0b00100100; //2.
  SYMBOL[13] = 0b00110000; //3.
  SYMBOL[14] = 0b00011001; //4.
  SYMBOL[15] = 0b00010010; //5.
  SYMBOL[16] = 0b00000010; //6.
  SYMBOL[17] = 0b01111000; //7.
  SYMBOL[18] = 0b00000000; //8.
  SYMBOL[19] = 0b00010000; //9.
}
