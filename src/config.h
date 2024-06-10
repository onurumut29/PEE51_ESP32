#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
#include "MQUnifiedsensor.h"
#include <HardwareSerial.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include "BluetoothSerial.h"
#include <SD.h>
#include <SPI.h>
#include "DFRobot_PH.h"
#include <time.h>
#include <nvs_flash.h>
#include <nvs.h>
#include "esp_attr.h"
#include "DFRobot_ESP_EC.h"
#include "soc/pcnt_struct.h"
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include <Preferences.h>
#include <EEPROM.h>

void BluetoothListen(void *parameter);
void DisplayMeasurements(void *parameter);
void Measuring(void *parameter);
void sendArray(void *parameter);

extern int coAmount, h2Amount, flowRateAmount , flowRate2Amount , temperatureAmount , phValueAmount , ecValueAmount , humidityAmount , ds18b20Amount, voltAmount, acsAmount;
const int MaxMeasurements = 500;
//#define MaxMeasurements 500

const int dht22_tempInterval  = MaxMeasurements / temperatureAmount; //40
const int phValueInterval     = MaxMeasurements / phValueAmount;
const int dht22_humInterval   = MaxMeasurements / humidityAmount;
const int ecValueInterval     = MaxMeasurements / ecValueAmount;
const int flowRateInterval    = MaxMeasurements / flowRateAmount;
const int flowRate2Interval   = MaxMeasurements / flowRate2Amount;
const int acsValueFInterval   = MaxMeasurements / acsAmount;
const int ds18b20Interval     = MaxMeasurements / ds18b20Amount; //4
const int voltInterval        = MaxMeasurements / voltAmount; //1
const int h2Interval          = MaxMeasurements / h2Amount;
const int coInterval          = MaxMeasurements / coAmount;

struct Measurement {
  float phValue;
  float ecValue;
  float AcsValueF;
  float flowRate;
  float flowRate2;
  float Volt;
  float DS18B20_1;
  float DS18B20_2;
  float DS18B20_3;
  float DS18B20_4;
  float DS18B20_5;
  float temperature;
  float humidity;
  float ppmH;
  float ppmCO;
  time_t timestamp;
  uint64_t timestamp_ms;
  uint64_t ts;
  int milliseconds; 
}; 
extern Measurement measurement[MaxMeasurements];

/*      Configuration     */
extern bool sendhttp;
extern String payload;
void generateJson(Measurement buff);

/*      Display      */
extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C smallOled;
extern U8G2_SH1106_128X64_NONAME_1_HW_I2C bigOled;

/*      MQ-7 MQ-8 sensor       */
void mq7_init(MQUnifiedsensor& MQ7);
void mq8_init(MQUnifiedsensor& MQ8);

//For GSMSerial output on OLED
extern U8G2LOG u8g2log;
extern volatile int stateBigOled, stateOled;

/*      GSM Functions    */
void saveTimestamp(uint64_t timestamp);
uint64_t getSavedTimestamp();

void parseDatetime();
//extern HardwareSerial gsmSerial;

extern String apn, apn_User, apn_Pass;
extern char httpapi[];

//void GA6_init();
void getTime();
void getTimeNow();  
void post_http(String j); 
time_t convertToUnixTimestamp(String date, String time);
void readGsmResponse();
void initialize_gsm();
void initialize_gsm2();


/*      Display Setup     */
void printBigOled(String x);
void init_displays();
void printSmallOled(String x);


/*      DS18B20 sensor       */
extern const int DS18B20_PIN;
void printDS18B20Address();
void AllDS18B20Sensors(Measurement& measurement);
//void AllDS18B20Sensors(Measurement& measurement);

/*      Flow sensor       */
void pcnt_example_init(pcnt_unit_t unit, int pulse_gpio_num);
extern float flowRate, flowRate2;

/*      Bluetooth          */
extern BluetoothSerial SerialBT;
extern String message;
extern char incomingChar;
void sendFileOverBluetooth(const char* path);
void readFileAndSendOverBluetooth(fs::FS &fs, const char *path);
void sendFileOverBluetoothInOneGo2(const char* path);

/*        SD Card         */
extern int CS_PIN;
void SD_init();
void listDir(fs::FS &fs, const char * dirname, uint8_t levels);
//void createDir(fs::FS &fs, const char * path);
//void removeDir(fs::FS &fs, const char * path);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
//void renameFile(fs::FS &fs, const char * path1, const char * path2);
//void deleteFile(fs::FS &fs, const char * path);
//void testFileIO(fs::FS &fs, const char * path);
void readFile(fs::FS &fs, const char * path);
void logMeasurement(String measurement);
void sendFileOverBluetoothInOneGo(const char* path);

/*      Configuration     */
extern int buttonbigOled, buttonsmallOled;
void buttonInterrupt_bigOled();
void buttonInterrupt_smallOled();

/*      Ph Sensor         */
extern DFRobot_PH ph;
extern int PH_PIN;
float pH();
float readTemperature();

/*      Current Sensor   */
extern int CurrentPin;
float CurrentSensor_quick();

/*      EC library (Conductivity sensor)        */
extern DFRobot_ESP_EC ec;
//extern float voltage_cond, temperature_cond;
extern int EC_PIN; // Potentiometer is connected to GPIO 34 (Analog ADC1_CH6) 
float Cond();

#endif // CONFIG_H
