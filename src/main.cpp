#include <Arduino.h>
#include <HardwareSerial.h>
#include <EEPROM.h>
#include "config.h"
#include <driver/adc.h>
/*      Save Time     */
#include <Preferences.h>
Preferences rtcPrefs;

/*      GSM Module Setup     */
HardwareSerial gsmSerial(2); // Use UART2
#define GSM_RX_PIN 16 //13
#define GSM_TX_PIN 17 //14
#define GSM_RST_PIN 12 //Not connected
extern String date;
extern String time_gsm;
String apn = "data.lycamobile.nl";
String apn_User = "lmnl";
String apn_Pass = "plus";
char httpapi[] = "http://jrbubuntu.ddns.net:5000/api/telemetry";
String mobileNumber = "+31614504288";
/*
String apn = "portalmmm.nl";
String apn_User = " ";
String apn_Pass = " ";
*/

extern time_t timestamp;

/*      MQ-7 CO2 sensor                  */
#include "MQUnifiedsensor.h"
#define Pin_MQ7 35
MQUnifiedsensor MQ7("ESP32", 3.3, 12, Pin_MQ7, "MQ-7");
/*      MQ-8 H2 sensor                   */
#define Pin_MQ8 32
MQUnifiedsensor MQ8("ESP32", 3.3, 12, Pin_MQ8, "MQ-8");
volatile float ppmH, ppmCO = 0.00;

/*      DHT22 - Temperature and Humidity */
#include "DHT.h"
#define DHT_SENSOR_PIN 25 
#define DHT_SENSOR_TYPE DHT22
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
volatile float temperature, humidity = 0.00;

/*      Setup Temperature sensor        */
const int DS18B20_PIN = 27;
extern volatile float DS18B20_1, DS18B20_2,DS18B20_3, DS18B20_4, DS18B20_5; // Initialize to a default value

/*      Setup Flowsensor                */
const int flowSensorPin = 36; //4

/*      Bluetooth                       */
BluetoothSerial SerialBT;
String message = "";
char incomingChar;

/*      SD card                         */
int CS_PIN = 5;

/*      Switch screen                   */
int buttonbigOled = 13; // Pin connected to the button
int buttonsmallOled = 14;
extern bool buttonPressed, buttonSmallPressed;
U8G2_WITH_HVLINE_SPEED_OPTIMIZATION
/*      Conductivity sensor                       */
int CondPin = 39; 

/*      Current sensor                  */
int CurrentPin = 33; 

/*      pH sensor                       */
DFRobot_PH ph;
int PH_PIN = 34; 

/*          Test for Array of JSON Objects         */
String payload = "";
bool sendhttp = false;
TaskStatus_t task_stats1, task_stats2;
// Define the queue handle
QueueHandle_t measurementQueue;
const int queueLength = 40; // Adjust the length according to your needs
Measurement measurement;
int temperatureAmount, phValueAmount, humidityAmount, ecValueAmount, flowRateAmount, acsAmount, ds18b20Amount, h2Amount, voltAmount = 10;
const int numMeasurements = std::max({temperatureAmount, phValueAmount, humidityAmount, ecValueAmount, flowRateAmount, acsAmount, ds18b20Amount, h2Amount, voltAmount});
const int shortMeasurement = 5;
const int longMeasurement = 100; //60
//const int numMeasurements = totMeasurements;
//Measurement measurements[50];
Measurement* measurements = new Measurement[numMeasurements];
//delete[] measurements;
int currentMeasurementIndex = 0;  
extern float phValue, AcsValueF, ecValue, flowRate;
float Volt = 0;

const int bufferSize = 8192; // 2048 Adjust this value based on your needs
char buffer[bufferSize];

int GSM_RX_PIN2, GSM_TX_PIN2, GSM_RST_PIN2, Pin_MQ72, Pin_MQ82, DHT_SENSOR_PIN2, DS18B20_PIN2, flowSensorPin2;


/*          Test for Array of JSON Objects         */
void sendArray(void *parameter)
{
  Serial.println("Now running sendArray task.");
  for (;;)
  {
    if(sendhttp==true){
      // Check if the queue is empty
      if (uxQueueMessagesWaiting(measurementQueue) > 0) {
        // Receive measurement from the queue
        if (xQueueReceive(measurementQueue, &measurement, portMAX_DELAY) == pdPASS) {
          TickType_t startTime = xTaskGetTickCount();          
          logMeasurement(buffer); //vandaag was payload (26/05)
          Serial.println("Message received in sendArray task: " + String(buffer));
          post_http(buffer);                
          TickType_t endTime = xTaskGetTickCount();
          TickType_t duration = endTime - startTime;          
          Serial.print("sendArrayTask duration: ");
          Serial.println(duration);
          Serial.println("");          
          sendhttp=false;
        }
      }
    } 
    vTaskDelay(10 / portTICK_PERIOD_MS);   
  }
}

const int chunkSize = 256;
void printBufferInChunks(const char* buffer, int bufferSize) {
    for (int i = 0; i < bufferSize; i += chunkSize) {
        int chunkLength = min(chunkSize, bufferSize - i);
        Serial.write(buffer + i, chunkLength);
        Serial.println(); // Add a newline after each chunk
    }
}

void Measuring(void *parameter) {
  Serial.println("Inside Measuring task.");
  for (;;) {
    TickType_t startTime = xTaskGetTickCount();          
 
    static int temperatureCount = 0, phValueCount = 0, humidityCount = 0, ecValueCount = 0, flowRateCount = 0;
    static int acsValueFCount = 0, ds18b20Count = 0, voltCount = 0, h2Count = 0;

    if (temperatureCount < temperatureAmount) {
      measurement.temperature = dht_sensor.readTemperature();
      if (isnan(measurement.temperature) || isinf(measurement.temperature)) measurement.temperature = 0;
      temperatureCount++;
    }

    if (phValueCount < phValueAmount) {
      measurement.phValue = pH();
      if (isnan(measurement.phValue) || isinf(measurement.phValue)) measurement.phValue = 0;
      phValueCount++;
    }

    if (humidityCount < humidityAmount) {
      measurement.humidity = dht_sensor.readHumidity();
      if (isnan(measurement.humidity) || isinf(measurement.humidity)) measurement.humidity = 0;
      humidityCount++;
    }

    if (ecValueCount < ecValueAmount) {
      measurement.ecValue = Cond();
      if (isnan(measurement.ecValue) || isinf(measurement.ecValue)) measurement.ecValue = 0;
      ecValueCount++;
    }

    if (flowRateCount < flowRateAmount) {
      measurement.flowRate = readFlowsensor();
      if (isnan(measurement.flowRate) || isinf(measurement.flowRate)) measurement.flowRate = 0;
      flowRateCount++;
    }

    if (acsValueFCount < acsAmount) {
      measurement.AcsValueF = CurrentSensor();
      if (isnan(measurement.AcsValueF) || isinf(measurement.AcsValueF)) measurement.AcsValueF = 0;
      acsValueFCount++;
    }

    if (ds18b20Count < ds18b20Amount) {
      AllDS18B20Sensors(measurement);      
      ds18b20Count++;
    }

    if (h2Count < h2Amount) {
      MQ8.update();
      measurement.ppmH = MQ8.readSensor();
      if (isnan(measurement.ppmH) || isinf(measurement.ppmH)) measurement.ppmH = 0;
      h2Count++;
    }

    if (voltCount < voltAmount) {
      measurement.Volt = 27.22; // Placeholder for voltage
      if (isnan(measurement.Volt) || isinf(measurement.Volt)) measurement.Volt = 0;
      voltCount++;
    }

    measurements[currentMeasurementIndex] = measurement;
    currentMeasurementIndex++;
    int bufferIndex;
    if (currentMeasurementIndex == longMeasurement) {
    TickType_t startTime = xTaskGetTickCount();          
      //long ts = convertToUnixTimestamp(date, time_gsm);
      time_t now = time(NULL);
      bufferIndex = snprintf(buffer, bufferSize, "{\"ts\": %ld, \"values\": {", now);

      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "\"Temperatuur_gas\": [");
      for (int i = 0; i < longMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].temperature);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Zuurtegraad\": [");
      for (int i = 0; i < longMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].phValue);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Stroom\": [");
      for (int i = 0; i < shortMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].AcsValueF);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Spanning\": [");
      for (int i = 0; i < longMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].Volt);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Temp1\": [");
      for (int i = 0; i < shortMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].DS18B20_1);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Temp2\": [");
      for (int i = 0; i < shortMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].DS18B20_2);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Temp3\": [");
      for (int i = 0; i < shortMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].DS18B20_3);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Temp4\": [");
      for (int i = 0; i < shortMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].DS18B20_4);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Temp5\": [");
      for (int i = 0; i < shortMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].DS18B20_5);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Luchtvochtigheid\": [");
      for (int i = 0; i < longMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].humidity);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Geleidbaarheid\": [");
      for (int i = 0; i < longMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].ecValue);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Flowsensor\": [");
      for (int i = 0; i < longMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].flowRate);
      }
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "], \"Waterstof\": [");
      for (int i = 0; i < shortMeasurement; i++) {
        bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurements[i].ppmH);
      }
      
      bufferIndex += snprintf(buffer + bufferIndex, bufferSize - bufferIndex, "]}}");

      currentMeasurementIndex = 0;      temperatureCount = 0;
      phValueCount = 0;                 humidityCount = 0;
      ecValueCount = 0;                 flowRateCount = 0;
      acsValueFCount = 0;               ds18b20Count = 0;
      h2Count = 0;                      voltCount = 0;

      if (xQueueSend(measurementQueue, &measurement, portMAX_DELAY) != pdPASS) {
        Serial.println("Failed to send to queue.");
      }
          sendhttp = true; // Set flag to send HTTP request
          TickType_t endTime = xTaskGetTickCount();
          TickType_t duration = endTime - startTime;            
          Serial.print("FormArray duration: ");
          Serial.println(duration);
          Serial.println("");
          Serial.println("Message formed in MeasureTask:");
          printBufferInChunks(buffer, bufferIndex);
          Serial.println("Size of message: " + String(bufferIndex));
    }

          TickType_t endTime = xTaskGetTickCount();
          TickType_t duration = endTime - startTime;          
         if(duration != 0) {
            Serial.print("MeasureTask duration: ");
            Serial.println(duration);
            Serial.println("");
          }   
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void BluetoothListen(void *parameter)
{
  Serial.println("Inside Bluetooth task.");
  for (;;)
  {
    if (SerialBT.available()){
      char incomingChar = SerialBT.read();
      if (incomingChar != '\n'){
        message += String(incomingChar);
      }
      else{
        message = "";
      }
      Serial.println("Received message:" + message);
    
      // Check if the command is to request a file
      if (message == "1") {
        Serial.println("Sending measurements.txt");
        sendFileOverBluetooth("/measurements.txt");
        //sendFileOverBluetooth(SD, "/foo.txt");
        delay(10);
        listDir(SD, "/", 0);
        message = "";
      }
      else if (message == "2") {
        Serial.println("Sending log.txt");
        sendFileOverBluetooth("/log.txt");
        delay(10);
        listDir(SD, "/", 0);
        message = "";
      }
      else if (message == "3") {
        size_t freeHeapBefore = esp_get_free_heap_size();
        Serial.println("Free heap before sending file: " + String(freeHeapBefore) + " bytes");

        sendFileOverBluetoothInOneGo("/log.txt");

        size_t freeHeapAfter = esp_get_free_heap_size();
        Serial.println("Free heap after sending file: " + String(freeHeapAfter) + " bytes");

        if (freeHeapAfter >= freeHeapBefore) {
            Serial.println("No memory leak detected");
        } else {
            Serial.println("Potential memory leak detected: " + String(freeHeapBefore - freeHeapAfter) + " bytes");
        }
      }
      else if (message == "4") {
        Serial.println("Sending log.txt");
        sendFileOverBluetoothInOneGo2("/log.txt");
        delay(10);
        listDir(SD, "/", 0);
        message = "";
      }
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}
void DisplayMeasurements(void *parameter)
{
  Serial.println("Inside Measuring task.");
  for (;;)
  {   
    String tempDis      = "Temp___:  " + String(temperature) + char(176) +" °C";
    String humidityDis  = "Humid__:  " + String(humidity)+     " %";
    String coDis        = "CO_____:  " + String(ppmCO) +      " ppm";
    String h2Dis        = "H2_____:  " + String(ppmH) +      " ppm";
    String flowDis      = "Flow___:  " + String(readFlowsensor()) +  " L/min"; 
    String ecDis        = "EC_____:  " + String(Cond()) +     " ms/cm";
    String DS18B20_1_Dis= "DStem_1:  " + String(DS18B20_1) + char(176) +" °C";
    String DS18B20_2_Dis= "DStem_2:  " + String(DS18B20_2) + char(176) +" °C";
    String DS18B20_3_Dis= "DStem_3:  " + String(DS18B20_3) + char(176) +" °C";
    String DS18B20_4_Dis= "DStem_4:  " + String(DS18B20_4) + char(176) +" °C";
    String Current_Dis  = "Current:  " + String(CurrentSensor()) + " A";
    String pH_Dis       = "pH_____:  " + String(pH()) + "";
    
    if(stateOled==1){
      smallOled.firstPage();
    do {
      smallOled.setFont(u8g2_font_6x10_tf); //Was u8g2_font_ncenB08_tr
      smallOled.drawStr(0, 10, h2Dis.c_str());
      smallOled.drawStr(0, 20, coDis.c_str());
      smallOled.drawStr(0, 30, flowDis.c_str());
      smallOled.drawStr(0, 40, pH_Dis.c_str());
      smallOled.drawStr(0, 50, ecDis.c_str());
      smallOled.drawStr(0, 60, Current_Dis.c_str());
    } while (smallOled.nextPage());
    }
     if(stateOled==2){
      smallOled.firstPage();
    do {
      smallOled.setFont(u8g2_font_6x10_tf); //Was u8g2_font_ncenB08_tr
      smallOled.drawStr(0, 10, tempDis.c_str());
      smallOled.drawStr(0, 20, DS18B20_1_Dis.c_str());
      smallOled.drawStr(0, 30, DS18B20_1_Dis.c_str());
      smallOled.drawStr(0, 40, DS18B20_1_Dis.c_str());
      smallOled.drawStr(0, 50, DS18B20_4_Dis.c_str());
      smallOled.drawStr(0, 60, humidityDis.c_str());
    } while (smallOled.nextPage());
    }
    
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  }
TaskHandle_t Task1, Task2, Task3, Task4 = NULL;

String createPayload(long timestamp, float temperature, float phValue, float AcsValueF, float Volt, float DS18B20_1, float DS18B20_2, float DS18B20_3, float humidity, float ecValue, float flowRate) 
                    {
                    if (isnan(temperature) || isinf(temperature)) temperature = 0;
                    if (isnan(phValue) || isinf(phValue)) phValue = 0;
                    if (isnan(AcsValueF) || isinf(AcsValueF)) AcsValueF = 0;
                    if (isnan(Volt) || isinf(Volt)) Volt = 0;
                    if (isnan(DS18B20_1) || isinf(DS18B20_1)) DS18B20_1 = 0;
                    if (isnan(DS18B20_2) || isinf(DS18B20_2)) DS18B20_2 = 0;
                    if (isnan(DS18B20_3) || isinf(DS18B20_3)) DS18B20_3 = 0;
                    if (isnan(humidity) || isinf(humidity)) humidity = 0;
                    if (isnan(ecValue) || isinf(ecValue)) ecValue = 0;
                    if (isnan(flowRate) || isinf(flowRate)) flowRate = 0;
    //String payload
    payload = "{\"ts\": " + String(timestamp) + 
                     " ,\"values\": {\"Temperatuur_gas\": " + String(temperature, 1) + 
                     ",\"Zuurtegraad\": " + String(phValue, 1) + 
                     ",\"Stroom\": " + String(AcsValueF, 1) + 
                     ",\"Spanning\": " + String(Volt, 1) + 
                     ",\"Temp1\": " + String(DS18B20_1, 1) + 
                     ",\"Temp2\": " + String(DS18B20_2, 1) + 
                     ",\"Temp3\": " + String(DS18B20_3, 1) + 
                     ",\"Luchtvochtigheid\": " + String(humidity, 1) + 
                     ",\"Geleidbaarheid\": " + String(ecValue, 1) + 
                     ",\"Flowsensor\": " + String(flowRate, 1) + "}}";
    return payload;
}

void generateJson(){
  float phValue;//String(pH());
  float ecValue;//String(Cond());
  float AcsValueF =27.0;//String(CurrentSensor());
  float flowRate;//String(readFlowsensor());
  float Volt = 27.22; //Placeholder for voltage
  float DS18B20_1;//String(DS18B20_1);
  float DS18B20_2;//String(DS18B20_2);
  float DS18B20_3;//String(DS18B20_3);
  float temperature;//String(dht_sensor.readTemperature());
  float humidity;//String(dht_sensor.readHumidity());
  time_t timestamp = convertToUnixTimestamp(date, time_gsm);
    String payload = 
            "{\"ts\": " + String(timestamp) + 
            ",\"values\": {\"Temperatuur_gas\": " + String(temperature) +
            ",\"Zuurtegraad\": " + String(phValue) +
            ",\"Stroom\": " + String(AcsValueF) +
            ",\"Spanning\": " + String(Volt) +
            ",\"Temp1\": " + String(DS18B20_1) +
            ",\"Temp2\": " + String(DS18B20_2) +
            ",\"Temp3\": " + String(DS18B20_3) +
            ",\"Luchtvochtigheid\": " + String(humidity) +
            ",\"Geleidbaarheid\": " + String(ecValue) +
            ",\"Flowsensor\": " + String(flowRate) + "}}";

    post_http(payload);  
}

void post2(){
  float Volt = 27.22; //Placeholder for voltage
  time_t timestamp = convertToUnixTimestamp(date, time_gsm);
  String payload = createPayload(timestamp, temperature, pH(), CurrentSensor(), Volt, DS18B20_1, DS18B20_2, DS18B20_3, humidity, Cond(), readFlowsensor());

    Serial.print("Payload in post2: ");
    Serial.println(payload);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    logMeasurement(payload);
    post_http(payload);
}

void processLine(String line){
  line.trim(); // Remove any leading or trailing whitespace  
  int colonIndex = line.indexOf(':');
  if (colonIndex == -1){
    Serial.println("Invalid line: " + line);    
    return; // Skip if no colon found
  }  
  String pinName = line.substring(0, colonIndex);
  String pinValueStr = line.substring(colonIndex + 1);  
  pinValueStr.trim(); // Remove any leading or trailing whitespace
  int pinValue = pinValueStr.toInt();
  
  
  if(pinName == "GSM_RX_PIN" || pinName == "GSM_TX_PIN" || pinName == "GSM_RST_PIN" || pinName == "Pin_MQ7" || pinName == "Pin_MQ8" || pinName == "DHT_SENSOR_PIN" || pinName == "DS18B20_PIN" || pinName == "flowSensorPin" || pinName == "buttonbigOled" || pinName == "buttonsmallOled" || pinName == "CondPin" || pinName == "CurrentPin" || pinName == "PH_PIN")
  {
    if(pinValue < 0 || pinValue > 39)
    {
      Serial.println("Invalid pin value: " + pinValueStr);
      Serial.println("For pin: " + pinName);
      return; // Validate pin number range for ESP32
    } 
    if (pinName == "GSM_RX_PIN") {
      GSM_RX_PIN2 = pinValue;
    } else if (pinName == "GSM_TX_PIN") {
      GSM_TX_PIN2 = pinValue;
    } else if (pinName == "GSM_RST_PIN") {
      GSM_RST_PIN2 = pinValue;
    } else if (pinName == "Pin_MQ7") {
      Pin_MQ72 = pinValue;
    } else if (pinName == "Pin_MQ8") {
      Pin_MQ82 = pinValue;
    } else if (pinName == "DHT_SENSOR_PIN") {
      DHT_SENSOR_PIN2 = pinValue;
    } else if (pinName == "DS18B20_PIN") {
      DS18B20_PIN2 = pinValue;
    } else if (pinName == "flowSensorPin") {
      flowSensorPin2 = pinValue;
    } else if (pinName == "buttonbigOled") {
      buttonbigOled = pinValue;
    } else if (pinName == "buttonsmallOled") {
      buttonsmallOled = pinValue;
    } else if (pinName == "CondPin") {
      CondPin = pinValue;
    } else if (pinName == "CurrentPin") {
      CurrentPin = pinValue;
    } else if (pinName == "PH_PIN") {
      PH_PIN = pinValue;
    } else if (pinName == "CS_PIN") {
      CS_PIN = pinValue;
    } 
  }
  
  if(pinName == "temperatureAmount" || pinName == "phValueAmount" || pinName == "humidityAmount" || pinName == "ecValueAmount" || pinName == "flowRateAmount" || pinName == "ds18b20Amount" || pinName == "acsAmount" || pinName == "h2Amount" || pinName == "voltAmount")
  { 
   if(pinValue < 0 || pinValue > 500)
    {
      Serial.println("Invalid pin value: " + pinValueStr);
      Serial.println("For pin: " + pinName);
      return; // Validate range for ESP32
    } else if(pinValue == 0)
    {
      Serial.println("Measurement for pin: " + pinName + " is disabled");
      Serial.println("Pin value: " + pinValueStr);
      //return; 
    } 
    if (pinName == "temperatureAmount") {
      temperatureAmount = pinValue;
    } else if (pinName == "phValueAmount") {
      phValueAmount = pinValue;
    } else if (pinName == "humidityAmount") {
      humidityAmount = pinValue;
    } else if (pinName == "ecValueAmount") {
      ecValueAmount = pinValue;
    } else if (pinName == "flowRateAmount") {
      flowRateAmount = pinValue;
    } else if (pinName == "acsAmount") {
      acsAmount = pinValue;
    } else if (pinName == "ds18b20Amount") {
      ds18b20Amount = pinValue;
    } else if (pinName == "h2Amount") {
      h2Amount = pinValue;
    } else if (pinName == "voltAmount") {
      voltAmount = pinValue;
    }
  }
  if (pinName == "mobileNumber") {
    if (pinValueStr.length() == 12 && pinValueStr.startsWith("+")) {
      mobileNumber = pinValueStr;
    } else {
      Serial.println("Invalid mobile number format: " + pinValueStr);
      Serial.println("Number length: " + pinValueStr.length());
      return;
    }
  }  
}

void read_configuration(){
  File file = SD.open("/config.txt");
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);
  
  while(file.available()){
    String line = file.readStringUntil('\n');
    processLine(line);
  }
  vTaskDelay(50 / portTICK_PERIOD_MS);
  
  file.close();
  vTaskDelay(50 / portTICK_PERIOD_MS);
  // Output the pin numbers for verification
  Serial.print("GSM_RX_PIN: ");       Serial.println(GSM_RX_PIN);
  Serial.print("GSM_TX_PIN: ");       Serial.println(GSM_TX_PIN);
  Serial.print("GSM_RST_PIN: ");      Serial.println(GSM_RST_PIN);
  Serial.print("Pin_MQ7: ");          Serial.println(Pin_MQ7);
  Serial.print("Pin_MQ8: ");          Serial.println(Pin_MQ8);
  Serial.print("DHT_SENSOR_PIN: ");   Serial.println(DHT_SENSOR_PIN);
  Serial.print("DS18B20_PIN: ");      Serial.println(DS18B20_PIN);
  Serial.print("flowSensorPin: ");    Serial.println(flowSensorPin);
  Serial.print("buttonbigOled: ");    Serial.println(buttonbigOled);
  Serial.print("buttonsmallOled: ");  Serial.println(buttonsmallOled);
  Serial.print("CondPin: ");          Serial.println(CondPin);
  Serial.print("CurrentPin: ");       Serial.println(CurrentPin);
  Serial.print("PH_PIN: ");           Serial.println(PH_PIN);
  Serial.print("CS_PIN: ");           Serial.println(CS_PIN);

  Serial.print("temperatureAmount: ");  Serial.println(temperatureAmount);
  Serial.print("phValueAmount: ");      Serial.println(phValueAmount);
  Serial.print("humidityAmount: ");     Serial.println(humidityAmount);
  Serial.print("ecValueAmount: ");      Serial.println(ecValueAmount);
  Serial.print("flowRateAmount: ");     Serial.println(flowRateAmount);
  Serial.print("acsAmount: ");          Serial.println(acsAmount);
  Serial.print("ds18b20Amount: ");      Serial.println(ds18b20Amount);
  Serial.print("h2Amount: ");           Serial.println(h2Amount);
  Serial.print("voltAmount: ");         Serial.println(voltAmount);

  Serial.print("Mobile phonenumber: ");         Serial.println(mobileNumber);
}

void setup() {
  Serial.begin(115200); // Initialize Serial for debug output
  setCpuFrequencyMhz(240);
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  SD_init();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  read_configuration();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  init_displays();
  vTaskDelay(3000 / portTICK_PERIOD_MS);
  bigOled.firstPage();
  do {
    bigOled.setFont(u8g2_font_5x7_tr); //u8g2_font_ncenB08_tr
    bigOled.drawStr(0, 20, "Giving time for SIM800L ");
    bigOled.drawStr(0, 40, "to start up.");
  } while (bigOled.nextPage());
  
  gsmSerial.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN, false); // 38400 Initialize gsmSerial with appropriate RX/TX pins
 
  vTaskDelay(3000 / portTICK_PERIOD_MS);// Give some time for the serial communication to establish
  gsmSerial.println("AT");
  vTaskDelay(100 / portTICK_PERIOD_MS); 
  gsmSerial.println("AT+IPR=115200");
  vTaskDelay(100 / portTICK_PERIOD_MS); 
  gsmSerial.println("AT&W");
  vTaskDelay(100 / portTICK_PERIOD_MS);
  //gsmSerial.println("AT&V");
  //vTaskDelay(100 / portTICK_PERIOD_MS);
  getTime();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  /*        Save obtained time   */
  rtcPrefs.begin("rtc", false); // Open the RTC preferences namespace
    time_t savedTimestamp = rtcPrefs.getUInt("timestamp", 0); // Retrieve the saved timestamp, or 0 if none

    if (savedTimestamp != 0) {
        // Set the local time using the saved timestamp
        struct timeval tv = { savedTimestamp, 0 };
        settimeofday(&tv, NULL);
    } else {
        // Obtain the time from the SIM800L and set the local time
        getTime();
        time_t unixTimestamp = convertToUnixTimestamp(date, time_gsm);
        struct timeval tv = { unixTimestamp, 0 };
        settimeofday(&tv, NULL);

        // Save the timestamp to the RTC memory
        rtcPrefs.getUInt("timestamp", unixTimestamp);
    }
  vTaskDelay(100 / portTICK_PERIOD_MS);

  gsmSerial.println("AT"); // Optional: Send an initial AT command to check if the GSM module is responsive
  initialize_gsm();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  mq7_init(MQ7);
  vTaskDelay(100 / portTICK_PERIOD_MS);  
  mq8_init(MQ8);
  vTaskDelay(100 / portTICK_PERIOD_MS);  
  dht_sensor.begin();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  printDS18B20Address();
  //vTaskDelay(100 / portTICK_PERIOD_MS);
  //SD_init();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  ph.begin();
   //Bluetooth
  SerialBT.begin("ESP32_BT"); 
  if (!SerialBT.connected()) {
    Serial.println("Failed to connect to remote device. Make sure Bluetooth is turned on!");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);

  /* Flow sensor */
  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);
  
  /* for switching screens  */
  pinMode(buttonbigOled, INPUT_PULLUP); // Set button pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(buttonbigOled), buttonInterrupt, FALLING); // Attach interrupt to the button pin
  pinMode(buttonsmallOled, INPUT_PULLUP); // Set button pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(buttonsmallOled), buttonInterrupt2, FALLING); // Attach interrupt to the button pin
  vTaskDelay(100 / portTICK_PERIOD_MS); 

  measurementQueue = xQueueCreate(queueLength, sizeof(Measurement));
  if (measurementQueue == NULL) {
    Serial.println("Failed to create queue.");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS); 
  esp_err_t esp_wifi_stop(); 

  xTaskCreatePinnedToCore(Measuring,              "Measuring",      4096,               NULL,   1,     &Task1,  1); // 20000 10000
  vTaskDelay(1000 / portTICK_PERIOD_MS);//pcName,  usStackDepth, pvParameters, uxPriority,   pvCreatedTask,  xCoreID 
  xTaskCreatePinnedToCore(sendArray,              "Send Array",     4096,               NULL,   1,     &Task2,  0); // 20000 10000
  vTaskDelay(1000 / portTICK_PERIOD_MS); 
  xTaskCreatePinnedToCore(DisplayMeasurements,   "Display Measurements",      4096,      NULL,   0,     &Task3,  0); // Core: 1
  xTaskCreatePinnedToCore(BluetoothListen,       "Listen to Bluetooth",       4096,      NULL,   0,     &Task4,  0); // Core: 1
  //xTaskCreatePinnedToCore(post,   "Post HTTP",      4096,      NULL,   1,     &Task1,  0); // Core: 1
  //vTaskDelay(1000 / portTICK_PERIOD_MS);//pcName,  usStackDepth, pvParameters, uxPriority,   pvCreatedTask,  xCoreID 
}

void loop() {  
  if (buttonPressed) {
    // Toggle state from 1 to 4
    state = (state % 4) + 1;
    Serial.print("State: ");
    Serial.println(state);
    buttonPressed = false; // Reset button press flag
  } 
  if (buttonSmallPressed) {
    // Toggle state from 1 to 4
    stateOled = (stateOled % 2) + 1;
    Serial.print("State: ");
    Serial.println(state);
    buttonSmallPressed = false; // Reset button press flag
  } 
    
  if (Serial.available()) 
    {            
        String inputString = Serial.readString(); // Read the contents of serial buffer as a string
        Serial.println();
        Serial.print("-- Input (");    
        Serial.print(inputString.length());    
        Serial.println(") --");
        
        if (inputString.startsWith("1")) { 
            Serial.println("Running initialize_gsm2()");
            initialize_gsm2();        
        }
        else if (inputString.startsWith("2")) { 
            Serial.println("Running getTimeNow()");
            getTimeNow();        
        }
        else if (inputString.startsWith("3")) { 
            Serial.println("Running generateJson()");
           generateJson();        
        } 
        else if (inputString.startsWith("4")) { 
            Serial.println("Running post2()");
           post2();        
        }
        //readGsmResponse();
    }  
    vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay to avoid overwhelming the loop    
}
/*
Backup Storing task
void storeMeasurement() {
  AllDS18B20Sensors();  
  temperature = dht_sensor.readTemperature();
  phValue = pH();
  AcsValueF = CurrentSensor();
  Volt = 27.22; // Placeholder for voltage
  DS18B20_1 = DS18B20_1;
  DS18B20_2 = DS18B20_2;
  DS18B20_3 = DS18B20_3;
  humidity = dht_sensor.readHumidity();
  ecValue = Cond();
  flowRate = readFlowsensor();

  //measurements[currentMeasurementIndex].timestamp = convertToUnixTimestamp(date, time_gsm); 
  Measurement measurement;
  if (isnan(phValue)      || isinf(phValue)) phValue          = 0;
  measurements[currentMeasurementIndex].phValue               = phValue;
  if (isnan(AcsValueF)    || isinf(AcsValueF)) AcsValueF      = 0;
  measurements[currentMeasurementIndex].AcsValueF             = AcsValueF;
  if (isnan(Volt)         || isinf(Volt)) Volt                = 0;
  measurements[currentMeasurementIndex].Volt                  = 27.22; // Placeholder for voltage
  if (isnan(DS18B20_1)    || isinf(DS18B20_1)) DS18B20_1      = 0;
  measurements[currentMeasurementIndex].DS18B20_1             = DS18B20_1;
  if (isnan(DS18B20_2)    || isinf(DS18B20_2)) DS18B20_2      = 0;
  measurements[currentMeasurementIndex].DS18B20_2             = DS18B20_2;
  if (isnan(DS18B20_3)    || isinf(DS18B20_3)) DS18B20_3      = 0;
  measurements[currentMeasurementIndex].DS18B20_3             = DS18B20_3;
  if (isnan(humidity)     || isinf(humidity)) humidity        = 0;
  measurements[currentMeasurementIndex].humidity              = humidity;
  if (isnan(ecValue)      || isinf(ecValue)) ecValue          = 0;
  measurements[currentMeasurementIndex].ecValue               = ecValue;
  if (isnan(flowRate)     || isinf(flowRate)) flowRate        = 0;
  measurements[currentMeasurementIndex].flowRate              = flowRate;
      
  currentMeasurementIndex++;
  if (currentMeasurementIndex == numMeasurements) {
    
    // Initialize a new payload structure
    payload = "{\"ts\": " + String(timestamp)  + ", \"values\": {"; //String(measurements[0].timestamp)
    // Collect arrays of values for each sensor type
    String temperatuurGasValues = "\"Temperatuur_gas\": [";
    String zuurtegradenValues = "\"Zuurtegraad\": [";
    String stroomValues = "\"Stroom\": [";
    String spanningValues = "\"Spanning\": [";
    String temp1Values = "\"Temp1\": [";
    String temp2Values = "\"Temp2\": [";
    String temp3Values = "\"Temp3\": [";
    String luchtvochtigheidValues = "\"Luchtvochtigheid\": [";
    String geleidbaarheidValues = "\"Geleidbaarheid\": [";
    String flowsensorValues = "\"Flowsensor\": [";

    for (int i = 0; i < numMeasurements; i++) {
      temperatuurGasValues += String(measurements[i].temperature);
      zuurtegradenValues += String(measurements[i].phValue);
      stroomValues += String(measurements[i].AcsValueF);
      spanningValues += String(measurements[i].Volt);
      temp1Values += String(measurements[i].DS18B20_1);
      temp2Values += String(measurements[i].DS18B20_2);
      temp3Values += String(measurements[i].DS18B20_3);
      luchtvochtigheidValues += String(measurements[i].humidity);
      geleidbaarheidValues += String(measurements[i].ecValue);
      flowsensorValues += String(measurements[i].flowRate);

      if (i < numMeasurements - 1) {
        temperatuurGasValues += ", ";
        zuurtegradenValues += ", ";
        stroomValues += ", ";
        spanningValues += ", ";
        temp1Values += ", ";
        temp2Values += ", ";
        temp3Values += ", ";
        luchtvochtigheidValues += ", ";
        geleidbaarheidValues += ", ";
        flowsensorValues += ", ";
      }
    }

    // Close the arrays
    temperatuurGasValues += "]";
    zuurtegradenValues += "]";
    stroomValues += "]";
    spanningValues += "]";
    temp1Values += "]";
    temp2Values += "]";
    temp3Values += "]";
    luchtvochtigheidValues += "]";
    geleidbaarheidValues += "]";
    flowsensorValues += "]";

    // Construct the final payload
    payload += temperatuurGasValues + ", " +
               zuurtegradenValues + ", " +
               stroomValues + ", " +
               spanningValues + ", " +
               temp1Values + ", " +
               temp2Values + ", " +
               temp3Values + ", " +
               luchtvochtigheidValues + ", " +
               geleidbaarheidValues + ", " +
               flowsensorValues + "}}";
      
    //Serial.println(payload);
    // Push the measurement to the queue
    if (xQueueSend(measurementQueue, &measurement, portMAX_DELAY) != pdPASS) {
      Serial.println("Failed to send to queue.");
    }
    currentMeasurementIndex = 0; // Reset the index for the next batch of measurements
    sendhttp=true;                // Set flag to send http request
  }
}
*/
//Backup of Measuring task
/*
void Measuring(void *parameter)
{
  Serial.println("Inside Measuring task.");
  for (;;)
  {              
  while(1){vTaskDelay(1000 / portTICK_PERIOD_MS);}
  String flowString = "Flow: " + String(readFlowsensor()) + " L/min";  
  printSmallOled(flowString);
  
  MQ7.update();
  ppmCO = MQ7.readSensor();
  Serial.print("CO value: ");
  Serial.println(ppmCO);
  //vTaskDelay(1000 / portTICK_PERIOD_MS);

  MQ8.update();
  ppmH = MQ8.readSensor();
  Serial.print("H value: ");
  Serial.println(ppmH);
  //vTaskDelay(1000 / portTICK_PERIOD_MS);
  temperature = dht_sensor.readTemperature();
  humidity = dht_sensor.readHumidity();
    
  Serial.print("Temperature: ");    Serial.println(temperature);
  Serial.print("Humidity: ");       Serial.println(humidity);

  AllDS18B20Sensors();
  //float p = pH();
  //float c = CurrentSensor();
  // Wait for a short period'
  // Update the Big display
  String tempDis      = "Temp___:" + String(temperature) + char(176) +"C";
  String humidityDis  = "Humid__:" + String(humidity)+     " %";
  String coDis        = "CO_____:" + String(ppmCO) +      " ppm";
  String flowDis      = "Flow___:" + String(readFlowsensor()) +  " L/min"; 
  String ecDis        = "EC_____:" + String(Cond()) +     " ms/cm";
  String DS18B20_1_Dis= "DStem_1:" + String(DS18B20_1) + char(176) +"C";
  String DS18B20_2_Dis= "DStem_2:" + String(DS18B20_2) + char(176) +"C";
  String DS18B20_3_Dis= "DStem_3:" + String(DS18B20_3) + char(176) +"C";
  String Current_Dis  = "Current:" + String(CurrentSensor()) + " A";
  String pH_Dis       = "pH:" + String(pH()) + " A";

  if(state==2){
    bigOled.firstPage();
    do {
      bigOled.setFont(u8g2_font_5x8_tr); //Was u8g2_font_ncenB08_tr
      bigOled.drawStr(0, 10, tempDis.c_str());
      bigOled.drawStr(0, 20, coDis.c_str());
      bigOled.drawStr(0, 30, flowDis.c_str());
      bigOled.drawStr(0, 40, humidityDis.c_str());
      bigOled.drawStr(0, 50, ecDis.c_str());
      bigOled.drawStr(0, 60, DS18B20_1_Dis.c_str());
    } while (bigOled.nextPage());
  }
  else if(state==3){
    bigOled.firstPage();
    do {
      bigOled.setFont(u8g2_font_6x10_tr); //Was u8g2_font_ncenB08_tr
      bigOled.drawStr(0, 10, tempDis.c_str());
      bigOled.drawStr(0, 20, coDis.c_str());
      bigOled.drawStr(0, 30, flowDis.c_str());
      bigOled.drawStr(0, 40, humidityDis.c_str());
      bigOled.drawStr(0, 50, ecDis.c_str());
      bigOled.drawStr(0, 60, DS18B20_1_Dis.c_str());
      bigOled.drawStr(0, 70, DS18B20_2_Dis.c_str());
      bigOled.drawStr(0, 80, Current_Dis.c_str());

    } while (bigOled.nextPage());
  }
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
*/

/*
void post_http_array(){  
  int dateTime2[6] ={1734087000, 1734087001, 1734087002, 1734087003, 1734087004, 1734087005};
  float ecValue2[6];
  float temperature_4[6];
  float humidity_4[6];
  float flowSensor2[6];
  
  // Prepare JSON payload
  String jsonPayload = "{";
  jsonPayload += "\"ts\": [" + String(dateTime2[0]);
  for (int i = 1; i < 6; i++) {
    jsonPayload += ", " + String(dateTime2[i]);
  }
  jsonPayload += "],";
  jsonPayload += "\"values\": {";
  jsonPayload += "\"Temperatuur_gas\": [" + String(flowSensor2[0]);
  for (int i = 1; i < 6; i++) {
    jsonPayload += ", " + String(flowSensor2[i]);
  }
  jsonPayload += "],";
  jsonPayload += "\"Temperatuur_vloeistof\": [" + String(flowSensor2[0]);
  for (int i = 1; i < 6; i++) {
    jsonPayload += ", " + String(flowSensor2[i]);
  }
   jsonPayload += "],";
  jsonPayload += "\"Stroom\": [" + String(ecValue2[0]);
  for (int i = 1; i < 6; i++) {
    jsonPayload += ", " + String(ecValue2[i]);
  }
  jsonPayload += "],";
  jsonPayload += "\"Ph\": [" + String(ecValue2[0]);
  for (int i = 1; i < 6; i++) {
    jsonPayload += ", " + String(ecValue2[i]);
  }
  jsonPayload += "],";
  jsonPayload += "\"Spanning\": [" + String(flowSensor2[0]);
  for (int i = 1; i < 6; i++) {
    jsonPayload += ", " + String(flowSensor2[i]);
  }
  jsonPayload += "],";
   jsonPayload += "\"Geleidbaarheid\": [" + String(flowSensor2[0]);
  for (int i = 1; i < 6; i++) {
    jsonPayload += ", " + String(flowSensor2[i]);
  }
  jsonPayload += "],";
  jsonPayload += "\"flowSensor4\": [" + String(flowSensor2[0]);
  for (int i = 1; i < 6; i++) {
    jsonPayload += ", " + String(flowSensor2[i]);
  }
  jsonPayload += "]";
  jsonPayload += "}}";




  Serial.println(String(jsonPayload));
}
*/
