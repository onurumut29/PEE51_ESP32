#include "config.h"

/*      GSM Module Setup     */
HardwareSerial gsmSerial(2); // Use UART2
#define GSM_RX_PIN 17 //16
#define GSM_TX_PIN 16 //17
#define GSM_RST_PIN 12 //Not connected
//extern String date, time_gsm;
String apn = "data.lycamobile.nl";
String apn_User = "lmnl";
String apn_Pass = "plus";
char httpapi[] = "http://jrbubuntu.ddns.net:5000/api/telemetry";
String mobileNumber = "+31614504288";

extern time_t timestamp;
uint64_t savedTimestamp;

/*      MQ-7 CO2 sensor                  */
#define Pin_MQ7 35
MQUnifiedsensor MQ7("ESP32", 5, 12, Pin_MQ7, "MQ-7");
/*      MQ-8 H2 sensor                   */
#define Pin_MQ8 32
MQUnifiedsensor MQ8("ESP32", 5, 12, Pin_MQ8, "MQ-8");

/*      DHT22 - Temperature and Humidity */
#include "DHT.h"
#define DHT_SENSOR_PIN 25 
#define DHT_SENSOR_TYPE DHT22
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
//volatile float temperature, humidity = 0.00;

/*      Setup Temperature sensor        */
const int DS18B20_PIN = 27;
extern volatile float DS18B20_1, DS18B20_2, DS18B20_3, DS18B20_4, DS18B20_5; // Initialize to a default value

/*      Setup Flowsensor                */
const int flowSensorPin = 36;
const float flowSensorCalibration = 21.00;
float flowRate, flowRate2 = 0.00;

const int flowSensor2Pin = 4;
const float flowSensorCalibration2 = 7.50;

#define PCNT_INPUT_SIG_IO1 flowSensorPin  // Pulse Input GPIO for PCNT_UNIT_0
#define PCNT_INPUT_SIG_IO2 flowSensor2Pin  // Pulse Input GPIO for PCNT_UNIT_1
#define PCNT_UNIT1         PCNT_UNIT_0
#define PCNT_UNIT2         PCNT_UNIT_1

const int FlowSensorTempPin = 26;

/*      Bluetooth                       */
BluetoothSerial SerialBT;
String message = "";
char incomingChar;

/*      SD card                         */
int CS_PIN = 5;
SemaphoreHandle_t fileMutex = NULL; //Handler for the log.txt file
/*      Switch screen                   */
int buttonbigOled = 13; // Pin connected to the button
int buttonsmallOled = 14;
extern bool buttonBigPressed, buttonSmallPressed;
U8G2_WITH_HVLINE_SPEED_OPTIMIZATION

/*          Conductivity sensor              */
int EC_PIN = 39; 

/*          Current sensor                  */
int CurrentPin = 33; 

/*          pH sensor                       */
DFRobot_PH ph;
int PH_PIN = 34; 

/*          Test for Array of JSON Objects         */
//String payload = "";
bool sendhttp = false;
// Define the queue handle
QueueHandle_t measurementQueue; // Define the queue handle
const int queueLength = 5;  //100   // Adjust the length according to your needs

//Ctrl + d for multiple cursors
int currentMeasurementIndex = 0;  
Measurement measurement[MaxMeasurements];

int h2Amount = 5; 
int coAmount = 5;
int flowRateAmount = 50;
int flowRate2Amount = 50;
int temperatureAmount = 50;
int phValueAmount = 50;
int ecValueAmount = 50;
int humidityAmount = 50;
int ds18b20Amount= 50;
int voltAmount = 200;
int acsAmount = 200;

//extern float phValue, AcsValueF, ecValue;
float Volt = 0;

const int bufferSize = 8192; //10240 
#define BUFFER_SIZE 1 //10
Measurement buffer[BUFFER_SIZE][MaxMeasurements];
int bufferI = 0;
char json[bufferSize];

int GSM_RX_PIN2, GSM_TX_PIN2, GSM_RST_PIN2, Pin_MQ72, Pin_MQ82, DHT_SENSOR_PIN2, DS18B20_PIN2, flowSensorPin2, flowSensor2Pin2;


/*          Test for Array of JSON Objects         */
void sendArray(void *parameter)
{
  Serial.println("Now running sendArray task.");
  Measurement buff;
  for (;;) 
  {    
      // Check if the queue is empty
      if (uxQueueMessagesWaiting(measurementQueue) > 0) {
        // Receive measurement from the queue
        if (xQueueReceive(measurementQueue, &buff, portMAX_DELAY) == pdPASS) {
          TickType_t startTime = xTaskGetTickCount();        
          generateJson(buff);      
          Serial.println("Message received in sendArray task: " + String(json));
          Serial.println("Size of message (In sendArray Task): " + String(strlen(json)));
          post_http(json);                
          TickType_t endTime = xTaskGetTickCount();
          TickType_t duration = endTime - startTime;          
          Serial.print("sendArrayTask duration: ");
          Serial.println(duration);
          Serial.println("");    
          if (xSemaphoreTake(fileMutex, pdMS_TO_TICKS(3000)) == pdTRUE) {      
          logMeasurement(json); 
          xSemaphoreGive(fileMutex);
          }
          else {
            Serial.println("sendArrayTask: Could not take fileMutex");
          }        
      }
    } 
    vTaskDelay(10 / portTICK_PERIOD_MS);   
  }
}

void Measuring(void *parameter) {
  Serial.println("Inside Measuring task.");
  int MaxMeasurements = std::max({temperatureAmount, phValueAmount, humidityAmount, ecValueAmount, flowRateAmount, flowRate2Amount, acsAmount, ds18b20Amount, voltAmount, h2Amount, coAmount});
  Serial.println("MaxMeasurements: " + String(MaxMeasurements));
  int mesureIndex = 0;
  for (;;) {
    
    static int temperatureCount = 0, phValueCount = 0, humidityCount = 0, ecValueCount = 0, flowRateCount = 0, flowRate2Count = 0, acsValueFCount = 0, ds18b20Count = 0, voltCount = 0, h2Count, coCount = 0;
    unsigned long start_time, end_time, duration_temperature, duration_phValue, duration_humidity, duration_ecValue, duration_flowRate, duration_flowRate2, duration_acsValueF, duration_ds18b20, duration_h2, duration_co, duration_volt;

    // Initialize durations to zero
    duration_temperature = duration_phValue = duration_humidity = duration_ecValue = duration_flowRate, duration_flowRate2 = duration_acsValueF = duration_ds18b20 = duration_h2, duration_co = duration_volt = 0;

  if ((currentMeasurementIndex % dht22_tempInterval == 0) && (temperatureCount < temperatureAmount)) {  
      start_time = micros();
      measurement[mesureIndex].temperature = dht_sensor.readTemperature();
      if (isnan(measurement[mesureIndex].temperature) || isinf(measurement[mesureIndex].temperature)) measurement[mesureIndex].temperature = 0;
      temperatureCount++;
      end_time = micros();
      duration_temperature = end_time - start_time;
  }

  if ((currentMeasurementIndex % phValueInterval == 0) && (phValueCount < phValueAmount)) {
      start_time = micros();
      measurement[mesureIndex].phValue = pH();
      if (isnan(measurement[mesureIndex].phValue) || isinf(measurement[mesureIndex].phValue)) measurement[mesureIndex].phValue = 0;
      phValueCount++;
      end_time = micros();
      duration_phValue = end_time - start_time;
  }

  if ((currentMeasurementIndex % dht22_humInterval == 0) &&  (humidityCount < humidityAmount)) {
      start_time = micros();
      measurement[mesureIndex].humidity = dht_sensor.readHumidity();
      if (isnan(measurement[mesureIndex].humidity) || isinf(measurement[mesureIndex].humidity)) measurement[mesureIndex].humidity = 0;
      humidityCount++;
      end_time = micros();
      duration_humidity = end_time - start_time;
  }

  if ((currentMeasurementIndex % ecValueInterval == 0) && (ecValueCount < ecValueAmount)) {
      start_time = micros();
      measurement[mesureIndex].ecValue = Cond();
      if (isnan(measurement[mesureIndex].ecValue) || isinf(measurement[mesureIndex].ecValue)) measurement[mesureIndex].ecValue = 0;
      ecValueCount++;
      end_time = micros();
      duration_ecValue = end_time - start_time;
  }

  if ((currentMeasurementIndex % flowRateInterval == 0) && (flowRateCount < flowRateAmount)) {
      start_time = micros();
      measurement[mesureIndex].flowRate = flowRate;
      if (isnan(measurement[mesureIndex].flowRate) || isinf(measurement[mesureIndex].flowRate)) measurement[mesureIndex].flowRate = 0;
      flowRateCount++;
      end_time = micros();
      duration_flowRate = end_time - start_time;
  }

  if ((currentMeasurementIndex % flowRate2Interval == 0) && (flowRate2Count < flowRate2Amount)) {
      start_time = micros();
      measurement[mesureIndex].flowRate2 = flowRate2;
      if (isnan(measurement[mesureIndex].flowRate2) || isinf(measurement[mesureIndex].flowRate2)) measurement[mesureIndex].flowRate2 = 0;
      flowRate2Count++;
      end_time = micros();
      duration_flowRate2 = end_time - start_time;
  }

  if ((currentMeasurementIndex % acsValueFInterval == 0) && (acsValueFCount < acsAmount)) {
      start_time = micros();
      measurement[mesureIndex].AcsValueF = CurrentSensor_quick();
      if (isnan(measurement[mesureIndex].AcsValueF) || isinf(measurement[mesureIndex].AcsValueF)) measurement[mesureIndex].AcsValueF = 0;
      acsValueFCount++;
      end_time = micros();
      duration_acsValueF = end_time - start_time;
  }

  if ((currentMeasurementIndex % ds18b20Interval == 0) && (ds18b20Count < ds18b20Amount)) {
      start_time = micros();
      AllDS18B20Sensors(measurement[mesureIndex]);
      ds18b20Count++;
      end_time = micros();
      duration_ds18b20 = end_time - start_time;
  }

  if ((currentMeasurementIndex % h2Interval == 0) && (h2Count < h2Amount)) {
      start_time = micros();
      MQ8.update();
      measurement[mesureIndex].ppmH = MQ8.readSensor();
      if (isnan(measurement[mesureIndex].ppmH) || isinf(measurement[mesureIndex].ppmH)) measurement[mesureIndex].ppmH = 0;
      h2Count++;
      end_time = micros();
      duration_h2 = end_time - start_time;
  } 

  if ((currentMeasurementIndex % coInterval == 0) && (coCount < coAmount)) {
      start_time = micros();
      MQ7.update();
      measurement[mesureIndex].ppmCO = MQ7.readSensor();
      if (isnan(measurement[mesureIndex].ppmCO) || isinf(measurement[mesureIndex].ppmCO)) measurement[mesureIndex].ppmCO = 0;
      coCount++;
      end_time = micros();
      duration_co = end_time - start_time;
  } 

  if ((currentMeasurementIndex % voltInterval == 0) && (voltCount < voltAmount)) {
      start_time = micros();
      measurement[mesureIndex].Volt = 27.22; // Placeholder for voltage
      if (isnan(measurement[mesureIndex].Volt) || isinf(measurement[mesureIndex].Volt)) measurement[mesureIndex].Volt = 0;
      voltCount++;
      end_time = micros();
      duration_volt = end_time - start_time;
  }
  measurement[mesureIndex].ts = savedTimestamp * 1000 + micros();  

/*
// Print the durations for each block
Serial.print("Temperature measurement time: "); Serial.println(duration_temperature);
Serial.print("pH measurement time: "); Serial.println(duration_phValue);
Serial.print("Humidity measurement time: "); Serial.println(duration_humidity);
Serial.print("EC value measurement time: "); Serial.println(duration_ecValue);
Serial.print("Flow rate measurement time: "); Serial.println(duration_flowRate);
Serial.print("ACS value measurement time: "); Serial.println(duration_acsValueF);
Serial.print("DS18B20 measurement time: "); Serial.println(duration_ds18b20);
Serial.print("H2 measurement time: "); Serial.println(duration_h2);
Serial.print("Voltage measurement time: "); Serial.println(duration_volt);
*/

    if (currentMeasurementIndex == MaxMeasurements) {      
      memcpy((void *) &buffer[bufferI], (void *) &measurement, sizeof(measurement));

      currentMeasurementIndex = 0;      temperatureCount = 0;
      phValueCount = 0;                 humidityCount = 0;
      ecValueCount = 0;                 flowRateCount = 0;
      acsValueFCount = 0;               ds18b20Count = 0;
      h2Count = 0;                      voltCount = 0;
      flowRate2Count = 0;               coCount = 0;
        
      if (xQueueSend(measurementQueue, &buffer[bufferI], portMAX_DELAY) != pdPASS) {
        Serial.println("Failed to send to queue.");
      }
      bufferI = (bufferI + 1) % BUFFER_SIZE;
      //printBufferInChunks(buffer, bufferIndex);
    }        
    currentMeasurementIndex++;
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // Monitor stack and heap usage
    //UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
    //size_t freeHeap = xPortGetFreeHeapSize();
    //Serial.print("MeasuringTask stack high water mark: ");
    //Serial.println(highWaterMark);
    //Serial.print("Free heap size: ");
    //Serial.println(freeHeap);
  }
}

void generateJson(Measurement measurement) {
  int bufferIndex = 0;
  Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex = snprintf(&json[bufferIndex], bufferSize, "{ \"values\": {");
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "\"ts\": [");
      // Add all timestamps     
      for (int i = 0; i < MaxMeasurements; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%llu", i > 0 ? "," : "", measurement.ts);
      }

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Temperatuur_gas\": [");
      for (int i = 0; i < temperatureAmount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.temperature);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Zuurtegraad\": [");
      for (int i = 0; i < phValueAmount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.phValue);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Stroom\": [");
      for (int i = 0; i < acsAmount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.AcsValueF);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Spanning\": [");
      for (int i = 0; i < voltAmount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.Volt);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Temp1\": [");
      for (int i = 0; i < ds18b20Amount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.DS18B20_1);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Temp2\": [");
      for (int i = 0; i < ds18b20Amount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.DS18B20_2);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Temp3\": [");
      for (int i = 0; i < ds18b20Amount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.DS18B20_3);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Temp4\": [");
      for (int i = 0; i < ds18b20Amount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.DS18B20_4);      
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Temp5\": [");
      for (int i = 0; i < ds18b20Amount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.DS18B20_5);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Luchtvochtigheid\": [");
      for (int i = 0; i < humidityAmount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.humidity);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Geleidbaarheid\": [");
      for (int i = 0; i < ecValueAmount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.ecValue);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Flowsensor\": [");
      for (int i = 0; i < flowRateAmount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.flowRate);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Flowsensor2\": [");
      for (int i = 0; i < flowRate2Amount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.flowRate2);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"CO\": [");
      for (int i = 0; i < coAmount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.ppmCO);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));

      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "], \"Waterstof\": [");
      for (int i = 0; i < h2Amount; i++) {
        bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "%s%g", i > 0 ? "," : "", measurement.ppmH);
      }
      Serial.println("BufferIndex size: " + String(bufferIndex));    
            
      bufferIndex += snprintf(&json[bufferIndex], bufferSize - bufferIndex, "]}}");      
      Serial.println("BufferIndex size: " + String(bufferIndex));
}

void BluetoothListen(void *parameter) {
    Serial.println("Inside Bluetooth task.");
    for (;;) {
        if (SerialBT.available()) {
            char incomingChar = SerialBT.read();
            if (incomingChar != '\n') {
                message += String(incomingChar);
            } else {
                message = "";
            }
            Serial.println("Received message:" + message);

            // Check if the command is to request a file
            if (message == "1" || message == "2" || message == "3" || message == "4") {
                // Acquire the mutex before accessing the file
                if (xSemaphoreTake(fileMutex, portMAX_DELAY) == pdTRUE) {
                    // File operations go here
                    if (message == "1") {
                        Serial.println("Sending measurements.txt");
                        sendFileOverBluetooth("/measurements.txt");
                        // ... (other file operations)
                    } else if (message == "2") {
                        Serial.println("Sending log.txt");
                        sendFileOverBluetooth("/log.txt");
                        // ... (other file operations)
                    } else if (message == "3") {
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
                    } else if (message == "4") {
                        Serial.println("Sending log.txt");
                        sendFileOverBluetoothInOneGo2("/log.txt");
                        // ... (other file operations)
                    }

                    // Release the mutex after the file operations are complete
                    xSemaphoreGive(fileMutex);
                } else {
                    Serial.println("Failed to acquire file mutex");
                }
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
    MQ7.update(); 
    measurement[1].ppmCO = MQ7.readSensor();
    float current_ACS = 0.01;
    String flowDis      = "Flow: " + String(flowRate) +  " L/min";                     
    //String flowDis      = "Flow: " + String(0.00) +  " L/min";              
    String flowDis2     = "Flow2: " + String(flowRate2) +  " L/min";               
    String tempDis      = "Temp: " + String(measurement[1].temperature) + " °C";
    String humidityDis  = "Hum_: " + String(measurement[1].humidity)    + " %";
    String coDis        = "CO__: " + String(measurement[1].ppmCO)       + " ppm";
    String h2Dis        = "H2__: " + String(measurement[1].ppmH)        + " ppm";
    String DS18B20_1_Dis= "DS_1: " + String(measurement[1].DS18B20_1)   + " °C";
    String DS18B20_2_Dis= "DS_2: " + String(measurement[1].DS18B20_2)   + " °C";
    String DS18B20_3_Dis= "DS_3: " + String(measurement[1].DS18B20_3)   + " °C";
    String DS18B20_4_Dis= "DS_4: " + String(measurement[1].DS18B20_4)   + " °C";
    String DS18B20_5_Dis= "DS_5: " + String(measurement[1].DS18B20_5)   + " °C";
    String Current_Dis  = "Amp_: " + String(current_ACS)             + " A";              
    String pH_Dis       = "pH__: " + String(pH())                    + "";                                 
    String VoltDis      = "Volt: " + String(Volt)                    + " V";        
    String ecDis        = "EC__: " + String(Cond())                  + " ms/cm";                     
    
   /*
    String tempDis      = "Temp: "  + String(100.99) + " °C";
    String humidityDis  = "Hum_: " + String(100.00) + " %";
    String coDis        = "CO__: " + String(100)    + " ppm";
    String h2Dis        = "H2__: " + String(999)    + " ppm";
    String flowDis      = "Flow: " + String((810.00)) + " L/min";              
    String ecDis        = "EC: "   + String(100.00) + " mS/cm";                     
    String DS18B20_1_Dis= "DS_1: " + String(100.99) + " °C";
    String DS18B20_2_Dis= "DS_2: " + String(100.99) + " °C";
    String DS18B20_3_Dis= "DS_3: " + String(100.99) + " °C";
    String DS18B20_4_Dis= "DS_4: " + String(100.99) + " °C";
    String DS18B20_5_Dis= "DS_5: " + String(100.99) + " °C";
    String Current_Dis  = "Amp_: " + String(100.99) + " A";              
    String pH_Dis       = "pH__: " + String(20.0)   + "";                                 
    String VoltDis      = "Volt: " + String(9999.99)+ " V";                                 
    */

    //#1 u8g2_font_micro_mr
    //#2 u8g2_font_3x5im_mr
    //For horizontal display u8g2_font_tinytim_tr
    if(stateBigOled==2){
      bigOled.firstPage();
    do {
      bigOled.setFont(u8g2_font_micro_mr); //Was u8g2_font_ncenB08_tr
      bigOled.setDisplayRotation(U8G2_R1);
      bigOled.drawStr(0, 8, h2Dis.c_str());
      bigOled.drawStr(0, 16, VoltDis.c_str());
      bigOled.drawStr(0, 24, Current_Dis.c_str());
      bigOled.drawStr(0, 32, pH_Dis.c_str());
      bigOled.drawStr(0, 40, ecDis.c_str());
      bigOled.drawStr(0, 48, flowDis.c_str());
      bigOled.drawStr(0, 56, tempDis.c_str());
      bigOled.drawStr(0, 64, DS18B20_1_Dis.c_str());
      bigOled.drawStr(0, 72, DS18B20_2_Dis.c_str());
      bigOled.drawStr(0, 80, DS18B20_3_Dis.c_str());
      bigOled.drawStr(0, 88, DS18B20_4_Dis.c_str());
      bigOled.drawStr(0, 96, humidityDis.c_str());
      bigOled.drawStr(0, 104, flowDis2.c_str());
      bigOled.drawStr(0, 112, coDis.c_str());
    } while (bigOled.nextPage());
    }   
    if(stateBigOled==3){
      bigOled.firstPage();
    do {
      bigOled.setFont(u8g2_font_3x5im_mr); //Was u8g2_font_ncenB08_tr
      bigOled.setDisplayRotation(U8G2_R1);
      bigOled.drawStr(0, 8, VoltDis.c_str());
      bigOled.drawStr(0, 16, Current_Dis.c_str());
      bigOled.drawStr(0, 24, h2Dis.c_str());
      bigOled.drawStr(0, 32, ecDis.c_str());
      bigOled.drawStr(0, 40, pH_Dis.c_str());
      bigOled.drawStr(0, 48, flowDis.c_str());
      bigOled.drawStr(0, 56, humidityDis.c_str());
      bigOled.drawStr(0, 64, tempDis.c_str());
      bigOled.drawStr(0, 72, DS18B20_1_Dis.c_str());
      bigOled.drawStr(0, 80, DS18B20_2_Dis.c_str());
      bigOled.drawStr(0, 88, DS18B20_3_Dis.c_str());
      bigOled.drawStr(0, 96, DS18B20_4_Dis.c_str());
      bigOled.drawStr(0, 104, DS18B20_5_Dis.c_str());
      bigOled.drawStr(0, 112, coDis.c_str());      
      bigOled.drawStr(0, 120, flowDis2.c_str());
    } while (bigOled.nextPage());
    } 
    if(stateBigOled==4){
      bigOled.firstPage();
    do {
      bigOled.setFont(u8g2_font_3x5im_mr); //Was u8g2_font_ncenB08_tr
      bigOled.setDisplayRotation(U8G2_R0);
      bigOled.drawStr(0, 8, VoltDis.c_str());
      bigOled.drawStr(65, 8, Current_Dis.c_str());
      bigOled.drawStr(0, 16, h2Dis.c_str());
      bigOled.drawStr(0, 24, ecDis.c_str());
      bigOled.drawStr(0, 32, tempDis.c_str());      
      bigOled.drawStr(65, 32, DS18B20_1_Dis.c_str());
      bigOled.drawStr(0, 40, DS18B20_2_Dis.c_str());
      bigOled.drawStr(65, 40, DS18B20_3_Dis.c_str());
      bigOled.drawStr(0, 48, DS18B20_4_Dis.c_str());
      bigOled.drawStr(65, 48, DS18B20_4_Dis.c_str());
      bigOled.drawStr(0, 56, pH_Dis.c_str());
      bigOled.drawStr(65, 56, humidityDis.c_str());
      bigOled.drawStr(0, 64, flowDis.c_str());
      bigOled.drawStr(70, 64, flowDis2.c_str());
    } while (bigOled.nextPage());
    } 
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  }

void counting(void *parameter){
  for (;;) {
    int16_t count1 = 0, count2 = 0;
    static int16_t last_count1 = 0, last_count2 = 0;
    static uint32_t last_time = 0;
    uint32_t current_time = millis();

    pcnt_get_counter_value(PCNT_UNIT1, &count1);
    pcnt_get_counter_value(PCNT_UNIT2, &count2);

    // Calculate frequency for PCNT_UNIT1
    uint32_t elapsed_time = current_time - last_time; // Time in milliseconds
    if (elapsed_time > 0) {
        int16_t pulses1 = count1 - last_count1;
        float frequency1 = (float)pulses1 / (elapsed_time / 1000.0); // Frequency in Hz
        //Serial.printf("Frequency on GPIO %d: %.2f Hz\n", PCNT_INPUT_SIG_IO1, frequency1);
        flowRate = frequency1/21.00;
        // Update last count for unit 1
        last_count1 = count1;
    }

    // Calculate frequency for PCNT_UNIT2
    if (elapsed_time > 0) {
        int16_t pulses2 = count2 - last_count2;
        float frequency2 = (float)pulses2 / (elapsed_time / 1000.0); // Frequency in Hz
        //Serial.printf("Frequency on GPIO %d: %.2f Hz\n", PCNT_INPUT_SIG_IO2, frequency2);
        flowRate2 = frequency2/7.50;
        // Update last count for unit 2
        last_count2 = count2;
    }

    // Update last time
    last_time = current_time;

    // Add a small delay to avoid flooding the serial output
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

TaskHandle_t Task1, Task2, Task3, Task4, Task5 = NULL;

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
  
  
  if(pinName == "GSM_RX_PIN" || pinName == "GSM_TX_PIN" || pinName == "GSM_RST_PIN" || pinName == "Pin_MQ7" || pinName == "Pin_MQ8" || pinName == "DHT_SENSOR_PIN" || pinName == "DS18B20_PIN" || pinName == "flowSensorPin" || pinName == "flowSensor2Pin" || pinName == "buttonbigOled" || pinName == "buttonsmallOled" || pinName == "EC_PIN" || pinName == "CurrentPin" || pinName == "PH_PIN")
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
    } else if (pinName == "flowSensor2Pin") {
      flowSensor2Pin2 = pinValue;
    } else if (pinName == "buttonbigOled") {
      buttonbigOled = pinValue;
    } else if (pinName == "buttonsmallOled") {
      buttonsmallOled = pinValue;
    } else if (pinName == "EC_PIN") {
      EC_PIN = pinValue;
    } else if (pinName == "CurrentPin") {
      CurrentPin = pinValue;
    } else if (pinName == "PH_PIN") {
      PH_PIN = pinValue;
    } else if (pinName == "CS_PIN") {
      CS_PIN = pinValue;
    } 
  }
  
  if(pinName == "temperatureAmount" || pinName == "phValueAmount" || pinName == "humidityAmount" || pinName == "ecValueAmount" || pinName == "flowRateAmount" || pinName == "flowRate2Amount" || pinName == "ds18b20Amount" || pinName == "acsAmount" || pinName == "h2Amount" || pinName == "voltAmount")
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
    }  else if (pinName == "flowRate2Amount") {
      flowRate2Amount = pinValue;
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
  Serial.print("flowSensor2Pin: ");    Serial.println(flowSensor2Pin);
  Serial.print("buttonbigOled: ");    Serial.println(buttonbigOled);
  Serial.print("buttonsmallOled: ");  Serial.println(buttonsmallOled);
  Serial.print("EC_PIN: ");          Serial.println(EC_PIN);
  Serial.print("CurrentPin: ");       Serial.println(CurrentPin);
  Serial.print("PH_PIN: ");           Serial.println(PH_PIN);
  Serial.print("CS_PIN: ");           Serial.println(CS_PIN);

  Serial.print("temperatureAmount: ");  Serial.println(temperatureAmount);
  Serial.print("phValueAmount: ");      Serial.println(phValueAmount);
  Serial.print("humidityAmount: ");     Serial.println(humidityAmount);
  Serial.print("ecValueAmount: ");      Serial.println(ecValueAmount);
  Serial.print("flowRateAmount: ");     Serial.println(flowRateAmount);
  Serial.print("flowRate2Amount: ");     Serial.println(flowRate2Amount);
  Serial.print("acsAmount: ");          Serial.println(acsAmount);
  Serial.print("ds18b20Amount: ");      Serial.println(ds18b20Amount);
  Serial.print("h2Amount: ");           Serial.println(h2Amount);
  Serial.print("voltAmount: ");         Serial.println(voltAmount);

  Serial.print("Mobile phonenumber: ");         Serial.println(mobileNumber);

  temperatureAmount = temperatureAmount;
  phValueAmount = phValueAmount;
  humidityAmount = humidityAmount;
  ecValueAmount = ecValueAmount;
  flowRateAmount = flowRateAmount;
  flowRate2Amount = flowRate2Amount;
  acsAmount = acsAmount;
  ds18b20Amount = ds18b20Amount;
  h2Amount = h2Amount;
  voltAmount = voltAmount;
  //const int MaxMeasurements = std::max({temperatureAmount, phValueAmount, humidityAmount, ecValueAmount, flowRateAmount, flowRate2Amount, acsAmount, ds18b20Amount, h2Amount, voltAmount});
}

void setup() {
  Serial.begin(115200); // Initialize Serial for debug output
  setCpuFrequencyMhz(240);
  vTaskDelay(10 / portTICK_PERIOD_MS);
  SD_init();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  if (SD.begin(CS_PIN)) {
         read_configuration();
    }
  vTaskDelay(100 / portTICK_PERIOD_MS);
  init_displays();
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  nvs_flash_init();  

  bigOled.firstPage();
  do {
    bigOled.setFont(u8g2_font_tinytim_tr); //u8g2_font_ncenB08_tr
    bigOled.drawStr(0, 20, "Giving time");
    bigOled.drawStr(0, 40, "for SIM800L ");
    bigOled.drawStr(0, 60, "to start up.");
  } while (bigOled.nextPage());
  
  gsmSerial.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN, false); // 38400 Initialize gsmSerial with appropriate RX/TX pins
 
  vTaskDelay(1000 / portTICK_PERIOD_MS);// Give some time for the serial communication to establish
  gsmSerial.println("AT");
  vTaskDelay(100 / portTICK_PERIOD_MS); 
  gsmSerial.println("AT+IPR=115200");
  vTaskDelay(100 / portTICK_PERIOD_MS); 
  gsmSerial.println("AT&W");  //gsmSerial.println("AT&V"); //Show saved GSM settings
  vTaskDelay(100 / portTICK_PERIOD_MS);
  stateBigOled = 1; 
  getTime();
  savedTimestamp = getSavedTimestamp();  
  Serial.println("Saved Timestamp: " + String(savedTimestamp));
  vTaskDelay(100 / portTICK_PERIOD_MS);
  gsmSerial.println("AT"); // Optional: Send an initial AT command to check if the GSM module is responsive
  vTaskDelay(10 / portTICK_PERIOD_MS);
  initialize_gsm();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  mq7_init(MQ7);
  vTaskDelay(100 / portTICK_PERIOD_MS);  
  mq8_init(MQ8);
  vTaskDelay(100 / portTICK_PERIOD_MS);  
  dht_sensor.begin();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  printDS18B20Address();
  vTaskDelay(100 / portTICK_PERIOD_MS);
  ph.begin();


//Conductivity sensor
  EEPROM.begin(32); //needed EEPROM.begin to store calibration k in eeprom
	ec.begin();       //by default lib store calibration k since 10 change it by set ec.begin(30); to start from 30


//Bluetooth
  SerialBT.begin("ESP32_BT"); 
  if (!SerialBT.connected()) {
    Serial.println("Failed to connect to remote device. Make sure Bluetooth is turned on!");
  }
  vTaskDelay(100 / portTICK_PERIOD_MS);

/* Flow sensor */
  pcnt_example_init(PCNT_UNIT1, PCNT_INPUT_SIG_IO1);
  pcnt_example_init(PCNT_UNIT2, PCNT_INPUT_SIG_IO2);

/* for switching screens  */
  pinMode(buttonbigOled, INPUT_PULLUP); // Set button pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(buttonbigOled), buttonInterrupt_bigOled, FALLING); // Attach interrupt to the button pin
  pinMode(buttonsmallOled, INPUT_PULLUP); // Set button pin as input with pull-up resistor
  attachInterrupt(digitalPinToInterrupt(buttonsmallOled), buttonInterrupt_smallOled, FALLING); // Attach interrupt to the button pin
  pinMode(CurrentPin, INPUT);
  interrupts();

  measurementQueue = xQueueCreate(queueLength, sizeof(Measurement));
  if (measurementQueue == NULL) {
    Serial.println("Failed to create queue.");
  }
  fileMutex = xSemaphoreCreateMutex();
    if (fileMutex == NULL) {
        Serial.println("Failed to create file mutex");
        // Handle the error appropriately
    }
  
  vTaskDelay(100 / portTICK_PERIOD_MS); 
  esp_err_t esp_wifi_stop(); 

  analogReadResolution(12); 

  xTaskCreatePinnedToCore(Measuring,              "Measuring",      8192,               NULL,   1,     &Task1,  1); // 4096
  vTaskDelay(100 / portTICK_PERIOD_MS);//pcName,  usStackDepth, pvParameters, uxPriority,   pvCreatedTask,  xCoreID 
  xTaskCreatePinnedToCore(sendArray,              "Send Array",     8192,               NULL,   1,     &Task2,  0); // 20000 10000
  vTaskDelay(100 / portTICK_PERIOD_MS); 
  xTaskCreatePinnedToCore(DisplayMeasurements,   "Display Measurements",      4096,     NULL,   0,     &Task3,  0); // Core: 1
  xTaskCreatePinnedToCore(BluetoothListen,       "Listen to Bluetooth",       4096,     NULL,   0,     &Task4,  0); // Core: 1
  vTaskDelay(100 / portTICK_PERIOD_MS); 
  xTaskCreatePinnedToCore(counting,              "Count pulses",    4096,               NULL,   1,     &Task5,  1); // Core: 1  
  vTaskDelay(100 / portTICK_PERIOD_MS); 
//xTaskCreatePinnedToCore(post,   "Post HTTP",      4096,      NULL,   1,     &Task1,  0); // Core: 1
  //vTaskDelay(1000 / portTICK_PERIOD_MS);//pcName,  usStackDepth, pvParameters, uxPriority,   pvCreatedTask,  xCoreID 
}

void loop() {  
  if (buttonBigPressed) {
    // Toggle stateBigOled from 1 to 4
    stateBigOled = (stateBigOled % 4) + 1;
    Serial.print("stateBigOled: ");
    Serial.println(stateBigOled);
    buttonBigPressed = false; // Reset button press flag
  } 
  if (buttonSmallPressed) {
    // Toggle state from 1 to 4
    stateOled = (stateOled % 2) + 1;
    Serial.print("stateOled: ");
    Serial.println(stateOled);
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
            Serial.println("No function set.");       
        } 
        //readGsmResponse();
    }  
    vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay to avoid overwhelming the loop    
}
