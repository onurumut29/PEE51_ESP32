#include "config.h"

/*      DS18B20 sensor            */
OneWire oneWire(DS18B20_PIN);         // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors(&oneWire);  // Pass our oneWire reference to Dallas Temperature. 
int numberOfDevices;                  // Number of temperature devices found
DeviceAddress tempDeviceAddress;      // We'll use this variable to store a found device address
volatile float DS18B20_1, DS18B20_2,DS18B20_3, DS18B20_4, DS18B20_5 = 0.0; // Initialize to a default value

//DS18B20 Find and print Address
void printDS18B20Address() {
  sensors.begin();
  numberOfDevices = sensors.getDeviceCount();  
  // Loop through each device, print out address
  for (int i = 0; i < numberOfDevices; i++) {
    // Search the wire for address
    if (sensors.getAddress(tempDeviceAddress, i)) {
      Serial.print("Found device ");
      Serial.print(i, DEC);
      Serial.print(" with address: ");      
      // Print the address
      for (uint8_t j = 0; j < 8; j++) {
          if (tempDeviceAddress[j] < 16) Serial.print("0");
          Serial.print(tempDeviceAddress[j], HEX);
        }
        Serial.println();
      } else {
        Serial.print("Found ghost device at ");
        Serial.print(i, DEC);
        Serial.print(" but could not detect address. Check power and cabling");
        Serial.println();
      }
    }
}
// Loop through each device, print out DS18B20 temperature data
void AllDS18B20Sensors(Measurement& measurement) {
  sensors.requestTemperatures(); // Send the command to get temperatures
  int numberOfDevices = sensors.getDeviceCount();
  for(int i = 0; i < numberOfDevices; i++) {
    // Search the wire for address
    if(sensors.getAddress(tempDeviceAddress, i)) {
        // Print the data
        float tempC = sensors.getTempC(tempDeviceAddress);
        // Assign readings to variables Tempc1 and Tempc2
        if(i == 0) {
            measurement.DS18B20_1 = tempC;
        } else if(i == 1) {
            //DS18B20_2 = tempC;
            measurement.DS18B20_2 = tempC;
        }
        else if(i == 2) {
            //DS18B20_3 = tempC; 
            measurement.DS18B20_3 = tempC; 
        }
        else if(i == 3) {
            //DS18B20_4 = tempC;
            measurement.DS18B20_4 = tempC;
        }
        else if(i == 4) {
            //DS18B20_5 = tempC;
            measurement.DS18B20_5 = tempC;
        }
    }
  }  
  sensors.setResolution(9);
}

/*              Setup Flowsensor    */
long currentMillis_flowsensor, previousMillis_flowsensor = 0;
int interval = 200; //50 is de flicker extreem
float calibrationFactor = 21.0;
volatile unsigned long pulseCount;
unsigned long pulse1Sec = 0;
float flowRate, flowSensorValue = 0.0;
unsigned int flowMilliLitres;

void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}

float readFlowsensor(){
  currentMillis_flowsensor = millis();
  if (currentMillis_flowsensor - previousMillis_flowsensor > interval) {
    
    pulse1Sec = pulseCount;
    pulseCount = 0;
    flowRate = ((1000.0 / interval) * pulse1Sec) / calibrationFactor;
    previousMillis_flowsensor += interval;
  
    flowMilliLitres = (flowRate / 60) * 1000;

    //Serial.print("Flow rate: ");
    //Serial.print(flowRate, 3);  
    //Serial.print("L/min");
    //Serial.print("\t"); 

    //Serial.print("Flow rate ml/min: ");
    //Serial.print(flowMilliLitres, 3);  
    //Serial.print("mL/min");
    //Serial.print("\t");    
    //Serial.println("");
  }
    return flowRate;
}

float readFlowSensorTemperature(int FlowSensorTempPin) {
  // Read the voltage at the ADC pin
  int adcValue = analogRead(FlowSensorTempPin);
  float voltage = adcValue * (3.3 / 4096.0);

  // Calculate the resistance of the NTC sensor
  float resistance = (10000 * voltage) / (3.3 - voltage);

  // Calculate the temperature in degrees Celsius
  float temperature = 1 / (log(resistance / 10000) / 3950 + 1 / 298.15) - 273.15;

  return temperature;
}
//Read water quality
float readFlowSensorTDS(int FlowSensorTDSPin, float voltageMultiplier, float voltageOffset, float tdsCalibrationFactor) {
  // Read the voltage at the ADC pin
  int adcValue = analogRead(FlowSensorTDSPin);
  float voltage = adcValue * (3.3 / 4096.0);

  // Apply voltage multiplier and offset
  voltage = voltage * voltageMultiplier + voltageOffset;

  // Calculate the TDS value
  float tdsValue = voltage * tdsCalibrationFactor;

  return tdsValue;
}
/*      Switching screens           */
//extern volatile int stateBigOled, stateOled; //  state 1 = GSM screen, state 2 = Oled screen
volatile bool buttonPressed, buttonSmallPressed = false;

/*      MQ-7 MQ-8 sensor            */
float RatioMQ7CleanAir = 27.5;
float RatioMQ8CleanAir = 70.0;
void mq7_init(MQUnifiedsensor& MQ7)
{
     vTaskDelay(500 / portTICK_PERIOD_MS);
    // CO
    MQ7.setRegressionMethod(1); //_PPM =  a*ratio^b
    // MQ7.setA(521853); MQ7.setB(-3.821); // Configurate the ecuation values to get Benzene concentration
    MQ7.setA(99.042);
    MQ7.setB(-1.518); // Configure the equation to calculate CO concentration value
    MQ7.init();
    vTaskDelay(5 / portTICK_PERIOD_MS);

    float calcR0 = 0;
    for (int i = 1; i <= 10; i++)
    {
        MQ7.update(); // Update data, the arduino will be read the voltage on the analog pin
        calcR0 += MQ7.calibrate(RatioMQ7CleanAir);
        Serial.print(".");
    }
    MQ7.setR0(calcR0 / 10);
    Serial.println("Done calculating R0 for MQ7!.");
    /*
      //If the RL value is different from 10K please assign your RL value with the following method:
      MQ7.setRL(9.87);
    */
    if (isinf(calcR0))
    {
        Serial.println("MQ7 Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    } // while(1);
    if (calcR0 == 0)
    {
        Serial.println("MQ7 Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    } // while(1);
    /*****************************  MQ CAlibration ********************************************/
    Serial.println("MQ7 initialized!");    
    MQ7.serialDebug(true);
}

void mq8_init(MQUnifiedsensor& MQ8)
{
     vTaskDelay(500 / portTICK_PERIOD_MS);
    // Hydrogen
    MQ8.setRegressionMethod(1); //_PPM =  a*ratio^b
    MQ8.setA(976.97);
    MQ8.setB(-0.688); // Configure the equation to to calculate H2 concentration
    MQ8.init();
    Serial.print("Calibrating MQ8 please wait.");
    float calcR0_MQ8 = 0;
    for (int i = 1; i <= 10; i++)
    {
        MQ8.update(); // Update data, the arduino will read the voltage from the analog pin
        calcR0_MQ8 += MQ8.calibrate(RatioMQ8CleanAir);
        Serial.print(".");
    }
    MQ8.setR0(calcR0_MQ8 / 10);
    Serial.println("R0 for MQ8 calculation done!.");

    if (isinf(calcR0_MQ8))
    {
        Serial.println("MQ8 Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    }
    if (calcR0_MQ8 == 0)
    {
        Serial.println("MQ8 Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    }
    /*****************************  MQ CAlibration ********************************************/
    Serial.println("MQ8 initialized!");    
    MQ8.serialDebug(true);
}

/*      Display Setup               */
U8G2_SH1106_128X64_NONAME_1_HW_I2C bigOled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);
U8G2_SSD1306_128X64_NONAME_1_HW_I2C smallOled(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 22, /* data=*/ 21);

//U8G2_SH1106_128X64_NONAME_1_HW_I2C
//For GSMSerial output on OLED
#define U8LOG_WIDTH 20 //25
#define U8LOG_HEIGHT 6 //8+
uint8_t u8log_buffer[U8LOG_WIDTH*U8LOG_HEIGHT];
U8G2LOG u8g2log;

void init_displays(){  
  smallOled.setI2CAddress(0x3D * 2);
  bigOled.setI2CAddress(0x3C * 2);
  Wire.setClock(100000); //400000

  smallOled.begin(); //0x78 (0x3D)
  bigOled.begin();  
  bigOled.clearBuffer();
  //was u8g2_font_ncenB08_tr 8x16 pixels 
  bigOled.setFont(u8g2_font_6x12_mf );	// set the font for the terminal window
  u8g2log.begin(bigOled, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer);
  u8g2log.setLineHeightOffset(2);	// set extra space between lines in pixel, this can be negative
  u8g2log.setRedrawMode(1);		// 0: Update screen with newline, 1: Update screen for every char 
  Serial.println("Displays initialized!");    
  //bigOled.firstPage();
  //do {
  //  bigOled.setFont(u8g2_font_5x7_tr); //u8g2_font_ncenB08_tr
  //  bigOled.drawStr(0, 20, "Hello ");
  //  bigOled.drawStr(0, 40, "World.");
  //} while (bigOled.nextPage());  
  //Serial.print("Big Display height: ");
  //Serial.println(bigOled.getDisplayHeight());
  //Serial.print("Display Width: ");
  //Serial.println(bigOled.getDisplayHeight());
  //Serial.print("Small Display height: ");
  //Serial.println(bigOled.getDisplayHeight());
  //Serial.print("Display Width: ");
  //Serial.println(bigOled.getDisplayHeight());
}

void printSmallOled(String x){
   smallOled.firstPage();
  do {
    smallOled.setFont(u8g2_font_ncenB08_tr);
    smallOled.drawStr(0, 20, "Flow: ");
    smallOled.drawStr(0, 40, x.c_str());
  } while (smallOled.nextPage()); 
}

void printBigOled(String x){
  bigOled.firstPage();
  do {
    bigOled.setFont(u8g2_font_ncenB08_tr);
    bigOled.drawStr(0, 20, "Flow: ");
    bigOled.drawStr(0, 40, x.c_str());
  } while (bigOled.nextPage()); 
}

/*              Setup Currentsensor    */
//Prints current in mA
float AcsValue, AvgAcs, AcsValueF, Samples =0.0;   //Sensor leesspanning | - | gem. leesspanning | Stroom

float CurrentSensor_quick(){
  for (int x = 0; x < 100; x++){  //150 samples
  AcsValue = analogRead(CurrentPin);      //uitlezen van de sensor   
  Samples = Samples + AcsValue;   //samples bij elkaar zetten
  vTaskDelay(1 / portTICK_PERIOD_MS);                     
}
AvgAcs=Samples/100.0;             //De gemiddeldes bij elkaar zetten

//((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
//2.5 is offset(I assumed that arduino is working on 5v so the viout at no current comes
//out to be 2.5 which is out offset. If your arduino is working on different voltage than 
//you must change the offset according to the input voltage)
//0.185v(185mV) is rise in output voltage when 1A current flows at input

//AcsValueF = (2.5 - (AvgAcs * (3.3 / 1024.0)) )/0.185;
int R1, R2 = 1000; //Voltage devider of the current sensor 
AcsValueF = (((AvgAcs * (3.3 / 1024.0)) * (R1+R2)/R2) -2.5)/1000; //Formula for voltage divider is inverted to compensate for itself
//Serial.println(AcsValueF);//Print the read current on Serial monitor
return AcsValueF;
}


/*      Conductivity Sensor   */
volatile float voltage,ecValue,temperature_ec = 25;  // variable for storing the potentiometer value
float Cond(){
  ecValue = 0;
  voltage = analogRead(CondPin);
  ecValue = 1000*voltage/RES2/ECREF;
  ecValue = ecValue / (1.0+0.0185*(temperature_ec-25.0));  //temperature compensation
  //Serial.print(ecValue,2);  Serial.println("ms/cm");
  return ecValue;
}

/*      pH Sensor             */
float voltage_pH, phValue, temperature_pH = 25;
float pH(){
  static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    
    if (currentTime - lastTime >= 1000) {  // tijdsinterval: 1s
        lastTime = currentTime;
        
        temperature_pH = readTemperature();  // Lees de temperatuur om temperatuurcompensatie uit te voeren
        voltage = analogRead(CondPin) / 1024.0 * 5000;  // Lees de spanning
        phValue = ph.readPH(voltage, temperature_pH);  // Converteer spanning naar pH met temperatuurcompensatie
        
        //Serial.print("Temperature: ");
        //Serial.println(temperature_pH, 1);
        //Serial.print("°C  pH: ");
        //Serial.println(phValue, 2);
        
        ph.calibration(voltage, temperature_pH);  // Kalibratieproces via Seriële CMD
    }
  return phValue;
}

float readTemperature() {
    // Voeg hier je code toe om de temperatuur van je temperatuursensor te krijgen
    // Bijvoorbeeld, als je een LM35 gebruikt:
    // int rawValue = analogRead(TEMP_SENSOR_PIN);
    // float millivolts = (rawValue / 1024.0) * 5000;
    // return millivolts / 10;
    // kan nog standaard waarde zoals 25 graden

    // Als placeholder, retourneer een constante waarde:
    return 25.0;
}
/*      SD card       */
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("Failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.name(), levels -1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if(!file){
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)){
      Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void SD_init(){
  if (!SD.begin(CS_PIN)) {
        Serial.println("SD Card initialization failed!");
        //Light up RED LED
    }
    Serial.println("SD Card initialized.");
  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    //Light up RED LED
    //return;
  }

  Serial.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);

  File file = SD.open("/log.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/log.txt", "Reading ID, Date, Hour, Temperature \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();

  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}

/*


void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\n", path);
  if(fs.remove(path)){
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if(file){
    len = file.size();
    size_t flen = len;
    start = millis();
    while(len){
      size_t toRead = len;
      if(toRead > 512){
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for(i=0; i<2048; i++){
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}


void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}
*/
/*      Bluetooth Setup   */
void sendFileOverBluetooth(const char* path) {    
        File file = SD.open(path, FILE_READ);
        if (!file) {
            Serial.println("Failed to open file for reading");
            return;
        }
        while (file.available()) {
            SerialBT.write(file.read());
        }
        file.close();
        Serial.println("File sent over Bluetooth");    
}

void logMeasurement(String s) {   
        File dataFile = SD.open("/log.txt", FILE_APPEND);
        if (dataFile) {
            dataFile.println(s);
            dataFile.close();
            Serial.println("Data written to file.");
        } else {
            Serial.println("Error opening file for writing.");
        }
}

void readFileAndSendOverBluetooth(fs::FS &fs, const char *path) {
  Serial.printf("Reading file: %s\n", path);
  File file = fs.open(path, FILE_READ);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  while (file.available()) {
    SerialBT.write(file.read());
  }

  file.close();
}
void sendFileOverBluetoothInOneGo(const char* path) {
    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading: " + String(path));
        return;
    }

    size_t fileSize = file.size();
    uint8_t* buffer = new uint8_t[fileSize];

    if (buffer == nullptr) {
        Serial.println("Failed to allocate memory for buffer");
        file.close();
        return;
    }

    size_t bytesRead = file.read(buffer, fileSize);
    if (bytesRead != fileSize) {
        Serial.println("Failed to read file: " + String(path) + ", bytesRead: " + String(bytesRead) + ", fileSize: " + String(fileSize));
        delete[] buffer;
        file.close();
        return;
    }

    file.close();

    SerialBT.write(buffer, fileSize);
    delete[] buffer;

    Serial.println("File sent over Bluetooth: " + String(path));
}





void sendFileOverBluetoothInOneGo2(const char* path) {
    File file = SD.open(path, FILE_READ);
    if (!file) {
        Serial.println("Failed to open file for reading");
        return;
    }

    size_t fileSize = file.size();
    uint8_t* buffer = new uint8_t[fileSize];

    if (file.read(buffer, fileSize) != fileSize) {
        Serial.println("Failed to read file");
        delete[] buffer;
        file.close();
        return;
    }

    file.close();

    SerialBT.write(buffer, fileSize);
    delete[] buffer;

    Serial.println("File sent over Bluetooth");
}


void buttonInterrupt() {
  buttonPressed = true; // Set button press flag
}
void buttonInterrupt2() {
  buttonSmallPressed = true; // Set button press flag
}