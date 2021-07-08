/*
 * Backup Emergency Recovery Transmitter
 * HW Version - 3.0
 * FW Version - 1.5
 * Matthew E. Nelson
 */

/*
 * Some code based on the following Libraries
 * - Sparkfun GNSS Library
 * - Iridium library
 * - Adafruit Arcada
 * 
 */

/********HARDWARE REQUIREMENTS*****************
 * - Adafruit Clue Board
 * - Sparkfun Neo GPS Unit 
 * - Sparkfun RockBlock 9603 SatComm
 * 
 * Hardware hookup is via QWICC Connector
 * (CLUE <-> [QWIIC] <----> [QWIIC] <-> Sparkfun GPS BOB
 *   ^
 *   | UART (SERIAL1)
 *   RockBlock 9603
 * 
 * NOTE - Clue board requires 3-6 VDC (different from Micro:bit)
 * Recommend using 3 AA or Boost board
 * RockBlock needs 5V DC
 * *********************************************/
#include <Arduino.h>
#include <time.h>
#include <Wire.h> //Needed for I2C to GNSS GPS
#include <Adafruit_Arcada.h>
#include <Adafruit_SPIFlash.h>
// #include <SdFat.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS33.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_SHT31.h>
#include <Adafruit_APDS9960.h>
#include <Adafruit_BMP280.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_u-blox_GNSS
#include <IridiumSBD.h> // Click here to get the library: http://librarymanager/All#IridiumSBDI2C
#include <time.h>

/**************************************************************************************************
** Declare global variables and instantiate classes                                              **
**************************************************************************************************/

// Define the two white LEDs on the front of the Clue Board
#define WHITE_LED 43

Adafruit_Arcada arcada;
Adafruit_LSM6DS33 lsm6ds33;
Adafruit_LIS3MDL lis3mdl;
Adafruit_SHT31 sht30;
Adafruit_APDS9960 apds9960;
Adafruit_BMP280 bmp280;
extern Adafruit_FlashTransport_QSPI flashTransport;
extern Adafruit_SPIFlash Arcada_QSPI_Flash;
SFE_UBLOX_GNSS myGNSS;

//Setup the second serial port that talks to the RockBloc
#define IridiumSerial Serial1
#define DIAGNOSTICS false // Change this to see diagnostics

// Declare the IridiumSBD object
IridiumSBD modem(IridiumSerial);


uint32_t buttons, last_buttons;
uint8_t j = 0;  // neopixel counter for rainbow

// Check the timer callback, this function is called every millisecond!
volatile uint16_t milliseconds = 0;
void timercallback() {
  analogWrite(LED_BUILTIN, milliseconds);  // pulse the LED
  if (milliseconds == 0) {
    milliseconds = 255;
  } else {
    milliseconds--;
  }
}

// Forward function declaration with default value for sea level
float altitude(const int32_t press, const float seaLevel = 1013.25);
float altitude(const int32_t press, const float seaLevel) {
  /*!
  @brief     This converts a pressure measurement into a height in meters
  @details   The corrected sea-level pressure can be passed into the function if it is known,
             otherwise the standard atmospheric pressure of 1013.25hPa is used (see
             https://en.wikipedia.org/wiki/Atmospheric_pressure) for details.
  @param[in] press    Pressure reading from BME680
  @param[in] seaLevel Sea-Level pressure in millibars
  @return    floating point altitude in meters.
  */
static float Altitude;
Altitude = 44330.0 * (1.0 - pow(((float)press / 100.0) / seaLevel, 0.1903));  // Convert into meters
  return (Altitude);
}  

/*
typedef struct gpsdata {
    long latitude;
    long longitude;
    long altitude;
    long altitudeMSL;
    byte SIV;
    byte fixType;
    byte RTK;
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
    
}
*/


// file system object from SdFat
 FatFileSystem fatfs;

// Configuration for the datalogging file:
#define FILE_NAME      "data.csv"

unsigned long myTime;
unsigned long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.
unsigned long lastTime2 = 0; //Simple local timer. Limits amount if I2C traffic to u-blox module.

void setup() {
  
  Serial.begin(115200);
  delay(500);
  Serial.println("Backup Emergency Recovery Transmitter (BERT)");
  Serial.println("===========================================");
  Serial.println(" HW Rev. 3.0 | FW Rev. 1.5");
  Serial.println("===========================================");
  delay(5000);
  pinMode(WHITE_LED, OUTPUT);
  digitalWrite(WHITE_LED, LOW);
  
  Wire.begin();
  Serial.println("Initializing I2C Bus....OK");

  Serial.print("Booting up Arcada...");
  if (!arcada.arcadaBegin()) {
    Serial.print("Failed to begin");
    while (1);
  }
  Serial.println("OK");
  arcada.displayBegin();
  Serial.print("Setting up Display...");

  for (int i=0; i<250; i+=10) {
    arcada.setBacklight(i);
    delay(1);
  }

  arcada.display->setCursor(0, 0);
  arcada.display->setTextWrap(true);
  arcada.display->setTextSize(2);
  Serial.println("OK");
  
  /********** Setup QSPI Flash Memory */
  Serial.print("Setting up Filesystem...");

  // Initialize flash library and check its chip ID.
  if (!Arcada_QSPI_Flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1);
  }
  Serial.println("Flash chip JEDEC ID: 0x"); Serial.println(Arcada_QSPI_Flash.getJEDECID(), HEX);
  Serial.print("Mounting Filesystem...");

  // First call begin to mount the filesystem.  Check that it returns true
  // to make sure the filesystem was mounted.
  if (!fatfs.begin(&Arcada_QSPI_Flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    Serial.println("Was the flash chip formatted with the fatfs_format example?");
    while(1);
  }
  Serial.println("Mounted filesystem!");
  
  arcada.display->setTextColor(ARCADA_WHITE);
  arcada.display->println("Getting a clue...");
  arcada.display->setTextColor(ARCADA_WHITE);
  arcada.display->println("Sensors Found: ");

 
  Serial.print("Checking APDS...");
  if (!apds9960.begin()) {
    Serial.println("No APDS9960 found");
    arcada.display->setTextColor(ARCADA_RED);
  } else {
    Serial.println("**APDS9960 OK!");
    arcada.display->setTextColor(ARCADA_GREEN);
    apds9960.enableColor(true);
  }
  arcada.display->print("APDS9960 ");


  /********** Check LSM6DS33 */
  Serial.print("Checking LSM6DS33...");
  if (!lsm6ds33.begin_I2C()) {
    Serial.println("No LSM6DS33 found");
    arcada.display->setTextColor(ARCADA_RED);
  } else {
    Serial.println("**LSM6DS33 OK!");
    arcada.display->setTextColor(ARCADA_GREEN);
  }
  arcada.display->println("LSM6DS33 ");
  
  /********** Check LIS3MDL */
  Serial.print("Checking LIS3MDL...");
  if (!lis3mdl.begin_I2C()) {
    Serial.println("No LIS3MDL found");
    arcada.display->setTextColor(ARCADA_RED);
  } else {
    Serial.println("**LIS3MDL OK!");
    arcada.display->setTextColor(ARCADA_GREEN);
  }
  arcada.display->print("LIS3MDL ");

  /********** Check SHT3x */
  Serial.print("Checking SHT30...");
  if (!sht30.begin(0x44)) {
    Serial.println("No SHT30 found");
    arcada.display->setTextColor(ARCADA_RED);
  } else {
    Serial.println("**SHT30 OK!");
    arcada.display->setTextColor(ARCADA_GREEN);
  }
  arcada.display->print("SHT30 ");

  /********** Check BMP280 */
  Serial.print("Checking BPM280...");
  if (!bmp280.begin()) {
    Serial.println("No BMP280 found");
    arcada.display->setTextColor(ARCADA_RED);
  } else {
    Serial.println("**BMP280 OK!");
    arcada.display->setTextColor(ARCADA_GREEN);
  }
  arcada.display->println("BMP280");

  buttons = last_buttons = 0;
  arcada.timerCallback(1000, timercallback);
  arcada.display->setTextWrap(false);

  Serial.print("Initializing GPS Sensor....");

  if (myGNSS.begin() == false)
  {
    Serial.println("FAILED");
    Serial.println(F("u-blox GNSS not detected at default I2C address. Please check wiring. Freezing."));
    arcada.display->setTextColor(ARCADA_RED);
    while (1);
  }
  else{
    arcada.display->setTextColor(ARCADA_GREEN);
    arcada.display->println("NEO-N9M GPS");
    myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    Serial.println("OK");
  }
  
 
  Serial.print("Initializing RockBloc Sat Modem");
  int signalQuality = -1;
  int err;

  // Start the serial port connected to the satellite modem
  IridiumSerial.begin(19200);

  // Begin satellite modem operation
  Serial.println(F("Starting SatComm..."));
  err = modem.begin();
  if (err != ISBD_SUCCESS)
  {
    Serial.print(F("SatComm: error "));
    Serial.println(err);
    arcada.display->setTextColor(ARCADA_RED);
    if (err == ISBD_NO_MODEM_DETECTED)
      Serial.println(F("No modem detected: check wiring."));
    return;
  }
  else{
    Serial.println("OK");
    Serial.println("Displaying SatCom FW and IMEI");
    // Example: Print the firmware revision
    char version[12];
    err = modem.getFirmwareVersion(version, sizeof(version));
    if (err != ISBD_SUCCESS)
    {
       Serial.print(F("FirmwareVersion failed: error "));
       Serial.println(err);
       return;
    }
    Serial.print(F("Firmware Version is "));
    Serial.print(version);
    Serial.println(F("."));
  
    // Example: Print the IMEI
    char IMEI[16];
    err = modem.getIMEI(IMEI, sizeof(IMEI));
    if (err != ISBD_SUCCESS)
    {
       Serial.print(F("getIMEI failed: error "));
       Serial.println(err);
       return;
    }
    Serial.print(F("IMEI is "));
    Serial.print(IMEI);
    Serial.println(F("."));

    // Example: Test the signal quality.
    // This returns a number between 0 and 5.
    // 2 or better is preferred.
    err = modem.getSignalQuality(signalQuality);
    if (err != ISBD_SUCCESS)
    {
      Serial.print(F("SignalQuality failed: error "));
      Serial.println(err);
      return;
    }
  
    Serial.print(F("On a scale of 0 to 5, signal quality is currently "));
    Serial.print(signalQuality);
    Serial.println(F("."));
    arcada.display->setTextColor(ARCADA_GREEN);
    arcada.display->println("Sat Radio");
  }

  Serial.println("Setup Process Comlplete...Booting BERTOS");
  arcada.display->setTextColor(ARCADA_GREEN);
  arcada.display->println("Bootup Complete!");
  delay(5000);
  arcada.display->fillScreen(ARCADA_BLACK);
  
}
/*=====================================================
* Main Loop
=========================================================*/
void loop() {

  float temp, pres, humidity;
  long GPSLat, GPSLon,GPSAlt;
  long altitudeMSL;
  byte SIV;
  int Year, Month, Day, Hour, Minute, Second;
  byte fixType;
  byte RTK;
  int signalQuality = -1;
  int err;
  char IMEI[16];

/*!
  @brief    1 Second Routine
  @details  At 1 second intervals, collect data, send to Serial and 
            send data to Flash memory on the Clue Board. This will append
            to the file in case of power failure. File format is as follows
            Temp,Pressure,Humidity,Lat,Lon,Altitude,FixType
  @return   void
*/
  
  if (millis() - lastTime > 1000)
  {
    Serial.println("In the 1 sec function");
    lastTime = millis(); //Update the timer
    temp = bmp280.readTemperature();
    pres = bmp280.readPressure()/100;
    humidity = sht30.readHumidity();
    GPSLat = myGNSS.getLatitude();
    GPSLon = myGNSS.getLongitude();
    GPSAlt = myGNSS.getAltitude();
    altitudeMSL = myGNSS.getAltitudeMSL();
    
    SIV = myGNSS.getSIV();
    Year = myGNSS.getYear();
    Month = myGNSS.getMonth();
    Day = myGNSS.getDay();
    Hour = myGNSS.getHour();
    Minute = myGNSS.getMinute();
    Second = myGNSS.getSecond();
    fixType = myGNSS.getFixType();

    // Open the datalogging file for writing.  The FILE_WRITE mode will open
  // the file for appending, i.e. it will add new data to the end of the file.
  File dataFile = fatfs.open(FILE_NAME, FILE_WRITE);
  // Check that the file opened successfully and write a line to it.
  if (dataFile) {
    dataFile.print(temp,2);
    dataFile.print(",");
    dataFile.print(pres, 2);
    dataFile.print(",");
    dataFile.print(humidity, 2);
    dataFile.print(",");
    dataFile.print(GPSLat);
    dataFile.print(",");
    dataFile.print(GPSLon);
    dataFile.print(",");
    dataFile.print(altitudeMSL);
    dataFile.print(",");
    dataFile.print(fixType);
    dataFile.println();
    // Finally close the file when done writing.  This is smart to do to make
    // sure all the data is written to the file.
    dataFile.close();
    Serial.println("Wrote new measurement to data file!");
  }
  else {
    Serial.println("Failed to open data file for writing!");
  }

  /*
   * Update the Arcada Display
   */

    arcada.display->fillScreen(ARCADA_BLACK);
    arcada.display->setTextColor(ARCADA_WHITE, ARCADA_BLACK);
    arcada.display->setCursor(0, 0);
    
    arcada.display->print("Temp: ");
    arcada.display->print(temp);
    arcada.display->print(" C");
    arcada.display->println("         ");
    
    arcada.display->print("Baro: ");
    arcada.display->print(pres);
    arcada.display->print(" hPa");
    arcada.display->println("         ");
    
    arcada.display->print("Humid: ");
    arcada.display->print(humidity);
    arcada.display->print(" %");
    arcada.display->println("         ");
  
    arcada.display->print("lat: ");
    arcada.display->print(GPSLat);
    arcada.display->println("         ");
  
    arcada.display->print("lon: ");
    arcada.display->print(GPSLon);
    arcada.display->println("         ");
  
    arcada.display->print("alt: ");
    arcada.display->print(GPSAlt);
    arcada.display->println("         ");

    /*
     * Print to Serial Output
    TODO: Add under DEBUG statement
  
    Serial.print(F("Temp: "));
    Serial.print(temp);
  
    Serial.print(F(" Pres: "));
    Serial.print(pres);
  
    Serial.print(F(" Humid: "));
    Serial.println(humidity);
  
    Serial.print("Lat: ");
    Serial.print(latitude);
    Serial.print(" ");
    Serial.print("Long: ");
    Serial.print(longitude);
    //Serial.print(" (degrees * 10^-7)");
    
  
    Serial.print(" Alt: ");
    Serial.print(altitude);
    Serial.print(" (mm)");
  
    Serial.print(" AltMSL: ");
    Serial.print(altitudeMSL);
    Serial.print(" (mm)");
  
    Serial.print(" SIV: ");
    Serial.print(SIV);
  
    Serial.print(" Fix: ");
    Serial.println(fixType);
    */
  
 }

 /*!
  @brief    5 min interval
  @details  Take the data collected and transmit via the SatComm
            It takes to send data, to the Iridium network and
            unfortunatly this is blocking code. Data is sent as 
            BERT,Lat,Lon,Altitude,FixType,Temp,Pressure,Humidity,SatQuality
  @return   void
*/

  if (millis() - lastTime2 > 300000)
  {
    Serial.println("In the 120 sec function");
    lastTime2 = millis(); //Update the timer
    // Example: Print the IMEI
    err = modem.getIMEI(IMEI, sizeof(IMEI));
    if (err != ISBD_SUCCESS)
    {
       Serial.print(F("getIMEI failed: error "));
       Serial.println(err);
       return;
    }
    Serial.print(F("IMEI is "));
    Serial.print(IMEI);
    Serial.println(F("."));

    temp = bmp280.readTemperature();
    pres = bmp280.readPressure()/100;
    humidity = sht30.readHumidity();
    GPSLat = myGNSS.getLatitude();
    GPSLon = myGNSS.getLongitude();
    GPSAlt = myGNSS.getAltitude();
    altitudeMSL = myGNSS.getAltitudeMSL();
    
    SIV = myGNSS.getSIV();
    Year = myGNSS.getYear();
    Month = myGNSS.getMonth();
    Day = myGNSS.getDay();
    Hour = myGNSS.getHour();
    Minute = myGNSS.getMinute();
    Second = myGNSS.getSecond();
    fixType = myGNSS.getFixType();

    RTK = myGNSS.getCarrierSolutionType();

    // Get Satellite Signal Quality
    /*
    if (RTK == 0) RTXType="No solution";
    else if (RTK == 1) RTXType="High precision floating fix";
    else if (RTK == 2) RTXType="High precision fix";
    */
    err = modem.getSignalQuality(signalQuality);
    //Can't have more than 340 bytes for transmission
    char telem_sat[300];
    //String telem = String("B,") + String(GPSLat) + ',' + String(GPSLon) + ',' + String(GPSAlt) + ',' + String(fixType) + ',' + String(temp,2) + ',' + String(pres,2) + ',' + String(humidity,2) + ',' + String(signalQuality);
    String telem = String("B,") + String(GPSLat) + ',' + String(GPSLon) + ',' + String(GPSAlt) + ',' + String(fixType) + ',' + String(temp,2) + ',' + String(pres,2) + ',' + String(humidity,2);
    Serial.println(telem);
    int telemlen = telem.length();
    telem.toCharArray(telem_sat,telemlen);
    // Send the message
    Serial.print(F("Sending Data: "));
    Serial.println(telem_sat);
    err = modem.sendSBDText(telem_sat);
    if (err != ISBD_SUCCESS)
    {
      Serial.print(F("sendSBDText failed: error "));
      Serial.println(err);
      if (err == ISBD_SENDRECEIVE_TIMEOUT)
        Serial.println(F("Try again with a better view of the sky."));
    }
  
    else
    {
      Serial.println(F("Message Sent!"));
    }
  
    // Clear the Mobile Originated message buffer
    Serial.println(F("Clearing the MO buffer."));
    err = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
    if (err != ISBD_SUCCESS)
    {
      Serial.print(F("clearBuffers failed: error "));
      Serial.println(err);
    }
    Serial.println(F("Done!"));
    
  }
}

//Iridium Diagnostic functions

#if DIAGNOSTICS
void ISBDConsoleCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  Serial.write(c);
}
#endif