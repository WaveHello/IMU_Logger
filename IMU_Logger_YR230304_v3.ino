/////////////////////////////////////////////////////////////////////////////////////////////////////////
//   Author: Jonathan Moore
//   Date: March 13, 2023
//  Version: 1.0

//  Purpose: The purpose of this code is to read in accelerometer, gyroscope, and magnometer data and write it to an sd card with a time step

//  Hardware used (All products from adafruit.com):
//      Stacking female headers - Part No. 2830
//      Adafruit feather M0 Express - ATSAMD21 Cortex M0 - Part No. 3403
//      MicroSD card - Part No. 1294
//      Lithium Ion Polymer Battery ideal for Feathers 3.7v 400mAh - Part No. 3898
//      Adafruit 9-Dof Orientation IMU Fusion Breakout - BNO085 - Stemma QT/Qwiic - Part No. 4754
//      CR1220 12 mm Diameter - 3V Lithium Coin Cell Battery
//      Adalogger Featherwing - RTC + SD Add-on for all Feather boards - Part No. 2922
/////////////////////////////////////////////////////////////////////////////////////////////////////////

//Libraries
#include <Adafruit_BNO08x.h>    // IMU library
#include "SdFat.h"              // Enables reading and quick writing on SD cards.
#include "FreeStack.h"
#include "IMULogger.h"
//------------------------------------------------------------------------------

// SD_FAT_TYPE = 0 for SdFat/File as defined in SdFatConfig.h,
// 1 for FAT16/FAT32, 2 for exFAT, 3 for FAT16/FAT32 and exFAT.
#define SD_FAT_TYPE 1
//------------------------------------------------------------------------------

// Set USE_RTC nonzero for file timestamps.
// RAM use will be marginal on Uno with RTClib.
// 0 - RTC not used
// 1 - DS1307
// 2 - DS3231
// 3 - PCF8523
#define USE_RTC 3 
#if USE_RTC
#include "RTClib.h"
#endif  // USE_RTC

/*
  Change the value of SD_CS_PIN if you are using SPI and
  your hardware does not use the default value, SS.
  Common values are:
  Arduino Ethernet shield: pin 4
  Sparkfun SD shield: pin 8
  Adafruit SD shields and modules: pin 10
*/

// SDCARD_SS_PIN is defined for the built-in SD on some boards.
#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = 10;
#else  // SDCARD_SS_PIN
// Assume built-in SD is used.
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Try max SPI clock for an SD. Reduce SPI_CLOCK if errors occur.
#define SPI_CLOCK SD_SCK_MHZ(12) // 12 is max for adafruit express m0

// Try to select the best SD card configuration.
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif  ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SPI_CLOCK)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SPI_CLOCK)
#endif  // HAS_SDIO_CLASS

//------------------------------------------------------------------------------
#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
const int RECORDING_TYPE_1 = 1; // Set 1 for accelerometer
// const int RECORDING_TYPE_2 = -1;
// const int RECORDING_TYPE_3 = 3;
//===============================================================================
void setIMUReports(int RECORDING_TYPE) { // Here is where you define the IMU sensor outputs you want to receive
  // Options: 
  // SH2_ACCELEROMETER, SH2_GYROSCOPE_CALIBRATED, SH2_MAGNETIC_FIELD_CALIBRATED, SH2_LINEAR_ACCELERATION, SH2_GRAVITY, SH2_ROTATION_VECTOR,
  // SH2_GEOMAGNETIC_ROTATION_VECTOR, SH2_GAME_ROTATION_VECTOR, SH2_STEP_COUNTER, SH2_STABILITY_CLASSIFIER, SH2_RAW_ACCELEROMETER, 
  // SH2_RAW_GYROSCOPE, SH2_RAW_MAGNETOMETER, SH2_SHAKE_DETECTOR
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(RECORDING_TYPE)) {
    Serial.println("Could not enable game vector");
  }
}
//------------------------------------------------------------------------------
void logIMU(data_t* data, int RECORDING_TYPE){ // Selects the correct values to output for set IMU report type
  // Change this so it returns all values so that they can be logged to a file with Datetime
  switch (RECORDING_TYPE) {
  case SH2_ACCELEROMETER:{

    Serial.print("Accelerometer - x: ");
    Serial.print(sensorValue.un.accelerometer.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.accelerometer.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.accelerometer.z);
    data->imu_data[0] = sensorValue.un.accelerometer.x;
    data->imu_data[1] = sensorValue.un.accelerometer.y;
    data->imu_data[2] = sensorValue.un.accelerometer.z;
    data->imu_data[3] = -999.9;
    break;
  }
  case SH2_GYROSCOPE_CALIBRATED:{
    Serial.print("Gyro - x: ");
    Serial.print(sensorValue.un.gyroscope.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.gyroscope.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.gyroscope.z);
    //Log Data
    data->imu_data[0] = sensorValue.un.gyroscope.x;
    data->imu_data[1] = sensorValue.un.gyroscope.y;
    data->imu_data[2] = sensorValue.un.gyroscope.z;
    data->imu_data[3] = -999.9;
    break;
  }
  case SH2_MAGNETIC_FIELD_CALIBRATED:{
    Serial.print("Magnetic Field - x: ");
    Serial.print(sensorValue.un.magneticField.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.magneticField.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.magneticField.z);
    //Log Data
    data->imu_data[0] = sensorValue.un.magneticField.x;
    data->imu_data[1] = sensorValue.un.magneticField.y;
    data->imu_data[2] = sensorValue.un.magneticField.z;
    data->imu_data[3] = -9999.9;
    break;
  }
  case SH2_LINEAR_ACCELERATION:{
    Serial.print("Linear Acceration - x: ");
    Serial.print(sensorValue.un.linearAcceleration.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.linearAcceleration.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.linearAcceleration.z);
    //Log Data
    data->imu_data[0] = sensorValue.un.linearAcceleration.x;
    data->imu_data[1] = sensorValue.un.linearAcceleration.y;
    data->imu_data[2] = sensorValue.un.linearAcceleration.z;
    data->imu_data[3] = -999.9;
    break;
  }
  case SH2_GRAVITY:{
    Serial.print("Gravity - x: ");
    Serial.print(sensorValue.un.gravity.x);
    Serial.print(" y: ");
    Serial.print(sensorValue.un.gravity.y);
    Serial.print(" z: ");
    Serial.println(sensorValue.un.gravity.z);
    //Log Data
    data->imu_data[0] = sensorValue.un.gravity.x;
    data->imu_data[1] = sensorValue.un.gravity.y;
    data->imu_data[2] = sensorValue.un.gravity.z;
    data->imu_data[3] = -999.9;
    break;
  }
  case SH2_ROTATION_VECTOR:{
    Serial.print("Rotation Vector - r: ");
    Serial.print(sensorValue.un.rotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.rotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.rotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.rotationVector.k);
    //Log Data
    data->imu_data[0] = sensorValue.un.rotationVector.real;
    data->imu_data[1] = sensorValue.un.rotationVector.i;
    data->imu_data[2] = sensorValue.un.rotationVector.j;
    data->imu_data[3] = sensorValue.un.rotationVector.k;
    break;
  }
  case SH2_GEOMAGNETIC_ROTATION_VECTOR:{
    Serial.print("Geo-Magnetic Rotation Vector - r: ");
    Serial.print(sensorValue.un.geoMagRotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.geoMagRotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.geoMagRotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.geoMagRotationVector.k);
    //Log data
    data->imu_data[0] = sensorValue.un.geoMagRotationVector.real;
    data->imu_data[1] = sensorValue.un.geoMagRotationVector.i;
    data->imu_data[2] = sensorValue.un.geoMagRotationVector.j;
    data->imu_data[3] = sensorValue.un.geoMagRotationVector.k;
    break;
  }
  case SH2_GAME_ROTATION_VECTOR:{
    Serial.print("Game Rotation Vector - r: ");
    Serial.print(sensorValue.un.gameRotationVector.real);
    Serial.print(" i: ");
    Serial.print(sensorValue.un.gameRotationVector.i);
    Serial.print(" j: ");
    Serial.print(sensorValue.un.gameRotationVector.j);
    Serial.print(" k: ");
    Serial.println(sensorValue.un.gameRotationVector.k);
    //Log data
    data->imu_data[0] = sensorValue.un.gameRotationVector.real;
    data->imu_data[1] = sensorValue.un.gameRotationVector.i;
    data->imu_data[2] = sensorValue.un.gameRotationVector.j;
    data->imu_data[3] = sensorValue.un.gameRotationVector.k;
    break;
  }

  }  
}

//===============================================================================

#if SD_FAT_TYPE == 0 // Depending on SD format type define sd_t and file_t
typedef SdFat sd_t;
typedef File file_t;
#elif SD_FAT_TYPE == 1
typedef SdFat32 sd_t;
typedef File32 file_t;
#elif SD_FAT_TYPE == 2
typedef SdExFat sd_t;
typedef ExFile file_t;
#elif SD_FAT_TYPE == 3
typedef SdFs sd_t;
typedef FsFile file_t;
#else  // SD_FAT_TYPE
#error Invalid SD_FAT_TYPE
#endif  // SD_FAT_TYPE

sd_t sd;

file_t csvFile;
// You may modify the filename.  Digits before the dot are file versions.
char csvName[] = "IMULogger00.csv";
data_t data;
data_t data_2;

//------------------------------------------------------------------------------
#if USE_RTC
#if USE_RTC == 1
RTC_DS1307 rtc;
#elif USE_RTC == 2
RTC_DS3231 rtc;
#elif USE_RTC == 3
RTC_PCF8523 rtc;
#else  // USE_RTC == type
#error USE_RTC type not implemented.
#endif  // USE_RTC == type
#endif
// Call back for file timestamps.  Only called for file create and sync().
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  DateTime now = rtc.now();

  // Return date using FS_DATE macro to format fields.
  *date = FS_DATE(now.year(), now.month(), now.day());

  // Return time using FS_TIME macro to format fields.
  *time = FS_TIME(now.hour(), now.minute(), now.second());

  // Return low time bits in units of 10 ms.
  *ms10 = now.second() & 1 ? 100 : 0;
}
//------------------------------------------------------------------------------
void printField(Print* pr, char sep, uint8_t v) {
  if (sep) {
    pr->write(sep);
  }
  if (v < 10) {
    pr->write('0');
  }
  pr->print(v);
}
//------------------------------------------------------------------------------
void printNow(Print* pr) {
  DateTime now = rtc.now();
  pr->print(now.year());
  printField(pr, '-',now.month());
  printField(pr, '-',now.day());
  printField(pr, ' ',now.hour());
  printField(pr, ':',now.minute());
  printField(pr, ':',now.second());
  pr->print(':');
  pr->print(millis());
  pr->print(':');
  pr->print(micros());
}
//------------------------------------------------------------------------------
void printRecord(Print* pr, data_t* data, int RECORDING_TYPE, int RECORDING_TYPE_2 = -1, int RECORDING_TYPE_3 = -1) {
  static uint32_t nr = 0;
  if (!data) {
    pr->print(F("Header, recording Setting,"));
    pr->print(RECORDING_TYPE);
    pr->print(RECORDING_TYPE_2);
    pr->println(RECORDING_TYPE_3);
    pr->print(F("rec#"));
    for (size_t i = 0; i < VECTOR_DIM; i++) {
      pr->print(F("comp"));
      pr->print(i);
    }
    pr->print(F("millimicro"));
    pr->println();
    nr = 0;
    return;
  }
    pr->print(nr++);
    pr->write(',');
    printNow(pr);
    for (size_t i = 0; i < VECTOR_DIM; i++) {
      pr->write(',');
      pr->print(data->imu_data[i]);
  
    }
    pr->println();

}
//------------------------------------------------------------------------------
#define error(s) sd.errorHalt(&Serial, F(s)) //Creates serial output function for errors
//------------------------------------------------------------------------------
void createCsvFile() {
  delay(1000);
  csvFile.close();
  while (sd.exists(csvName)) {
    char* p = strchr(csvName, '.');
    if (!p) {
      error("no dot in filename");
    }
    while (true) {
      p--;
      if (p < csvName || *p < '0' || *p > '9') {
        error("Can't create file name");
      }
      if (p[0] != '9') {
        p[0]++;
        break;
      }
      p[0] = '0';
    }
  }
  if (!csvFile.open(csvName, O_RDWR | O_CREAT)) {
    error("open csvName failed");
  }
  Serial.println(csvName);
}
//-----------------------------------------------------------------------------
void testSensor() {
  const uint32_t interval = 200000;
  int32_t diff;
  data_t data;
  Serial.println(F("\nTesting - type any character to stop\n"));
  delay(1000);
  printRecord(&Serial, nullptr, RECORDING_TYPE_1);// Change to RECORDING_TYPE_2 for other values
  uint32_t m = micros();
  while (!Serial.available()) {
    m += interval;
    do {
      diff = m - micros();
    } while (diff > 0);
    logIMU(&data, RECORDING_TYPE_1);
    printRecord(&Serial, &data, RECORDING_TYPE_1);
  }
}

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  // while (!Serial) {
  //   yield(); // wait for serial port to connect. Needed for native USB port only
  // }

  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
  //if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte UART buffer!
  //if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");

  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version :");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionPatch);
    Serial.print(" Build ");
    Serial.println(bno08x.prodIds.entry[n].swBuildNumber);
  }


  delay(1000);
    // Initialize SD.
    if (!sd.begin(SD_CONFIG)) {
      sd.initErrorHalt(&Serial);
    }
  #if USE_RTC
    if (!rtc.begin()) {
      error("rtc.begin failed");
    }
    if (!rtc.isrunning()) {
      // Set RTC to sketch compile date & time.
      // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
      error("RTC is NOT running!");
    }
    // Set callback
    FsDateTime::setCallback(dateTime);
  #endif  // USE_RTC
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  delay(500);
  digitalWrite(13, HIGH);
  delay(500);
  digitalWrite(13, LOW);
  createCsvFile(); //creates CSV file, you need to have a log data line or this while run forever
  
  // printRecord(&csvFile, nullptr, RECORDING_TYPE_1, RECORDING_TYPE_2);
  csvFile.sync();

}

void loop() {

  // testSensor();

  while(true){

    int i = 0;
    int time1 = micros();
    digitalWrite(13, HIGH);
    while(i < 500){
      delay(5);

      setIMUReports(RECORDING_TYPE_1);

      if (bno08x.wasReset()) {
        Serial.print("sensor was reset ");
      }

      if (! bno08x.getSensorEvent(&sensorValue)) {
        return;
      }
      logIMU(&data, RECORDING_TYPE_1);
      printRecord(&csvFile, &data, RECORDING_TYPE_1);
      // printRecord(&Serial, &data,  RECORDING_TYPE_1);

      Serial.println("Reading events");
      
      // delay(10);
      // //set the imu report type
      // // print the report
      // // set the report type to the second option
      // // print the report of the second option

      // setIMUReports(RECORDING_TYPE_2);
      // // if (bno08x.wasReset()) {
      // //   Serial.print("sensor was reset ");
      // // }
      // // Serial.println("Reading events");
      // if (! bno08x.getSensorEvent(&sensorValue)) {

      //   return;
      // }
      // logIMU(&data_2, RECORDING_TYPE_2);
      // printRecord(&csvFile, &data_2,  RECORDING_TYPE_2);
      // printRecord(&Serial, &data_2,  RECORDING_TYPE_2);

      // delay(10);
      // //set the imu report type
      // // print the report
      // // set the report type to the second option
      // // print the report of the second option

      // setIMUReports(RECORDING_TYPE_3);
      // // Serial.println("Reading events");
      // if (! bno08x.getSensorEvent(&sensorValue)) {

      //   return;
      // }
      // logIMU(&data, RECORDING_TYPE_3);
      // printRecord(&csvFile, &data,  RECORDING_TYPE_3);
      // printRecord(&Serial, &data,  RECORDING_TYPE_3);
      i++;
    }

    csvFile.sync();
    int diff = micros() - time1;
    Serial.print(diff);
  }


}









