// Libraries
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include "SensirionI2CSen5x.h"
#include "Adafruit_INA219.h"
#include "Adafruit_SGP30.h"

// The used commands use up to 48 bytes. On some Arduino's the default buffer
// space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

SensirionI2CSen5x sen5x;
Adafruit_INA219 ina219;
Adafruit_SGP30 sgp;

void printModuleVersions() {
    uint16_t error;
    char errorMessage[256];

    unsigned char productName[32];
    uint8_t productNameSize = 32;

    error = sen5x.getProductName(productName, productNameSize);

    if (error) {
        Serial.print("Error trying to execute getProductName(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("ProductName:");
        Serial.println((char*)productName);
    }

    uint8_t firmwareMajor;
    uint8_t firmwareMinor;
    bool firmwareDebug;
    uint8_t hardwareMajor;
    uint8_t hardwareMinor;
    uint8_t protocolMajor;
    uint8_t protocolMinor;

    error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug,
                             hardwareMajor, hardwareMinor, protocolMajor,
                             protocolMinor);
    if (error) {
        Serial.print("Error trying to execute getVersion(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Firmware: ");
        Serial.print(firmwareMajor);
        Serial.print(".");
        Serial.print(firmwareMinor);
        Serial.print(", ");

        Serial.print("Hardware: ");
        Serial.print(hardwareMajor);
        Serial.print(".");
        Serial.println(hardwareMinor);
    }
}

void printSerialNumber() {
    uint16_t error;
    char errorMessage[256];
    unsigned char serialNumber[32];
    uint8_t serialNumberSize = 32;

    error = sen5x.getSerialNumber(serialNumber, serialNumberSize);
    if (error) {
        Serial.print("Error trying to execute getSerialNumber(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("SerialNumber:");
        Serial.println((char*)serialNumber);
    }
}

// Write to the SD card
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;

float CO2;
float TVOC;

float shuntvoltage;
float busvoltage;
float current_mA;
float loadvoltage;
float power_mW;

String dataMessage;

void SGP30_read() {
  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }

  TVOC = sgp.TVOC;
  CO2 = sgp.eCO2;

  Serial.print("TVOC: "); Serial.print(TVOC); Serial.print(" ppb\r");
  Serial.print("eCO2: "); Serial.print(CO2); Serial.println(" ppm\r");
}

void SEN55_read() {
  uint16_t error;
  char errorMessage[256];

  error = sen5x.readMeasuredValues(
    massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0,
    massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex,
    noxIndex);

  if (error) {
    Serial.print("Error trying to execute readMeasuredValues(): ");
    errorToString(error, errorMessage, 256);
    Serial.println(errorMessage);
  } else {
    Serial.print("MassConcentrationPm1p0:");
    Serial.print(massConcentrationPm1p0);
    Serial.print("\r");
    Serial.print("MassConcentrationPm2p5:");
    Serial.print(massConcentrationPm2p5);
    Serial.print("\r");
    Serial.print("MassConcentrationPm4p0:");
    Serial.print(massConcentrationPm4p0);
    Serial.print("\r");
    Serial.print("MassConcentrationPm10p0:");
    Serial.print(massConcentrationPm10p0);
    Serial.print("\r");
    Serial.print("AmbientHumidity:");
    if (isnan(ambientHumidity)) {
        Serial.print("n/a");
    } else {
        Serial.print(ambientHumidity);
    }
    Serial.print("\r");
    Serial.print("AmbientTemperature:");
    if (isnan(ambientTemperature)) {
        Serial.print("n/a");
    } else {
        Serial.print(ambientTemperature);
    }
    Serial.print("\r");
    Serial.print("VocIndex:");
    if (isnan(vocIndex)) {
        Serial.print("n/a");
    } else {
        Serial.print(vocIndex);
    }
    Serial.print("\r");
    Serial.print("NoxIndex:");
    if (isnan(noxIndex)) {
        Serial.println("n/a");
    } else {
        Serial.println(noxIndex);
    }
  }
  Serial.println("\r");
}

void INA219_read() {
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busvoltage); Serial.println(" V\r");
  Serial.print("Shunt Voltage: "); Serial.print(shuntvoltage); Serial.println(" mV\r");
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V\r");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA\r");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW\r");
}

void SD_log() {
  //Concatenate all info separated by commas
  dataMessage = String(ambientTemperature) + ", " + String(ambientHumidity) + ", " + String(CO2) + ", " + String(TVOC) + ", " + String(massConcentrationPm2p5) + ", " + String(massConcentrationPm10p0) + ", " + String(busvoltage) + ", " + String(power_mW) + "\r\n";
  Serial.print("Saving data: ");
  Serial.println(dataMessage);

  //Append the data to file
  appendFile(SD, "/log.txt", dataMessage.c_str());
}

void setup() {
  Serial.begin(460800);

  while (!Serial) {
        delay(100);
  }

  Wire.begin();

  sen5x.begin(Wire);

  uint16_t error;
  char errorMessage[256];
  error = sen5x.deviceReset();
    if (error) {
        Serial.print("Error trying to execute deviceReset(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

// Print SEN55 module information if i2c buffers are large enough
#ifdef USE_PRODUCT_INFO
    printSerialNumber();
    printModuleVersions();
#endif

    // set a temperature offset in degrees celsius
    // Note: supported by SEN54 and SEN55 sensors
    // By default, the temperature and humidity outputs from the sensor
    // are compensated for the modules self-heating. If the module is
    // designed into a device, the temperature compensation might need
    // to be adapted to incorporate the change in thermal coupling and
    // self-heating of other device components.
    //
    // A guide to achieve optimal performance, including references
    // to mechanical design-in examples can be found in the app note
    // “SEN5x – Temperature Compensation Instruction” at www.sensirion.com.
    // Please refer to those application notes for further information
    // on the advanced compensation settings used
    // in `setTemperatureOffsetParameters`, `setWarmStartParameter` and
    // `setRhtAccelerationMode`.
    //
    // Adjust tempOffset to account for additional temperature offsets
    // exceeding the SEN module's self heating.
    float tempOffset = 0.0;
    error = sen5x.setTemperatureOffsetSimple(tempOffset);
    if (error) {
        Serial.print("Error trying to execute setTemperatureOffsetSimple(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    } else {
        Serial.print("Temperature Offset set to ");
        Serial.print(tempOffset);
        Serial.println(" deg. Celsius (SEN54/SEN55 only");
    }

    // Start Measurement
    error = sen5x.startMeasurement();
    if (error) {
        Serial.print("Error trying to execute startMeasurement(): ");
        errorToString(error, errorMessage, 256);
        Serial.println(errorMessage);
    }

    if (!ina219.begin()) {
      Serial.println("Failed to find INA219 chip");
      while (1) { delay(10); }
    }

    Serial.println("Measuring voltage and current with INA219 ...");

    if (! sgp.begin()){
      Serial.println("Sensor not found :(");
      while (1);
    }

    Serial.print("Found SGP30 serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);

    // Initialize SD Card
    while (!SD.begin(0)) {
      Serial.println("Card Mount Failed");
    }

    File file = SD.open("/log.txt");
    if(!file) {
      Serial.println("File doesn't exist");
      Serial.println("Creating file...");
      writeFile(SD, "/log.txt", "Temperature (°C), Humidity (%), CO2 (ppm), TVOC (ppb), PM2.5 (ppm), PM10 (ppm), Voltage (V), Power (mW) \r\n");
    }
    else {
      Serial.println("File already exists");  
    }
    file.close();
}

void loop() {
  SGP30_read();
  delay(1000);
  SEN55_read();
  delay(1000);
  SD_log();
  delay(5000);
}
