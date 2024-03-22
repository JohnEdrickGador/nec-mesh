// Libraries
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include "SensirionI2CSen5x.h"
#include "Adafruit_INA219.h"
#include "Adafruit_SGP30.h"

#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include <string.h>

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

/*Mesh Details*/
#define   WIFI_CHANNEL    6 //Check the access point on your router for the channel - 6 is not the same for everyone
#define   MESH_PREFIX     "whateveryouwant"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

/****** WiFi Connection Details *******/
#define   STATION_SSID     "CARE_407"
#define   STATION_PASSWORD "nec_c@re"
// #define   BRIDGE_NODE
#define   HOSTNAME  "MQTT_Bridge"

/******* MQTT Broker Connection Details *******/
const char* mqtt_server = "7037541e66a1404980eb5c1c4f000874.s1.eu.hivemq.cloud";
const char* mqtt_username = "ESP32-Test";
const char* mqtt_password = "ESP32test01";
const int mqtt_port = 8883;

const char* publishTopic ="ESP32PubTest";
const char* subscribeTopic = "ESP32SubTest"; 

/**** Secure WiFi Connectivity Initialisation *****/
WiFiClientSecure espClient;

/*Prototypes*/
void receivedCallback( const uint32_t &from, const String &msg );
void mqttCallback(char* topic, byte* payload, unsigned int length);
void sendMessage(); // Prototype so PlatformIO doesn't complain
void publishMQTT();
void SEN55_read();
void SGP30_read();
void INA219_read();
void SD_log();
void SensorRead();

IPAddress getlocalIP();
IPAddress myIP(0,0,0,0);

/*Mesh*/
painlessMesh  mesh;
Scheduler userScheduler;

/*Tasks*/
Task taskSendMessage( TASK_SECOND * 10 , TASK_FOREVER, &sendMessage );

/**** MQTT Client Initialisation Using WiFi Connection *****/
PubSubClient client(espClient);

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

/****** root certificate *********/

static const char *root_ca PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)EOF";

/************* Connect to MQTT Broker ***********/
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP8266Client-";   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("connected");

      client.subscribe("led_state");   // subscribe the topics here

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

/**** Method for Publishing MQTT Messages **********/
void publishMessage(const char* topic, String payload , boolean retained){
  if (client.publish(topic, payload.c_str(), true))
      Serial.println("Message publised ["+String(topic)+"]: "+payload);
}

bool isInternet = false;

void setup() {
  Serial.begin(115200);

  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);

  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  // Channel set to 6. Make sure to use the same channel for your mesh and for you other
  // network (STATION_SSID)
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, WIFI_CHANNEL );
  mesh.onReceive(&receivedCallback);

  userScheduler.addTask( taskSendMessage );
  // userScheduler.addTask( taskPublishMQTT );
  taskSendMessage.enable();
  // taskPublishMQTT.enable();

  #ifdef BRIDGE_NODE
  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);
  #endif

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

    // if (! sgp.begin()){
    //   Serial.println("Sensor not found :(");
    //   while (1);
    // }

    Serial.print("Found SGP30 serial #");
    Serial.print(sgp.serialnumber[0], HEX);
    Serial.print(sgp.serialnumber[1], HEX);
    Serial.println(sgp.serialnumber[2], HEX);

    // Initialize SD Card
    while (!SD.begin(0)) {
      Serial.println("Card Mount Failed");
    }

    File file = SD.open("/InitialDeployment.txt");
    if(!file) {
      Serial.println("File doesn't exist");
      Serial.println("Creating file...");
      writeFile(SD, "/InitialDeployment.txt", "Temperature (°C), Humidity (%), CO2 (ppm), TVOC (ppb), PM2.5 (ppm), PM10 (ppm), Voltage (V), Power (mW) \r\n");
    }
    else {
      Serial.println("File already exists");  
    }
    file.close();
}

void loop() {
  mesh.update();
  if(myIP != getlocalIP()) {
    myIP = getlocalIP();
    Serial.println("My IP is " + myIP.toString());
    isInternet = true;
  }

  if (isInternet == true) {
    #ifdef BRIDGE_NODE
    if (!client.connected()) reconnect(); // check if client is connected
    client.loop();
    #endif
  } 
}

void receivedCallback( const uint32_t &from, const String &msg ) {
  Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
  uint32_t target = 1973942425;
  if (mesh.sendSingle(target, "Hello world!") == 0) {
    mesh.sendBroadcast(msg);
  }
}

void sendMessage() {
  SensorRead();

  JsonDocument doc;
  doc["deviceId"] = "CARE_Office_1";
  doc["Site"] = "Care Office";
  doc["humidity"] = ambientHumidity;
  doc["temperature"] = ambientTemperature;
  doc["eCO2"] = CO2;
  doc["TVOC"] = TVOC;
  doc["PM2.5"] = massConcentrationPm2p5;
  doc["PM10"] = massConcentrationPm10p0;
  doc["Voltage"] = busvoltage;
  doc["Power"] = power_mW;

  char mqtt_message[512];
  serializeJson(doc, mqtt_message);
  uint32_t target = 1973942425;
  if (mesh.sendSingle(target, mqtt_message) == 0) {
    mesh.sendBroadcast(mqtt_message);
  }
  else {
    Serial.println("Message sent!");
  }
}

void publishMQTT() {
  JsonDocument doc;
  doc["deviceId"] = "ESP32C3-Beetle";
  doc["Site"] = "Edrick House";
  doc["humidity"] = random(1,20);
  doc["temperature"] = random(1,25);

  char mqtt_message[128];
  serializeJson(doc, mqtt_message);
  publishMessage(publishTopic,mqtt_message,true);
}

IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
}

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
  appendFile(SD, "/InitialDeployment.txt", dataMessage.c_str());
}

void SensorRead() {
  // SGP30_read();
  SEN55_read();
  INA219_read();
  SD_log();
}