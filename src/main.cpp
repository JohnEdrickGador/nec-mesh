/*** Libraries ***/
#include <painlessMesh.h>
#include <WiFi.h>
#include <map>
#include <FS.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "SensirionI2CSen5x.h"
#include "Adafruit_SGP30.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include "Adafruit_INA219.h"
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "time.h"
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

/*** Initialize Libraries ***/
painlessMesh mesh;
Scheduler userScheduler;
HTTPClient http;
WiFiClientSecure espClient;
PubSubClient client(espClient);
SensirionI2CSen5x sen5x;
Adafruit_INA219 ina219;
Adafruit_SGP30 sgp;

/*** Mesh Details ***/
#define   WIFI_CHANNEL                6 //Check the access point on your router for the channel - 6 is not the same for everyone
#define   MESH_PREFIX                 "NEC_4TH_FLOOR"
#define   MESH_PASSWORD               "CARE_OFFICE"
#define   MESH_PORT                   5555
#define   MESH_SIZE                   4
#define   NODE_SOURCE                 "18"
#define   INDOOR_ANEMOMETER_SOURCE    "19"
#define   OUTDOOR_ANEMOMETER_SOURCE   "20"

/*** Access Point Credentials ***/
#define   STATION_SSID          "CARE_407"
#define   STATION_PASSWORD      "nec_c@re"

/*** Host Names ***/
#define   SINK_HOSTNAME         "Sink_Node"

/*** MQTT Broker Connection Details ***/
const char* mqttServer = "ed7632329e6e4fbcbe77b1fa917585a1.s1.eu.hivemq.cloud";
const char* mqttUsername = "gador.e";
const char* mqttPassword = "CAREspice1b";
const int   mqttPort = 8883;
const char* publishTopic ="UPCARE/UNDERGRAD/EEE199_NEC";
const char* subscribeTopic = "UPCARE/UNDERGRAD/EEE199_NEC"; 

/*** Buffer Space ***/
// The used commands use up to 48 bytes. On some Arduino's the default buffer space is not large enough
#define MAXBUF_REQUIREMENT 48

#if (defined(I2C_BUFFER_LENGTH) &&                 \
     (I2C_BUFFER_LENGTH >= MAXBUF_REQUIREMENT)) || \
    (defined(BUFFER_LENGTH) && BUFFER_LENGTH >= MAXBUF_REQUIREMENT)
#define USE_PRODUCT_INFO
#endif

/*** Buffer ***/
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];

/*** Root certificate ***/
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

/*** Functions ***/

//Mesh Network
void receivedCallback(const uint32_t &from, const String &msg);
void sendMessage();

//Sink Node Election
void sinkNodeElectionInit();
void sinkNodeElection();
void connectToWifi();
void broadcastRSSI();

//MQTT Protocol
void reconnect();
void publishMessage(const char* topic, String payload , boolean retained, String dataType);
void publishSensorData();
void publishOutdoorAnemometerData();
void publishIndoorAnemometerData();

//SD Card Operations
void sdInit();
void sdCreateFile(const char* fileName, String dataType);
void appendFile(fs::FS &fs, const char * path, const char * message);
void writeFile(fs::FS &fs, const char * path, const char * message);
void sdOutdoorAnemometerLog(const char* fileName);
void sdIndoorAnemometerLog(const char* fileName);
void sdSensorLog(const char* fileName);

// SEN55
void printSerialNumber();
void printModuleVersions();
void sen55Init();
void sen55Read();

// SGP30
void sgp30Init();
void sgp30Read();

// Air Quality Index
void getAQI();

// INA219
void ina219Init();
void ina219Read();

// Anemometer
void getAnemometerData(String url, float &windSpeed, float &windGust, float &windDirection);

// Time
void getTime();
void broadcastTime();

/*** Variables ***/

//Mesh Network
int myRSSI;
String nodeIDRSSIString;
std::map<uint32_t, int> nodeIDRSSIMap;
uint32_t target;
bool sneDone = false;
bool isSinkNode = false;
bool isConnected = false; //flag to check if node is connected to the internet
bool mapReconstructed = false;
bool mqttConnected = false;

// SEN55
float massConcentrationPm1p0;
float massConcentrationPm2p5;
float massConcentrationPm4p0;
float massConcentrationPm10p0;
float ambientHumidity;
float ambientTemperature;
float vocIndex;
float noxIndex;

// SGP30
float CO2;
float TVOC;

// Air Quality Index
int Cp, AQI;
float BPhi, BPlo, Ihi, Ilo;
float pm2p5Ip = 999999999, pm10Ip = 999999999, tvocIp = 999999999, co2Ip = 999999999;
String aqiDescription;
std::vector<std::vector<int>> aqiValues = {{0, 50}, {51, 75}, {76, 100}, {101, 150}};
std::vector<String> aqiDescriptionVector = {"Green- Good", "Yellow- Moderate", "Orange- Unhealthy for Sensitive Groups", "Red- Unhealthy"};
std::vector<std::vector<int>> pm2p5Breakpoints = {{0, 15}, {16, 20}, {21, 30}, {30, 999999999}};          // PM2.5- from DENR
std::vector<std::vector<int>> pm10Breakpoints = {{0, 50}, {51, 75}, {76, 100}, {100, 999999999}};         // PM10- from US-EPA
std::vector<std::vector<int>> co2Breakpoints = {{0, 800}, {801, 1150}, {1151, 1500}, {1501, 999999999}};  // CO2- from US-EPA (Researchgate)
std::vector<std::vector<int>> tvocBreakpoints = {{0, 400}, {401, 600}, {601, 800}, {800, 999999999}};     // TVOC- from BiyaHero

// INA219
float shuntVoltage;
float busVoltage;
float currentmA;
float loadVoltage;
float powermW;

// Anemometer
float outdoorWindSpeed;
float outdoorWindGust;
float outdoorWindDirection;
float indoorWindSpeed;
float indoorWindGust;
float indoorWindDirection;
String outdoorURL = "https://api.weather.com/v2/pws/observations/current?stationId=IQUEZO15&format=json&units=m&apiKey=9d4f41efcb5647a58f41efcb56d7a5d3&numericPrecision=decimal";
String indoorURL = "https://api.weather.com/v2/pws/observations/current?stationId=IQUEZO20&format=json&units=m&apiKey=9d4f41efcb5647a58f41efcb56d7a5d3&numericPrecision=decimal";

// Time
String timeStamp = "";
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 28800;
const int   daylightOffset_sec = 0;

// SD Card
String sensorData;
String outdoorAnemometerData;
String indoorAnemometerData;

/*** Tasks ***/
// All nodes
Task taskBroadcastRSSI (TASK_SECOND * 10, TASK_FOREVER, &broadcastRSSI);

// Sink Node
Task taskBroadcastTime(TASK_SECOND * 30 , TASK_FOREVER, &broadcastTime);
Task taskPublishSensorData(TASK_MINUTE * 2, TASK_FOREVER, &publishSensorData);
Task taskPublishOutdoorAnemometerData(TASK_MINUTE * 2, TASK_FOREVER, &publishOutdoorAnemometerData);
Task taskPublishIndoorAnemometerData(TASK_MINUTE * 2, TASK_FOREVER, &publishIndoorAnemometerData);

// Child Node
Task taskSendMessage(TASK_MINUTE * 2, TASK_FOREVER, &sendMessage);

void setup() {
  Serial.begin(115200);
  pinMode(10, OUTPUT);

  //Reboot Sink Node Election
  sinkNodeElectionInit();

  //Initialize sensors and sd card
  sen55Init();
  sgp30Init();
  ina219Init();
  sdInit();

  //Initialize Mesh Network
  mesh.setDebugMsgTypes(ERROR | STARTUP | CONNECTION);  // set before init() so that you can see startup messages

  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, WIFI_CHANNEL);
  mesh.onReceive(&receivedCallback);

  //MQTT
  espClient.setCACert(root_ca);
  client.setServer(mqttServer, mqttPort);

  userScheduler.addTask(taskBroadcastRSSI);
  userScheduler.addTask(taskBroadcastTime);
  userScheduler.addTask(taskPublishSensorData);
  userScheduler.addTask(taskPublishOutdoorAnemometerData);
  userScheduler.addTask(taskPublishIndoorAnemometerData);
  userScheduler.addTask(taskSendMessage);

  // Configure to synchronize with PH time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  if (!mapReconstructed) {
    while(mesh.getNodeList().size() + 1 != MESH_SIZE) {
      mesh.update();
    }
    Serial.println("All nodes connected!");
  }

  //Perform sink node election
  sinkNodeElection();

}

void loop() {
  mesh.update();
  if(isConnected == true) {
    if(mesh.getNodeId() == target) {
      if (!client.connected()) reconnect(); // check if client is connected
      client.loop();
    }
  }

  if(isSinkNode) {
    if(mqttConnected) {
      taskBroadcastTime.enableIfNot();
      taskPublishSensorData.enableIfNot();
      taskPublishOutdoorAnemometerData.enableIfNot();
      taskPublishIndoorAnemometerData.enableIfNot();
    }
  }

  else {
    if(timeStamp != "") {
      taskSendMessage.enableIfNot();
    }
  }
}

void broadcastRSSI() {
  mesh.sendBroadcast("RSSI: " + String(myRSSI));
  Serial.print("RSSI Broadcasted: ");
  Serial.println(myRSSI);
}

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

void receivedCallback(const uint32_t &from, const String &msg) {
  Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
  if(msg.substring(0,4) == "RSSI") {
    nodeIDRSSIMap.insert({from, msg.substring(6).toInt()});
    Serial.println("RSSI and nodeID pushed to map!");
  }
  else if(msg.substring(0,7) == "Payload") {
    if(isSinkNode) {
      publishMessage(publishTopic, msg.substring(9), true, "Sensor");
    }
  }
  else if (msg.substring(0,4) == "Time"){
    timeStamp = msg.substring(6);
  }
}

void connectToWifi() {
  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  while(WiFi.status() != WL_CONNECTED){mesh.update();}
  Serial.println("Connected to wifi!");
  isConnected = true;
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

  error = sen5x.getVersion(firmwareMajor, firmwareMinor, firmwareDebug, hardwareMajor, hardwareMinor, protocolMajor, protocolMinor);

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

void sen55Init() {
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
}

void sen55Read() {
  uint16_t error;
  char errorMessage[256];

  error = sen5x.readMeasuredValues(massConcentrationPm1p0, massConcentrationPm2p5, massConcentrationPm4p0, massConcentrationPm10p0, ambientHumidity, ambientTemperature, vocIndex, noxIndex);

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

void sgp30Init() {
  if (!sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }

  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
}

void sgp30Read() {
  if (!sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }

  TVOC = sgp.TVOC;
  CO2 = sgp.eCO2;

  Serial.print("TVOC: "); Serial.print(TVOC); Serial.print(" ppb\r");
  Serial.print("eCO2: "); Serial.print(CO2); Serial.println(" ppm\r");
}

void getAQI() {
  for (int i=0; i<4; ++i) {
    int pm2p5Rounded = std::round(massConcentrationPm2p5);
    if ((pm2p5Rounded <= pm2p5Breakpoints[i][1]) && (pm2p5Rounded >= pm2p5Breakpoints[i][0])) { 
      Cp = std::trunc(massConcentrationPm2p5);
      BPhi = pm2p5Breakpoints[i][1];
      BPlo = pm2p5Breakpoints[i][0];
      Ihi = aqiValues[i][1];
      Ilo = aqiValues[i][0];
      pm2p5Ip = ((Ihi - Ilo) / (BPhi -BPlo)) * (Cp - BPlo) + Ilo;
    }

    int pm10Rounded = std::round(massConcentrationPm10p0);
    if ((pm10Rounded <= pm10Breakpoints[i][1]) && (pm10Rounded >= pm10Breakpoints[i][0])) { 
      Cp = std::trunc(massConcentrationPm10p0);
      BPhi = pm10Breakpoints[i][1];
      BPlo = pm10Breakpoints[i][0];
      Ihi = aqiValues[i][1];
      Ilo = aqiValues[i][0];
      pm10Ip = ((Ihi - Ilo) / (BPhi -BPlo)) * (Cp - BPlo) + Ilo;
    }

    int co2Rounded = std::round(CO2);
    if ((co2Rounded <= co2Breakpoints[i][1]) && (co2Rounded >= co2Breakpoints[i][0])) { 
      Cp = std::trunc(CO2);
      BPhi = co2Breakpoints[i][1];
      BPlo = co2Breakpoints[i][0];
      Ihi = aqiValues[i][1];
      Ilo = aqiValues[i][0];
      co2Ip = ((Ihi - Ilo) / (BPhi -BPlo)) * (Cp - BPlo) + Ilo;
    }

    int tvocRounded = std::round(TVOC);
    if ((tvocRounded <= tvocBreakpoints[i][1]) && (tvocRounded >= tvocBreakpoints[i][0])) { 
      Cp = std::trunc(TVOC);
      BPhi = tvocBreakpoints[i][1];
      BPlo = tvocBreakpoints[i][0];
      Ihi = aqiValues[i][1];
      Ilo = aqiValues[i][0];
      tvocIp = ((Ihi - Ilo) / (BPhi -BPlo)) * (Cp - BPlo) + Ilo;
    }
  }

  if (pm2p5Ip != 999999999 && pm10Ip != 999999999 && co2Ip != 999999999 && tvocIp != 999999999) {
    // Max
    AQI = std::round(std::max({pm2p5Ip, pm10Ip, co2Ip, tvocIp}));
    for (int i=0; i<4; ++i) {
      if ((AQI <= aqiValues[i][1]) && (AQI >= aqiValues[i][0])) {
        aqiDescription = aqiDescriptionVector[i];
        break;
      }
    }

    Serial.print("AQI Indices (PM2.5, PM10, CO2, TVOC): "); Serial.print(pm2p5Ip); Serial.print(", "); Serial.print(pm10Ip); Serial.print(", "); Serial.print(co2Ip); Serial.print(", "); Serial.println(tvocIp);
    Serial.print("AQI Value: "); Serial.println(AQI);
    Serial.print("AQI Description: "); Serial.println(aqiDescription);
  }

  else {
    if (pm2p5Ip == 999999999) {
      Serial.print("PM2.5");
    }
    if (pm10Ip == 999999999) {
      Serial.print(" and PM10");
    }
    if (co2Ip == 999999999) {
      Serial.print(" and CO2");
    }
    if (tvocIp == 999999999) {
      Serial.print(" and TVOC");
    }
    Serial.println(" reading/s is/are out of range! ERROR!");
  }
}

void ina219Init() {
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1);
  }

  Serial.println("Measuring voltage and current with INA219 ...");
}

void ina219Read() {
  shuntVoltage = ina219.getShuntVoltage_mV();
  busVoltage = ina219.getBusVoltage_V();
  currentmA = ina219.getCurrent_mA();
  powermW = ina219.getPower_mW();
  loadVoltage = busVoltage + (shuntVoltage / 1000);
  
  Serial.print("Bus Voltage:   "); Serial.print(busVoltage); Serial.println(" V\r");
  Serial.print("Shunt Voltage: "); Serial.print(shuntVoltage); Serial.println(" mV\r");
  Serial.print("Load Voltage:  "); Serial.print(loadVoltage); Serial.println(" V\r");
  Serial.print("Current:       "); Serial.print(currentmA); Serial.println(" mA\r");
  Serial.print("Power:         "); Serial.print(powermW); Serial.println(" mW\r");
}

void getAnemometerData(String url, float &windSpeed, float &windGust, float &windDirection) {
  // Send HTTP request
  http.begin(url);
  int httpCode = http.GET();
    
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
        
    // Parse JSON response
    DynamicJsonDocument doc(2048); // Adjust the size according to your response
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
      Serial.println("Failed to parse JSON");
    } else {
      // Extract wind speed, direction, and gust
      JsonObject data = doc["observations"][0];
      windSpeed = data["metric"]["windSpeed"];
      windGust = data["metric"]["windGust"];
      windDirection = data["winddir"];

      // Print wind data
      Serial.print("Wind Speed: "); Serial.print(windSpeed); Serial.println(" m/s\r");
      Serial.print("Wind Gust: "); Serial.print(windGust); Serial.println(" m/s\r");
      Serial.print("Wind Direction: "); Serial.println(windDirection); Serial.println(" degrees\r");
    }
  } else {
    Serial.print("Error: ");
    Serial.println(httpCode);
  }

  http.end();
}

void getTime() {
  struct tm timeinfo;
  while(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    delay(2000);
  }

  char buffer[50]; // Adjust the buffer size as needed
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  
  timeStamp = buffer;
  Serial.println(timeStamp);
}

void broadcastTime() {
  getTime();
  mesh.sendBroadcast("Time: " + timeStamp);
}

void sdCreateFile(const char* fileName, String dataType) {
  // Initialize SD Card
  while (!SD.begin(0)) {
    Serial.println("Card Mount Failed");
  }

  File file = SD.open(fileName);

  if(!file) {
    Serial.println("File doesn't exist");
    if(dataType == "Sensor") {
      Serial.println("Creating sensor log file...");
      writeFile(SD, fileName, "Time, Temperature (°C), Humidity (%), CO2 (ppm), TVOC (ppb), PM2.5 (ppm), PM10 (ppm), AQI, AQI Description, Voltage (V), Power (mW) \r\n");
    }
    else {
      Serial.println("Creating anemometer log file...");
      writeFile(SD, fileName, "Time, Wind Speed (m/s), Wind Gust (m/s), Wind Direction (°) \r\n");
    }
  }
  else {
    Serial.println("File already exists");  
  }

  file.close();
}

void sdInit() {
  sdCreateFile("/SensorLog.txt", "Sensor");
  sdCreateFile("/SensorLogFailedSend.txt", "Sensor");
  sdCreateFile("/SensorLogFailedPublish.txt", "Sensor");
  sdCreateFile("/OutdoorAnemometerLog.txt", "Anemometer");
  sdCreateFile("/IndoorAnemometerLog.txt", "Anemometer");
  sdCreateFile("/OutdoorAnemometerLogFailedPublish.txt", "Anemometer");
  sdCreateFile("/IndoorAnemometerLogFailedPublish.txt", "Anemometer");
}

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

void sdSensorLog(const char* fileName) {
  //Concatenate all info separated by commas
  sensorData = timeStamp + ", " + String(ambientTemperature) + ", " + String(ambientHumidity) + ", " + String(CO2) + ", " + String(TVOC) + ", " + String(massConcentrationPm2p5) + ", " + String(massConcentrationPm10p0) + ", " + String(AQI) + ", " + aqiDescription + ", " + String(busVoltage) + ", " + String(powermW) + "\r\n";
  Serial.print("Saving data: ");
  Serial.println(sensorData);

  //Append the data to file
  appendFile(SD, fileName, sensorData.c_str());
}

void sdOutdoorAnemometerLog(const char* fileName) {
  //Concatenate all info separated by commas
  outdoorAnemometerData = timeStamp + ", " + String(outdoorWindSpeed) + ", " + String(outdoorWindGust) + ", " + String(outdoorWindDirection) + "\r\n";
  Serial.print("Saving data: ");
  Serial.println(outdoorAnemometerData);

  //Append the data to file
  appendFile(SD, fileName, outdoorAnemometerData.c_str());
}

void sdIndoorAnemometerLog(const char* fileName) {
  //Concatenate all info separated by commas
  indoorAnemometerData = timeStamp + ", " + String(indoorWindSpeed) + ", " + String(indoorWindGust) + ", " + String(indoorWindDirection) + "\r\n";
  Serial.print("Saving data: ");
  Serial.println(indoorAnemometerData);

  //Append the data to file
  appendFile(SD, fileName, indoorAnemometerData.c_str());
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32Client-";   // Create a random client ID
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqttUsername, mqttPassword)) {
      Serial.println("connected");
      client.subscribe("led_state");   // subscribe the topics here
      mqttConnected = true;
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
    }
    delay(5000);
  }
}

void publishMessage(const char* topic, String payload , boolean retained, String dataType){
  if (client.publish(topic, payload.c_str(), true)) {
    Serial.println("Message published ["+String(topic)+"]: "+payload);
  } else {
    Serial.println("Message not published");
    if(dataType == "Sensor") {
      sdSensorLog("/SensorLogFailedPublish.txt");
    } else if(dataType == "Outdoor Anemometer") {
      sdOutdoorAnemometerLog("/OutdoorAnemometerLogFailedPublish.txt");
    } else {
      sdIndoorAnemometerLog("/IndoorAnemometerLogFailedPublish.txt");
    }
  }
}

void publishSensorData() {
  getTime();
  sen55Read();
  sgp30Read();
  ina219Read();
  getAQI();
  sdSensorLog("/SensorLog.txt");

  JsonDocument doc;
  doc["source"] = NODE_SOURCE;
  doc["local_time"] = timeStamp;
  doc["TMP"] = String(ambientTemperature);
  doc["RH"] = String(ambientHumidity);
  doc["PM2p5"] = String(massConcentrationPm2p5);
  doc["PM10"] = String(massConcentrationPm10p0);
  doc["CO2"] = String(CO2);
  doc["TVOC"] = String(TVOC);
  doc["VOL"] = String(busVoltage);
  doc["POW"] = String(powermW);
  doc["AQI"] = String(AQI);
  doc["type"] = "data";

  String mqttMessage;
  serializeJson(doc, mqttMessage);
  Serial.println(mqttMessage);

  publishMessage(publishTopic, mqttMessage, true, "Sensor");
}

void publishOutdoorAnemometerData() {
  getTime();
  getAnemometerData(outdoorURL, outdoorWindSpeed, outdoorWindGust, outdoorWindDirection);
  sdOutdoorAnemometerLog("/OutdoorAnemometerLog.txt");

  JsonDocument doc;
  doc["source"] = OUTDOOR_ANEMOMETER_SOURCE;
  doc["local_time"] = timeStamp;
  doc["WSPD"] = String(outdoorWindSpeed);
  doc["WGUST"] = String(outdoorWindGust);
  doc["WDIR"] = String(outdoorWindDirection);
  doc["type"] = "data";

  String mqttMessage;
  serializeJson(doc, mqttMessage);
  Serial.println(mqttMessage);

  publishMessage(publishTopic, mqttMessage, true, "Outdoor Anemometer");
}

void publishIndoorAnemometerData() {
  getTime();
  getAnemometerData(indoorURL, indoorWindSpeed, indoorWindGust, indoorWindDirection);
  sdIndoorAnemometerLog("/IndoorAnemometerLog.txt");

  JsonDocument doc;
  doc["source"] = INDOOR_ANEMOMETER_SOURCE;
  doc["local_time"] = timeStamp;
  doc["WSPD"] = String(indoorWindSpeed);
  doc["WGUST"] = String(indoorWindGust);
  doc["WDIR"] = String(indoorWindDirection);
  doc["type"] = "data";

  String mqttMessage;
  serializeJson(doc, mqttMessage);
  Serial.println(mqttMessage);

  publishMessage(publishTopic, mqttMessage, true, "Indoor Anemometer");
}

void sendMessage() {
  sen55Read();
  sgp30Read();
  ina219Read();
  getAQI();
  sdSensorLog("/SensorLog.txt");

  JsonDocument doc;
  doc["source"] = NODE_SOURCE;
  doc["local_time"] = timeStamp;
  doc["TMP"] = String(ambientTemperature);
  doc["RH"] = String(ambientHumidity);
  doc["PM2p5"] = String(massConcentrationPm2p5);
  doc["PM10"] = String(massConcentrationPm10p0);
  doc["CO2"] = String(CO2);
  doc["TVOC"] = String(TVOC);
  doc["VOL"] = String(busVoltage);
  doc["POW"] = String(powermW);
  doc["AQI"] = String(AQI);
  doc["type"] = "data";

  String mqttMessage;
  serializeJson(doc, mqttMessage);
  mqttMessage = "Payload: " + mqttMessage;
  Serial.println(mqttMessage);

  if(mesh.sendSingle(target, mqttMessage)) {
    Serial.println("Message sent to the sink node!");
  }
  else {
    Serial.println("Message failed to send!");
    sdSensorLog("/SensorLogFailedSend.txt");
  }
}

void sinkNodeElectionInit() {
  while (!SD.begin(0)) {
    Serial.println("Card Mount Failed");
  }

  File file = SD.open("/NodeID_RSSI.txt");
  if(!file) {
    WiFi.begin(STATION_SSID, STATION_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {}
    Serial.println("Connected to WiFi!");
    myRSSI = WiFi.RSSI();
    WiFi.disconnect();
    Serial.println(myRSSI);
  }
  else {
    mapReconstructed = true;
    String numberString = "";
    int nodeID = 0;
    int RSSI = 0;
    while (file.available()) {
      char c = file.read();
      if (c == ':') {
        nodeID = numberString.toInt();
        Serial.println(nodeID);
        numberString = "";
      } else if (c == ',') {
        RSSI = numberString.toInt();
        Serial.println(RSSI);
        numberString = "";
        nodeIDRSSIMap.insert({nodeID, RSSI});
      } 
      else {
        numberString = numberString + c;
      }
    }
  }
  file.close();
}

void sinkNodeElection() {
  nodeIDRSSIMap.insert({mesh.getNodeId(), myRSSI});
  
  while (nodeIDRSSIMap.size() != MESH_SIZE) {
    taskBroadcastRSSI.enableIfNot();
    mesh.update();
  }

  if(!mapReconstructed) {
    taskBroadcastRSSI.enableIfNot();
    taskBroadcastRSSI.setIterations(5);
  }

  int maxRSSI = INT_MIN; // Initialize to the smallest possible integer

  // Iterate through the map
  for (const auto& pair : nodeIDRSSIMap) {
    nodeIDRSSIString = nodeIDRSSIString + String(pair.first) + ":" + String(pair.second) + ",";

    // Check if the current RSSI is greater than the maximum RSSI found so far
    if (pair.second > maxRSSI) {
      // Update the maximum RSSI and corresponding node ID
      maxRSSI = pair.second;
      target = pair.first;
    }
    else if (pair.second == maxRSSI) {
      if (pair.first < target) {
        target = pair.first;
      }
    }
  }

  sneDone = true;
  Serial.println("Sink node election done!");
  Serial.println("Sink node is " + String(target));

  File file = SD.open("/NodeID_RSSI.txt");

  if(!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/NodeID_RSSI.txt", nodeIDRSSIString.c_str());
  }
  else {
    Serial.println("File already exists");  
  }

  file.close();

  if(mesh.getNodeId() == target) {
    digitalWrite(10, HIGH);
    isSinkNode = true;
    mesh.setHostname(SINK_HOSTNAME);
    connectToWifi();
    mesh.setRoot(true);
  }

  else {
    digitalWrite(10, LOW);
    mesh.setRoot(false);
  }

  mesh.setContainsRoot(true);
}