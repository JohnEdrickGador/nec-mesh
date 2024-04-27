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
#define   WIFI_CHANNEL    6 //Check the access point on your router for the channel - 6 is not the same for everyone
#define   MESH_PREFIX     "NEC_4TH_FLOOR"
#define   MESH_PASSWORD   "CARE_OFFICE"
#define   MESH_PORT       5555
#define   MESH_SIZE       3

/*** Access Point Credentials ***/
#define   STATION_SSID          "CARE_407"
#define   STATION_PASSWORD      "nec_c@re"

/*** Host Names ***/
#define   SINK_HOSTNAME         "Sink_Node"
#define   CHILD_HOSTNAME        "Child_Node"

/*** MQTT Broker Connection Details ***/
const char* mqttServer = "198e7235f58349c4abe133a3e05ed706.s1.eu.hivemq.cloud";
const char* mqttUsername = "ESP32-Test";
const char* mqttPassword = "ESP32test01";
const int   mqttPort = 8883;
const char* publishTopic ="ESP32PubTest";
const char* subscribeTopic = "ESP32SubTest"; 

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

/***Functions***/

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

/***Variables***/

//Mesh Network
int myRSSI;
String nodeIDRSSIString;
std::map<uint32_t, int> nodeIDRSSIMap;
uint32_t target;
bool sneDone = false;
bool isConnected = false; //flag to check if node is connected to the internet

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
std::vector<std::vector<int>> aqiValues = {{0, 50}, {51, 100}, {101, 150}, {151, 200}, {201, 300}, {301, 500}};
std::vector<String> aqiDescriptionVector = {"Green- Good", "Yellow- Moderate", "Orange- Unhealthy for Sensitive Groups", "Red- Unhealthy", "Purple- Very Unhealthy", "Maroon- Hazardous"};
std::vector<std::vector<float>> pm2p5Breakpoints = {{0.0, 25.0}, {25.1, 35.0}, {35.1, 45.0}, {45.1, 55.0}, {55.1, 90.0}, {91.0, 999999999.0}};  // PM2.5- from DENR
std::vector<std::vector<int>> pm10Breakpoints = {{0, 54}, {55, 154}, {155, 254}, {255, 354}, {355, 424}, {425, 604}};                          // PM10- from US-EPA
std::vector<std::vector<int>> co2Breakpoints = {{0, 500}, {501, 1000}, {1001, 1500}, {1501, 2000}, {2001, 3000}, {3001, 5000}};                // CO2- from US-EPA (Researchgate)
std::vector<std::vector<int>> tvocBreakpoints = {{0, 500}, {501, 1000}, {1001, 1500}, {1501, 2000}, {2001, 3000}, {3001, 5000}};                // TVOC- from US-EPA

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
String indoorURL = "https://api.weather.com/v2/pws/observations/current?stationId=IQUEZO15&format=json&units=m&apiKey=9d4f41efcb5647a58f41efcb56d7a5d3&numericPrecision=decimal";

// Time
String timeStamp;
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 28800;
const int   daylightOffset_sec = 0;

// SD Card
String sensorData;
String outdoorAnemometerData;
String indoorAnemometerData;