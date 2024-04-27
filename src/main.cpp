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