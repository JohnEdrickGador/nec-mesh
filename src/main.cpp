#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <painlessMesh.h>
#include <HTTPClient.h>
#include <time.h>

#include <WiFi.h>
// #include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>

/*Mesh Details*/
#define   WIFI_CHANNEL    6 //Check the access point on your router for the channel - 6 is not the same for everyone
#define   MESH_PREFIX     "whateveryouwant"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

/****** WiFi Connection Details *******/
// #define   STATION_SSID     "CARE_407"
// #define   STATION_PASSWORD "nec_c@re"
// #define   STATION_SSID     "HUAWEI-2.4G-kCj7_EXT"
// #define   STATION_PASSWORD "dtaY9jsJ"
#define   STATION_SSID     "PLDTHOMEFIBRe3e58"
#define   STATION_PASSWORD "Florenda@1124"
#define   BRIDGE_NODE
#define   HOSTNAME  "MQTT_Bridge"

/******* MQTT Broker Connection Details *******/
const char* mqtt_server = "198e7235f58349c4abe133a3e05ed706.s1.eu.hivemq.cloud";
const char* mqtt_username = "ESP32-Test";
const char* mqtt_password = "ESP32test01";
const int mqtt_port = 8883;

const char* publishTopic ="ESP32PubTest";
const char* subscribeTopic = "ESP32SubTest"; 

/*Time*/
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 28800;
const int   daylightOffset_sec = 0;

String url = "https://api.weather.com/v2/pws/observations/current?stationId=IQUEZO15&format=json&units=m&apiKey=9d4f41efcb5647a58f41efcb56d7a5d3&numericPrecision=decimal";
String url1 = "https://api.weather.com/v2/pws/observations/current?stationId=IQUEZO20&format=json&units=m&apiKey=9d4f41efcb5647a58f41efcb56d7a5d3&numericPrecision=decimal";
float windSpeed;
float windGust;
float windDirection;
float windSpeed1;
float windGust1;
float windDirection1;
String timeString;

/**** Secure WiFi Connectivity Initialisation *****/
WiFiClientSecure espClient;

/*Prototypes*/
void receivedCallback( const uint32_t &from, const String &msg );
void mqttCallback(char* topic, byte* payload, unsigned int length);
void sendMessage(); // Prototype so PlatformIO doesn't complain
void publishMQTT();
void publishMQTT1();
void getTime();
void sendTime();
// void getAnemometerData();
void getAnemometerData(String url, float &windSpeed, float &windGust, float &windDirection);

IPAddress getlocalIP();
IPAddress myIP(0,0,0,0);

/*Mesh*/
painlessMesh  mesh;
Scheduler userScheduler;

/*Tasks*/
Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );
Task taskPublishMQTT( TASK_SECOND * 60, TASK_FOREVER, &publishMQTT );
Task taskPublishMQTT1( TASK_SECOND * 60, TASK_FOREVER, &publishMQTT1 );
Task taskSendTime( TASK_SECOND * 30 , TASK_FOREVER, &sendTime );

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
  if (client.publish(topic, payload.c_str(), true)) {
    Serial.println("Message publised ["+String(topic)+"]: "+payload);
  } else {
    Serial.println("Message not sent");
  }
}

bool isInternet = false;

void setup() {
  Serial.begin(115200);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  espClient.setCACert(root_ca);
  client.setServer(mqtt_server, mqtt_port);

  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  // Channel set to 6. Make sure to use the same channel for your mesh and for you other
  // network (STATION_SSID)
  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, WIFI_CHANNEL );
  mesh.onReceive(&receivedCallback);

  #ifdef BRIDGE_NODE
  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);
  #endif

  userScheduler.addTask( taskSendMessage );
  userScheduler.addTask( taskPublishMQTT );
  // userScheduler.addTask( taskPublishMQTT1 );
  userScheduler.addTask( taskSendTime );
  taskSendTime.enable();
  // taskSendMessage.enable();
  taskPublishMQTT.enable();

  pinMode(10, OUTPUT);
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
    digitalWrite(10, HIGH);
  } else {
    digitalWrite(10,LOW);
  }
}

void receivedCallback( const uint32_t &from, const String &msg ) {
  #ifdef BRIDGE_NODE
    Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
    publishMessage(publishTopic,msg,true);
  #endif
}

void sendMessage() {
  String msg = "Random number: " + String(random(1,20));
  mesh.sendBroadcast( msg );
  taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}

void publishMQTT() {
  getAnemometerData(url, windSpeed, windGust, windDirection);
  // getAnemometerData(url1, windSpeed1, windGust1, windDirection1);
  getTime();
  JsonDocument doc;
  // doc["deviceId"] = "ESP32C3-Beetle";
  // doc["Site"] = "Edrick House";
  // doc["humidity"] = random(1,20);
  // doc["temperature"] = random(1,25);

  doc["Source"] = "1";
  doc["local_time"] = timeString;

  JsonArray SEN55_data = doc.createNestedArray("SEN55_data");
  SEN55_data.add(NAN);
  SEN55_data.add(NAN);
  SEN55_data.add(NAN);
  SEN55_data.add(NAN);

  JsonArray SGP30_data = doc.createNestedArray("SGP30_data");
  SGP30_data.add(NAN);
  SGP30_data.add(NAN);

  JsonArray INA219_data = doc.createNestedArray("INA219_data");
  INA219_data.add(NAN);
  INA219_data.add(NAN);

  JsonArray Urageuxy_data = doc.createNestedArray("Urageuxy_data");
  Urageuxy_data.add(std::round(windSpeed * 100.0)/ 100.0);
  Urageuxy_data.add(std::round(windGust * 100.0)/ 100.0);
  Urageuxy_data.add(windDirection);

  // JsonArray AQI_data = doc.createNestedArray("AQI_data");
  // AQI_data.add(NAN);
  // AQI_data.add(NAN);

  doc["AQI"] = NAN;
  doc["type"] = "data";

  char mqtt_message[1024];
  serializeJson(doc, mqtt_message);
  Serial.println(mqtt_message);
  publishMessage(publishTopic,mqtt_message,true);
}

void publishMQTT1() {
  // getAnemometerData(url, windSpeed, windGust, windDirection);
  getAnemometerData(url1, windSpeed1, windGust1, windDirection1);
  getTime();
  JsonDocument doc;
  // doc["deviceId"] = "ESP32C3-Beetle";
  // doc["Site"] = "Edrick House";
  // doc["humidity"] = random(1,20);
  // doc["temperature"] = random(1,25);

  doc["Source"] = "1";
  doc["local_time"] = timeString;

  JsonArray SEN55_data = doc.createNestedArray("SEN55_data");
  SEN55_data.add(NAN);
  SEN55_data.add(NAN);
  SEN55_data.add(NAN);
  SEN55_data.add(NAN);

  JsonArray SGP30_data = doc.createNestedArray("SGP30_data");
  SGP30_data.add(NAN);
  SGP30_data.add(NAN);

  JsonArray INA219_data = doc.createNestedArray("INA219_data");
  INA219_data.add(NAN);
  INA219_data.add(NAN);

  JsonArray Urageuxy_data = doc.createNestedArray("Urageuxy_data1");
  Urageuxy_data.add(std::round(windSpeed1 * 100.0)/ 100.0);
  Urageuxy_data.add(std::round(windGust1 * 100.0)/ 100.0);
  Urageuxy_data.add(windDirection1);

  // JsonArray AQI_data = doc.createNestedArray("AQI_data");
  // AQI_data.add(NAN);
  // AQI_data.add(NAN);

  doc["AQI"] = NAN;
  doc["type"] = "data";

  char mqtt_message[1024];
  serializeJson(doc, mqtt_message);
  Serial.println(mqtt_message);
  publishMessage(publishTopic,mqtt_message,true);
}

IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
}

void getTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return;
  }

  char buffer[50]; // Adjust the buffer size as needed
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  
  timeString = buffer;

  Serial.println(timeString);
}

void sendTime() {
  getTime();
  mesh.sendBroadcast( timeString );
  Serial.println("Time broadcasted!");
}

void getAnemometerData(String url, float &windSpeed, float &windGust, float &windDirection) {
  // Send HTTP request
  HTTPClient http;
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

      // Print parsed JSON data
      // Serial.println("Complete JSON data:");
      // serializeJsonPretty(doc, Serial);
    }
  } else {
    Serial.print("Error: ");
    Serial.println(httpCode);
  }

  http.end();
}