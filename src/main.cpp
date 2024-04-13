/*** Libraries ***/
#include <string.h>
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <painlessMesh.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include "Adafruit_INA219.h"
#include "Adafruit_SGP30.h"
#include "SensirionI2CSen5x.h"
#include "time.h"

#include <WiFi.h>

/*** Variables ***/
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

float windSpeed;
float windGust;
float windDirection;

String dataMessage;
String time_stamp;

bool isInternet = false;

/*** Mesh Details ***/
#define   WIFI_CHANNEL    6 //Check the access point on your router for the channel - 6 is not the same for everyone
#define   MESH_PREFIX     "whateveryouwant"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

/*** WiFi Connection Details ***/
#define   STATION_SSID     "CARE_407"
#define   STATION_PASSWORD "nec_c@re"
// #define   BRIDGE_NODE
#define   HOSTNAME  "MQTT_Bridge"

/*** MQTT Broker Connection Details ***/
const char* mqtt_server = "7037541e66a1404980eb5c1c4f000874.s1.eu.hivemq.cloud";
const char* mqtt_username = "ESP32-Test";
const char* mqtt_password = "ESP32test01";
const int   mqtt_port = 8883;
const char* publishTopic ="ESP32PubTest";
const char* subscribeTopic = "ESP32SubTest"; 

/*** Time Servers ***/
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 28800;
const int   daylightOffset_sec = 0;

/*** Weather Underground Cloud details ***/
// const char* ssid = "PLDTHOMEFIBRe3e58";
// const char* password = "Florenda@1124";
const char* API_KEY = "9d4f41efcb5647a58f41efcb56d7a5d3";
const char* device_id = "IQUEZO15";
const int   numericPrecision = 2;
String url = "https://api.weather.com/v2/pws/observations/current?stationId=IQUEZO15&format=json&units=m&apiKey=9d4f41efcb5647a58f41efcb56d7a5d3&numericPrecision=decimal";

/*** Functions ***/
void printSerialNumber();
void printModuleVersions();
void SEN55_init();
void SGP30_init();
void INA219_init();
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void SD_init();
void SEN55_read();
void SGP30_read();
void INA219_read();
void getAnemometerData();
void SD_log();
void receivedCallback( const uint32_t &from, const String &msg );
// void mqttCallback(char* topic, byte* payload, unsigned int length);
void reconnect();
void publishMessage(const char* topic, String payload , boolean retained);
void publishMQTT();
void sendTime();
void sendMessage();

/*** Secure WiFi Connectivity Initialization ***/
WiFiClientSecure espClient;

/*** MQTT Client Initialization Using WiFi Connection ***/
PubSubClient client(espClient);

/*** HTTP Client Initialization ***/
HTTPClient http;

/*** IP Address Initialization ***/
IPAddress getlocalIP();
IPAddress myIP(0,0,0,0);

/*** Mesh Initialization ***/
painlessMesh  mesh;
Scheduler userScheduler;

/*** Sensor Initialization ***/
SensirionI2CSen5x sen5x;
Adafruit_INA219 ina219;
Adafruit_SGP30 sgp;

/*** Tasks ***/
Task taskSendTime( TASK_SECOND * 30 , TASK_FOREVER, &sendTime );
Task taskSendMessage( TASK_SECOND * 35 , TASK_FOREVER, &sendMessage );

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

void setup() {
  Serial.begin(115200);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

  #ifdef BRIDGE_NODE
  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);

  userScheduler.addTask( taskSendTime );
  taskSendTime.enable();
  #endif

  SEN55_init();
  SGP30_init();
  INA219_init();
  SD_init();

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

  pinMode(10, OUTPUT);
}

void loop() {
  mesh.update();
  if(myIP != getlocalIP()) {
    myIP = getlocalIP();
    Serial.println("My IP is " + myIP.toString());
    Serial.println(mesh.getNodeId());
    isInternet = true;
  }

  if (isInternet == true) {
    #ifdef BRIDGE_NODE
    if (!client.connected()) reconnect(); // check if client is connected
    client.loop();
    #endif
  } 
}

IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
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

/*** Initialize the sensors ***/
void SEN55_init() {
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

void SGP30_init() {
  if (! sgp.begin()){
    Serial.println("Sensor not found :(");
    while (1);
  }

  Serial.print("Found SGP30 serial #");
  Serial.print(sgp.serialnumber[0], HEX);
  Serial.print(sgp.serialnumber[1], HEX);
  Serial.println(sgp.serialnumber[2], HEX);
}

void INA219_init() {
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    while (1) { delay(10); }
  }

  Serial.println("Measuring voltage and current with INA219 ...");
}

/*** Creating a file in SD card ***/
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

/*** Appending data to a file ***/
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

/*** Initialize the SD card ***/
void SD_init() {
  // Initialize SD Card
  while (!SD.begin(0)) {
    Serial.println("Card Mount Failed");
  }

  File file = SD.open("/TestPhase_3.txt");
  if(!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/TestPhase_3.txt", "Time, Temperature (°C), Humidity (%), Wind Speed (m/s), Wind Direction (°), CO2 (ppm), TVOC (ppb), PM2.5 (ppm), PM10 (ppm), Voltage (V), Power (mW) \r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();
}

/*** Sensor measurement ***/
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

void getAnemometerData() {
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

/*** Saving readings to SD card ***/
void SD_log() {
  //Concatenate all info separated by commas
  dataMessage = time_stamp + ", " + String(ambientTemperature) + ", " + String(ambientHumidity) + ", " + String(windSpeed) + ", " + String(windGust) + ", " + String(windDirection) + ", " + String(CO2) + ", " + String(TVOC) + ", " + String(massConcentrationPm2p5) + ", " + String(massConcentrationPm10p0) + ", " + String(busvoltage) + ", " + String(power_mW) + "\r\n";
  Serial.print("Saving data: ");
  Serial.println(dataMessage);

  //Append the data to file
  appendFile(SD, "/TestPhase_3.txt", dataMessage.c_str());
}

/*** Called when a message is received ***/
void receivedCallback( const uint32_t &from, const String &msg ) {
  time_stamp = msg;
  Serial.println(time_stamp);
}

/*** Connect to MQTT Broker ***/
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

/*** Method for Publishing MQTT Messages ***/
void publishMessage(const char* topic, String payload , boolean retained){
  if (client.publish(topic, payload.c_str(), true))
    Serial.println("Message publised ["+String(topic)+"]: "+payload);
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

/*** Sink node function to broadcast time ***/
void sendTime() {
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return;
  }

  char buffer[50]; // Adjust the buffer size as needed
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", &timeinfo);
  
  String timeString = buffer;

  Serial.println(timeString);

  mesh.sendBroadcast( timeString );
}

/*** Child node function to send data to sink node ***/
void sendMessage() {
  SEN55_read();
  SGP30_read();
  INA219_read();
  // getAnemometerData();
  SD_log();

  JsonDocument doc;
  doc["deviceId"] = "CARE_Office_1";
  doc["Source"] = "Care Office";
  doc["local_time"] = time_stamp;

  JsonArray SEN55_data = doc.createNestedArray("SEN55_data");
  SEN55_data.add(ambientHumidity);
  SEN55_data.add(ambientTemperature);
  SEN55_data.add(massConcentrationPm2p5);
  SEN55_data.add(massConcentrationPm10p0);

  JsonArray SGP30_data = doc.createNestedArray("SGP30_data");
  SGP30_data.add(CO2);
  SGP30_data.add(TVOC);

  JsonArray INA219_data = doc.createNestedArray("INA219_data");
  INA219_data.add(busvoltage);
  INA219_data.add(power_mW);

  doc["type"] = "data";

  String mqtt_message;
  serializeJson(doc, mqtt_message);
  uint32_t target = 1973942425;
  if (mesh.sendSingle(target, mqtt_message) == 0) {
    Serial.println("Message not sent!");
    digitalWrite(10, LOW);
  }
  else {
    Serial.println("Message sent!");
    digitalWrite(10, HIGH);
  }
}