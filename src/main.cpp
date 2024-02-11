#include <Arduino.h>

//************************************************************
// this is a simple example that uses the painlessMesh library to
// connect to a another network and relay messages from a MQTT broker to the nodes of the mesh network.
// To send a message to a mesh node, you can publish it to "painlessMesh/to/12345678" where 12345678 equals the nodeId.
// To broadcast a message to all nodes in the mesh you can publish it to "painlessMesh/to/broadcast".
// When you publish "getNodes" to "painlessMesh/to/gateway" you receive the mesh topology as JSON
// Every message from the mesh which is send to the gateway node will be published to "painlessMesh/from/12345678" where 12345678 
// is the nodeId from which the packet was send.
//************************************************************
#include <painlessMesh.h>
#include <PubSubClient.h>
#include <WiFiClient.h>

#define   WIFI_CHANNEL    1 //Check the access point on your router for the channel - 6 is not the same for everyone
#define   MESH_PREFIX     "whateveryouwant"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

#define   STATION_SSID     "Jarvis"
#define   STATION_PASSWORD "0n3_Voy@ger!"

//MQTT Server Configs
const char* mqtt_server = "mqtt3.thingspeak.com";
const char* publishTopic ="channels/2407401/publish";
const char* subscribeTopic1 = "channels/2407401/subscribe/fields/field1";   

#define   BRIDGE_NODE

#define HOSTNAME "MQTT_Bridge"

// Prototypes
void receivedCallback( const uint32_t &from, const String &msg );
void mqttCallback(char* topic, byte* payload, unsigned int length);
void sendMessage(); // Prototype so PlatformIO doesn't complain
void publishMQTT();
void mqttReconnect();

IPAddress getlocalIP();

IPAddress myIP(0,0,0,0);
IPAddress mqttBroker(192, 168, 1, 128);

painlessMesh  mesh;
WiFiClient wifiClient;
Scheduler userScheduler;
PubSubClient mqttClient("mqtt3.thingspeak.com", 1883, wifiClient);

//Tasks
Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );
Task taskPublishMQTT( TASK_SECOND * 20, TASK_FOREVER, &publishMQTT );
Task taskMQTTReconnect( TASK_SECOND * 5, TASK_FOREVER, &mqttReconnect );

void setup() {
  Serial.begin(115200);

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
  userScheduler.addTask( taskMQTTReconnect );
  taskSendMessage.enable();
  taskPublishMQTT.enable();
}

void loop() {
  mesh.update();
  #ifdef BRIDGE_NODE
  mqttClient.loop();
  #endif

  if(myIP != getlocalIP()){
    myIP = getlocalIP();
    Serial.println("My IP is " + myIP.toString());

    #ifdef BRIDGE_NODE
    if (!mqttClient.connected()) {
      taskMQTTReconnect.enable();
    } else {
      taskMQTTReconnect.disable();
    }
    #endif
  }

}

void receivedCallback( const uint32_t &from, const String &msg ) {
  Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
  // String topic = "painlessMesh/from/" + String(from);
  // mqttClient.publish(topic.c_str(), msg.c_str());
}

void sendMessage() {
  String msg = "Random number: " + String(random(1,20));
  mesh.sendBroadcast( msg );
  taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}

void publishMQTT() {
  String payload = "field1=" + String(random(30));
  if (mqttClient.publish(publishTopic, payload.c_str())) {
    Serial.println("Message Published");
  }
}

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect("AhAkNAQDMg8cBB01OjcmOQ0", "AhAkNAQDMg8cBB01OjcmOQ0", "W3L+BcOWPGT6/LbG1xBvasAN")) {
      Serial.println("connected");
      mqttClient.subscribe(subscribeTopic1);
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");   // Wait 5 seconds before retrying
      
    }
  }
}



IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
}