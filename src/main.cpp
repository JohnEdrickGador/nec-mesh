#include <Arduino.h>
#include <painlessMesh.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <WiFi.h>

#define   WIFI_CHANNEL    11 //Check the access point on your router for the channel - 6 is not the same for everyone
#define   MESH_PREFIX     "whateveryouwant"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555
#define   MESH_SIZE       2

#define   STATION_SSID     "CARE_407"
#define   STATION_PASSWORD "nec_c@re"

#define   HOSTNAME         "MQTT_Bridge"

//Prototypes
void receivedCallback( const uint32_t &from, const String &msg );
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void broadcast_rssi();
void connect_to_wifi();
void send_message();

Task taskSendMessage( TASK_MINUTE * 1 , TASK_FOREVER, &send_message );

IPAddress getlocalIP();

IPAddress myIP(0,0,0,0);

painlessMesh  mesh;
WiFiClient wifiClient;
Scheduler userScheduler;

bool isInternet = false;
int8_t my_rssi = 0;
uint32_t target = 0;
bool sink_node_election = false;
bool isConnected = false;
bool task_enabled = false;

void setup() {
  Serial.begin(115200);
  pinMode(10, OUTPUT);

  WiFi.begin(STATION_SSID, STATION_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {}
  Serial.println("Connected to WiFi!");
  my_rssi = WiFi.RSSI();
  WiFi.disconnect();

  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, WIFI_CHANNEL );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);

  // Channel set to 6. Make sure to use the same channel for your mesh and for you other
  // network (STATION_SSID)

  broadcast_rssi();

  userScheduler.addTask( taskSendMessage );
}

void loop() {
  mesh.update();

  if(myIP != getlocalIP()){
    myIP = getlocalIP();
    Serial.println("My IP is " + myIP.toString());
    isInternet = true;
  }
  
  if (mesh.getNodeList().size() + 1 == MESH_SIZE) {
    if(mesh.getNodeId() == target && !isConnected) {
      digitalWrite(10, HIGH);
      Serial.println("I am the sink node!");
      connect_to_wifi();
    }
    else if (mesh.getNodeId() != target && !task_enabled) {
      taskSendMessage.enable();
      task_enabled = true;
    }
  }
}

void receivedCallback( const uint32_t &from, const String &msg ) {
  Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
  if (msg.toInt() != 0) {
    if(my_rssi > msg.toInt()) {
      target = mesh.getNodeId();
      Serial.print("Sink node ID is ");
      Serial.println(target);
    }
    else {
      target = from;
      Serial.print("Sink node ID is ");
      Serial.println(target);
    }
  }
}

IPAddress getlocalIP() {
  return IPAddress(mesh.getStationIP());
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
  broadcast_rssi();
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
  broadcast_rssi();
}

void broadcast_rssi() {
  mesh.sendBroadcast(String(my_rssi));
  Serial.print("RSSI Broadcasted: ");
  Serial.println(my_rssi);
}

void connect_to_wifi() {
  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);
  Serial.println("Connected to wifi!");
  isConnected = true;
}

void send_message() {
  mesh.sendSingle(target, "Hello from the child node!");
  Serial.println("Message sent!");
}