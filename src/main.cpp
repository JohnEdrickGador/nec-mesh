#include <Arduino.h>
#include <WiFi.h>
#include <string>
#include <iostream>
#include <map>

//************************************************************
// this is a simple example that uses the painlessMesh library
//
// 1. sends a silly message to every node on the mesh at a random time between 1 and 5 seconds
// 2. prints anything it receives to Serial.print
//
//
//************************************************************
#include "painlessMesh.h"

#define   MESH_PREFIX     "whateverYouLike"
#define   MESH_PASSWORD   "somethingSneaky"
#define   MESH_PORT       5555

const char* ssid = "CARE-MESH";
const char* password = "edrick123";

//global variables
bool isGateway = false;
int gatewayNodeId = 0;
unsigned long lastBroadcast = 0;
unsigned long lastGatewayCheck = 0;

//create a dictionary for the rssi of each node
std::map<int, int> node_rssi;


Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

// User stub
void sendMessage() ; // Prototype so PlatformIO doesn't complain
void checkGatewayStatus();
void broadcastWifiInfo();

Task taskSendMessage( TASK_SECOND * 1 , TASK_FOREVER, &sendMessage );
Task taskCheckGateway(TASK_SECOND * 5 , TASK_FOREVER , &checkGatewayStatus);
Task taskBroadcastWifiInfo(TASK_SECOND * 10, TASK_FOREVER, &broadcastWifiInfo);

void sendMessage() {
  String msg = "Hello from node 2 ";
  /*msg += mesh.getNodeId(); */
  mesh.sendBroadcast( msg );
  taskSendMessage.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 5 ));
}

void checkGatewayStatus() {
  // ... logic to check gateway status and initiate failover if needed ...
  if (!WiFi.isConnected() || (isGateway && !mesh.isRoot())) {
    String electionMessage = "ELECTION";
    mesh.sendBroadcast(electionMessage);
    isGateway = false;
    return;
  }
}

void broadcastWifiInfo() {
  // ... logic to broadcast Wi-Fi information to other nodes ...
  // Check if connected to Wi-Fi before broadcasting:
  if (WiFi.isConnected()) {
    // Construct the message in the "WIFI_INFO:rssi,status" format:
    String wifiInfo = String(WiFi.RSSI()) + "," + String(WiFi.status()) + "," + String (mesh.getNodeId());

    // Broadcast the message using painlessMesh:
    mesh.sendBroadcast("WIFI_INFO:" + wifiInfo, true);
  }
}

void voteForGateway() {
  // ... logic to select the best gateway candidate and announce the winner ...
  broadcastWifiInfo();
}

void becomeGateway() {
  isGateway = true;
  mesh.setRoot(true);
  // ... additional setup for gateway node ...
}

void relinquishGateway() {
  isGateway = false;
  mesh.setRoot(false);
  // ... cleanup for non-gateway node ...
}

bool startsWith(const String& str, const String& prefix) {
  if (str.length() < prefix.length()) return false;
  return strncmp(str.c_str(), prefix.c_str(), prefix.length()) == 0;
}

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  if (msg == "ELECTION") {
    voteForGateway();
  } else if (msg.startsWith("WIFI_INFO")) {
    // ... parse Wi-Fi information and update candidate list ...
    String messageData = msg.substring(10);

    if (messageData.indexOf(',') < 0) {
      Serial.println("Invalid WIFI_INFO message format: missing commas");
      return;
    }

    String rssiString = messageData.substring(0, messageData.indexOf(','));
    int rssi;
    if (!rssiString.toInt()) {
      Serial.println("Failed to convert rssi");
      return;
    }
    rssi = rssiString.toInt();

    String nodeIdString = messageData.substring(messageData.indexOf(','), messageData.indexOf(',') + 2);
    int nodeId;
    if (!nodeIdString.toInt()) {
      Serial.println("Failed to convert nodeId");
    }
    nodeId = nodeIdString.toInt();

    node_rssi[nodeId] = rssi;

  }
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void setup() {
  Serial.begin(115200);

//mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  userScheduler.addTask( taskSendMessage );
  taskSendMessage.enable();
  //Connect to WiFi if not already the gateway
  if (!isGateway) {
    WiFi.begin(ssid, password);
    Serial.println("connected to wifi");
  }

  //start a task for periodic gateway cecks and wifi broadcasts
  userScheduler.addTask(taskCheckGateway);
  taskCheckGateway.enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
