#include <painlessMesh.h>
#include <WiFi.h>
#include <vector>
#include <algorithm>

#define   WIFI_CHANNEL    11 //Check the access point on your router for the channel - 6 is not the same for everyone
#define   MESH_PREFIX     "test"
#define   MESH_PASSWORD   "password"
#define   MESH_PORT       5555

#define   STATION_SSID     "HG8145V5_7185A"
#define   STATION_PASSWORD "X7BBwEDt"

#define   HOSTNAME         "MQTT_Bridge"

//Prototypes
void receivedCallback( const uint32_t &from, const String &msg );
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void sink_node_election();
void broadcast_rssi();
void broadcast_dc();
void connect_to_wifi();
void send_message();

Task taskBroadcastRSSI (TASK_SECOND * 5, TASK_FOREVER, &broadcast_rssi);
Task taskBroadcastDC (TASK_SECOND * 5, TASK_FOREVER, &broadcast_dc);
Task taskSendMessage( TASK_MINUTE * 1 , TASK_FOREVER, &send_message );

painlessMesh  mesh;
Scheduler userScheduler;

int my_rssi = 0;
int mesh_size = 4;
std::vector<int> RSSI_vec;
std::vector<uint32_t> nodeID_vec;
int target_idx = 0;
uint32_t target = 0;
bool sne_done = false;
bool isConnected = false;

void setup() {
  Serial.begin(115200);
  pinMode(10, OUTPUT);

  WiFi.begin(STATION_SSID, STATION_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {}
  Serial.println("Connected to WiFi!");
  my_rssi = WiFi.RSSI();
  WiFi.disconnect();
  Serial.println(my_rssi);

  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, WIFI_CHANNEL );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);

  userScheduler.addTask( taskBroadcastRSSI );
  userScheduler.addTask( taskBroadcastDC );
  userScheduler.addTask( taskSendMessage );

  while(mesh.getNodeList().size() + 1 != mesh_size) {
    mesh.update();
  }
  Serial.println("All nodes connected!");

  sink_node_election();
}

void loop() {
  mesh.update();
  if(sne_done) {
    if(!mesh.isConnected(target)) {
      sne_done = false;
      taskSendMessage.abort();
      Serial.println("Sink node has been disconnected!");
      mesh_size = mesh.getNodeList().size() + 1;
      sink_node_election();
    }

    // if(mesh.getNodeId() == target) {
    //   if (WiFi.status() != WL_CONNECTED) {
    //     taskBroadcastDC.enableIfNot();
    //     taskBroadcastDC.setIterations(5);
    //   }
    // }
  }
}

void receivedCallback( const uint32_t &from, const String &msg ) {
  Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
  if(!sne_done) {
    if(count(nodeID_vec.begin(), nodeID_vec.end(), from) == 0) {
      RSSI_vec.push_back(msg.toInt());
      nodeID_vec.push_back(from);
      Serial.println("RSSI and nodeID pushed to vector!");
    }
  }
  
  // else if (msg == "Disconnected!") {
  //   sne_done = false;
  //   taskSendMessage.abort();
  //   Serial.println("Sink node has been disconnected from the internet!");
  //   mesh_size = mesh.getNodeList().size() + 1;
  //   sink_node_election();
  // }
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void broadcast_rssi() {
  mesh.sendBroadcast(String(my_rssi));
  Serial.print("RSSI Broadcasted: ");
  Serial.println(my_rssi);
}

void connect_to_wifi() {
  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to wifi!");
    isConnected = true;
  }
}

void broadcast_dc() {
  mesh.sendBroadcast("Disconnected!");
  Serial.println("Message broadcasted!");
}

void send_message() {
  mesh.sendSingle(target, "Hello from the child node!");
  Serial.println("Message sent!");
}

void sink_node_election() {
  RSSI_vec.push_back(my_rssi);
  nodeID_vec.push_back(mesh.getNodeId());
  
  while (RSSI_vec.size() != mesh_size) {
    taskBroadcastRSSI.enableIfNot();
    mesh.update();
  }

  target_idx = std::max_element(RSSI_vec.begin(), RSSI_vec.end()) - RSSI_vec.begin();
  target = nodeID_vec[target_idx];
  Serial.print("Sink node is ");
  Serial.println(target);

  RSSI_vec.clear();
  nodeID_vec.clear();

  sne_done = true;
  Serial.println("Sink node election done!");

  taskBroadcastRSSI.setIterations(5);

  if(mesh.getNodeId() == target) {
    connect_to_wifi();
    digitalWrite(10, HIGH);
  }

  else {
    taskSendMessage.enableIfNot();
    digitalWrite(10, LOW);
  }
}