#include <painlessMesh.h>
#include <WiFi.h>
#include <map>

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
void connect_to_wifi();
void broadcast_status();
void send_message();

Task taskBroadcastRSSI (TASK_SECOND * 10, TASK_FOREVER, &broadcast_rssi);
Task taskSendMessage( TASK_MINUTE * 1 , TASK_FOREVER, &send_message );
Task taskBroadcastStatus(TASK_SECOND * 30 , TASK_FOREVER, &broadcast_status );

painlessMesh  mesh;
Scheduler userScheduler;

int my_rssi = 0;
int mesh_size = 3;
std::map<uint32_t, int> nodeRSSIMap;
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
  userScheduler.addTask( taskBroadcastStatus );
  userScheduler.addTask( taskSendMessage );

  while(mesh.getNodeList().size() + 1 != mesh_size) {
    mesh.update();
  }
  Serial.println("All nodes connected!");

  sink_node_election();
}

void loop() {
  mesh.update();
}

void receivedCallback( const uint32_t &from, const String &msg ) {
  Serial.printf("bridge: Received from %u msg=%s\n", from, msg.c_str());
  if(!sne_done && msg.toInt() != 0) {
    nodeRSSIMap.insert({from, msg.toInt()});
    Serial.println("RSSI and nodeID pushed to map!");
  }
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

void broadcast_status() {
  mesh.sendBroadcast("I am still connected!");
}

void connect_to_wifi() {
  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to wifi!");
    isConnected = true;
  }
}

void send_message() {
  if(mesh.sendSingle(target, "Hello from the child node!")) {
    Serial.println("Message sent to the sink node!");
  }
  else {
    Serial.println("Message to the sink node failed to send!");
  }
}

void sink_node_election() {
  nodeRSSIMap.clear();
  
  nodeRSSIMap.insert({mesh.getNodeId(), my_rssi});
  
  while (nodeRSSIMap.size() != mesh_size) {
    taskBroadcastRSSI.enableIfNot();
    mesh.update();
  }

  int maxRSSI = INT_MIN; // Initialize to the smallest possible integer

  // Iterate through the map
  for (const auto& pair : nodeRSSIMap) {
    // Check if the current RSSI is greater than the maximum RSSI found so far
    if (pair.second > maxRSSI) {
      // Update the maximum RSSI and corresponding node ID
      maxRSSI = pair.second;
      target = pair.first;
    }
  }

  sne_done = true;
  Serial.println("Sink node election done!");
  Serial.println("Sink node is " + String(target));

  taskBroadcastRSSI.setIterations(1);

  if(mesh.getNodeId() == target) {
    connect_to_wifi();
    digitalWrite(10, HIGH);
    taskBroadcastStatus.enable();
  }

  else {
    taskSendMessage.enableDelayed(30000);
    mesh.sendBroadcast("My sink node is " + String(target));
    digitalWrite(10, LOW);
  }
}