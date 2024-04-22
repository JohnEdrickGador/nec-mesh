#include <painlessMesh.h>
#include <WiFi.h>
#include <map>
#include <SPI.h>
#include <SD.h>

#define   WIFI_CHANNEL    6 //Check the access point on your router for the channel - 6 is not the same for everyone
#define   MESH_PREFIX     "test"
#define   MESH_PASSWORD   "password"
#define   MESH_PORT       5555

#define   STATION_SSID     "CARE_407"
#define   STATION_PASSWORD "nec_c@re"

#define   HOSTNAME         "MQTT_Bridge"

//Prototypes
void receivedCallback( const uint32_t &from, const String &msg );
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void sinkNodeElection();
void broadcastRSSI();
void connectToWifi();
void writeFile(fs::FS &fs, const char * path, const char * message);
void sendMessage();

Task taskBroadcastRSSI (TASK_SECOND * 10, TASK_FOREVER, &broadcastRSSI);
Task taskSinkNodeElection (TASK_SECOND * 60, TASK_FOREVER, &sinkNodeElection );

painlessMesh  mesh;
Scheduler userScheduler;

int my_rssi = 0;
int mesh_size = 3;
String nodeRSSIString = "";
std::map<uint32_t, int> nodeRSSIMap;
uint32_t target = 0;
bool sne_done = false;
bool isConnected = false;

void setup() {
  Serial.begin(115200);
  pinMode(10, OUTPUT);

  while (!SD.begin(0)) {
    Serial.println("Card Mount Failed");
  }

  File file = SD.open("/NodeID_RSSI4.txt");
  if(!file) {
    WiFi.begin(STATION_SSID, STATION_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {}
    Serial.println("Connected to WiFi!");
    my_rssi = WiFi.RSSI();
    WiFi.disconnect();
    Serial.println(my_rssi);
  }
  else {
    sne_done = true;
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
        nodeRSSIMap.insert({nodeID, RSSI});
      } 
      else {
        numberString = numberString + c;
      }
    }
  }
  file.close();

  mesh.setDebugMsgTypes( ERROR | STARTUP | CONNECTION );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT, WIFI_AP_STA, WIFI_CHANNEL );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);

  userScheduler.addTask( taskBroadcastRSSI );
  userScheduler.addTask( taskSinkNodeElection );

  while(mesh.getNodeList().size() + 1 != mesh_size) {
    mesh.update();
  }
  Serial.println("All nodes connected!");

  sinkNodeElection();
}

void loop() {
  mesh.update();
  if(!taskSinkNodeElection.isEnabled()){
    taskSinkNodeElection.enableDelayed(60000);
  }
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

void broadcastRSSI() {
  mesh.sendBroadcast(String(my_rssi));
  Serial.print("RSSI Broadcasted: ");
  Serial.println(my_rssi);
}

void connectToWifi() {
  mesh.stationManual(STATION_SSID, STATION_PASSWORD);
  mesh.setHostname(HOSTNAME);
  Serial.println("Connected to wifi!");
  isConnected = true;
  mesh.sendBroadcast("I am connected to the access point!");
}

void sendMessage() {
  if(mesh.sendSingle(target, "Hello from the child node!")) {
    Serial.println("Message sent to the sink node!");
  }
  else {
    Serial.println("Message to the sink node failed to send!");
  }
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

void sinkNodeElection() {
  nodeRSSIMap.insert({mesh.getNodeId(), my_rssi});
  
  while (nodeRSSIMap.size() != mesh_size) {
    taskBroadcastRSSI.enableIfNot();
    mesh.update();
  }

  if(!sne_done) {
    taskBroadcastRSSI.enableIfNot();
    taskBroadcastRSSI.setIterations(1);
  }

  int maxRSSI = INT_MIN; // Initialize to the smallest possible integer

  // Iterate through the map
  for (const auto& pair : nodeRSSIMap) {
    nodeRSSIString = nodeRSSIString + String(pair.first) + ":" + String(pair.second) + ",";
    Serial.println(nodeRSSIString);

    // Check if the current RSSI is greater than the maximum RSSI found so far
    if (pair.second > maxRSSI) {
      // Update the maximum RSSI and corresponding node ID
      if(pair.first == mesh.getNodeId() || (mesh.isConnected(pair.first))) {
        maxRSSI = pair.second;
        target = pair.first;
      }
    }
    else if (pair.second == maxRSSI) {
      if (pair.first < target) {
        if(pair.first == mesh.getNodeId() || mesh.isConnected(pair.first)){target = pair.first;}
      }
    }
  }

  sne_done = true;
  Serial.println("Sink node election done!");
  Serial.println("Sink node is " + String(target));

  File file = SD.open("/NodeID_RSSI4.txt");

  if(!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/NodeID_RSSI4.txt", nodeRSSIString.c_str());
  }
  else {
    Serial.println("File already exists");  
  }

  file.close();

  nodeRSSIString = "";

  if(mesh.getNodeId() == target) {
    connectToWifi();
    digitalWrite(10, HIGH);
  }

  else {
    sendMessage();
    mesh.sendBroadcast("My sink node is " + String(target));
    digitalWrite(10, LOW);
  }
}