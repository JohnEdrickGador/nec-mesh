#include <painlessMesh.h>
#include <WiFi.h>
#include <map>
#include <SPI.h>
#include <SD.h>

#define   WIFI_CHANNEL    1 //Check the access point on your router for the channel - 6 is not the same for everyone
#define   MESH_PREFIX     "NEC_1ST_FLOOR"
#define   MESH_PASSWORD   "CARE_OFFICE"
#define   MESH_PORT       5555

#define   STATION_SSID     "CARE_407"
#define   STATION_PASSWORD "nec_c@re"

#define   HOSTNAME         "MQTT_Bridge"

//Prototypes
void receivedCallback( const uint32_t &from, const String &msg );
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeDelayReceivedCallback(uint32_t from, int32_t delay);
void measureDelay();
void sdCreateFile(const char* fileName);
void sinkNodeElection();
void broadcastRSSI();
void connectToWifi();
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void sendMessage();

Task taskBroadcastRSSI (TASK_SECOND * 10, TASK_FOREVER, &broadcastRSSI);
Task taskMeasureDelay (TASK_SECOND * 10, TASK_FOREVER, &measureDelay);

painlessMesh  mesh;
Scheduler userScheduler;

int my_rssi = 0;
int mesh_size = 4;
String nodeRSSIString = "";
std::map<uint32_t, int> nodeRSSIMap;
uint32_t target = 0;
bool sne_done = false;
bool isConnected = false;
String delayString;

void setup() {
  Serial.begin(115200);
  pinMode(10, OUTPUT);

  while (!SD.begin(0)) {
    Serial.println("Card Mount Failed");
  }

  File file = SD.open("/NodeID_RSSI.txt");
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
  mesh.onNodeDelayReceived(&nodeDelayReceivedCallback);

  userScheduler.addTask( taskBroadcastRSSI );
  userScheduler.addTask( taskMeasureDelay );

  if(!sne_done) {
    while(mesh.getNodeList().size() + 1 != mesh_size) {
      mesh.update();
    }
    Serial.println("All nodes connected!");
  }

  Serial.println(mesh.getNodeId());

  sinkNodeElection();
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

void nodeDelayReceivedCallback(uint32_t from, int32_t delay) {
  delayString = String(delay) + "\r\n";
  if(from == 1974061657) {
    appendFile(SD, "/Node16DelayLog.txt", delayString.c_str());
  }
  else if (from == 1973094313) {
    appendFile(SD, "/Node17DelayLog.txt", delayString.c_str());
  }
  else if (from == 1973938737) {
    appendFile(SD, "/Node21DelayLog.txt", delayString.c_str());
  }
}

void measureDelay() {
  if (mesh.startDelayMeas(target)) {
    Serial.printf("Started delay measurement to node %u\n", target);
  } else {
    Serial.printf("Failed to start delay measurement to node %u\n", target);
  }
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

void sdCreateFile(const char* fileName) {
  // Initialize SD Card
  while (!SD.begin(0)) {
    Serial.println("Card Mount Failed");
  }

  File file = SD.open(fileName);

  if(!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating delay log file...");
    writeFile(SD, fileName, "Delay Time \r\n");
  }
  else {
    Serial.println("File already exists");  
  }

  file.close();
}

void sinkNodeElection() {
  nodeRSSIMap.insert({mesh.getNodeId(), my_rssi});
  
  while (nodeRSSIMap.size() != mesh_size) {
    mesh.update();
  }

  int maxRSSI = INT_MIN; // Initialize to the smallest possible integer

  // Iterate through the map
  for (const auto& pair : nodeRSSIMap) {
    nodeRSSIString = nodeRSSIString + String(pair.first) + ":" + String(pair.second) + ",";

    // Check if the current RSSI is greater than the maximum RSSI found so far
    if (pair.second > maxRSSI) {
      // Update the maximum RSSI and corresponding node ID
      maxRSSI = pair.second;
      target = pair.first;
    }
    else if (pair.second == maxRSSI) {
      if (pair.first < target) {
        target = pair.first;
      }
    }
  }

  Serial.println("Sink node election done!");
  Serial.println("Sink node is " + String(target));

  File file = SD.open("/NodeID_RSSI.txt");

  if(!file) {
    Serial.println("File doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/NodeID_RSSI.txt", nodeRSSIString.c_str());
  }
  else {
    Serial.println("File already exists");
  }

  file.close();

  if(mesh.getNodeId() == target) {
    // if(!isConnected) {connectToWifi();}
    digitalWrite(10, HIGH);
    sdCreateFile("/Node16DelayLog.txt");
    sdCreateFile("/Node17DelayLog.txt");
    sdCreateFile("/Node21DelayLog.txt");
  }

  else {
    // sendMessage();
    digitalWrite(10, LOW);
    mesh.sendBroadcast("My sink node is " + String(target));
    taskMeasureDelay.enable();
  }
}