#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include "C:\Users\user\Documents\Arduino\libraries\MAVLink\common\mavlink.h"
#include <chrono>
#include <vector>
#include <deque>
#include <string>
#include <queue>
#include <map>



#define DRONE_ID 1
#define RX_PIN 16
#define TX_PIN 17

HardwareSerial pixhawkSerial(2);

std::deque<std::string> statusMessages;
const size_t MAX_STATUS_MESSAGES = 100;

unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 3000; // 5 seconds
const unsigned long MAVLINK_TIMEOUT = 3000;
const unsigned long TELEMETRY_INTERVAL = 1000;
const unsigned long GPS_TIME_INTERVAL = 1000;
const unsigned long STATUS_MESSAGE_INTERVAL = 1000;

std::queue<mavlink_message_t> message_queue;
const int MAX_QUEUE_SIZE = 20;

// WiFi and MQTT settings
const char* ssid = "TP-Link_5734";
const char* password = "70628739";
const char* mqtt_server = "192.168.0.18";
const char* mqtt_command_topic = "drone/command";
const char* mqtt_rtcm_topic = "drone/rtcm";
char mqtt_status_topic[50];
char mqtt_mission_topic[50];
char mqtt_telemetry_topic[50];
char mqtt_individual_command_topic[50];
char mqtt_individual_status_topic[50];

#define MQTT_MAX_PACKET_SIZE 2048

// GPS timing info
unsigned long systemTimeAtLastGPSUpdate = 0;
uint64_t last_gps_time_us = 0;
bool timeInitialized = false;

// MAVLink IDs
#define SYSTEM_ID 255
#define COMPONENT_ID MAV_COMP_ID_MISSIONPLANNER

// Flight modes
#define STABILIZE_MODE 0
#define GUIDED_MODE 4
#define LAND_MODE 9 
#define AUTO_MODE 3

struct Waypoint {
    float lat;
    float lon;
    float alt;
    uint64_t time;  // Timestamp in microseconds
    bool visited;   // Flag to track if we've navigated to this waypoint
};

std::vector<Waypoint> mission;
size_t currentWaypointIndex = 0;
uint64_t missionStartTime = 0;
bool missionStarted = false;
bool armed = false;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastTelemetryTime = 0;
unsigned long lastGPSTime = 0;
unsigned long lastStatusRequestTime = 0;

mavlink_message_t msg;
mavlink_status_t status;

const std::map<uint8_t, const char*> flightModes = {
    {0, "STABILIZE"},
    {1, "ACRO"},
    {2, "ALT_HOLD"},
    {3, "AUTO"},
    {4, "GUIDED"},
    {5, "LOITER"},
    {6, "RTL"},
    {7, "CIRCLE"},
    {9, "LAND"},
    {11, "DRIFT"},
    {13, "SPORT"},
    {14, "FLIP"},
    {15, "AUTOTUNE"},
    {16, "POSHOLD"},
    {17, "BRAKE"},
    {18, "THROW"},
    {19, "AVOID_ADSB"},
    {20, "GUIDED_NOGPS"},
    {21, "SMART_RTL"},
    {22, "FLOWHOLD"},
    {23, "FOLLOW"},
    {24, "ZIGZAG"},
    {25, "SYSTEMID"},
    {26, "AUTOROTATE"},
    {27, "AUTO_RTL"}
};

void setup() {
  Serial.begin(115200);
  pixhawkSerial.begin(57600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
  
  delay(1000);
  
  snprintf(mqtt_status_topic, sizeof(mqtt_status_topic), "drone/%d/status", DRONE_ID);
  snprintf(mqtt_mission_topic, sizeof(mqtt_mission_topic), "drone/%d/mission", DRONE_ID);
  snprintf(mqtt_telemetry_topic, sizeof(mqtt_telemetry_topic), "drone/%d/telemetry", DRONE_ID);
  snprintf(mqtt_individual_command_topic, sizeof(mqtt_individual_command_topic), "drone/%d/command", DRONE_ID);
  snprintf(mqtt_individual_status_topic, sizeof(mqtt_individual_status_topic), "drone/%d/status_messages", DRONE_ID);
  
  Serial.println("ESP32 started. Waiting for MQTT commands.");
}

void loop() {
  if (!client.connected()) {
    if (reconnect()) {
      lastReconnectAttempt = 0;
    }
  }
  client.loop();
  
  while (pixhawkSerial.available()) {
    uint8_t c = pixhawkSerial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (message_queue.size() < MAX_QUEUE_SIZE) {
        message_queue.push(msg);
      }
    }
  }
 // Process messages in the queue
  while (!message_queue.empty()) {
    mavlink_message_t current_msg = message_queue.front();
    message_queue.pop();
    handle_mavlink_message(&current_msg);
  }


  executeMission();

  // Send telemetry data at regular intervals
  unsigned long currentTime = millis();
 
  // if (currentTime - lastStatusRequestTime > STATUS_MESSAGE_INTERVAL) { 
  //   requestStatusMessages();
  //   lastStatusRequestTime = currentTime;
  // }
  if (currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    sendTelemetryData();
    lastTelemetryTime = currentTime;
    // printDebugInfo();
  }
  currentTime = millis();
  if (currentTime - lastGPSTime >= GPS_TIME_INTERVAL) {
    getCurrentTimeUs();
    lastGPSTime = currentTime;
  }
}

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println("IP address: " + WiFi.localIP().toString());
}

boolean reconnect() {
  if (millis() - lastReconnectAttempt < RECONNECT_INTERVAL) {
    return false;
  }
  
  lastReconnectAttempt = millis();
  Serial.print("Attempting MQTT connection...");
  
  // Create a unique client ID
  String clientId = "ESP32Client_";
  clientId += String(DRONE_ID);
  clientId += "_";
  clientId += String(random(0xffff), HEX);
  
  if (client.connect(clientId.c_str())) {
    Serial.println("connected");
    client.subscribe(mqtt_command_topic);
    client.subscribe(mqtt_mission_topic);
    client.subscribe(mqtt_individual_command_topic);
    client.subscribe(mqtt_rtcm_topic);
    publishStatus("connected");
    Serial.println("Subscribed to topics: ");
    Serial.println(mqtt_command_topic);
    Serial.println(mqtt_mission_topic);
    Serial.println(mqtt_individual_command_topic);
    Serial.println(mqtt_rtcm_topic);
    return true;
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
    return false;
  }
}

bool waitForAck(uint8_t expectedMsgId, unsigned long timeout) {
    auto start = std::chrono::high_resolution_clock::now();
    while (true) {
        if (pixhawkSerial.available()) {
            mavlink_message_t msg;
            mavlink_status_t status;
            uint8_t c = pixhawkSerial.read();
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                // Serial.print("Received message ID: ");
                // Serial.println(msg.msgid);
                if (msg.msgid == expectedMsgId || msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST || msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
                    if (msg.msgid == MAVLINK_MSG_ID_MISSION_ACK) {
                        mavlink_mission_ack_t ack;
                        mavlink_msg_mission_ack_decode(&msg, &ack);
                        Serial.print("Mission ACK type: ");
                        Serial.println(ack.type);
                        if (ack.type != MAV_MISSION_ACCEPTED) {
                            Serial.print("Mission command not accepted. Error: ");
                            Serial.println(ack.type);
                            return false;
                        }
                    }
                    return true;
                }
            }
        }
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - start);
        if (duration.count() > timeout) {
            Serial.println("Timeout waiting for ACK");
            return false;
        }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.println("Message received on topic: " + String(topic));
  
  DynamicJsonDocument doc(2048);
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (strcmp(topic, mqtt_mission_topic) == 0) {
    Serial.println("Received mission data");
    handleMissionUpload(doc);
  } else if (strcmp(topic, mqtt_command_topic) == 0 || strcmp(topic, mqtt_individual_command_topic) == 0) {
    if (doc.containsKey("command")) {
      String command = doc["command"];
      Serial.print("Received command: ");
      Serial.println(command);
      if (command == "arm") {
        armDrone();
      } else if (command == "takeoff") {
        float altitude = doc["altitude"];
        takeoff(altitude);
      } else if (command == "change_mode") {
        String mode = doc["mode"];
        if (mode == "stabilize") {
          changeFlightMode(STABILIZE_MODE);
        } else if (mode == "guided") {
          changeFlightMode(GUIDED_MODE);
        } else if (mode == "land") {
          Serial.println("Starting to land");
          changeFlightMode(LAND_MODE);
        } else if (mode == "auto") {
          changeFlightMode(AUTO_MODE);
        }
        else{
          Serial.println("Going to else part");
        }
      } else if (command == "start_mission") {
        float takeoffAltitude = doc["takeoffAltitude"].as<float>();
        startMission(takeoffAltitude);
      }
    }
  } else if (strcmp(topic, mqtt_rtcm_topic) == 0) {
    JsonArray rtcmData = doc["data"].as<JsonArray>();
    uint8_t buffer[2000];  // Increased to 2000 bytes
    size_t dataLen = 0;
    
    if (rtcmData.size() > sizeof(buffer)) {
        Serial.print("RTCM message too large: ");
        Serial.println(rtcmData.size());
        Serial.print("Maximum size supported: ");
        Serial.println(sizeof(buffer));
        return;
    }
    
    for (JsonVariant v : rtcmData) {
        buffer[dataLen++] = v.as<uint8_t>();
    }
    
    // MAVLink GPS_RTCM_DATA has a maximum payload size limitation
    const size_t MAX_CHUNK_SIZE = 180; // Maximum safe size for GPS_RTCM_DATA message
    size_t chunks = (dataLen + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE; // Calculate number of chunks needed
    
    Serial.print("RTCM data size: ");
    Serial.print(dataLen);
    Serial.print(" bytes, splitting into ");
    Serial.print(chunks);
    Serial.println(" chunks");
    
    for (size_t i = 0; i < dataLen; i += MAX_CHUNK_SIZE) {
      mavlink_message_t msg;
      uint8_t mavBuffer[MAVLINK_MAX_PACKET_LEN];
      
      size_t chunkSize = min(MAX_CHUNK_SIZE, dataLen - i);
      
      mavlink_msg_gps_rtcm_data_pack(SYSTEM_ID, COMPONENT_ID, &msg,
          (chunks > 1) ? 1 : 0,  // Set flag if message is fragmented
          chunkSize,             // Length of this chunk
          &buffer[i]             // Pointer to start of this chunk
      );
      
      uint16_t len = mavlink_msg_to_send_buffer(mavBuffer, &msg);
      pixhawkSerial.write(mavBuffer, len);
      
      // Log chunk information
      Serial.print("Sent chunk ");
      Serial.print(i / MAX_CHUNK_SIZE + 1);
      Serial.print(" of ");
      Serial.print(chunks);
      Serial.print(", size: ");
      Serial.println(chunkSize);
      
      // Small delay between chunks to prevent buffer overflow
      if (chunks > 1) {
          delay(5);
      }
    }
  }
}

void handle_mavlink_message(mavlink_message_t *msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      // Process heartbeat
      mavlink_heartbeat_t heartbeat;
      mavlink_msg_heartbeat_decode(msg, &heartbeat);
      armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
      // Update connection status
      publishStatus("connected");
      break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      // Process global position
      mavlink_global_position_int_t global_pos;
      mavlink_msg_global_position_int_decode(msg, &global_pos);
      // Update position in telemetry data
      break;
    case MAVLINK_MSG_ID_SYS_STATUS:
      // Process system status
      mavlink_sys_status_t sys_status;
      mavlink_msg_sys_status_decode(msg, &sys_status);
      // Update battery status in telemetry data
      break;
    case MAVLINK_MSG_ID_STATUSTEXT: {
      mavlink_statustext_t statustext;
      mavlink_msg_statustext_decode(msg, &statustext);
      Serial.println(statustext.text);
      addStatusMessage(statustext.text);
      break;
    }
    // Add more message handlers as needed
  }
}


// Helper function to format timestamp
void formatTimestamp(char* buffer, size_t bufferSize, uint64_t timestamp_us) {
    uint64_t timestamp_s = timestamp_us / 1000000; // Convert to seconds
    uint32_t hours = (timestamp_s % 86400) / 3600;
    uint32_t minutes = (timestamp_s % 3600) / 60;
    uint32_t seconds = timestamp_s % 60;
    
    snprintf(buffer, bufferSize, "%02d:%02d:%02d", hours, minutes, seconds);
}

void printDebugInfo() {
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 5000) { // Print every 5 seconds
    Serial.println("Debug Info:");
    Serial.print("Messages in queue: ");
    Serial.println(message_queue.size());
    Serial.print("Status messages stored: ");
    Serial.println(statusMessages.size());
    Serial.print("Last known armed state: ");
    Serial.println(armed ? "ARMED" : "DISARMED");
    lastDebugTime = millis();
  }
}

// Add this new function to handle status messages
void addStatusMessage(const char* message) {
    char timestamp[20];
    formatTimestamp(timestamp, sizeof(timestamp), getCurrentTimeUsLite());
    
    char fullMessage[256];
    snprintf(fullMessage, sizeof(fullMessage), "%s: %s", timestamp, message);
    
    statusMessages.push_front(std::string(fullMessage));
    if (statusMessages.size() > MAX_STATUS_MESSAGES) {
        statusMessages.pop_back();
    }
    
    char mqttMessage[300];
    Serial.println("Status Message::");
    Serial.println(fullMessage);
    snprintf(mqttMessage, sizeof(mqttMessage), "{\"message\": \"%s\"}", fullMessage);
    client.publish(mqtt_individual_status_topic, mqttMessage);
}

// Add this new function to request status messages
void requestStatusMessages() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_REQUEST_MESSAGE, 0, MAVLINK_MSG_ID_STATUSTEXT, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);
}


void publishStatus(const char* status) {
  DynamicJsonDocument doc(256);
  doc["status"] = status;
  char buffer[256];
  serializeJson(doc, buffer);
  client.publish(mqtt_status_topic, buffer);
}

void requestMAVLinkData() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Request GLOBAL_POSITION_INT
  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_REQUEST_MESSAGE, 0, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);

  // Request SYS_STATUS for battery info
  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_REQUEST_MESSAGE, 0, MAVLINK_MSG_ID_SYS_STATUS, 0, 0, 0, 0, 0, 0);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);
}


void sendTelemetryData() {
  requestMAVLinkData();

  unsigned long startTime = millis();
  bool gotPosition = false;
  bool gotStatus = false;
  bool gotHeartbeat = false;
  float altitude = 0;
  float battery_voltage = 0;
  uint8_t base_mode = 0;
  uint32_t custom_mode = 0;

  while ((!gotPosition || !gotStatus || !gotHeartbeat) && (millis() - startTime < 1000)) {
    if (pixhawkSerial.available()) {
      mavlink_message_t msg;
      mavlink_status_t status;
      uint8_t c = pixhawkSerial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        switch (msg.msgid) {
          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(&msg, &pos);
            altitude = pos.relative_alt / 1000.0f;
            gotPosition = true;
            break;
          }
          case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            battery_voltage = sys_status.voltage_battery / 1000.0f;
            gotStatus = true;
            break;
          }
          case MAVLINK_MSG_ID_HEARTBEAT: {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&msg, &heartbeat);
            base_mode = heartbeat.base_mode;
            custom_mode = heartbeat.custom_mode;
            gotHeartbeat = true;
            break;
          }
        }
      }
    }
  }

  const char* mode_str = getFlightModeString(base_mode, custom_mode);

  DynamicJsonDocument doc(1024);
  doc["altitude"] = altitude;
  doc["battery"] = battery_voltage;
  doc["mode"] = mode_str;
  // Serial.println("Telemetry data: ALTITUDE");
  // Serial.print(altitude);
  // Serial.println("Telemetry data: Battery Voltage");
  // Serial.print(battery_voltage);
  // Serial.println("Telemetry data: Mode");
  // Serial.print(mode_str);
  
  if (timeInitialized) {
    doc["timestamp"] = getCurrentTimeUsLite();
  } else {
    doc["timestamp"] = "not_initialized";
  }

  char buffer[1024];
  serializeJson(doc, buffer);
  client.publish(mqtt_telemetry_topic, buffer);
}

const char* getFlightModeString(uint8_t base_mode, uint32_t custom_mode) {
  if (base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
    const char* mode_str;
    auto it = flightModes.find(custom_mode);
    if (it != flightModes.end()) {
        mode_str = it->second;
    } else {
        mode_str = "UNKNOWN";
    }
    return mode_str;
  } else if (base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED) {
    return "Manual";
  } else {
    return "Unknown";
  }
}

void handleMissionUpload(JsonDocument& doc) {
    Serial.println("Processing mission upload");
    JsonArray waypoints = doc["waypoints"].as<JsonArray>();
    
    if (waypoints.size() > 0) {
        mission.clear();
        for (JsonObject waypoint : waypoints) {
            float lat = waypoint["lat"];
            float lon = waypoint["lon"];
            float alt = waypoint["alt"];
            uint32_t time = waypoint["time"];
            mission.push_back({lat, lon, alt, time * 1000ULL, false});  // Convert milli to microseconds
        }
        missionStarted = false;
        currentWaypointIndex = 0;
        client.publish(mqtt_status_topic, "{\"status\": \"mission uploaded successfully\"}");
    } else {
        client.publish(mqtt_status_topic, "{\"status\": \"error: invalid mission data\"}");
    }
    publishStatus("mission uploaded");
}


void clearMission() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_clear_all_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);
}

void startMission(float takeoffAltitude) {
  if (!mission.empty() && !missionStarted) {
    // Arm the drone if not armed
    if (!isArmed()) {
      publishStatus("Drone not armed");
    }
    
    // Takeoff to specified altitude
    // This should automatically switch to GUIDED mode if necessary
    takeoff(takeoffAltitude);
    
    // Wait for altitude to be reached
    unsigned long start = millis();
    while (getCurrentAltitude() < takeoffAltitude * 0.95 && millis() - start < 10000) { // Wait up to 10 seconds
      delay(500);
    }
    
    missionStartTime = getCurrentTimeUsLite();
    missionStarted = true;
    currentWaypointIndex = 0;
    publishStatus("mission started");
  } else {
    publishStatus("Cannot start mission: Either mission is empty or already started");
  }
}


void executeMission() {
    if (!missionStarted || mission.empty()) return;

    uint64_t currentTime = getCurrentTimeUsLite();
    uint64_t elapsedTime = currentTime - missionStartTime;

    // Find the next unvisited waypoint based on elapsed time
    while (currentWaypointIndex < mission.size() - 1 && 
           elapsedTime >= mission[currentWaypointIndex + 1].time) {
        currentWaypointIndex++;
    }

    if (currentWaypointIndex < mission.size()) {
        Waypoint& currentWaypoint = mission[currentWaypointIndex];
        if (!currentWaypoint.visited) {
            navigateToWaypoint(currentWaypoint);
            currentWaypoint.visited = true;
        }
    } else {
        // Mission complete
        land();
        missionStarted = false;
        publishStatus("mission completed");
    }
}

void navigateToWaypoint(const Waypoint& waypoint) {
    // Implement navigation logic here
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_mission_item_int_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                      1, 1, 0, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                      MAV_CMD_NAV_WAYPOINT,
                                      2, 0, 0, 0, 0, 0,
                                      waypoint.lat * 1e7, waypoint.lon * 1e7, waypoint.alt);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    pixhawkSerial.write(buf, len);

    char status_message[100];
    snprintf(status_message, sizeof(status_message), "Navigating to waypoint %d: Lat %.6f, Lon %.6f, Alt %.2f", 
             currentWaypointIndex, waypoint.lat, waypoint.lon, waypoint.alt);
    publishStatus(status_message);
}

void changeFlightMode(uint8_t mode) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_set_mode_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);

  const char* mode_str;
  auto it = flightModes.find(mode);
  if (it != flightModes.end()) {
      mode_str = it->second;
  } else {
      mode_str = "UNKNOWN";
  }

  char status_message[50];
  snprintf(status_message, sizeof(status_message), "Mode changed to %s", mode_str);
  publishStatus(status_message);
  Serial.println(status_message);
}


void handle_gps_time(const mavlink_system_time_t& sys_time) {
  if (!timeInitialized || sys_time.time_unix_usec > last_gps_time_us) {
    last_gps_time_us = sys_time.time_unix_usec;
    systemTimeAtLastGPSUpdate = micros();
    timeInitialized = true;
    Serial.println("Time reference updated from GPS");
    Serial.print("GPS Unix time (us): ");
    Serial.println(last_gps_time_us);
    Serial.print("System time at update (ms): ");
    Serial.println(systemTimeAtLastGPSUpdate);
  }
}

uint64_t getCurrentTimeUs() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_REQUEST_MESSAGE, 0, MAVLINK_MSG_ID_SYSTEM_TIME, 0, 0, 0, 0, 0, 0);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);
  
  unsigned long start = millis();
  while (millis() - start < 1000) {
    if (pixhawkSerial.available()) {
      uint8_t c = pixhawkSerial.read();
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
        if (msg.msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
          mavlink_system_time_t sys_time;
          mavlink_msg_system_time_decode(&msg, &sys_time);
          handle_gps_time(sys_time);
          return sys_time.time_unix_usec;
        }
      }
    }
  }
  
  if (timeInitialized) {
    uint64_t time_since_last_gps = (uint64_t)(micros() - systemTimeAtLastGPSUpdate);
    return last_gps_time_us + time_since_last_gps;
  }
  
  return (uint64_t)micros();
}

//Give absolute time without mavlink call, based on last gps call
uint64_t getCurrentTimeUsLite() {
  if (!timeInitialized) {
      return (uint64_t)micros();
  }
  uint64_t time_since_last_gps = (uint64_t)(micros() - systemTimeAtLastGPSUpdate);
  return last_gps_time_us + time_since_last_gps;
}

bool isArmed() {
    // Request a HEARTBEAT message
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                  1, 1, // target system, target component
                                  MAV_CMD_REQUEST_MESSAGE, 0, // command, confirmation
                                  MAVLINK_MSG_ID_HEARTBEAT, 0, 0, 0, 0, 0, 0); // params
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    pixhawkSerial.write(buf, len);
    
    // Wait for a short time to receive the HEARTBEAT
    unsigned long start = millis();
    while (millis() - start < 1000) { // Wait up to 1 second
        if (pixhawkSerial.available()) {
            uint8_t c = pixhawkSerial.read();
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                    armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
                    return armed;
                }
            }
        }
    }
    
    // If we didn't receive a new HEARTBEAT, return the last known arm status
    return armed;
}

void armDrone() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);

  publishStatus("Sent arm command");
  Serial.println("Sent arm command");
}

void takeoff(float altitude) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);

  publishStatus("Sent takeoff command");
  Serial.println("Sent takeoff command");
}

void land() {
  Serial.println("landing");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);
  publishStatus("Sent land command");
  Serial.println("Sent land command");
}

float getCurrentAltitude() {
    float altitude = 0.0f;
    // Request a GLOBAL_POSITION_INT message
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    
    mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                  1, 1, // target system, target component
                                  MAV_CMD_REQUEST_MESSAGE, 0, // command, confirmation
                                  MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 0, 0, 0, 0, 0, 0); // params
    
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    pixhawkSerial.write(buf, len);
    
    // Wait for a short time to receive the GLOBAL_POSITION_INT
    unsigned long start = millis();
    while (millis() - start < 1000) { // Wait up to 1 second
        if (pixhawkSerial.available()) {
            uint8_t c = pixhawkSerial.read();
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
                    mavlink_global_position_int_t global_pos;
                    mavlink_msg_global_position_int_decode(&msg, &global_pos);
                    altitude = global_pos.relative_alt / 1000.0f; // Convert from mm to m
                    return altitude;
                }
            }
        }
    }
    
    // If we didn't receive a new GLOBAL_POSITION_INT, return the default altitude (0)
    return altitude;
}

