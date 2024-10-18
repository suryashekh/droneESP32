#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include "C:\Users\user\Documents\Arduino\libraries\MAVLink\common\mavlink.h"
#include <chrono>
#include <vector>


#define DRONE_ID 1
#define RX_PIN 16
#define TX_PIN 17

HardwareSerial pixhawkSerial(2);

const unsigned long MAVLINK_TIMEOUT = 3000;
const unsigned long TELEMETRY_INTERVAL = 1000;
const unsigned long GPS_TIME_INTERVAL = 1000;

// WiFi and MQTT settings
const char* ssid = "TP-Link_5734";
const char* password = "70628739";
const char* mqtt_server = "192.168.0.18";
const char* mqtt_command_topic = "drone/command";
char mqtt_status_topic[50];
char mqtt_mission_topic[50];
char mqtt_telemetry_topic[50];
char mqtt_individual_command_topic[50];

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

mavlink_message_t msg;
mavlink_status_t status;

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
  
  Serial.println("ESP32 started. Waiting for MQTT commands.");
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  while (pixhawkSerial.available()) {
    uint8_t c = pixhawkSerial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      handle_mavlink_message(&msg);
    }
  }

  executeMission();

  // Send telemetry data at regular intervals
  unsigned long currentTime = millis();
  if (currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    sendTelemetryData();
    lastTelemetryTime = currentTime;
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

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(mqtt_command_topic);
      client.subscribe(mqtt_mission_topic);
      client.subscribe(mqtt_individual_command_topic);
      publishStatus("connected");
      Serial.println("Subscribed to topics: ");
      Serial.println(mqtt_command_topic);
      Serial.println(mqtt_mission_topic);
      Serial.println(mqtt_individual_command_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
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
    // Add more message handlers as needed
  }
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
  
  Serial.println("Telemetry data: ALTITUDE");
  Serial.print(altitude);
  Serial.println("Telemetry data: Battery Voltage");
  Serial.print(battery_voltage);
  Serial.println("Telemetry data: Mode");
  Serial.print(mode_str);
  
  if (timeInitialized) {
    uint64_t time_since_last_gps = (uint64_t)(micros() - systemTimeAtLastGPSUpdate);
    uint64_t estimated_current_time = last_gps_time_us + time_since_last_gps;
    doc["timestamp"] = estimated_current_time;
  } else {
    doc["timestamp"] = "not_initialized";
  }

  char buffer[1024];
  serializeJson(doc, buffer);
  client.publish(mqtt_telemetry_topic, buffer);
}

const char* getFlightModeString(uint8_t base_mode, uint32_t custom_mode) {
  if (base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
    switch (custom_mode) {
      case STABILIZE_MODE: return "Stabilize";
      case GUIDED_MODE: return "Guided";
      case AUTO_MODE: return "Auto";
      case LAND_MODE: return "Land";
      default: return "Unknown";
    }
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
            mission.push_back({lat, lon, alt, time * 1000000ULL, false});  // Convert seconds to microseconds
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
  switch(mode) {
    case STABILIZE_MODE:
      mode_str = "stabilize";
      break;
    case GUIDED_MODE:
      mode_str = "guided";
      break;
    case AUTO_MODE:
      mode_str = "auto";
      break;
    case LAND_MODE:
      mode_str = "land";
      break;
    default:
      mode_str = "unknown";
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

  publishStatus("armed");
  Serial.println("Sent arm command");
}

void takeoff(float altitude) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);

  publishStatus("taking off");
  Serial.println("Sent takeoff command");
}

void land() {
  Serial.println("landing");
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);

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

