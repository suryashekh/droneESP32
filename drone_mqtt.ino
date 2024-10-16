#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include "C:\Users\user\Documents\Arduino\libraries\MAVLink\common\mavlink.h"
#include <chrono>

#define DRONE_ID 1  // Make sure this matches the ID you're using in JavaScript
#define RX_PIN 16
#define TX_PIN 17

HardwareSerial pixhawkSerial(2);

const unsigned long MAVLINK_TIMEOUT = 3000; // 3 seconds timeout
const unsigned long TELEMETRY_INTERVAL = 1000; // 1 second interval for telemetry updates

// WiFi credentials
const char* ssid = "TP-Link_5734";
const char* password = "70628739";

// MQTT Broker
const char* mqtt_server = "192.168.0.18";

// MQTT topics
const char* mqtt_command_topic = "drone/command";
char mqtt_status_topic[50];
char mqtt_mission_topic[50];
char mqtt_telemetry_topic[50];
char mqtt_individual_command_topic[50];

#define MQTT_MAX_PACKET_SIZE 2048

// MAVLink system and component IDs
#define SYSTEM_ID 255
#define COMPONENT_ID MAV_COMP_ID_MISSIONPLANNER

// Flight modes
#define STABILIZE_MODE 0
#define GUIDED_MODE 4
#define LAND_MODE 9 
#define AUTO_MODE 3

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastTelemetryTime = 0;

void setup() {
  Serial.begin(115200);
  pixhawkSerial.begin(57600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.setBufferSize(2048);  // Increase buffer size
  
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
  
  // Read and process any incoming MAVLink messages
  mavlink_message_t msg;
  mavlink_status_t status;
  while (pixhawkSerial.available()) {
    uint8_t c = pixhawkSerial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      handle_mavlink_message(&msg);
    }
  }

  // Send telemetry data at regular intervals
  unsigned long currentTime = millis();
  if (currentTime - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    sendTelemetryData();
    lastTelemetryTime = currentTime;
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

  // Wait for and process MAVLink messages
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
            altitude = pos.relative_alt / 1000.0f; // Convert mm to m
            gotPosition = true;
            break;
          }
          case MAVLINK_MSG_ID_SYS_STATUS: {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            battery_voltage = sys_status.voltage_battery / 1000.0f; // Convert mV to V
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
    if (uploadMission(waypoints)) {
      client.publish(mqtt_status_topic, "{\"status\": \"mission uploaded successfully\"}");
    } else {
      client.publish(mqtt_status_topic, "{\"status\": \"error: mission upload failed\"}");
    }
  } else {
    client.publish(mqtt_status_topic, "{\"status\": \"error: invalid mission data\"}");
  }
    publishStatus("mission uploaded");
}


bool uploadMission(const JsonArray& waypoints) {
  uint16_t missionCount = waypoints.size() * 2;  // Each waypoint has a move and a delay command
  
  // Clear current mission
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_mission_clear_all_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);
  
  if (!waitForAck(MAVLINK_MSG_ID_MISSION_ACK, MAVLINK_TIMEOUT)) {
    Serial.println("Failed to clear existing mission");
    return false;
  }

  // Send mission count
  mavlink_msg_mission_count_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, missionCount);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);

  if (!waitForAck(MAVLINK_MSG_ID_MISSION_REQUEST, MAVLINK_TIMEOUT)) {
    Serial.println("No response to mission count");
    return false;
  }

  uint16_t seq = 0;
  for (JsonObject waypoint : waypoints) {
    float lat = waypoint["lat"];
    float lon = waypoint["lon"];
    float alt = waypoint["alt"];
    uint32_t time = waypoint["time"];

    // Send waypoint
    mavlink_msg_mission_item_int_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                      1, 1, seq, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                      MAV_CMD_NAV_WAYPOINT,
                                      0, 1, 0, 0, 0, 0,
                                      lat * 1e7, lon * 1e7, alt);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    pixhawkSerial.write(buf, len);

    if (!waitForAck(MAVLINK_MSG_ID_MISSION_REQUEST, MAVLINK_TIMEOUT)) {
      Serial.print("Failed to receive ACK for waypoint ");
      Serial.println(seq);
      return false;
    }

    seq++;

    // Send delay
    mavlink_msg_mission_item_int_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                      1, 1, seq, MAV_FRAME_MISSION,
                                      MAV_CMD_NAV_DELAY,
                                      0, 1, time, -1, -1, -1,
                                      0, 0, 0);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    pixhawkSerial.write(buf, len);

    if (!waitForAck(MAVLINK_MSG_ID_MISSION_REQUEST, MAVLINK_TIMEOUT)) {
      Serial.print("Failed to receive ACK for delay ");
      Serial.println(seq);
      return false;
    }

    seq++;
  }

    // Send mission end
    mavlink_msg_mission_ack_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_MISSION_ACCEPTED);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    pixhawkSerial.write(buf, len);
  // Wait for final MISSION_ACK
  // if (!waitForAck(MAVLINK_MSG_ID_MISSION_ACK, MAVLINK_TIMEOUT)) {
  //   Serial.println("Failed to receive final MISSION_ACK");
  //   return false;
  // }

  Serial.println("Mission upload complete");
  return true;
}

void clearMission() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_mission_clear_all_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);
}

void startMission() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                1, 1, MAV_CMD_MISSION_START, 0,
                                0, 0, 0, 0, 0, 0, 0);

  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  pixhawkSerial.write(buf, len);

  Serial.println("Mission start command sent");
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
