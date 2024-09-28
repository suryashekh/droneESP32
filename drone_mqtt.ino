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

const unsigned long MAVLINK_TIMEOUT = 3000; // 1 second timeout

// WiFi credentials
const char* ssid = "NETGEAR20F0BF-SETUP";
const char* password = "sharedsecret";

// MQTT Broker
const char* mqtt_server = "192.168.21.96";

// MQTT topics
const char* mqtt_command_topic = "drone/command";
const char* mqtt_status_topic = "drone/status";
char mqtt_mission_topic[50];

// Increase MQTT_MAX_PACKET_SIZE in PubSubClient.h or define it here
#define MQTT_MAX_PACKET_SIZE 2048

// MAVLink system and component IDs
#define SYSTEM_ID 255
#define COMPONENT_ID MAV_COMP_ID_MISSIONPLANNER

// Flight modes
#define STABILIZE_MODE 0
#define GUIDED_MODE 4

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  pixhawkSerial.begin(57600, SERIAL_8N1, RX_PIN, TX_PIN);
  
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.setBufferSize(2048);  // Increase buffer size
  
  delay(1000);
  
  snprintf(mqtt_mission_topic, sizeof(mqtt_mission_topic), "drone/%d/mission", DRONE_ID);
  
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
  Serial.println(length);
  Serial.print("Payload: ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  
  DynamicJsonDocument doc(2048);  // Increase JSON document size
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("deserializeJson() failed: ");
    Serial.println(error.c_str());
    return;
  }

  if (strcmp(topic, mqtt_mission_topic) == 0) {
    Serial.println("Received mission data");
    handleMissionUpload(doc);
  } else if (strcmp(topic, mqtt_command_topic) == 0) {
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
        }
      }
    }
  } else {
    Serial.println("Received message on unexpected topic");
  }
}


void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe(mqtt_command_topic);
      client.subscribe(mqtt_mission_topic);
      Serial.println("Subscribed to topics: ");
      Serial.println(mqtt_command_topic);
      Serial.println(mqtt_mission_topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}
void handle_mavlink_message(mavlink_message_t *msg) {
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      // Process heartbeat
      mavlink_heartbeat_t heartbeat;
      mavlink_msg_heartbeat_decode(msg, &heartbeat);
      // Serial.println("Received heartbeat");
      break;
    // Add more message handlers as needed
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

  client.publish(mqtt_status_topic, "{\"status\": \"mode changed\"}");
  Serial.println("Sent mode change command");
}

void armDrone() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  pixhawkSerial.write(buf, len);

  client.publish(mqtt_status_topic, "{\"status\": \"armed\"}");
  Serial.println("Sent arm command");
}

void takeoff(float altitude) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude);
  
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  pixhawkSerial.write(buf, len);

  char status[50];
  snprintf(status, sizeof(status), "{\"status\": \"taking off\", \"altitude\": %.2f}", altitude);
  client.publish(mqtt_status_topic, status);
  Serial.println("Sent takeoff command");
}