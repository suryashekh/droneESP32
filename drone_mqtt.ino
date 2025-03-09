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
#include <FastLED.h>


#define DRONE_ID 10
#define RX_PIN 16
#define TX_PIN 18

#define MAX_LIGHT_STEPS 2000  // Increased from default to handle larger sequences
#define LED_PIN 13   // GPIO2 for ESP32
#define NUM_LEDS 49  // Number of LEDs in your strip
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define BRIGHTNESS 255        // Default brightness (0-255)
#define MAX_LIGHT_SEQUENCE 7  //7*7 total 49

// Add this with other global variable declarations
CRGB leds[NUM_LEDS];

HardwareSerial pixhawkSerial(2);

std::deque<std::string> statusMessages;
const size_t MAX_STATUS_MESSAGES = 100;

unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 3000;  // 5 seconds
const unsigned long MAVLINK_TIMEOUT = 3000;
const unsigned long TELEMETRY_INTERVAL = 1000;
const unsigned long GPS_TIME_INTERVAL = 1000;
const unsigned long STATUS_MESSAGE_INTERVAL = 1000;

std::queue<mavlink_message_t> message_queue;
const int MAX_STATUS_MESSAGE_QUEUE_SIZE = 5;
#define MQTT_MAX_PACKET_SIZE 2048


// WiFi and MQTT settings
const char* ssid = "Tenda_BEB620";
const char* password = "12345678";
const char* mqtt_server = "192.168.2.50";
const char* mqtt_command_topic = "drone/command";
const char* mqtt_rtcm_topic = "drone/rtcm";
char mqtt_status_topic[50];
char mqtt_mission_topic[50];
char mqtt_telemetry_topic[50];
char mqtt_individual_command_topic[50];
char mqtt_individual_status_topic[50];


uint64_t systemTimeOffset = 0;
bool timeInitialized = false;

RTC_NOINIT_ATTR int resetCount = 0;  // Survives reboots
RTC_NOINIT_ATTR uint32_t lastResetTime = 0;

bool telemetryStreamsInitialized = false;
const unsigned long STREAM_REINIT_INTERVAL = 30000;

unsigned long lastMissionExecutionTime = 0;
const unsigned long MISSION_EXECUTION_INTERVAL = 100; // Check mission every 100ms
// MAVLink IDs
#define SYSTEM_ID 255
#define COMPONENT_ID MAV_COMP_ID_MISSIONPLANNER

// Flight modes
#define STABILIZE_MODE 0
#define GUIDED_MODE 4
#define LAND_MODE 9
#define AUTO_MODE 3

struct Waypoint {
  double lat;
  double lon;
  double alt;
  uint64_t time;  // Timestamp in microseconds
  bool visited;   // Flag to track if we've navigated to this waypoint
};

struct MissionBuffer {
  char* buffer;
  size_t totalSize;
  size_t receivedSize;
  bool* receivedChunks;
  int totalChunks;
  unsigned long lastChunkTime;
} missionBuffer = { nullptr, 0, 0, nullptr, 0, 0 };

enum DroneStartupState {
    INIT,
    MODE_CHANGE_WAIT,
    ARMING_WAIT,
    TAKING_OFF
};

DroneStartupState startupState = INIT;
unsigned long stateStartTime = 0;

// Structure for light sequence steps (keep this with other struct definitions)
struct LightStep {
  uint64_t time;  // Time in microseconds since mission start
  uint8_t r;      // Red value (0-255)
  uint8_t g;      // Green value (0-255)
  uint8_t b;      // Blue value (0-255)
  bool executed;  // Flag to track if this step has been executed
};

enum CommandPriority {
  PRIORITY_LOW,  // For RTCM messages
  PRIORITY_HIGH  // For commands and navigation
};

struct CommMonitor {
  unsigned long lastMavlinkMsg = 0;
  const unsigned long MAVLINK_TIMEOUT = 3000;  // 2 seconds without any MAVLink message is concerning
  bool wasTimeout = false;  // Track if we were in timeout state
} commMonitor;

struct CommandMessage {
  mavlink_message_t msg;
  unsigned long timestamp;
  uint8_t retries;
  bool requiresAck;
  CommandPriority priority;
};

class CompareCommandPriority {
public:
  bool operator()(const CommandMessage& a, const CommandMessage& b) {
    return a.priority < b.priority;
  }
};

std::priority_queue<CommandMessage, std::vector<CommandMessage>, CompareCommandPriority> commandQueue;
const size_t MAX_QUEUE_SIZE = 10;  // Increased to handle RTCM messages
unsigned long lastCommandSentTime = 0;
const unsigned long COMMAND_SPACING = 250;
float current_altitude = 0.0f;
float battery_voltage = 0.0f;
uint8_t base_mode = 0;
uint32_t custom_mode = 0;
uint8_t current_satellites_visible = 0;
uint8_t current_gps_fix_type = GPS_FIX_TYPE_NO_GPS;
// Keep these global variables
// std::vector<LightStep> lightSequence;
struct LightSequenceManager {
    LightStep steps[MAX_LIGHT_STEPS];
    size_t count;
    size_t currentIndex;
};

// Replace the global vector with our new structure
LightSequenceManager lightSequence;
size_t currentLightIndex = 0;
// Add these global variables at the top with other global declarations
bool missionCancelled = false;
unsigned long missionCancelTime = 0;
bool isLightSequenceOnly = false;


#define MAX_WAYPOINTS 150
#define MAX_MISSION_SIZE (MAX_WAYPOINTS * sizeof(Waypoint))
#define COMMAND_DOC_SIZE 2048   // Size for regular commands and RTCM
#define CHUNK_DOC_SIZE 1024     // Size for parsing mission chunks
#define MISSION_CHUNK_SIZE 512  // Size of each mission data chunk
// Add these definitions at the top with other #defines
// GPS Fix Type Definitions
#define GPS_FIX_TYPE_NO_GPS 0     // No GPS connected
#define GPS_FIX_TYPE_NO_FIX 1     // No position information
#define GPS_FIX_TYPE_2D_FIX 2     // 2D position
#define GPS_FIX_TYPE_3D_FIX 3     // 3D position
#define GPS_FIX_TYPE_DGPS 4       // DGPS/SBAS aided
#define GPS_FIX_TYPE_RTK_FLOAT 5  // RTK float
#define GPS_FIX_TYPE_RTK_FIXED 6  // RTK fixed
#define GPS_FIX_TYPE_STATIC 7     // Static fixed
#define GPS_FIX_TYPE_PPP 8        // PPP solution

// Add these structs/classes at the top of your drone_mqtt.ino file
struct PositionAccuracy {
  float horizontal_accuracy = 0.0f;  // meters (from EPH)
  float vertical_accuracy = 0.0f;    // meters (from EPV)
  uint32_t last_rtcm_time = 0;
  uint8_t fix_type = 0;
  uint8_t satellites = 0;
  double latitude = 0.0; 
  double longitude = 0.0; 


  String getFixTypeString() {
    switch (fix_type) {
      case GPS_FIX_TYPE_NO_GPS: return "NO_GPS";
      case GPS_FIX_TYPE_NO_FIX: return "NO_FIX";
      case GPS_FIX_TYPE_2D_FIX: return "2D";
      case GPS_FIX_TYPE_3D_FIX: return "3D";
      case GPS_FIX_TYPE_DGPS: return "DGPS";
      case GPS_FIX_TYPE_RTK_FLOAT: return "RTK_FLOAT";
      case GPS_FIX_TYPE_RTK_FIXED: return "RTK_FIXED";
      case GPS_FIX_TYPE_STATIC: return "STATIC";
      default: return "UNKNOWN";
    }
  }

  float getEstimatedAccuracy() {
    // Base accuracy based on fix type
    float base_accuracy = 2.5f;  // Default GPS accuracy
    switch (fix_type) {
      case GPS_FIX_TYPE_RTK_FIXED:
        base_accuracy = 0.03f;  // 3cm
        break;
      case GPS_FIX_TYPE_RTK_FLOAT:
        base_accuracy = 0.3f;  // 30cm
        break;
      case GPS_FIX_TYPE_DGPS:
        base_accuracy = 0.8f;  // 80cm
        break;
    }

    // Calculate 3D position error using horizontal and vertical components
    float position_error = sqrt(
      pow(horizontal_accuracy, 2) + pow(vertical_accuracy, 2));

    float accuracy_factor = position_error / 0.5f;  // Normalize to 50cm
    float age_factor = min(2.0f, (float)(millis() - last_rtcm_time) / 1000.0f);

    return base_accuracy * accuracy_factor * age_factor;
  }
};

PositionAccuracy posAccuracy;  // Global instance

std::vector<Waypoint> mission;
size_t currentWaypointIndex = 0;
uint64_t missionStartTime = 0;
bool missionStarted = false;
bool armed = false;
bool waitingForTakeoff = false;
unsigned long takeoffStartTime = 0;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastSuccessfulPublish = 0;
const unsigned long PUBLISH_TIMEOUT = 3000;
unsigned long lastTelemetryUpdate = 0;
const unsigned long TELEMETRY_TIMEOUT = 2000;
unsigned long lastTelemetryTime = 0;
bool waitingForMissionStart = false;

uint32_t targetStartTimeSeconds = 0;
float targetTakeoffAltitude = 0;
unsigned long lastArmingCheck = -1;

mavlink_message_t msg;
mavlink_status_t status;

enum ConnectionState {
    DISCONNECTED,
    CONNECTING_WIFI,
    CONNECTING_MQTT,
    CONNECTED
};

const unsigned long MQTT_ATTEMPT_TIMEOUT = 1000;  // 1 second timeout for MQTT connection attempt
unsigned long mqttAttemptStart = 0;

ConnectionState connectionState = DISCONNECTED;
unsigned long connectionStartTime = 0;
const unsigned long CONNECTION_TIMEOUT = 5000; // 5 second timeout for connection attempts


const std::map<uint8_t, const char*> flightModes = {
  { 0, "STABILIZE" },
  { 1, "ACRO" },
  { 2, "ALT_HOLD" },
  { 3, "AUTO" },
  { 4, "GUIDED" },
  { 5, "LOITER" },
  { 6, "RTL" },
  { 7, "CIRCLE" },
  { 9, "LAND" },
  { 11, "DRIFT" },
  { 13, "SPORT" },
  { 14, "FLIP" },
  { 15, "AUTOTUNE" },
  { 16, "POSHOLD" },
  { 17, "BRAKE" },
  { 18, "THROW" },
  { 19, "AVOID_ADSB" },
  { 20, "GUIDED_NOGPS" },
  { 21, "SMART_RTL" },
  { 22, "FLOWHOLD" },
  { 23, "FOLLOW" },
  { 24, "ZIGZAG" },
  { 25, "SYSTEMID" },
  { 26, "AUTOROTATE" },
  { 27, "AUTO_RTL" }
};

void setup() {
  printHeapStats();
  Serial.begin(115200);
  pixhawkSerial.begin(57600, SERIAL_8N1, RX_PIN, TX_PIN);

  setupLEDs();
  FastLED.clear();
  FastLED.show();
  setup_wifi();

  client.setBufferSize(MQTT_MAX_PACKET_SIZE);
  client.setKeepAlive(5);      // Increase keep alive time
  client.setSocketTimeout(10);  // Increase socket timeout
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  setupTelemetryStreams();

  delay(1000);

  snprintf(mqtt_status_topic, sizeof(mqtt_status_topic), "drone/%d/status", DRONE_ID);
  snprintf(mqtt_mission_topic, sizeof(mqtt_mission_topic), "drone/%d/mission", DRONE_ID);
  snprintf(mqtt_telemetry_topic, sizeof(mqtt_telemetry_topic), "drone/%d/telemetry", DRONE_ID);
  snprintf(mqtt_individual_command_topic, sizeof(mqtt_individual_command_topic), "drone/%d/command", DRONE_ID);
  snprintf(mqtt_individual_status_topic, sizeof(mqtt_individual_status_topic), "drone/%d/status_messages", DRONE_ID);

  Serial.println("ESP32 started. Waiting for MQTT commands.");
}

void loop() {
  unsigned long currentMillis = millis();

  // Handle mission execution independently of MQTT connection
  if (currentMillis - lastMissionExecutionTime >= MISSION_EXECUTION_INTERVAL) {
    handleMissionExecution();
    lastMissionExecutionTime = currentMillis;
  }

  handleConnection();

  // Only run client.loop() if connected
  if (connectionState == CONNECTED) {
      client.loop();
  }

  checkDroneComms();
  
  while (pixhawkSerial.available()) {
    uint8_t c = pixhawkSerial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
      if (message_queue.size() < MAX_STATUS_MESSAGE_QUEUE_SIZE) {
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

  // Check if we're waiting for mission start time
  if (waitingForMissionStart && !missionCancelled) {
    uint64_t currentTime = getCurrentTimeUsLite();
    uint64_t currentSeconds = (currentTime / 1000000ULL) % 86400ULL;

    if (currentSeconds >= targetStartTimeSeconds) {
      startMissionExecution();
    }
  }

  // Adding this to reset the cancel flag after some time
  if (missionCancelled && (millis() - missionCancelTime > 5000)) {
    missionCancelled = false;
  }

  if (waitingForTakeoff) {
    missionTakeoff(targetTakeoffAltitude);
  }

  handlePreMissionBlink();
  executeLightSequence();
  processCommandQueue();
  setupTelemetryStreams();
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

void handleConnection() {
  static unsigned long lastAttempt = 0;
  unsigned long currentMillis = millis();

  // Only attempt reconnection after RECONNECT_INTERVAL
  if (currentMillis - lastAttempt < RECONNECT_INTERVAL) {
      return;
  }

  // Check for connection timeout
  if ((connectionState == CONNECTING_WIFI || connectionState == CONNECTING_MQTT) &&
      (currentMillis - connectionStartTime > CONNECTION_TIMEOUT)) {
      connectionState = DISCONNECTED;
      Serial.println("Connection attempt timed out");
  }

  switch (connectionState) {
      case DISCONNECTED:
          if (WiFi.status() != WL_CONNECTED) {
              Serial.println("Connecting to WiFi...");
              WiFi.begin(ssid, password);
              connectionState = CONNECTING_WIFI;
              connectionStartTime = currentMillis;
              lastAttempt = currentMillis;
          } else if (!client.connected()) {
              connectionState = CONNECTING_MQTT;
              connectionStartTime = currentMillis;
              mqttAttemptStart = currentMillis;  // Start MQTT connection timer
          }
          break;

      case CONNECTING_WIFI:
          if (WiFi.status() == WL_CONNECTED) {
              Serial.println("WiFi connected");
              Serial.println("IP address: " + WiFi.localIP().toString());
              connectionState = CONNECTING_MQTT;
              connectionStartTime = currentMillis;
              mqttAttemptStart = currentMillis;  // Start MQTT connection timer
          }
          break;

      case CONNECTING_MQTT:
          if (!client.connected()) {
              // Only attempt MQTT connection if we haven't started or previous attempt timed out
              if (currentMillis - mqttAttemptStart >= MQTT_ATTEMPT_TIMEOUT) {
                  String clientId = "ESP32Client_";
                  clientId += String(DRONE_ID);
                  clientId += "_";
                  clientId += String(random(0xffff), HEX);

                  Serial.print("Attempting MQTT connection...");
                  
                  // Set shorter socket timeout for connection attempt
                  client.setSocketTimeout(1);  // 1 second socket timeout
                  
                  if (client.connect(clientId.c_str())) {
                      Serial.println("connected");
                      client.setSocketTimeout(15);  // Reset to normal timeout
                      // Subscribe to topics
                      client.subscribe(mqtt_command_topic);
                      client.subscribe(mqtt_mission_topic);
                      client.subscribe(mqtt_individual_command_topic);
                      client.subscribe(mqtt_rtcm_topic);
                      publishStatus("connected");
                      connectionState = CONNECTED;
                  } else {
                      Serial.print("failed, rc=");
                      Serial.print(client.state());
                      Serial.println(" try again after interval");
                      connectionState = DISCONNECTED;
                      lastAttempt = currentMillis;
                  }
                  mqttAttemptStart = currentMillis;  // Reset timer for next attempt
              }
          }
          break;

      case CONNECTED:
          if (WiFi.status() != WL_CONNECTED || !client.connected()) {
              connectionState = DISCONNECTED;
              Serial.println("Connection lost");
          }
          break;
  }
}

void checkDroneComms() {
  if (millis() - commMonitor.lastMavlinkMsg > commMonitor.MAVLINK_TIMEOUT) {
      if (!commMonitor.wasTimeout) {  // Only do this once when entering timeout
          // Clear the command queue
          while (!commandQueue.empty()) {
              commandQueue.pop();
          }
          publishStatus("Warning: Communication timeout - cleared command queue");
          commMonitor.wasTimeout = true;
      }
  } else if (commMonitor.wasTimeout) {
      // Just recovered from timeout
      commMonitor.wasTimeout = false;
      publishStatus("Communication restored");
  }
}

bool queueCommand(mavlink_message_t& msg, bool requiresAck, bool isRTCM = false) {
  if (commandQueue.size() >= MAX_QUEUE_SIZE) {
    if (isRTCM) {
      // For RTCM messages, try to find and replace the oldest RTCM message
      std::priority_queue<CommandMessage, std::vector<CommandMessage>, CompareCommandPriority> tempQueue;
      bool replacedOldRTCM = false;
      unsigned long oldestRTCMTime = ULONG_MAX;

      // First pass: find oldest RTCM message
      std::vector<CommandMessage> savedCommands;
      while (!commandQueue.empty()) {
        CommandMessage cmd = commandQueue.top();
        commandQueue.pop();
        if (cmd.priority == PRIORITY_LOW && cmd.timestamp < oldestRTCMTime) {
          oldestRTCMTime = cmd.timestamp;
        }
        savedCommands.push_back(cmd);
      }

      // Second pass: rebuild queue, skipping oldest RTCM if found
      for (const auto& cmd : savedCommands) {
        if (cmd.priority == PRIORITY_LOW && cmd.timestamp == oldestRTCMTime && !replacedOldRTCM) {
          replacedOldRTCM = true;
          continue;
        }
        tempQueue.push(cmd);
      }

      if (!replacedOldRTCM) {
        Serial.println("Queue full, dropping RTCM message");
        // Restore original queue
        for (const auto& cmd : savedCommands) {
          commandQueue.push(cmd);
        }
        return false;
      }
      commandQueue = tempQueue;
    } else {
      // For high priority commands, keep all existing high priority and newest RTCM messages
      std::priority_queue<CommandMessage, std::vector<CommandMessage>, CompareCommandPriority> tempQueue;
      std::vector<CommandMessage> rtcmMessages;

      while (!commandQueue.empty()) {
        CommandMessage cmd = commandQueue.top();
        commandQueue.pop();
        if (cmd.priority == PRIORITY_HIGH) {
          tempQueue.push(cmd);
        } else if (cmd.priority == PRIORITY_LOW) {
          rtcmMessages.push_back(cmd);
        }
      }

      // Sort RTCM messages by timestamp (newest first)
      std::sort(rtcmMessages.begin(), rtcmMessages.end(),
                [](const CommandMessage& a, const CommandMessage& b) {
                  return a.timestamp > b.timestamp;
                });

      // Add back newest RTCM messages that fit
      size_t spaceLeft = MAX_QUEUE_SIZE - tempQueue.size() - 1;  // -1 for new command
      for (size_t i = 0; i < min(spaceLeft, rtcmMessages.size()); i++) {
        tempQueue.push(rtcmMessages[i]);
      }

      commandQueue = tempQueue;
    }
  }

  CommandMessage cmd = {
    .msg = msg,
    .timestamp = millis(),
    .retries = 0,
    .requiresAck = requiresAck,
    .priority = isRTCM ? PRIORITY_LOW : PRIORITY_HIGH
  };

  commandQueue.push(cmd);
  return true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Raw payload (first 32 bytes): ");
  for (int i = 0; i < min(32, (int)length); i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.print("Payload length: ");
  Serial.println(length);

  if (strcmp(topic, mqtt_mission_topic) == 0) {
    DynamicJsonDocument chunkDoc(CHUNK_DOC_SIZE);
    DeserializationError error = deserializeJson(chunkDoc, payload, length);

    if (error) {
      Serial.print("Mission chunk parse failed: ");
      Serial.println(error.c_str());
      return;
    }

    // Reset the flag at the start of new mission upload (chunk 0)
    if (chunkDoc["chunk"]["chunk"] == 0) {
        isLightSequenceOnly = false;  // Reset at start of new mission
        Serial.println("Reset light sequence flag");
    }

    if (chunkDoc.containsKey("light_sequence_only")) {
      isLightSequenceOnly = chunkDoc["light_sequence_only"];
      publishStatus("Starting new mission upload");
      Serial.printf("Light sequence only mode: %s\n", isLightSequenceOnly ? "true" : "false");
    }

    if (!chunkDoc.containsKey("chunk") || 
      !chunkDoc["chunk"].containsKey("chunk") || 
      !chunkDoc["chunk"].containsKey("totalChunks") || 
      !chunkDoc["chunk"].containsKey("totalSize") || 
      !chunkDoc["chunk"].containsKey("data")) {
      Serial.println("Invalid chunk format");
      return;
    }

    int chunk = chunkDoc["chunk"]["chunk"];
    int totalChunks = chunkDoc["chunk"]["totalChunks"];
    size_t totalSize = chunkDoc["chunk"]["totalSize"];
    const char* data = chunkDoc["chunk"]["data"];
    size_t dataSize = strlen(data);

    Serial.printf("Received chunk %d of %d, size: %d\n",
                  chunk + 1, totalChunks, dataSize);
    // Clear previous mission data if this is the first chunk
    if (chunk == 0) {
      clearMissionData();
      publishStatus("Starting new mission upload");
    }
    // Initialize buffer if this is a new mission
    if (chunk == 0 || missionBuffer.buffer == nullptr) {
      // Cleanup any existing mission buffer
      cleanupMissionBuffer();

      // Allocate new buffer
      missionBuffer.buffer = (char*)malloc(totalSize + 1);
      missionBuffer.receivedChunks = (bool*)calloc(totalChunks, sizeof(bool));

      if (missionBuffer.buffer == nullptr || missionBuffer.receivedChunks == nullptr) {
        Serial.println("Failed to allocate mission buffer");
        cleanupMissionBuffer();
        return;
      }

      missionBuffer.totalSize = totalSize;
      missionBuffer.totalChunks = totalChunks;
      missionBuffer.receivedSize = 0;
      memset(missionBuffer.buffer, 0, totalSize + 1);
      memset(missionBuffer.receivedChunks, 0, totalChunks * sizeof(bool));
    }

    // Validate chunk
    if (chunk >= missionBuffer.totalChunks) {
      Serial.println("Invalid chunk number");
      return;
    }

    // Store chunk data
    if (!missionBuffer.receivedChunks[chunk]) {
      size_t offset = chunk * MISSION_CHUNK_SIZE;
      if (offset + dataSize <= missionBuffer.totalSize) {
        memcpy(missionBuffer.buffer + offset, data, dataSize);
        missionBuffer.receivedChunks[chunk] = true;
        missionBuffer.receivedSize += dataSize;
        missionBuffer.lastChunkTime = millis();

        Serial.printf("Stored chunk %d, total received: %d/%d bytes\n",
                      chunk, missionBuffer.receivedSize, missionBuffer.totalSize);

        // Check if all chunks received
        bool allReceived = true;
        for (int i = 0; i < missionBuffer.totalChunks; i++) {
          if (!missionBuffer.receivedChunks[i]) {
            allReceived = false;
            break;
          }
        }

        if (allReceived) {
          Serial.println("All chunks received, processing mission");
          missionBuffer.buffer[missionBuffer.totalSize] = '\0';

          // Print buffer stats
          Serial.printf("Buffer size: %d, Received size: %d\n",
                        missionBuffer.totalSize, missionBuffer.receivedSize);

          // Print the first and last 100 characters of the buffer
          Serial.println("Start of buffer:");
          Serial.println(String(missionBuffer.buffer).substring(0, 100));
          Serial.println("End of buffer:");
          Serial.println(String(missionBuffer.buffer).substring(max(0, (int)missionBuffer.receivedSize - 100)));

          // Print memory stats before parsing
          printHeapStats();

          // Process complete mission
          DynamicJsonDocument missionDoc(65536);
          error = deserializeJson(missionDoc, missionBuffer.buffer);

          if (error) {
            Serial.print("Final mission parse failed: ");
            Serial.println(error.c_str());
            // Print the problematic JSON
            Serial.println("Failed JSON:");
            Serial.println(missionBuffer.buffer);
          } else {
            handleMissionUpload(missionDoc);
          }

          // Print memory stats after parsing
          printHeapStats();

          // Cleanup
          cleanupMissionBuffer();
        }
      } else {
        Serial.println("Chunk size exceeds buffer bounds");
      }
    } else {
      Serial.printf("Chunk %d already received\n", chunk);
    }
  } else {
    DynamicJsonDocument doc(COMMAND_DOC_SIZE);
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
      Serial.print("deserializeJson() failed: ");
      Serial.println(error.c_str());
      return;
    }

    if (strcmp(topic, mqtt_command_topic) == 0 || strcmp(topic, mqtt_individual_command_topic) == 0) {
      if (doc.containsKey("command")) {
        String command = doc["command"];
        Serial.print("Received command: ");
        Serial.println(command);
        if (command == "arm") {
          armDrone();
        } else if (command == "takeoff") {
          float altitude = doc["altitude"];
          takeoff(altitude);
        } else if (command == "cancel_mission") {
          handleMissionCancel();
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
          } else {
            Serial.println("Going to else part");
          }
        } else if (command == "start_mission") {
          targetTakeoffAltitude = doc["takeoffAltitude"].as<float>();
          targetStartTimeSeconds = doc["startTimeSeconds"].as<uint32_t>();
          startMission();
        } else if (command == "time_sync") {
          handleTimeSync(doc);
        }
      }
    } else if (strcmp(topic, mqtt_rtcm_topic) == 0) {
      JsonArray rtcmData = doc["data"].as<JsonArray>();
      uint8_t buffer[2000];  // Increased to 2000 bytes
      size_t dataLen = 0;
      posAccuracy.last_rtcm_time = millis();
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
      const size_t MAX_CHUNK_SIZE = 180;                                // Maximum safe size for GPS_RTCM_DATA message
      size_t chunks = (dataLen + MAX_CHUNK_SIZE - 1) / MAX_CHUNK_SIZE;  // Calculate number of chunks needed

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

        queueCommand(msg, false, true);  // RTCM messages are low priority

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
}

// Add this function to clean up mission buffer
void cleanupMissionBuffer() {
  if (missionBuffer.buffer != nullptr) {
    free(missionBuffer.buffer);
    missionBuffer.buffer = nullptr;
  }
  if (missionBuffer.receivedChunks != nullptr) {
    free(missionBuffer.receivedChunks);
    missionBuffer.receivedChunks = nullptr;
  }
  missionBuffer.totalSize = 0;
  missionBuffer.receivedSize = 0;
  missionBuffer.totalChunks = 0;
}

// Add this to your loop() function to handle timeout
void checkMissionTimeout() {
  if (missionBuffer.buffer != nullptr && (millis() - missionBuffer.lastChunkTime) > 10000) {  // 10 second timeout
    Serial.println("Mission upload timeout, cleaning up");
    cleanupMissionBuffer();
    publishStatus("Mission upload timeout");
  }
}

// New function to handle all mission-related execution
void handleMissionExecution() {
  // Check if we're waiting for mission start time
  if (waitingForMissionStart && !missionCancelled) {
    uint64_t currentTime = getCurrentTimeUsLite();
    uint64_t currentSeconds = (currentTime / 1000000ULL) % 86400ULL;

    if (currentSeconds >= targetStartTimeSeconds) {
      startMissionExecution();
    }
  }


  // Handle pre-mission blinking
  handlePreMissionBlink();
  
  // Execute mission components regardless of connection status
  if (!missionCancelled) {
    executeMission();
    executeLightSequence();
  }
}

void handle_mavlink_message(mavlink_message_t* msg) {

  commMonitor.lastMavlinkMsg = millis();

  if (!commandQueue.empty() && commandQueue.top().requiresAck) {
    if (msg->msgid == MAVLINK_MSG_ID_MISSION_ACK || msg->msgid == MAVLINK_MSG_ID_COMMAND_ACK) {
      commandQueue.pop();
      lastCommandSentTime = millis();
    }
  }

  if (msg->msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT || msg->msgid == MAVLINK_MSG_ID_SYS_STATUS || msg->msgid == MAVLINK_MSG_ID_GPS_RAW_INT) {
    lastTelemetryUpdate = millis();
  }

  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      mavlink_heartbeat_t heartbeat;
      mavlink_msg_heartbeat_decode(msg, &heartbeat);
      armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
      base_mode = heartbeat.base_mode;
      custom_mode = heartbeat.custom_mode;
      publishStatus("connected");
      break;

    case MAVLINK_MSG_ID_GPS_RAW_INT:
      mavlink_gps_raw_int_t gps_raw;
      mavlink_msg_gps_raw_int_decode(msg, &gps_raw);
      // current_gps_fix_type = gps_raw.fix_type;
      // current_satellites_visible = gps_raw.satellites_visible;
      // Update accuracy metrics
      posAccuracy.fix_type = gps_raw.fix_type;
      posAccuracy.satellites = gps_raw.satellites_visible;
      if (gps_raw.eph != UINT16_MAX) {
        posAccuracy.horizontal_accuracy = gps_raw.eph / 100.0f;
      }
      // EPV is vertical position error in cm, convert to meters
      if (gps_raw.epv != UINT16_MAX) {
        posAccuracy.vertical_accuracy = gps_raw.epv / 100.0f;
      }
      posAccuracy.latitude = gps_raw.lat / 1e7;
      posAccuracy.longitude = gps_raw.lon / 1e7;
      break;

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
      mavlink_global_position_int_t global_pos;
      mavlink_msg_global_position_int_decode(msg, &global_pos);
      current_altitude = global_pos.relative_alt / 1000.0f;
      break;

    case MAVLINK_MSG_ID_SYS_STATUS:
      mavlink_sys_status_t sys_status;
      mavlink_msg_sys_status_decode(msg, &sys_status);
      battery_voltage = sys_status.voltage_battery / 1000.0f;
      break;

    case MAVLINK_MSG_ID_STATUSTEXT:
      {
        mavlink_statustext_t statustext;
        mavlink_msg_statustext_decode(msg, &statustext);
        addStatusMessage(statustext.text);
        break;
      }
  }
}


void processCommandQueue() {
  if (commandQueue.empty()) {
    return;
  }

  unsigned long currentTime = millis();
  if (currentTime - lastCommandSentTime < COMMAND_SPACING) {
    return;
  }

  CommandMessage cmd = commandQueue.top();

  // Check for timeout
  if (currentTime - cmd.timestamp > 2000) {  // 2 second timeout
    if (cmd.requiresAck && cmd.retries < 3 && cmd.priority == PRIORITY_HIGH) {
      // Only retry high priority commands
      uint8_t buf[MAVLINK_MAX_PACKET_LEN];
      uint16_t len = mavlink_msg_to_send_buffer(buf, &cmd.msg);
      pixhawkSerial.write(buf, len);
      lastCommandSentTime = currentTime;

      // Update and requeue the command
      cmd.timestamp = currentTime;
      cmd.retries++;
      commandQueue.pop();
      commandQueue.push(cmd);

      Serial.printf("Retrying high priority command, attempt %d/3\n", cmd.retries + 1);
    } else {
      // Drop the command if:
      // - Max retries reached for high priority
      // - Low priority command timed out (no retries)
      // - Non-ACK command timed out
      Serial.printf("Dropping %s priority command\n",
                    cmd.priority == PRIORITY_HIGH ? "high" : "low");
      commandQueue.pop();
    }
    return;
  }

  // Send the command
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &cmd.msg);
  pixhawkSerial.write(buf, len);
  lastCommandSentTime = currentTime;

  if (!cmd.requiresAck) {
    commandQueue.pop();
  }
}

// Helper function to format timestamp
void formatTimestamp(char* buffer, size_t bufferSize, uint64_t timestamp_us) {
  uint64_t timestamp_s = timestamp_us / 1000000;  // Convert to seconds
  uint32_t hours = (timestamp_s % 86400) / 3600;
  uint32_t minutes = (timestamp_s % 3600) / 60;
  uint32_t seconds = timestamp_s % 60;

  snprintf(buffer, bufferSize, "%02d:%02d:%02d", hours, minutes, seconds);
}


// Replace the old setupLEDs() with this new version
void setupLEDs() {
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS)
    .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  Serial.println("LED strip initialized");
}

// Add this helper function to set all LEDs to the same color
void setRGB(uint8_t red, uint8_t green, uint8_t blue) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(red, green, blue);
  }
  FastLED.show();
}

void parseLightSequence(JsonArray& sequence) {
    // Reset the sequence
    lightSequence.count = 0;
    lightSequence.currentIndex = 0;
    
    size_t stepCount = 0;
    for (JsonObject step : sequence) {
        if (stepCount >= MAX_LIGHT_STEPS) {
            Serial.printf("Warning: Light sequence truncated at %d steps\n", MAX_LIGHT_STEPS);
            break;
        }
        
        uint32_t time = step["time"];
        uint8_t r = step["r"];
        uint8_t g = step["g"];
        uint8_t b = step["b"];

        lightSequence.steps[stepCount] = {
            .time = time * 1000ULL,  // Convert to microseconds
            .r = r,
            .g = g,
            .b = b,
            .executed = false
        };
        
        Serial.printf("Added light step %d: time=%u, R=%u, G=%u, B=%u\n",
                     stepCount, time, r, g, b);
        
        stepCount++;
        
        // Add a small delay every 100 steps to prevent watchdog issues
        if (stepCount % 100 == 0) {
            delay(1);
            yield();
        }
    }
    
    lightSequence.count = stepCount;
    Serial.printf("Successfully loaded %d light steps\n", stepCount);
}
// Add this function to turn off LEDs when mission ends or on errors
void turnOffLEDs() {
  FastLED.clear();
  FastLED.show();
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


void publishStatus(const char* status) {
  static unsigned long lastConnected = 0;

  if (strcmp(status, "connected") == 0 && millis() - lastConnected < 10000) {  //so that I do not get multiple connected too often
    return;
  }
  lastConnected = millis();

  DynamicJsonDocument doc(256);
  doc["status"] = status;
  char buffer[256];
  serializeJson(doc, buffer);
  client.publish(mqtt_status_topic, buffer);
}

void setupTelemetryStreams() {

  if (millis() - lastTelemetryUpdate <= TELEMETRY_TIMEOUT) {
    return;
  }
  Serial.println("hey");
  mavlink_message_t msg;

  // Position data at 2Hz (includes GPS, global & local position)
  mavlink_msg_request_data_stream_pack(
    SYSTEM_ID, COMPONENT_ID, &msg,
    1, 1,
    MAV_DATA_STREAM_POSITION,
    2,  // 2 Hz
    1   // Start
  );
  queueCommand(msg, false, false);

  // Extended status data at 2Hz (includes SYS_STATUS)
  mavlink_msg_request_data_stream_pack(
    SYSTEM_ID, COMPONENT_ID, &msg,
    1, 1,
    MAV_DATA_STREAM_EXTENDED_STATUS,
    2,  // 2 Hz
    1);
  queueCommand(msg, false, false);

  // Extra data at 2Hz (includes GPS_RAW)
  mavlink_msg_request_data_stream_pack(
    SYSTEM_ID, COMPONENT_ID, &msg,
    1, 1,
    MAV_DATA_STREAM_EXTRA1,
    2,  // 2 Hz
    1);
  queueCommand(msg, false, false);

  telemetryStreamsInitialized = true;
}


void sendTelemetryData() {
  // Use the global variables that are updated by handle_mavlink_message
  const char* mode_str = getFlightModeString(base_mode, custom_mode);

  DynamicJsonDocument doc(1024);
  doc["altitude"] = current_altitude;
  doc["battery"] = battery_voltage;
  doc["mode"] = mode_str;
  doc["gps_fix"] = posAccuracy.getFixTypeString();
  doc["satellites_visible"] = posAccuracy.satellites;
  doc["latitude"] = posAccuracy.latitude;
  doc["longitude"] = posAccuracy.longitude;

  // Add accuracy information
  JsonObject accuracy = doc.createNestedObject("accuracy");
  accuracy["estimated"] = posAccuracy.getEstimatedAccuracy();
  accuracy["horizontal"] = posAccuracy.horizontal_accuracy;
  accuracy["vertical"] = posAccuracy.vertical_accuracy;
  accuracy["rtcm_age"] = (millis() - posAccuracy.last_rtcm_time) / 1000.0f;
  if (timeInitialized) {
    doc["timestamp"] = getCurrentTimeUsLite();
  } else {
    doc["timestamp"] = "not_initialized";
  }

  char buffer[1024];
  serializeJson(doc, buffer);
  if (client.publish(mqtt_telemetry_topic, buffer)) {
    lastSuccessfulPublish = millis();  // Update only on successful publish
  } else if (millis() - lastSuccessfulPublish > PUBLISH_TIMEOUT) {
    // If publish fails and we haven't had a successful publish in a while
    client.disconnect();  // Force disconnect
    // Next loop iteration will attempt reconnection
  }
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
  Serial.println("\nProcessing mission upload");

  // Print the raw JSON for debugging
  // String jsonStr;
  // serializeJson(doc, jsonStr);
  // Serial.println("Received JSON:");
  // Serial.println(jsonStr);

  // Check if the document is an array
  JsonArray waypoints;
  if (doc.is<JsonArray>()) {
    waypoints = doc.as<JsonArray>();
  } else if (doc.containsKey("waypoints")) {
    // Fallback for object format
    waypoints = doc["waypoints"].as<JsonArray>();
  } else {
    Serial.println("Error: Invalid mission format");
    publishStatus("error: invalid mission format");
    clearMissionData();  // Clear data on error
    client.publish(mqtt_status_topic, "{\"status\": \"error: invalid mission format\"}");
    return;
  }

  Serial.printf("Number of waypoints: %d\n, lighsequenceonly: %d", waypoints.size(),isLightSequenceOnly );
  if (isLightSequenceOnly || waypoints.size() > 0) {
    mission.clear();
    int waypointIndex = 0;
    for (JsonObject waypoint : waypoints) {
      double lat = waypoint["lat"];
      double lon = waypoint["lon"];
      double alt = waypoint["alt"];
      uint32_t time = waypoint["time"];
      Serial.printf("Waypoint %d: lat=%.14f, lon=%.14f, alt=%.2f, time=%u\n",
                    waypointIndex++, lat, lon, alt, time);
      mission.push_back({ lat, lon, alt, time * 1000ULL, false });  // Convert milli to microseconds
    }
    missionStarted = false;
    currentWaypointIndex = 0;
    client.publish(mqtt_status_topic, "{\"status\": \"waypoint mission uploaded successfully\"}");
  } else {
    client.publish(mqtt_status_topic, "{\"status\": \"error: invalid mission data\"}");
    publishStatus("error: invalid mission data");
    Serial.println("Error: Invalid mission format");
    clearMissionData();
    return;
  }

  // Parse light sequence if available
  if (doc.containsKey("light_sequence")) {
    Serial.println("light sequesnce was present");
    JsonArray lightSeq = doc["light_sequence"].as<JsonArray>();
    parseLightSequence(lightSeq);
    Serial.printf("Parsed %d light sequence steps\n", lightSequence.count);
    client.publish(mqtt_status_topic, "{\"status\": \"leds mission uploaded successfully\"}");
  }
  currentLightIndex = 0;
  publishStatus("mission uploaded");
}


void startMission() {
  if (isLightSequenceOnly || (!mission.empty() && !missionStarted)) {
    uint64_t currentTime = getCurrentTimeUsLite();
    uint64_t currentSeconds = (currentTime / 1000000ULL) % 86400ULL;

    if (targetStartTimeSeconds > currentSeconds) {
      // Instead of waiting here, set flag and return
      waitingForMissionStart = true;
      Serial.printf("Will start mission at target time: ");
      char timestamp[20];
      formatTimestamp(timestamp, sizeof(timestamp), targetStartTimeSeconds * 1000000ULL);
      Serial.printf("Will start mission at target time: %s\n", timestamp);
      publishStatus("Waiting for start time");
      return;
    }

    // Actual mission start logic
    startMissionExecution();
  }
}

void clearMissionData() {
  // Clear waypoints
  mission.clear();
  currentWaypointIndex = 0;

  // Clear light sequence
  // Clear light sequence
  lightSequence.count = 0;
  lightSequence.currentIndex = 0;

  // Reset mission states
  missionStarted = false;
  waitingForMissionStart = false;
  waitingForTakeoff = false;
  missionCancelled = false;
  lastArmingCheck = -1;

  // Clear LEDs
  FastLED.clear();
  FastLED.show();

  // Reset timing variables
  missionStartTime = 0;
  targetStartTimeSeconds = 0;
  targetTakeoffAltitude = 0;

  // Clear any existing mission buffer
  cleanupMissionBuffer();
}

// Adding this new function to handle mission cancellation
void handleMissionCancel() {
  missionCancelled = true;
  missionCancelTime = millis();
  waitingForMissionStart = false;
  waitingForTakeoff = false;
  missionStarted = false;

  // Clear all mission-related states
  currentWaypointIndex = 0;
  currentLightIndex = 0;
  startupState = INIT; 

  // Turn off LEDs
  FastLED.clear();
  FastLED.show();

  // If drone is armed and flying, switch to LAND mode
  if (armed) {
    changeFlightMode(LAND_MODE);
  }

  publishStatus("mission cancelled");
}

void startMissionExecution() {
    if (missionCancelled) {
        return;
    }
    // Serial.printf("Mission state=%d\n",startupState);
    unsigned long currentTime = millis();

    switch (startupState) {
        case INIT:
            changeFlightMode(GUIDED_MODE);
            stateStartTime = currentTime;
            startupState = MODE_CHANGE_WAIT;
            publishStatus("changing to guided mode");
            break;

        case MODE_CHANGE_WAIT:
            if (currentTime - stateStartTime >= 500) {
                startupState = ARMING_WAIT;
                stateStartTime = currentTime;
                armDrone();
                publishStatus("arming drone");
            }
            break;

        case ARMING_WAIT:
            if (currentTime - stateStartTime >= 1000) {
                startupState = TAKING_OFF;
                currentWaypointIndex = 0;
                currentLightIndex = 0;

                // Reset light sequence
                for (size_t i = 0; i < lightSequence.count; i++) {
                    lightSequence.steps[i].executed = false;
                }

                // Clear LEDs
                FastLED.clear();
                FastLED.show();

                waitingForTakeoff = true;
                missionTakeoff(targetTakeoffAltitude);
                publishStatus("starting takeoff");
            }
            break;

        case TAKING_OFF:
            // Only start mission after takeoff is complete
            if (!waitingForTakeoff) {
                waitingForMissionStart = false;
                missionStartTime = getCurrentTimeUsLite();
                missionStarted = true;
                publishStatus("takeoff complete, starting waypoint navigation");
                // startupState = INIT;  // Reset for next time
            }
            break;
    }
}

void missionTakeoff(float altitude) {
    static bool takeoffCommandSent = false;

    // Send takeoff command if not sent yet
    if (!takeoffCommandSent) {
        takeoff(altitude);
        takeoffCommandSent = true;
        takeoffStartTime = millis();
        publishStatus("Sending takeoff command");
        return;
    }

    // Check altitude progress
    float currentAlt = current_altitude;
    unsigned long elapsedTime = millis() - takeoffStartTime;

    // Check if we've reached target altitude or timed out
    if (currentAlt >= altitude * 0.95 || elapsedTime >= 10000) {
        waitingForTakeoff = false;
        takeoffCommandSent = false; // Reset for potential future takeoffs

        if (currentAlt >= altitude * 0.95) {
            publishStatus("Reached target altitude, starting mission");
            // Important: Don't set missionStarted here
            // startupState = INIT;  // Reset state machine for next time
            // Let startMissionExecution handle the mission start
        } else {
            publishStatus("Takeoff timeout");
            // handleMissionCancel();  // Cancel mission on timeout
            return;
        }
    } else {
        // Still waiting for altitude...
        if (elapsedTime % 1000 == 0) {  // Update status every second
            char status[50];
            snprintf(status, sizeof(status), "Takeoff in progress: %.1fm/%.1fm", currentAlt, altitude);
            publishStatus(status);
        }
        return;  // Important: return while still taking off
    }
}

void handlePreMissionBlink() {
  if (missionStarted) return;  // Don't run if mission has started

  // Get current second using getCurrentTimeUsLite()
  uint64_t currentTimeUs = getCurrentTimeUsLite();
  uint64_t currentSecond = (currentTimeUs / 1000000ULL) % 86400ULL;  // Seconds since midnight

  // Turn on white for even seconds, off for odd seconds
  if (currentSecond % 2 == 0) {
    // Set white color (R=255, G=255, B=255)
    setRGB(255, 255, 255);
  } else {
    // Turn off LEDs
    FastLED.clear();
    FastLED.show();
  }
}

void executeLightSequence() {
    if (!missionStarted || lightSequence.count == 0 || missionCancelled) return;

    uint64_t currentTime = getCurrentTimeUsLite();
    uint64_t elapsedTime = currentTime - missionStartTime;

    // Find and execute all light steps that should be active
    while (lightSequence.currentIndex < lightSequence.count && 
           elapsedTime >= lightSequence.steps[lightSequence.currentIndex].time) {
        
        if (!lightSequence.steps[lightSequence.currentIndex].executed) {
            const LightStep& step = lightSequence.steps[lightSequence.currentIndex];
            
            // Update LED values using FastLED
            setRGB(step.r, step.g, step.b);

            Serial.printf("Updated lights: R=%u, G=%u, B=%u at time %llu\n",
                        step.r, step.g, step.b, elapsedTime);

            lightSequence.steps[lightSequence.currentIndex].executed = true;
        }
        lightSequence.currentIndex++;
    }
}

// Modify the executeMission() function to be more resilient
void executeMission() {
  if (!missionStarted || mission.empty() || missionCancelled) return;

  uint64_t currentTime = getCurrentTimeUsLite();
  uint64_t elapsedTime = currentTime - missionStartTime;

  // Don't start navigation until we reach the time of the first waypoint
  if (elapsedTime < mission[0].time) {
    if (!mission[0].visited) {
      char status_message[100];
      snprintf(status_message, sizeof(status_message), 
               "Waiting for first waypoint: %.1f seconds remaining", 
               (mission[0].time - elapsedTime) / 1000000.0);
      publishStatus(status_message);
    }
    return;
  }

  // Special case for the first waypoint - if it hasn't been visited yet
  if (!mission[0].visited) {
      if (elapsedTime >= mission[0].time) {
          navigateToWaypoint(mission[0]);
          mission[0].visited = true;
          currentWaypointIndex = 0;
          
          char status_message[100];
          snprintf(status_message, sizeof(status_message),
                  "Navigating to first waypoint at time %llu",
                  mission[0].time / 1000);
          publishStatus(status_message);
          return;
      }
      return;  // Still waiting for first waypoint time
  }

  // // Check if we're at the last waypoint and it's been visited - initiate landing
  // if (currentWaypointIndex == mission.size() - 1 && mission[currentWaypointIndex].visited) {
  //     // Mission complete
  //     changeFlightMode(LAND_MODE);
  //     missionStarted = false;
  //     if (client.connected()) {
  //         publishStatus("mission completed, initiating landing");
  //     }
  //     return;
  // }
  // Find the target waypoint based on elapsed time
  size_t targetIndex = currentWaypointIndex;
  while (targetIndex < mission.size() - 1 && elapsedTime >= mission[targetIndex + 1].time) {
    targetIndex++;
  }

  // Only update waypoint if we've moved to a new one
  if (targetIndex != currentWaypointIndex) {
    currentWaypointIndex = targetIndex;
      Waypoint& currentWaypoint = mission[currentWaypointIndex];
      navigateToWaypoint(currentWaypoint);
      currentWaypoint.visited = true;
      
      // Try to publish status if connected, but don't block if disconnected
      char status_message[100];
       snprintf(status_message, sizeof(status_message),
               "Navigating to waypoint %d at time %llu",
               currentWaypointIndex, 
               currentWaypoint.time / 1000); // Convert to seconds
      
      publishStatus(status_message);
  }
}


// Modify the navigateToWaypoint function to be more resilient
void navigateToWaypoint(const Waypoint& waypoint) {
  mavlink_message_t msg;
  mavlink_msg_mission_item_int_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                   1, 1, 0, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                   MAV_CMD_NAV_WAYPOINT,
                                   2, 0, 0, 0, 0, 0,
                                   waypoint.lat * 1e7, waypoint.lon * 1e7, waypoint.alt);
  
  // Always queue the command regardless of connection status
  queueCommand(msg, true, false);

  // Log locally even if we can't publish
  Serial.printf("Navigating to waypoint %d: Lat %.6f, Lon %.6f, Alt %.2f\n",
                currentWaypointIndex, waypoint.lat, waypoint.lon, waypoint.alt);
}

void changeFlightMode(uint8_t mode) {
  mavlink_message_t msg;
  mavlink_msg_set_mode_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode);

  if (queueCommand(msg, true, false)) {  // false for non-critical
    const char* mode_str = getFlightModeString(MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode);
    char status_message[50];
    snprintf(status_message, sizeof(status_message), "Queued mode change to %s", mode_str);
    publishStatus(status_message);
    Serial.println(status_message);
  };
}

// Simplified function to handle time sync message from GCS
void handleTimeSync(const JsonDocument& doc) {
  if (!doc.containsKey("timestamp")) {
    return;  // Only check for valid timestamp
  }

  uint64_t gcsTime = doc["timestamp"].as<uint64_t>();
  uint64_t oldOffset = systemTimeOffset;  // Store old offset for logging
  systemTimeOffset = gcsTime - micros();
  timeInitialized = true;  // Keep this flag for other features that need it

  char status_message[100];
  snprintf(status_message, sizeof(status_message),
           "Time re-synchronized. Offset adjustment: %lld microseconds",
           (systemTimeOffset - oldOffset));
  publishStatus(status_message);

  Serial.printf("Time sync updated. Old offset: %llu, New offset: %llu, Adjustment: %lld\n",
                oldOffset, systemTimeOffset, (systemTimeOffset - oldOffset));
}

//Give absolute time without mavlink call, based on last gps call
uint64_t getCurrentTimeUsLite() {
  if (!timeInitialized) {
    return micros();
  }
  return micros() + systemTimeOffset;
}

// bool isArmed() { // change for queueCommand if requried
//     // Request a HEARTBEAT message
//     mavlink_message_t msg;
//     uint8_t buf[MAVLINK_MAX_PACKET_LEN];

//     mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
//                                   1, 1, // target system, target component
//                                   MAV_CMD_REQUEST_MESSAGE, 0, // command, confirmation
//                                   MAVLINK_MSG_ID_HEARTBEAT, 0, 0, 0, 0, 0, 0); // params

//     uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
//     pixhawkSerial.write(buf, len);

//     // Wait for a short time to receive the HEARTBEAT
//     unsigned long start = millis();
//     while (millis() - start < 1000) { // Wait up to 1 second
//         if (pixhawkSerial.available()) {
//             uint8_t c = pixhawkSerial.read();
//             if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
//                 if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
//                     mavlink_heartbeat_t heartbeat;
//                     mavlink_msg_heartbeat_decode(&msg, &heartbeat);
//                     armed = (heartbeat.base_mode & MAV_MODE_FLAG_SAFETY_ARMED);
//                     return armed;
//                 }
//             }
//         }
//     }

//     // If we didn't receive a new HEARTBEAT, return the last known arm status
//     return armed;
// }

void armDrone() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0);

  if (queueCommand(msg, true, false)) {  // High priority, needs ACK
    publishStatus("Queued arm command");
    Serial.println("Queued arm command");
  }
}

void takeoff(float altitude) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg, 1, 1, MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude);

  if (queueCommand(msg, true, false)) {  // High priority, needs ACK
    publishStatus("Queued takeoff command");
    Serial.println("Queued takeoff command");
  }
}

// Add this helper function to print the free heap memory
void printHeapStats() {
  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("Largest free block: %u bytes\n", ESP.getMaxAllocHeap());
}