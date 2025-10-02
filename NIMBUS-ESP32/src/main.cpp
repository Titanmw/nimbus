#include <Arduino.h>
#include <MAVLink.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// Camera Definition
#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define LED_LEDC_GPIO 33        // configure LED pin
#define CONFIG_LED_MAX_INTENSITY 255

#include "esp_camera.h"
#include "camera_pins.h"

// UART Pins
#define BAUD_RATE 57600

// MAVLink System ID
#define SYSTEM_ID 70
#define COMPONENT_ID 1
#define TARGET_SYSTEM_ID 0
#define TARGET_COMPONENT_ID 1

// WiFi credentials
#define WIFI_SSID "Sebastians Kurz IPhone"
#define WIFI_PASSWORD "Korruption"

// UART buffer size
#define BUFFER_SIZE 1024

struct Waypoint
{
  uint16_t seq;         // Reihenfolge-Nummer des Waypoints
  uint8_t frame;        // Frame-Typ (z. B. global, relativ, lokal)
  uint16_t command;     // MAVLink-Befehl für den Waypoint (z. B. NAV_WAYPOINT)
  uint8_t current;      // 1 = aktueller Waypoint, 0 = zukünftiger Waypoint
  uint8_t autocontinue; // Automatischer Übergang zum nächsten Waypoint
  float param1;         // Parameter 1 (z. B. Verweilzeit)
  float param2;         // Parameter 2 (z. B. Radius)
  float param3;         // Parameter 3 (z. B. Yaw-Orientierung)
  float param4;         // Parameter 4 (z. B. Heading)
  float latitude;       // Breitengrad des Waypoints (in Grad)
  float longitude;      // Längengrad des Waypoints (in Grad)
  float altitude;       // Höhe des Waypoints (in Metern)
  uint8_t mission_type; // Missionstyp (z. B. Standardmission, VTOL, Rover)
};

// Global variables to store battery status and waypoints
float batteryVoltage = 0.0;
float batteryCurrent = 0.0;
float gpsLatitude = 0.0;
float gpsLongitude = 0.0;
float gpsAltitude = 0.0;
uint8_t droneBaseMode = 0;
uint8_t droneCustomMode = 0;
uint8_t droneSystemStatus = 0;
uint8_t gpsFixType = 0;    // GPS-Fix-Status (0 = Kein Fix, 2 = 2D Fix, 3 = 3D Fix)
uint8_t gpsSatellites = 0; // Anzahl der sichtbaren Satelliten
std::vector<Waypoint> waypoints;

// WiFi and WebServer
WebServer server(80);
uint8_t rxBuffer[BUFFER_SIZE];
size_t rxBufferIndex = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastGPSRequestTime = 0;

int led_duty = 0;

void setupLedFlash(int pin)
{
#if CONFIG_LED_ILLUMINATOR_ENABLED
  // ledcAttach(pin, 5000, 8);

  ledcSetup(0, 5000, 8); // Kanal 0, Frequenz 5000 Hz, 8 Bit Auflösung
  ledcAttachPin(pin, 0); // Pin an Kanal 0 anhängen

#else
  log_i("LED flash is disabled -> CONFIG_LED_ILLUMINATOR_ENABLED = 0");
#endif
}

void enable_led(bool en)
{ // Turn LED On or Off
  int duty = en ? CONFIG_LED_MAX_INTENSITY : 0;
  ledcWrite(LED_LEDC_GPIO, duty);
}

void setupWiFi()
{
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // WLAN-SSID und Passwort

  // Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    // Serial.print(".");
  }
  // Serial.println("\nWiFi connected");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP()); // Zeigt die lokale IP-Adresse des ESP32 im Netzwerk an
}

void sendHeartbeat()
{
  try
  {
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(
        SYSTEM_ID, COMPONENT_ID, &msg,
        MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC,
        MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY);

    // Serialize the message to the buffer
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    // Send the serialized message via UART
    Serial.write(buffer, len);
  }
  catch (const std::exception &e)
  {
    // Serial.println("Error sending Heartbeat: ");
    // Serial.println(e.what());
  }
}

void sendMAVLinkMessage(mavlink_message_t &msg)
{
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  Serial.write(buffer, len);
}

bool receiveMAVLinkMessage(mavlink_message_t &msg, uint8_t msg_id)
{
  mavlink_status_t status;
  while (Serial.available())
  {
    uint8_t c = Serial.read();
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      if (msg.msgid == msg_id)
      {
        return true;
      }
    }
  }
  return false;
}

void requestWaypoints()
{
  mavlink_message_t msg;
  mavlink_msg_mission_request_list_pack(
      SYSTEM_ID, COMPONENT_ID, &msg,
      TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, 0 // Add the final argument as required by the function
  );

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

  // Send the serialized message via UART
  Serial.write(buffer, len);
  // //Serial.println("Mission Request List sent");
}

void sendMissionCount()
{
  uint16_t count = waypoints.size();

  if (count < 0)
  {
    count = 0;
  }

  mavlink_message_t msg;
  uint8_t mission_type = 0;      // Standardmission
  uint32_t mission_checksum = 0; // Falls nicht benötigt, auf 0 setzen

  mavlink_msg_mission_count_pack(
      SYSTEM_ID, COMPONENT_ID, &msg,
      TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
      count, mission_type, mission_checksum);

  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  Serial.write(buffer, len);
}

void uploadWaypoints()
{
  // Serial.println("Lösche vorhandene Wegpunkte...");
  mavlink_message_t msg;
  mavlink_msg_mission_clear_all_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, 0); // Hier das fehlende Argument hinzufügen
  sendMAVLinkMessage(msg);
  delay(500);

  // Serial.printf("Sende %d neue Wegpunkte...\n", waypoints.size());
  mavlink_msg_mission_count_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, waypoints.size(), 0, 0); // Fehlende Argumente ergänzt
  sendMAVLinkMessage(msg);
  delay(500);

  for (const auto &wp : waypoints)
  {
    mavlink_message_t wp_msg;
    mavlink_msg_mission_item_int_pack(SYSTEM_ID, COMPONENT_ID, &wp_msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
                                      wp.seq, wp.frame, wp.command, wp.current, wp.autocontinue,
                                      wp.param1, wp.param2, wp.param3, wp.param4,
                                      (int32_t)(wp.latitude * 1e7), (int32_t)(wp.longitude * 1e7), wp.altitude, wp.mission_type);
    sendMAVLinkMessage(wp_msg);
    delay(500);
  }
  // Serial.println("Alle neuen Wegpunkte erfolgreich hochgeladen.");
}

void setMissionMode()
{
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

  mavlink_msg_set_mode_pack(
      SYSTEM_ID, COMPONENT_ID, &msg,
      TARGET_SYSTEM_ID,
      MAV_MODE_GUIDED_ARMED, // Setzt den Mission-Modus
      0                      // Keine spezifische Basis-Modus-Konfiguration
  );

  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  Serial.write(buffer, len);

  // Serial.println("Mission Mode gesetzt");
}

void setFlightMode(uint8_t mode)
{
  mavlink_message_t msg;
  mavlink_msg_set_mode_pack(
      SYSTEM_ID, COMPONENT_ID, &msg,
      TARGET_SYSTEM_ID, mode, 0 // 0 für Basis-Modus-Konfiguration
  );

  sendMAVLinkMessage(msg);

  // Serial.print("Mode change requested: ");
  // Serial.println(mode);
}

void deleteAllWaypoints()
{
  mavlink_message_t msg;
  mavlink_msg_mission_clear_all_pack(
      SYSTEM_ID, COMPONENT_ID, &msg,
      TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, 0);
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
  Serial.write(buffer, len);

  // Lokale Waypoints-Liste ebenfalls leeren
  waypoints.clear();
}

void requestGPSData()
{
  mavlink_message_t msg;
  mavlink_msg_command_long_pack(
      SYSTEM_ID, COMPONENT_ID, &msg,
      TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
      MAV_CMD_REQUEST_MESSAGE, 0,
      MAVLINK_MSG_ID_GPS_RAW_INT, 0, 0, 0, 0, 0, 0);

  sendMAVLinkMessage(msg);
  // Serial.println("Requested GPS Data...");
}

String getFlightModeName(uint8_t baseMode, uint32_t customMode)
{
  if (baseMode == 81 || baseMode == 89) // ArduPilot: 81 = Armed, 89 = Flugsteuerung aktiv
  {
    switch (customMode)
    {
    case 0:
      return "STABILIZE";
    case 1:
      return "ACRO";
    case 2:
      return "ALT_HOLD";
    case 3:
      return "AUTO";
    case 4:
      return "GUIDED";
    case 5:
      return "LOITER";
    case 6:
      return "RTL";
    case 7:
      return "CIRCLE";
    case 9:
      return "LAND";
    case 11:
      return "DRIFT";
    case 13:
      return "SPORT";
    case 14:
      return "FLIP";
    case 15:
      return "AUTOTUNE";
    case 16:
      return "POSHOLD";
    case 17:
      return "BRAKE";
    case 18:
      return "THROW";
    case 19:
      return "AVOID_ADSB";
    case 20:
      return "GUIDED_NOGPS";
    case 21:
      return "SMART_RTL";
    case 22:
      return "TAKEOFF";
    case 23:
      return "QSTABILIZE";
    case 24:
      return "QHOVER";
    case 25:
      return "QLOITER";
    case 26:
      return "QLAND";
    case 27:
      return "QRTL";
    default:
      return "UNKNOWN";
    }
  }

  return "UNKNOWN"; // Falls baseMode nicht bekannt ist
}

String getSystemStatus(uint8_t status)
{
  switch (status)
  {
  case MAV_STATE_UNINIT:
    return "UNINITIALIZED";
  case MAV_STATE_BOOT:
    return "BOOT";
  case MAV_STATE_STANDBY:
    return "STANDBY";
  case MAV_STATE_ACTIVE:
    return "ACTIVE";
  case MAV_STATE_CRITICAL:
    return "CRITICAL";
  case MAV_STATE_EMERGENCY:
    return "EMERGENCY";
  default:
    return "UNKNOWN";
  }
}

uint8_t getModeValueFromString(String modeString)
{
  if (modeString.equalsIgnoreCase("MANUAL"))
    return 0;
  if (modeString.equalsIgnoreCase("CIRCLE"))
    return 1;
  if (modeString.equalsIgnoreCase("STABILIZE"))
    return 2;
  if (modeString.equalsIgnoreCase("TRAINING"))
    return 3;
  if (modeString.equalsIgnoreCase("ACRO"))
    return 4;
  if (modeString.equalsIgnoreCase("FBWA"))
    return 5;
  if (modeString.equalsIgnoreCase("FBWB"))
    return 6;
  if (modeString.equalsIgnoreCase("CRUISE"))
    return 7;
  if (modeString.equalsIgnoreCase("AUTO"))
    return 8;
  if (modeString.equalsIgnoreCase("RTL"))
    return 9;
  if (modeString.equalsIgnoreCase("LOITER"))
    return 10;
  if (modeString.equalsIgnoreCase("TAKEOFF"))
    return 11;
  if (modeString.equalsIgnoreCase("LAND"))
    return 12;
  if (modeString.equalsIgnoreCase("GUIDED"))
    return 13;
  if (modeString.equalsIgnoreCase("INITIALIZING"))
    return 14;
  if (modeString.equalsIgnoreCase("QSTABILIZE"))
    return 15;
  if (modeString.equalsIgnoreCase("QHOVER"))
    return 16;
  if (modeString.equalsIgnoreCase("QLOITER"))
    return 17;
  if (modeString.equalsIgnoreCase("QLAND"))
    return 18;
  if (modeString.equalsIgnoreCase("QRTL"))
    return 19;
  if (modeString.equalsIgnoreCase("POSHOLD"))
    return 20;
  if (modeString.equalsIgnoreCase("BRAKE"))
    return 21;
  if (modeString.equalsIgnoreCase("THROW"))
    return 22;
  if (modeString.equalsIgnoreCase("AVOID_ADSB"))
    return 23;
  if (modeString.equalsIgnoreCase("GUIDED_NOGPS"))
    return 24;
  if (modeString.equalsIgnoreCase("SMART_RTL"))
    return 25;

  return 255; // Ungültiger Modus
}

void checkForIncomingMessages()
{
  try
  {
    while (Serial.available())
    {
      uint8_t c = Serial.read();
      rxBuffer[rxBufferIndex++] = c;

      if (rxBufferIndex >= BUFFER_SIZE)
      {
        rxBufferIndex = 0; // Prevent buffer overflow
      }

      mavlink_message_t msg;
      mavlink_status_t status;

      // Try to decode a MAVLink message
      if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
      {
        // //Serial.println("MAVLink message received!");
        // //Serial.print("Message ID: ");
        // //Serial.println(msg.msgid);

        switch (msg.msgid)
        {
        case MAVLINK_MSG_ID_COMMAND_ACK:
        {
          mavlink_command_ack_t ack;
          mavlink_msg_command_ack_decode(&msg, &ack);

          // Serial.print("[ACK] Command ID: ");
          // Serial.print(ack.command);
          // Serial.print(" | Result: ");
          // Serial.println(ack.result == MAV_RESULT_ACCEPTED ? "Accepted ✅" : "Rejected ❌");

          break;
        }

        case MAVLINK_MSG_ID_HEARTBEAT:
        {
          mavlink_heartbeat_t heartbeat;
          mavlink_msg_heartbeat_decode(&msg, &heartbeat);

          droneCustomMode = heartbeat.custom_mode;
          droneBaseMode = heartbeat.base_mode;
          droneSystemStatus = heartbeat.system_status;

          // //Serial.print("Drone Mode: ");
          // //Serial.println(droneBaseMode);
          // //Serial.print("System Status: ");
          // //Serial.println(droneSystemStatus);
          break;
        }

        case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
          mavlink_gps_raw_int_t gps_data;
          mavlink_msg_gps_raw_int_decode(&msg, &gps_data);

          gpsLatitude = gps_data.lat / 1e7;
          gpsLongitude = gps_data.lon / 1e7;
          gpsAltitude = gps_data.alt / 1000.0;
          gpsFixType = gps_data.fix_type;
          gpsSatellites = gps_data.satellites_visible;

          // Serial.print("GPS Data - Lat: ");
          // Serial.print(gpsLatitude, 7);
          // Serial.print(", Lon: ");
          // Serial.print(gpsLongitude, 7);
          // Serial.print(", Alt: ");
          // Serial.print(gpsAltitude);
          // Serial.print(", Fix: ");
          // Serial.print(gpsFixType);
          // Serial.print(", Satellites: ");
          // Serial.println(gpsSatellites);
          break;
        }

        case MAVLINK_MSG_ID_BATTERY_STATUS:
        {
          // //Serial.println("Battery Status received");
          mavlink_battery_status_t battery_status;
          mavlink_msg_battery_status_decode(&msg, &battery_status);

          // Update global variables for voltage and current
          if (battery_status.voltages[0] != UINT16_MAX)
          {
            batteryVoltage = battery_status.voltages[0] / 1000.0; // Convert mV to V
          }

          if (battery_status.current_battery != -1)
          {
            batteryCurrent = battery_status.current_battery / 100.0; // Convert mA to A
          }

          // //Serial.print("Voltage (V): ");
          // //Serial.println(batteryVoltage);
          // //Serial.print("Current (A): ");
          // //Serial.println(batteryCurrent);
          break;
        }

        case MAVLINK_MSG_ID_MISSION_ACK:
        {
          // Serial.println("MISSION_ACK received!");
          // Serial.print("ACK details: ");
          // Serial.print("msgid: ");
          // Serial.print(msg.msgid);
          // Serial.print(", sysid: ");
          // Serial.print(msg.sysid);
          // Serial.print(", compid: ");
          // Serial.println(msg.compid);
          break;
        }

        case MAVLINK_MSG_ID_MISSION_COUNT:
        {
          mavlink_mission_count_t mission_count;
          mavlink_msg_mission_count_decode(&msg, &mission_count);
          // //Serial.print("Mission Count: ");
          // //Serial.println(mission_count.count);
          waypoints.clear();
          for (uint16_t i = 0; i < mission_count.count; i++)
          {
            mavlink_message_t req_msg;
            mavlink_msg_mission_request_int_pack(
                SYSTEM_ID, COMPONENT_ID, &req_msg,
                TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, i, 0 // Add 0 as last argument
            );
            uint16_t len = mavlink_msg_to_send_buffer(rxBuffer, &req_msg);
            Serial.write(rxBuffer, len);
          }
          break;
        }

        case MAVLINK_MSG_ID_MISSION_ITEM_INT:
        {
          mavlink_mission_item_int_t mission_item;
          mavlink_msg_mission_item_int_decode(&msg, &mission_item);

          // Serial.print("Waypoint received - Seq: ");
          // Serial.print(mission_item.seq);
          // Serial.print(", Lat: ");
          // Serial.print(mission_item.x / 1e7, 7);
          // Serial.print(", Lon: ");
          // Serial.print(mission_item.y / 1e7, 7);
          // Serial.print(", Alt: ");
          // Serial.println(mission_item.z);

          Waypoint wp = {
              mission_item.seq,
              mission_item.frame,
              mission_item.command,
              mission_item.current,
              mission_item.autocontinue,
              mission_item.param1,
              mission_item.param2,
              mission_item.param3,
              mission_item.param4,
              mission_item.x / 1e7,
              mission_item.y / 1e7,
              mission_item.z,
              mission_item.mission_type};
          waypoints.push_back(wp);
          break;
        }

        default:
          // //Serial.print("Unhandled message ID: ");
          // //Serial.println(msg.msgid);
          break;
        }
      }
    }
  }
  catch (const std::exception &e)
  {
    // Serial.println("Error checking incoming messages: ");
    // Serial.println(e.what());
  }
}

void handleGetMissions()
{
  // Serial.print("Sending waypoints, count: ");
  // Serial.println(waypoints.size());

  StaticJsonDocument<2048> doc;
  JsonArray array = doc.to<JsonArray>();

  for (const auto &wp : waypoints)
  {
    JsonObject obj = array.createNestedObject();
    obj["seq"] = wp.seq;
    obj["frame"] = wp.frame;
    obj["command"] = wp.command;
    obj["current"] = wp.current;
    obj["autocontinue"] = wp.autocontinue;
    obj["param1"] = wp.param1;
    obj["param2"] = wp.param2;
    obj["param3"] = wp.param3;
    obj["param4"] = wp.param4;
    obj["latitude"] = wp.latitude;
    obj["longitude"] = wp.longitude;
    obj["altitude"] = wp.altitude;
    obj["mission_type"] = wp.mission_type;

    // Serial.print("Seq: ");
    // Serial.print(wp.seq);
    // Serial.print(", Lat: ");
    // Serial.print(wp.latitude, 7);
    // Serial.print(", Lon: ");
    // Serial.print(wp.longitude, 7);
    // Serial.print(", Alt: ");
    // Serial.println(wp.altitude);
  }

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleAddWayPoint()
{
  if (server.hasArg("plain"))
  {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));

    if (error)
    {
      server.send(400, "application/json", "{\"error\": \"Invalid JSON\"}");
      return;
    }

    // Serial.println("Neue Wegpunktdaten empfangen:");
    // Serial.print("Latitude: ");
    // Serial.println(doc["latitude"].as<float>(), 7);
    // Serial.print("Longitude: ");
    // Serial.println(doc["longitude"].as<float>(), 7);
    // Serial.print("Altitude: ");
    // Serial.println(doc["altitude"].as<float>(), 2);
    // Serial.print("Command: ");
    // Serial.println((uint16_t)doc["command"]);

    // Neuen Waypoint lokal speichern
    uint16_t newSeq = waypoints.empty() ? 0 : waypoints.back().seq + 1;

    Waypoint wp = {
        newSeq,
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        (uint16_t)doc["command"],
        0, // current (0 = nicht aktiv)
        1, // autocontinue
        0.0, 0.0, 0.0, 0.0,
        (float)doc["latitude"],
        (float)doc["longitude"],
        (float)doc["altitude"],
        0};

    waypoints.push_back(wp); // Speichern des neuen Waypoints

    uploadWaypoints();

    server.send(200, "application/json", "{\"status\": \"Waypoint added\"}");
  }
  else
  {
    server.send(400, "application/json", "{\"error\": \"Invalid JSON\"}");
  }
}

void handleClearMissions()
{
  waypoints.clear();
  deleteAllWaypoints();

  server.send(200, "application/json", "{\"status\": \"All waypoints deleted\"}");
}

void handleGetStatus()
{
  StaticJsonDocument<256> doc;

  // Batterie-Daten
  doc["voltage"] = batteryVoltage;
  doc["current"] = batteryCurrent;

  // GPS-Daten
  doc["latitude"] = gpsLatitude;
  doc["longitude"] = gpsLongitude;
  doc["altitude"] = gpsAltitude;
  doc["gps_fix"] = gpsFixType;
  doc["satellites"] = gpsSatellites;

  // Flugmodus & Systemstatus
  doc["mode"] = getFlightModeName(droneBaseMode, droneCustomMode);
  doc["base_mode"] = droneBaseMode;     // Roher base_mode-Wert
  doc["custom_mode"] = droneCustomMode; // Roher custom_mode-Wert

  doc["system_status"] = getSystemStatus(droneSystemStatus);

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleSetMode()
{
  if (server.hasArg("plain"))
  {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, server.arg("plain"));

    if (error)
    {
      server.send(400, "application/json", "{\"error\": \"Invalid JSON\"}");
      return;
    }

    String modeString = doc["mode"].as<String>();
    uint8_t modeValue = getModeValueFromString(modeString);

    if (modeValue == 255)
    {
      server.send(400, "application/json", "{\"error\": \"Invalid mode\"}");
      return;
    }

    // Modus setzen
    setFlightMode(modeValue);

    // Antwort senden
    StaticJsonDocument<128> response;
    response["status"] = "Mode change requested";
    response["mode"] = modeString;

    String responseString;
    serializeJson(response, responseString);
    server.send(200, "application/json", responseString);
  }
  else
  {
    server.send(400, "application/json", "{\"error\": \"Invalid request\"}");
  }
}

void handleSetMissions()
{
  // geplant
}

void handleStartMission()
{
  // geplant
}

void handleGoto()
{
  // geplant
}

void handleRotate()
{
  // geplant
}

void handleArm()
{
  // geplant
}

void handleDisarm()
{
  // geplant
}

void handleTakeoff()
{
  // geplant
}

void handleLand()
{
  // geplant
}

void handleCaptureCamera()
{
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_SVGA);

  // Optional: Bildhelligkeit, Kontrast, Sättigung
  s->set_brightness(s, 1); // -2 bis 2
  s->set_contrast(s, 1);   // -2 bis 2
  s->set_saturation(s, 1); // -2 bis 2

  // Optional: Auto Belichtung & Belichtungszeit
  s->set_exposure_ctrl(s, 1); // 0 = aus, 1 = an
  s->set_gain_ctrl(s, 1);     // Auto-Gain
  s->set_awb_gain(s, 1);      // Auto-White-Balance

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb)
  {
    server.send(500, "text/plain", "Camera capture failed");
    return;
  }

  server.sendHeader("Content-Type", "image/jpeg");
  server.send_P(200, "image/jpeg", (char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);

  enable_led(true);
}

void initCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  // config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.frame_size = FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    if (psramFound())
    {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    }
    else
    {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  }
  else
  {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    // Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID)
  {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG)
  {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif
}

void setup()
{
  try
  {
    // Initialize Serial1 for MAVLink communication
    // Serial.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    Serial.begin(BAUD_RATE);
    // Serial.begin(115200); // For debugging

    // Serial.println("ESP32-S3 MAVLink with WebServer - Starting");

    // Set up WiFi and start web server
    initCamera();

    setupWiFi();
    server.on("/get_missions", HTTP_GET, handleGetMissions);
    server.on("/set_missions", HTTP_POST, handleSetMissions);
    server.on("/clear_missions", HTTP_DELETE, handleClearMissions);
    server.on("/start_mission", HTTP_POST, handleStartMission);
    server.on("/goto", HTTP_POST, handleGoto);
    server.on("/rotate", HTTP_POST, handleRotate);
    server.on("/arm", HTTP_POST, handleArm);
    server.on("/disarm", HTTP_POST, handleDisarm);
    server.on("/takeoff", HTTP_POST, handleTakeoff);
    server.on("/land", HTTP_POST, handleLand);
    server.on("/set_mode", HTTP_POST, handleSetMode);
    server.on("/get_status", HTTP_GET, handleGetStatus);
    server.on("/camera", HTTP_GET, handleCaptureCamera);
    server.begin();
    // Serial.println("Web server started");

    // Request waypoints at startup
    requestWaypoints();
  }
  catch (const std::exception &e)
  {
    // Serial.println("Error initializing MAVLink communication: ");
    // Serial.println(e.what());
  }
}

void loop()
{
  try
  {
    // Handle HTTP requests
    server.handleClient();

    // Check for incoming MAVLink messages continuously
    checkForIncomingMessages();

    // Send a MAVLink Heartbeat message every second
    if (millis() - lastHeartbeatTime >= 10000)
    {
      sendHeartbeat();
      lastHeartbeatTime = millis();

      // Request waypoints
      requestWaypoints();
    }

    if (millis() - lastGPSRequestTime > 5000)
    {
      requestGPSData();
      lastGPSRequestTime = millis();
    }
  }
  catch (const std::exception &e)
  {
    // Serial.println("Error in MAVLink loop: ");
    // Serial.println(e.what());
  }
}
