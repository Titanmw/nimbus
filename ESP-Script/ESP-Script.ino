#include <Arduino.h>
#include <MAVLink.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// UART Pins
#define RX_PIN 4
#define TX_PIN 5
#define BAUD_RATE 57600

// MAVLink System ID
#define SYSTEM_ID 69
#define COMPONENT_ID 1
#define TARGET_SYSTEM_ID 0
#define TARGET_COMPONENT_ID 1

// WiFi credentials
#define WIFI_SSID "Sebastians Kurz IPhone"
#define WIFI_PASSWORD "Korruption"

// UART buffer size
#define BUFFER_SIZE 1024

// Global variables to store battery status and waypoints
float batteryVoltage = 0.0;
float batteryCurrent = 0.0;
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

std::vector<Waypoint> waypoints;

// WiFi and WebServer
WebServer server(80);
uint8_t rxBuffer[BUFFER_SIZE];
size_t rxBufferIndex = 0;
unsigned long lastHeartbeatTime = 0;


void setup()
{
    try
    {
        // Initialize Serial1 for MAVLink communication
        Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
        Serial.begin(115200); // For debugging

        Serial.println("ESP32-S3 MAVLink with WebServer - Starting");

        // Set up WiFi and start web server
        setupWiFi();
        server.on("/getBattery", HTTP_GET, handleGetBattery);
        server.on("/getWayPoints", HTTP_GET, handleGetWayPoints);
        server.on("/addWayPoint", HTTP_POST, handleAddWayPoint);
        server.on("/deleteAllWaypoints", HTTP_DELETE, handleDeleteAllWaypoints);
        server.begin();
        Serial.println("Web server started");

        // Request waypoints at startup
        requestWaypoints();
    }
    catch (const std::exception &e)
    {
        Serial.println("Error initializing MAVLink communication: ");
        Serial.println(e.what());
    }
}

void setupWiFi()
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // WLAN-SSID und Passwort

    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); // Zeigt die lokale IP-Adresse des ESP32 im Netzwerk an
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
    }
    catch (const std::exception &e)
    {
        Serial.println("Error in MAVLink loop: ");
        Serial.println(e.what());
    }
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
        Serial1.write(buffer, len);
    }
    catch (const std::exception &e)
    {
        Serial.println("Error sending Heartbeat: ");
        Serial.println(e.what());
    }
}

void sendMAVLinkMessage(mavlink_message_t &msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial1.write(buffer, len);
}

bool receiveMAVLinkMessage(mavlink_message_t &msg, uint8_t msg_id) {
    mavlink_status_t status;
    while (Serial1.available()) {
        uint8_t c = Serial1.read();
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            if (msg.msgid == msg_id) {
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
    Serial1.write(buffer, len);
    // Serial.println("Mission Request List sent");
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
    Serial1.write(buffer, len);
}

void uploadWaypoints() {
    Serial.println("Lösche vorhandene Wegpunkte...");
    mavlink_message_t msg;
    mavlink_msg_mission_clear_all_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, 0); // Hier das fehlende Argument hinzufügen
    sendMAVLinkMessage(msg);
    delay(500);

    Serial.printf("Sende %d neue Wegpunkte...\n", waypoints.size());
    mavlink_msg_mission_count_pack(SYSTEM_ID, COMPONENT_ID, &msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, waypoints.size(), 0, 0); // Fehlende Argumente ergänzt
    sendMAVLinkMessage(msg);
    delay(500);

    for (const auto& wp : waypoints) {
        mavlink_message_t wp_msg;
        mavlink_msg_mission_item_int_pack(SYSTEM_ID, COMPONENT_ID, &wp_msg, TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
                                          wp.seq, wp.frame, wp.command, wp.current, wp.autocontinue,
                                          wp.param1, wp.param2, wp.param3, wp.param4,
                                          (int32_t)(wp.latitude * 1e7), (int32_t)(wp.longitude * 1e7), wp.altitude, wp.mission_type);
        sendMAVLinkMessage(wp_msg);
        delay(500);
    }
    Serial.println("Alle neuen Wegpunkte erfolgreich hochgeladen.");
}

void setMissionMode()
{
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_set_mode_pack(
        SYSTEM_ID, COMPONENT_ID, &msg,
        TARGET_SYSTEM_ID,
        MAV_MODE_GUIDED_ARMED, // Setzt den Mission-Modus
        0 // Keine spezifische Basis-Modus-Konfiguration
    );

    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial1.write(buffer, len);

    Serial.println("Mission Mode gesetzt");
}

void deleteAllWaypoints()
{
    mavlink_message_t msg;
    mavlink_msg_mission_clear_all_pack(
        SYSTEM_ID, COMPONENT_ID, &msg,
        TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, 0);
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    Serial1.write(buffer, len);

    // Lokale Waypoints-Liste ebenfalls leeren
    waypoints.clear();
}

void checkForIncomingMessages()
{
    try
    {
        while (Serial1.available())
        {
            uint8_t c = Serial1.read();
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
                // Serial.println("MAVLink message received!");
                // Serial.print("Message ID: ");
                // Serial.println(msg.msgid);

                switch (msg.msgid)
                {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                    // Serial.print("System Status: ");
                    // Serial.println(heartbeat.system_status); // Aktueller Zustand der Drohne
                    // Serial.print("Base Mode: ");
                    // Serial.println(heartbeat.base_mode); // Modus der Drohne
                    break;
                }

                case MAVLINK_MSG_ID_BATTERY_STATUS:
                {
                    // Serial.println("Battery Status received");
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

                    // Serial.print("Voltage (V): ");
                    // Serial.println(batteryVoltage);
                    // Serial.print("Current (A): ");
                    // Serial.println(batteryCurrent);
                    break;
                }

                case MAVLINK_MSG_ID_MISSION_ACK:
                {
                    Serial.println("MISSION_ACK received!");
                    Serial.print("ACK details: ");
                    Serial.print("msgid: ");
                    Serial.print(msg.msgid);
                    Serial.print(", sysid: ");
                    Serial.print(msg.sysid);
                    Serial.print(", compid: ");
                    Serial.println(msg.compid);
                    break;
                }

                case MAVLINK_MSG_ID_MISSION_COUNT:
                {
                    mavlink_mission_count_t mission_count;
                    mavlink_msg_mission_count_decode(&msg, &mission_count);
                    // Serial.print("Mission Count: ");
                    // Serial.println(mission_count.count);
                    waypoints.clear();
                    for (uint16_t i = 0; i < mission_count.count; i++)
                    {
                        mavlink_message_t req_msg;
                        mavlink_msg_mission_request_int_pack(
                            SYSTEM_ID, COMPONENT_ID, &req_msg,
                            TARGET_SYSTEM_ID, TARGET_COMPONENT_ID, i, 0 // Add 0 as last argument
                        );
                        uint16_t len = mavlink_msg_to_send_buffer(rxBuffer, &req_msg);
                        Serial1.write(rxBuffer, len);
                    }
                    break;
                }

                case MAVLINK_MSG_ID_MISSION_ITEM_INT:
                {
                    mavlink_mission_item_int_t mission_item;
                    mavlink_msg_mission_item_int_decode(&msg, &mission_item);

                    Serial.print("Waypoint received - Seq: ");
                    Serial.print(mission_item.seq);
                    Serial.print(", Lat: ");
                    Serial.print(mission_item.x / 1e7, 7);
                    Serial.print(", Lon: ");
                    Serial.print(mission_item.y / 1e7, 7);
                    Serial.print(", Alt: ");
                    Serial.println(mission_item.z);

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
                    // Serial.print("Unhandled message ID: ");
                    // Serial.println(msg.msgid);
                    break;
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        Serial.println("Error checking incoming messages: ");
        Serial.println(e.what());
    }
}

void handleGetBattery()
{
    String response = "{";
    response += "\"voltage\": " + String(batteryVoltage, 2) + ", ";
    response += "\"current\": " + String(batteryCurrent, 2);
    response += "}";

    server.send(200, "application/json", response);
}

void handleGetWayPoints()
{
    Serial.print("Sending waypoints, count: ");
    Serial.println(waypoints.size());

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

        Serial.print("Seq: ");
        Serial.print(wp.seq);
        Serial.print(", Lat: ");
        Serial.print(wp.latitude, 7);
        Serial.print(", Lon: ");
        Serial.print(wp.longitude, 7);
        Serial.print(", Alt: ");
        Serial.println(wp.altitude);
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

        Serial.println("Neue Wegpunktdaten empfangen:");
        Serial.print("Latitude: ");
        Serial.println(doc["latitude"].as<float>(), 7);
        Serial.print("Longitude: ");
        Serial.println(doc["longitude"].as<float>(), 7);
        Serial.print("Altitude: ");
        Serial.println(doc["altitude"].as<float>(), 2);
        Serial.print("Command: ");
        Serial.println((uint16_t)doc["command"]);

        // Neuen Waypoint lokal speichern
        uint16_t newSeq = waypoints.empty() ? 0 : waypoints.back().seq + 1;

        Waypoint wp = {
            newSeq,
            MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            (uint16_t)doc["command"],
            0,  // current (0 = nicht aktiv)
            1,  // autocontinue
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

void handleDeleteAllWaypoints()
{
    waypoints.clear();
    deleteAllWaypoints();

    server.send(200, "application/json", "{\"status\": \"All waypoints deleted\"}");
}
