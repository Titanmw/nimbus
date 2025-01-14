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
#define TARGET_COMPONENT_ID 0

// WiFi credentials
#define WIFI_SSID "Sebastians Kurz IPhone"
#define WIFI_PASSWORD "Korruption"

// UART buffer size
#define BUFFER_SIZE 256

// Global variables to store battery status and waypoints
float batteryVoltage = 0.0;
float batteryCurrent = 0.0;
struct Waypoint {
    uint16_t seq;
    uint8_t frame;
    uint16_t command;
    float latitude;
    float longitude;
    float altitude;
};
std::vector<Waypoint> waypoints;

bool readyToArm = false; // Status, ob die Drohne bereit ist zu armen

// WiFi and WebServer
WebServer server(80);
uint8_t rxBuffer[BUFFER_SIZE];
size_t rxBufferIndex = 0;
unsigned long lastHeartbeatTime = 0;

void setup() {
    try {
        // Initialize Serial1 for MAVLink communication
        Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
        Serial.begin(115200); // For debugging

        Serial.println("ESP32-S3 MAVLink with WebServer - Starting");

        // Set up WiFi and start web server
        setupWiFi();
        server.on("/getBattery", handleGetBattery);
        server.on("/getWayPoints", handleGetWayPoints);
        server.on("/arm", HTTP_PUT, handleArm);
        server.on("/readyToArm", handleGetReadyToArm);
        server.begin();
        Serial.println("Web server started");

        // Request waypoints at startup
        requestWaypoints();
    } catch (const std::exception &e) {
        Serial.println("Error initializing MAVLink communication: ");
        Serial.println(e.what());
    }
}

void setupWiFi() {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD); // WLAN-SSID und Passwort

    Serial.println("Connecting to WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nWiFi connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); // Zeigt die lokale IP-Adresse des ESP32 im Netzwerk an
}

void loop() {
    try {
        // Handle HTTP requests
        server.handleClient();

        // Check for incoming MAVLink messages continuously
        checkForIncomingMessages();

        // Send a MAVLink Heartbeat message every second
        if (millis() - lastHeartbeatTime >= 10000) {
            sendHeartbeat();
            lastHeartbeatTime = millis();

          // Request waypoints
          requestWaypoints();
        }
    } catch (const std::exception &e) {
        Serial.println("Error in MAVLink loop: ");
        Serial.println(e.what());
    }
}

void sendHeartbeat() {
    try {
        mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_heartbeat_pack(
            SYSTEM_ID, COMPONENT_ID, &msg,
            MAV_TYPE_GENERIC, MAV_AUTOPILOT_GENERIC,
            MAV_MODE_PREFLIGHT, 0, MAV_STATE_STANDBY
        );

        // Serialize the message to the buffer
        uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

        // Send the serialized message via UART
        Serial1.write(buffer, len);
    } catch (const std::exception &e) {
        Serial.println("Error sending Heartbeat: ");
        Serial.println(e.what());
    }
}

void requestWaypoints() {
    mavlink_message_t msg;
    mavlink_msg_mission_request_list_pack(
        SYSTEM_ID, COMPONENT_ID, &msg,
        TARGET_SYSTEM_ID, TARGET_COMPONENT_ID
        , 0 // Add the final argument as required by the function
    );

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);

    // Send the serialized message via UART
    Serial1.write(buffer, len);
    //Serial.println("Mission Request List sent");
}

void checkForIncomingMessages() {
    try {
        while (Serial1.available()) {
            uint8_t c = Serial1.read();
            rxBuffer[rxBufferIndex++] = c;

            if (rxBufferIndex >= BUFFER_SIZE) {
                rxBufferIndex = 0; // Prevent buffer overflow
            }

            mavlink_message_t msg;
            mavlink_status_t status;

            // Try to decode a MAVLink message
            if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
                //Serial.println("MAVLink message received!");
                //Serial.print("Message ID: ");
                //Serial.println(msg.msgid);

                switch (msg.msgid) {
                    case MAVLINK_MSG_ID_HEARTBEAT: {
                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);

                        Serial.print("System Status: ");
                        Serial.println(heartbeat.system_status); // Aktueller Zustand der Drohne
                        Serial.print("Base Mode: ");
                        Serial.println(heartbeat.base_mode); // Modus der Drohne

                        // Aktualisiere readyToArm basierend auf dem Systemstatus
                        if (heartbeat.system_status == MAV_STATE_STANDBY) {
                            readyToArm = true; // Drohne ist bereit zu armen
                            Serial.println("Drone is ready to arm.");
                        } else {
                            readyToArm = false; // Drohne ist nicht bereit zu armen
                            Serial.println("Drone is not ready to arm.");
                        }
                        break;
                    }

                    case MAVLINK_MSG_ID_BATTERY_STATUS: {
                        //Serial.println("Battery Status received");
                        mavlink_battery_status_t battery_status;
                        mavlink_msg_battery_status_decode(&msg, &battery_status);

                        // Update global variables for voltage and current
                        if (battery_status.voltages[0] != UINT16_MAX) {
                            batteryVoltage = battery_status.voltages[0] / 1000.0; // Convert mV to V
                        }

                        if (battery_status.current_battery != -1) {
                            batteryCurrent = battery_status.current_battery / 100.0; // Convert mA to A
                        }

                        //Serial.print("Voltage (V): ");
                        //Serial.println(batteryVoltage);
                        //Serial.print("Current (A): ");
                        //Serial.println(batteryCurrent);
                        break;
                    }

                    case MAVLINK_MSG_ID_MISSION_COUNT: {
                        mavlink_mission_count_t mission_count;
                        mavlink_msg_mission_count_decode(&msg, &mission_count);
                        //Serial.print("Mission Count: ");
                        //Serial.println(mission_count.count);
                        waypoints.clear();
                        for (uint16_t i = 0; i < mission_count.count; i++) {
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

                    case MAVLINK_MSG_ID_MISSION_ITEM_INT: {
                        mavlink_mission_item_int_t mission_item;
                        mavlink_msg_mission_item_int_decode(&msg, &mission_item);
                        Waypoint wp = {
                            mission_item.seq,
                            mission_item.frame,
                            mission_item.command,
                            mission_item.x / 1e7,
                            mission_item.y / 1e7,
                            mission_item.z
                        };
                        waypoints.push_back(wp);
                        //Serial.print("Waypoint received: ");
                        //Serial.println(wp.seq);
                        break;
                    }

                    default:
                        //Serial.print("Unhandled message ID: ");
                        //Serial.println(msg.msgid);
                        break;
                }
            }
        }
    } catch (const std::exception &e) {
        Serial.println("Error checking incoming messages: ");
        Serial.println(e.what());
    }
}

void handleGetBattery() {
    String response = "{";
    response += "\"voltage\": " + String(batteryVoltage, 2) + ", ";
    response += "\"current\": " + String(batteryCurrent, 2);
    response += "}";

    server.send(200, "application/json", response);
}

void handleGetWayPoints() {
    String response = "[";
    for (size_t i = 0; i < waypoints.size(); i++) {
        response += "{";
        response += "\"seq\": " + String(waypoints[i].seq) + ", ";
        response += "\"frame\": " + String(waypoints[i].frame) + ", ";
        response += "\"command\": " + String(waypoints[i].command) + ", ";
        response += "\"latitude\": " + String(waypoints[i].latitude, 7) + ", ";
        response += "\"longitude\": " + String(waypoints[i].longitude, 7) + ", ";
        response += "\"altitude\": " + String(waypoints[i].altitude, 2);
        response += "}";
        if (i < waypoints.size() - 1) {
            response += ", ";
        }
    }
    response += "]";

    server.send(200, "application/json", response);
}

void handleArm() {
    if (server.hasArg("plain")) {
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, server.arg("plain"));

        if (error) {
            Serial.print("JSON Parsing Error: ");
            Serial.println(error.c_str());
            server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON format.\"}");
            return;
        }

        bool armCommand = doc["state"] == 1;
        Serial.print("Arm Command: ");
        Serial.println(armCommand ? "ARM" : "DISARM");

        String response;

        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_command_long_pack(SYSTEM_ID, COMPONENT_ID, &msg,
                                      TARGET_SYSTEM_ID, TARGET_COMPONENT_ID,
                                      MAV_CMD_COMPONENT_ARM_DISARM,
                                      0, armCommand ? 1 : 0, 0, 0, 0, 0, 0, 0);

        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        Serial.print("MAVLink Message Length: ");
        Serial.println(len);

        // Debug: Output the MAVLink message buffer
        for (int i = 0; i < len; i++) {
            Serial.print(buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();

        // Send the MAVLink message
        Serial2.write(buf, len);

        response = armCommand
            ? "{\"status\":\"success\",\"message\":\"Drone armed successfully.\"}"
            : "{\"status\":\"success\",\"message\":\"Drone disarmed successfully.\"}";

        server.send(200, "application/json", response);
    } else {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing JSON body.\"}");
    }
}

void handleGetReadyToArm() {
    String response = "{";
    response += "\"readyToArm\": " + String(readyToArm ? "true" : "false");
    response += "}";

    server.send(200, "application/json", response);
}

