#include <HardwareSerial.h>
#include <MAVLink.h>

HardwareSerial MAVLinkSerial(2);

// Zielsystem und Komponente (normalerweise 1 für Pixhawk)
const uint8_t target_system = 1;
const uint8_t target_component = 1;

// Timer für regelmäßige PING-Anfragen
unsigned long lastPingTime = 0;
const unsigned long pingInterval = 2000; // 2 Sekunden
uint32_t ping_seq = 0; // Sequenznummer für PING

void setup() {
    Serial.begin(115200); // Debugging
    MAVLinkSerial.begin(115200, SERIAL_8N1, 16, 21); // MAVLink UART

    Serial.println("Starting MAVLink PING test...");
}

void loop() {
    mavlink_message_t msg;
    mavlink_status_t status;

    // Alle 2 Sekunden einen PING senden
    if (millis() - lastPingTime > pingInterval) {
        lastPingTime = millis();
        sendPing();
    }

    // Eingehende Nachrichten verarbeiten
    while (MAVLinkSerial.available()) {
        uint8_t byteReceived = MAVLinkSerial.read();

        if (mavlink_parse_char(MAVLINK_COMM_0, byteReceived, &msg, &status)) {
            if (msg.msgid == MAVLINK_MSG_ID_PING) {
                processPing(msg);
            }
        }
    }
}

// Funktion: PING senden
void sendPing() {
    mavlink_message_t msg;
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];

    uint64_t current_time = millis();

    // PING-Nachricht erstellen
    mavlink_msg_ping_pack(
        target_system,               // System-ID des ESP32
        target_component,            // Komponente des ESP32
        &msg,                        // MAVLink-Nachricht
        current_time,                // Zeitstempel
        ping_seq++,                  // Sequenznummer
        target_system,               // Zielsystem (Pixhawk)
        target_component             // Zielkomponente
    );

    // Nachricht senden
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    MAVLinkSerial.write(buffer, len);

    Serial.println("Sent PING");
}

// Funktion: PING-Antwort verarbeiten
void processPing(const mavlink_message_t& msg) {
    mavlink_ping_t ping;
    mavlink_msg_ping_decode(&msg, &ping);

    Serial.println("Received PING response:");
    Serial.print("Time: ");
    Serial.println(ping.time_usec);
    Serial.print("Seq: ");
    Serial.println(ping.seq);
    Serial.print("Target System: ");
    Serial.println(ping.target_system);
    Serial.print("Target Component: ");
    Serial.println(ping.target_component);
}
