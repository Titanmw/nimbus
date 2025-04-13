#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

const char *ssid = "NIMBUS";
const char *password = "12345678";

#define SERVO_PIN 18
#define SERVO_HZ 50
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_RANGE_DEG 270
#define SERVO_DEADBAND_US 6

Servo myServo;
int currentPos = 0;
bool attached = false;

WebServer server(80);

void moveServoSmooth(int fromDeg, int toDeg, int stepDeg = 2, int delayMs = 25)
{
  if (fromDeg == toDeg)
    return;

  int dir = (toDeg > fromDeg) ? 1 : -1;
  int pos = fromDeg;
  static int lastMicros = -1;

  while (pos != toDeg)
  {
    pos += dir * stepDeg;
    if ((dir > 0 && pos > toDeg) || (dir < 0 && pos < toDeg))
      pos = toDeg;

    int pulse = map(pos, 0, SERVO_RANGE_DEG, SERVO_MIN_US, SERVO_MAX_US);

    if (abs(pulse - lastMicros) > SERVO_DEADBAND_US)
    {
      myServo.writeMicroseconds(pulse);
      lastMicros = pulse;
    }

    delay(delayMs);
  }

  currentPos = toDeg;
}

void handleInfo()
{
  server.send(200, "application/json", "{\"type\":\"koffer\"}");
}

void handleGetPosition()
{
  String response = "{\"position\":" + String(currentPos) + ", \"min\":0, \"max\":" + String(SERVO_RANGE_DEG) + "}";
  server.send(200, "application/json", response.c_str());
}

void handleSetPosition()
{
  if (!server.hasArg("pos"))
  {
    String response = "{\"error\":\"Fehlender Parameter pos\", \"min\":0, \"max\":" + String(SERVO_RANGE_DEG) + "}";
    server.send(400, "application/json", response.c_str());
    return;
  }

  int target = server.arg("pos").toInt();
  if (target < 0 || target > SERVO_RANGE_DEG)
  {
    String response = "{\"error\":\"Ungültiger Parameter pos\", \"min\":0, \"max\":" + String(SERVO_RANGE_DEG) + "}";
    server.send(400, "application/json", response.c_str());
    return;
  }

  Serial.printf("Servo: %d° -> %d°\n", currentPos, target);
  moveServoSmooth(currentPos, target);

  String response = "{\"position\":" + String(currentPos) + ", \"min\":0, \"max\":" + String(SERVO_RANGE_DEG) + "}";
  server.send(200, "application/json", response.c_str());
}

void setup()
{
  Serial.begin(115200);
  delay(100);

  // WLAN verbinden
  WiFi.begin(ssid, password);
  Serial.printf("Verbinde mit WLAN \"%s\"...\n", ssid);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000)
  {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWLAN verbunden!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\nWLAN-Verbindung fehlgeschlagen.");
  }

  // Servo verbinden
  myServo.setPeriodHertz(SERVO_HZ);
  attached = myServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  if (attached)
  {
    Serial.println("Servo erfolgreich verbunden.");
    int startPulse = map(currentPos, 0, SERVO_RANGE_DEG, SERVO_MIN_US, SERVO_MAX_US);
    myServo.writeMicroseconds(startPulse);
  }
  else
  {
    Serial.println("Fehler beim Servo-Attach.");
  }

  // Webserver einrichten
  server.on("/info", handleInfo);
  server.on("/position", handleGetPosition);
  server.on("/set", handleSetPosition);
  server.begin();
  Serial.println("Webserver gestartet.");
}

void loop()
{
  server.handleClient();

  // WLAN überwachen & reconnecten
  static unsigned long lastWiFiCheck = 0;
  if (millis() - lastWiFiCheck > 5000)
  {
    lastWiFiCheck = millis();

    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WLAN getrennt. Versuche Reconnect...");
      WiFi.disconnect(); // optional: vorherige Verbindung hart trennen
      WiFi.begin(ssid, password);
    }
    else
    {
      // optional: Debug-Ausgabe
      // Serial.println("WLAN OK");
    }
  }
}
