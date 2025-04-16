import 'dart:convert';
import 'dart:typed_data';
import 'package:http/http.dart' as http;
import 'package:route_cli/drone_status.dart';
import 'package:route_cli/waypoint.dart';

class DroneController {
  final String baseUrl;

  DroneController(this.baseUrl);

  Future<void> setWaypoints(List<Waypoint> waypoints) async {
    final url = Uri.parse('$baseUrl/addWayPoints');
    final response = await http.post(url,
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode(waypoints));

    if (response.statusCode == 200) {
      print("✅ Waypoints gesetzt.");
    } else {
      print("❌ Fehler beim Setzen der Waypoints: ${response.body}");
    }
  }

  Future<bool> setMode(String mode) async {
    final url = Uri.parse('$baseUrl/setMode');
    final response = await http.post(
      url,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'mode': mode}),
    );

    if (response.statusCode == 200) {
      final json = jsonDecode(response.body);

      final isAccepted = json['status'] == 'ack' &&
          json['mode'] == mode.toUpperCase() &&
          json['result_text'] == 'MAV_RESULT_ACCEPTED';

      if (isAccepted) {
        print("✅ Modus erfolgreich gesetzt: $mode");
        return true;
      } else {
        print(
            "⚠️ Modusantwort erhalten, aber nicht akzeptiert: ${json['result_text']}");
        return false;
      }
    } else {
      print("❌ Fehler beim Setzen des Modus: ${response.body}");
      return false;
    }
  }

  Future<bool> arm() async {
    final url = Uri.parse('$baseUrl/arm');
    final response = await http.post(url);

    if (response.statusCode == 200) {
      print("✅ Drohne ist bewaffnet.");
      return true;
    } else {
      print("❌ Fehler beim Armen: ${response.body}");
      return false;
    }
  }

  Future<bool> startMission() async {
    final url = Uri.parse('$baseUrl/startMission');
    final response = await http.post(url);

    if (response.statusCode == 200) {
      print("✅ Mission gestartet.");
      return true;
    } else {
      print("❌ Fehler beim Starten der Mission: ${response.body}");
      return false;
    }
  }

  Future<DroneStatus?> getStatus() async {
    final url = Uri.parse('$baseUrl/getStatus');
    final response = await http.get(url);

    if (response.statusCode == 200) {
      final json = jsonDecode(response.body);
      return DroneStatus.fromJson(json);
    } else {
      print("❌ Fehler beim Abrufen des Status: ${response.body}");
      return null;
    }
  }

  Future<Uint8List?> getCameraImage() async {
    final url = Uri.parse('$baseUrl/camera');
    final response = await http.get(url);

    if (response.statusCode == 200 &&
        response.headers['content-type'] == 'image/jpeg') {
      return response.bodyBytes;
    } else {
      print('❌ Fehler beim Abrufen des Kamerabilds: ${response.statusCode}');
      return null;
    }
  }

  Future<void> takeoffAndFly(List<Waypoint> waypoints,
      {double takeoffAltitude = 10.0}) async {
    final fullMission = [
      Waypoint.takeoff(altitude: takeoffAltitude),
      ...waypoints
    ];
    await setWaypoints(fullMission);
    await setMode("AUTO");
    await arm();
    await startMission();
  }
}
