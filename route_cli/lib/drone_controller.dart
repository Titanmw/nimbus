import 'dart:convert';
import 'dart:typed_data';
import 'package:http/http.dart' as http;
import 'package:route_cli/drone_status.dart';
import 'package:route_cli/waypoint.dart';

class DroneController {
  final String baseUrl;

  DroneController(this.baseUrl);

  Future<void> setMissions(List<Waypoint> waypoints) async {
    final url = Uri.parse('$baseUrl/set_missions');
    final response = await http.post(url,
        headers: {'Content-Type': 'application/json'},
        body: jsonEncode(waypoints));

    if (response.statusCode == 200) {
      print("Waypoints gesetzt.");
    } else {
      print("Fehler beim Setzen der Waypoints: ${response.body}");
    }
  }

  Future<List<Waypoint>> getMissions() async {
    final url = Uri.parse('$baseUrl/get_missions');
    final response = await http.get(url);

    if (response.statusCode == 200) {
      final json = jsonDecode(response.body);
      if (json is List) {
        return json.map((wp) => Waypoint.fromJson(wp)).toList();
      } else {
        print("Unerwartetes Format bei Waypoints: $json");
      }
    } else {
      print("Fehler beim Abrufen der Waypoints: ${response.statusCode}");
    }

    return [];
  }

  Future<bool> clearMissions() async {
    final url = Uri.parse('$baseUrl/clear_missions');
    final response = await http.delete(
      url,
      headers: {'Content-Type': 'application/json'},
    );

    if (response.statusCode == 200) {
      print("Missionen erfolgreich gelöscht.");
      return true;
    } else {
      print("Fehler beim Löschen der Missionen: ${response.body}");
      return false;
    }
  }

  Future<bool> setMode(String mode) async {
    final url = Uri.parse('$baseUrl/set_mode');
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
        print("Modus erfolgreich gesetzt: $mode");
        return true;
      } else {
        print(
            "Modusantwort erhalten, aber nicht akzeptiert: ${json['result_text']}");
        return false;
      }
    } else {
      print("Fehler beim Setzen des Modus: ${response.body}");
      return false;
    }
  }

  Future<bool> arm() async {
    final url = Uri.parse('$baseUrl/arm');
    final response = await http.post(url);

    if (response.statusCode == 200) {
      print("Drohne ist scharf.");
      return true;
    } else {
      print("Fehler beim Armen: ${response.body}");
      return false;
    }
  }

  Future<bool> disarm(bool force) async {
    final url = Uri.parse('$baseUrl/disarm');
    final response = await http.post(
      url,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'force': force}),
    );

    if (response.statusCode == 200) {
      print("Drohne wurde entschärft.");
      return true;
    } else {
      print("Fehler beim Entschärfen: ${response.body}");
      return false;
    }
  }

  Future<bool> startMission() async {
    final url = Uri.parse('$baseUrl/start_mission');
    final response = await http.post(url);

    if (response.statusCode == 200) {
      print("Mission gestartet.");
      return true;
    } else {
      print("Fehler beim Starten der Mission: ${response.body}");
      return false;
    }
  }

  Future<bool> goto({
    required double latitude,
    required double longitude,
    required double altitude,
  }) async {
    final url = Uri.parse('$baseUrl/goto');
    final response = await http.post(
      url,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'latitude': latitude,
        'longitude': longitude,
        'altitude': altitude,
      }),
    );

    if (response.statusCode == 200) {
      print("Goto-Befehl erfolgreich gesendet.");
      return true;
    } else {
      print("Fehler beim Senden des Goto-Kommandos: ${response.body}");
      return false;
    }
  }

  Future<bool> rotate({
    required double yaw,
    required double speed,
    required int direction,
    required bool relative,
  }) async {
    final url = Uri.parse('$baseUrl/rotate');
    final response = await http.post(
      url,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({
        'yaw': yaw,
        'speed': speed,
        'direction': direction,
        'relative': relative,
      }),
    );

    if (response.statusCode == 200) {
      print("Rotationsbefehl erfolgreich gesendet.");
      return true;
    } else {
      print("Fehler beim Senden des Rotationsbefehls: ${response.body}");
      return false;
    }
  }

  Future<bool> takeoff(double altitude) async {
    final url = Uri.parse('$baseUrl/takeoff');
    final response = await http.post(
      url,
      headers: {'Content-Type': 'application/json'},
      body: jsonEncode({'altitude': altitude}),
    );

    if (response.statusCode == 200) {
      print("Takeoff-Befehl erfolgreich gesendet.");
      return true;
    } else {
      print("Fehler beim Takeoff-Befehl: ${response.body}");
      return false;
    }
  }

  Future<bool> land() async {
    final url = Uri.parse('$baseUrl/land');
    final response = await http.post(
      url,
      headers: {'Content-Type': 'application/json'},
    );

    if (response.statusCode == 200) {
      print("Landeanfrage erfolgreich gesendet.");
      return true;
    } else {
      print("Fehler beim Landeanfrage: ${response.body}");
      return false;
    }
  }

  Future<DroneStatus?> getStatus() async {
    final url = Uri.parse('$baseUrl/get_status');
    final response = await http.get(url);

    if (response.statusCode == 200) {
      final json = jsonDecode(response.body);
      return DroneStatus.fromJson(json);
    } else {
      print("Fehler beim Abrufen des Status: ${response.body}");
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
      print('Fehler beim Abrufen des Kamerabilds: ${response.statusCode}');
      return null;
    }
  }

  Future<void> takeoffAndFly(List<Waypoint> waypoints,
      {double takeoffAltitude = 10.0}) async {
    // Prüfe, ob bereits ein Takeoff-Waypoint enthalten ist
    final hasTakeoff = waypoints.any((wp) => wp.command == 22);

    final fullMission = hasTakeoff
        ? waypoints
        : [Waypoint.takeoff(altitude: takeoffAltitude), ...waypoints];

    await setMissions(fullMission);
    await setMode("AUTO");
    await arm();
    await startMission();
  }
}
