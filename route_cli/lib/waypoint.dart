import 'package:route_cli/place.dart';

class Waypoint {
  final double latitude;
  final double longitude;
  final double altitude;
  final int command;
  final int frame;
  final List<double> params;

  Waypoint({
    required this.latitude,
    required this.longitude,
    required this.altitude,
    required this.command,
    this.frame = 3, // MAV_FRAME_GLOBAL_RELATIVE_ALT
    this.params = const [0.0, 0.0, 0.0, 0.0],
  });

  /// Factory für NAV_TAKEOFF (MAV_CMD_NAV_TAKEOFF = 22)
  factory Waypoint.takeoff({required double altitude}) {
    return Waypoint(
      latitude: 0.0,
      longitude: 0.0,
      altitude: altitude,
      command: 22,
    );
  }

  /// Factory für NAV_WAYPOINT (MAV_CMD_NAV_WAYPOINT = 16)
  factory Waypoint.fromPlace({required Place place, required double altitude}) {
    return Waypoint(
      latitude: place.lat,
      longitude: place.lon,
      altitude: altitude,
      command: 16, // MAV_CMD_NAV_WAYPOINT = 16
    );
  }

  /// Erzeugt ein Waypoint-Objekt aus JSON
  factory Waypoint.fromJson(Map<String, dynamic> json) {
    return Waypoint(
      latitude: json["latitude"],
      longitude: json["longitude"],
      altitude: json["altitude"],
      command: json["command"],
      frame: json["frame"] ?? 3,
      params: List<double>.from(json["params"] ?? [0.0, 0.0, 0.0, 0.0]),
    );
  }

  /// Konvertiert in Map (für JSON-Body)
  Map<String, dynamic> toJson() => {
        "latitude": latitude,
        "longitude": longitude,
        "altitude": altitude,
        "command": command,
        "frame": frame,
        "params": params,
      };

  @override
  String toString() =>
      "WP(command: $command, lat: $latitude, lon: $longitude, alt: $altitude)";
}
