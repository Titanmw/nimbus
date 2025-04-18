import 'package:route_cli/waypoint.dart';

class DroneStatus {
  final double voltage;
  final double current;
  final int gpsFix;
  final int satellites;
  final String mode;
  final int baseMode;
  final int customMode;
  final int? systemStatus;
  late Waypoint currentPosition;

  DroneStatus(
      {required this.voltage,
      required this.current,
      required latitude,
      required longitude,
      required altitude,
      required this.gpsFix,
      required this.satellites,
      required this.mode,
      required this.baseMode,
      required this.customMode,
      required this.systemStatus}) {
    currentPosition = Waypoint(
      latitude: latitude,
      longitude: longitude,
      altitude: altitude,
      command: 179,
    );
  }

  factory DroneStatus.fromJson(Map<String, dynamic> json) {
    int? parseOptionalInt(dynamic value) {
      if (value is int) return value;
      if (value is String) return int.tryParse(value);
      return null;
    }

    return DroneStatus(
      voltage: (json["voltage"] ?? 0.0).toDouble(),
      current: (json["current"] ?? 0.0).toDouble(),
      latitude: (json["latitude"] ?? 0.0).toDouble(),
      longitude: (json["longitude"] ?? 0.0).toDouble(),
      altitude: (json["altitude"] ?? 0.0).toDouble(),
      gpsFix: (json["gps_fix"] ?? 0).toInt(),
      satellites: (json["satellites"] ?? 0).toInt(),
      mode: json["mode"] ?? "UNKNOWN",
      baseMode: (json["base_mode"] ?? 0).toInt(),
      customMode: (json["custom_mode"] ?? 0).toInt(),
      systemStatus: parseOptionalInt(json["system_status"]),
    );
  }

  @override
  String toString() {
    final systemStatusStr =
        systemStatus != null ? systemStatus.toString() : "UNKNOWN";
    return '''
      Mode: $mode | Lat: ${currentPosition.latitude}, Lon: ${currentPosition.longitude}, Alt: ${currentPosition.altitude.toStringAsFixed(1)} m
      GPS Fix: $gpsFix (Satellites: $satellites)
      Voltage: ${voltage.toStringAsFixed(1)} V, Current: ${current.toStringAsFixed(1)} A
      System Status: $systemStatusStr, BaseMode: $baseMode
    ''';
  }
}
