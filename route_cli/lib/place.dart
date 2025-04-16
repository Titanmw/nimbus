import 'dart:math';

class Place {
  static double haversine(double lat1, double lon1, double lat2, double lon2) {
    const R = 6371000; // Erdradius in Meter
    final dLat = _deg2rad(lat2 - lat1);
    final dLon = _deg2rad(lon2 - lon1);

    final a = sin(dLat / 2) * sin(dLat / 2) +
        cos(_deg2rad(lat1)) *
            cos(_deg2rad(lat2)) *
            sin(dLon / 2) *
            sin(dLon / 2);
    final c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
  }

  static double _deg2rad(double deg) => deg * pi / 180.0;

  final String name;
  final String label;
  final double lat;
  final double lon;

  Place({
    required this.name,
    required this.label,
    required this.lat,
    required this.lon,
  });

  factory Place.fromJson(Map<String, dynamic> json) {
    final coords = json['geometry']['coordinates'];
    final props = json['properties'];

    return Place(
      name: props['name'] ?? '',
      label: props['label'] ?? '',
      lat: coords[1],
      lon: coords[0],
    );
  }

  @override
  String toString() {
    return '$label\n→ Lat: $lat, Lon: $lon';
  }
}
