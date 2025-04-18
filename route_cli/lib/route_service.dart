import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:polyline_codec/polyline_codec.dart';
import 'package:route_cli/place.dart';
import 'package:route_cli/waypoint.dart';

class RouteService {
  RouteService(this.apiKey);
  final String apiKey;

  Future<List<List<double>>?> getWalkingRoute(Place a, Place b) async {
    final url = Uri.parse(
        'https://api.openrouteservice.org/v2/directions/foot-walking');

    final response = await http.post(
      url,
      headers: {
        'Authorization': apiKey,
        'Content-Type': 'application/json',
      },
      body: jsonEncode({
        'coordinates': [
          [a.lon, a.lat],
          [b.lon, b.lat],
        ]
      }),
    );

    if (response.statusCode == 200) {
      final json = jsonDecode(response.body);
      final encoded = json['routes'][0]['geometry'];
      final decoded = PolylineCodec.decode(encoded);
      return decoded
          .map((point) => [point[0].toDouble(), point[1].toDouble()])
          .toList();
    } else {
      throw Exception('Fehler bei der Anfrage: ${response.statusCode}');
    }
  }

  Future<List<Place>> getWalkingRouteAsPlaces(Place a, Place b) async {
    final list = await getWalkingRoute(a, b);
    if (list == null) return [];

    return List.generate(list.length, (index) {
      final e = list[index];
      return Place(
        lat: e[0],
        lon: e[1],
        name: 'Wegpunkt ${index + 1}',
        label: 'Wegpunkt ${index + 1} → Lat: ${e[1]}, Lon: ${e[0]}',
      );
    });
  }

  Future<List<Waypoint>> getWalkingRouteAsWaypoints(Place a, Place b,
      {double altitude = 10.0}) async {
    final list = await getWalkingRoute(a, b);
    if (list == null) return [];

    return list
        .map((e) => Waypoint(
              latitude: e[0],
              longitude: e[1],
              altitude: altitude,
              command: 16, // MAV_CMD_NAV_WAYPOINT
            ))
        .toList();
  }

  Future<List<Place>> searchPlaces(String text) async {
    final url = Uri.parse(
      'https://api.openrouteservice.org/geocode/search?api_key=$apiKey&text=${Uri.encodeComponent(text)}',
    );

    final response = await http.get(url);

    if (response.statusCode == 200) {
      final json = jsonDecode(response.body);
      return json['features'].map<Place>((f) => Place.fromJson(f)).toList();
    } else {
      throw Exception('Geocoding fehlgeschlagen: ${response.statusCode}');
    }
  }
}
