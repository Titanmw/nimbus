import 'dart:convert';
import 'package:http/http.dart' as http;
import 'package:polyline_codec/polyline_codec.dart';
import 'package:route_cli/place.dart';

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
