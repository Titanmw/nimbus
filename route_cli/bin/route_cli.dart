import 'package:route_cli/place.dart';
import 'package:route_cli/route_service.dart';

Future<void> main(List<String> arguments) async {
  final routeService =
      RouteService('5b3ce3597851110001cf6248cb75cce6ad4c4a44a5c409c3faaecffe');

  final places = await routeService.searchPlaces("HTL-Donaustadt");

  final myPlace = Place(
    name: "Me",
    label: "Me",
    lat: 48.21919,
    lon: 16.4484621,
  );

  if (places.isEmpty) {
    print("Kein Treffer");
    return;
  }

  // Liste nach Distanz sortieren
  places.sort((a, b) {
    final distA = Place.haversine(myPlace.lat, myPlace.lon, a.lat, a.lon);
    final distB = Place.haversine(myPlace.lat, myPlace.lon, b.lat, b.lon);
    return distA.compareTo(distB);
  });

  // Ausgabe mit Distanz
  for (final place in places) {
    final dist = Place.haversine(
      myPlace.lat,
      myPlace.lon,
      place.lat,
      place.lon,
    );
    final distKm = (dist / 1000).toStringAsFixed(2);
    print("$place");
    print("Entfernung: $distKm km\n");
  }

  final waypoints = await routeService.getWalkingRoute(myPlace, places[1]);
  print(waypoints);
}
