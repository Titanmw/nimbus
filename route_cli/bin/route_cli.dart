import 'dart:io';

import 'package:route_cli/drone_controller.dart';
import 'package:route_cli/place.dart';
import 'package:route_cli/route_service.dart';
import 'package:route_cli/waypoint.dart';

Future<void> main(List<String> arguments) async {
  final routeService =
      RouteService('5b3ce3597851110001cf6248cb75cce6ad4c4a44a5c409c3faaecffe');

  final drone = DroneController('http://127.0.0.1:5000');

  final places = await routeService.searchPlaces("HTL-Donaustadt");

  var status = await drone.getStatus();
  if (status != null) {
    print("Drohnenstatus:\n$status");
  }

  final myPlace = Place.fromWaypoint(status!.currentPosition);

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

  Place goal = places[1];
  double flightAltitude = 10.0;

  final fullMission = [
    status.currentPosition, // Startposition als Home
    Waypoint.takeoff(altitude: flightAltitude), // Takeoff
    ...await routeService.getWalkingRouteAsWaypoints(
      myPlace,
      goal,
      altitude: flightAltitude,
    ), // Die Route
    Waypoint.landHere(), // Landung
  ];

  print("Waypoints:");
  for (final waypoint in fullMission) {
    print(waypoint);
  }

  await drone.setMissions(fullMission);

  // await drone.setMode("GUIDED");
  // await drone.arm();
  // await drone.startMission();

  status = await drone.getStatus();
  if (status != null) {
    print("Drohnenstatus:\n$status");
  }

  // final imageBytes = await drone.getCameraImage();
  // if (imageBytes != null) {
  //   final file = File('snapshot.jpg');
  //   await file.writeAsBytes(imageBytes);
  //   print('📸 Bild gespeichert als snapshot.jpg');
  // }
}
