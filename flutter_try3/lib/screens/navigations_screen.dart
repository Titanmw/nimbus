import 'dart:async';
import 'dart:math';

import 'package:flutter/material.dart';
import 'package:flutter_try3/reusable_widgets/reusable_widget.dart';
import 'package:flutter_try3/screens/home_screen.dart';
import 'package:flutter_try3/screens/showcase_screen.dart';
import 'package:flutter_try3/screens/user_settings.dart';

class NavigationsScreen extends StatefulWidget {
  const NavigationsScreen({super.key, required this.username});

  final String username;

  @override
  // ignore: library_private_types_in_public_api
  _NavigationsScreenState createState() => _NavigationsScreenState();
}

class _NavigationsScreenState extends State<NavigationsScreen> {
  bool isRouteMode = true;
  String destination = "HTL Donaustadt";
  String currentObstacle = "Kein Hindernis";
  List<String> destinations = ["HTL Donaustadt"];
  Timer? navigationTimer;
  Timer? obstacleTimer;
  int currentInstructionIndex = 0;
  bool isNavigating = false;

  final List<String> demoInstructions = [
    "500 Meter geradeaus",
    "Nach 500 Metern links abbiegen",
    "Nach 100 Metern rechts abbiegen",
    "Hindernis in 2 Metern, rechts ausweichen"
        "200 Meter geradeaus bis zum Ziel"
  ];

  final List<String> obstacles = [
    "Mülltonne -> 8 Meter entfernt rechts ausweichen",
    "Baustelle -> 12 Meter entfernt links ausweichen",
    "Fahrradfahrer -> 6 Meter entfernt links ausweichen",
    "Kein Hindernis"
  ];

  void selectDestination() async {
    final selectedDestination = await showModalBottomSheet<String>(
      context: context,
      builder: (context) {
        final destinations = [
          "HTL Donaustadt",
          "Stephansplatz",
          "Prater",
          "Schönbrunn",
          "Donauinsel",
          "Kagran",
          "Kagraner Platz"
        ]; // Beispielziele

        return ListView.builder(
          itemCount: destinations.length,
          itemBuilder: (context, index) {
            return ListTile(
              title: Text(destinations[index]),
              onTap: () {
                Navigator.pop(context, destinations[index]);
              },
            );
          },
        );
      },
    );

    if (selectedDestination != null) {
      setState(() {
        destination = selectedDestination;
        destinations = [selectedDestination];
      });
    }
  }

  void addExtraStop() async {
    final extraStop = await showModalBottomSheet<String>(
      context: context,
      builder: (context) {
        final destinationsList = [
          "HTL Donaustadt",
          "Stephansplatz",
          "Prater",
          "Schönbrunn",
          "Donauinsel",
          "Kagran",
          "Kagraner Platz"
        ];

        return ListView.builder(
          itemCount: destinationsList.length,
          itemBuilder: (context, index) {
            return ListTile(
              title: Text(destinationsList[index]),
              onTap: () {
                Navigator.pop(context, destinationsList[index]);
              },
            );
          },
        );
      },
    );

    if (extraStop != null) {
      setState(() {
        destinations.add(extraStop);
      });
    }
  }

  void startNavigation() {
    if (destinations.isEmpty) return;

    setState(() {
      isNavigating = true;
      currentInstructionIndex = 0;
      currentObstacle = "Navigation gestartet: ${destinations.first}";
    });

    navigationTimer = Timer.periodic(const Duration(seconds: 10), (timer) {
      if (currentInstructionIndex < demoInstructions.length) {
        setState(() {
          currentObstacle = demoInstructions[currentInstructionIndex];
          currentInstructionIndex++;
        });
      } else if (destinations.length > 1) {
        setState(() {
          destinations.removeAt(0);
          currentInstructionIndex = 0;
          currentObstacle = "Weiter zum nächsten Ziel: ${destinations.first}";
        });
      } else {
        timer.cancel();
        setState(() {
          isNavigating = false;
          currentObstacle = "Am Ziel angekommen: ${destinations.first}";
        });
      }
    });
  }

  void startFreeMode() {
    // Zeige sofort ein Hindernis
    final randomObstacle =
        obstacles[Random().nextInt(obstacles.length)]; // Zufälliges Hindernis
    setState(() {
      currentObstacle = randomObstacle;
    });

    // Starte den Timer für weitere Hindernisse
    obstacleTimer = Timer.periodic(const Duration(seconds: 10), (timer) {
      final randomObstacle =
          obstacles[Random().nextInt(obstacles.length)]; // Zufälliges Hindernis
      setState(() {
        currentObstacle = randomObstacle;
      });
    });
  }

  void stopFreeMode() {
    if (obstacleTimer != null) {
      obstacleTimer!.cancel(); // Sofort stoppen
      obstacleTimer = null; // Timer-Referenz aufheben
    }
    setState(() {
      currentObstacle = "Kein Hindernis"; // Anzeige sofort aktualisieren
    });
  }

  void endNavigation() {
    if (navigationTimer != null) {
      navigationTimer!.cancel(); // Navigationstimer stoppen
      navigationTimer = null; // Referenz aufheben
    }
    if (obstacleTimer != null) {
      obstacleTimer!.cancel(); // Free-Mode-Timer stoppen
      obstacleTimer = null; // Referenz aufheben
    }
    setState(() {
      isNavigating = false;
      currentObstacle = "Navigation beendet"; // Status sofort aktualisieren
    });
  }

  @override
  void dispose() {
    navigationTimer?.cancel();
    obstacleTimer?.cancel();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    bool isNotificationFilled = false;
    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        title: const Text('Navigation'),
        backgroundColor: Colors.white,
        elevation: 0,
        leading: Builder(
          builder: (BuildContext context) {
            return IconButton(
              icon: const Icon(Icons.menu, color: Colors.black),
              onPressed: () {
                Scaffold.of(context).openDrawer(); // Öffnet den Drawer
              },
            );
          },
        ),
        actions: [
          Row(
            children: [
              Transform.rotate(
                angle: 90 *
                    (3.14 / 180), // Drehe das Icon um 90 Grad (in Bogenmaß)
                child: const Icon(
                  Icons.battery_charging_full_rounded,
                  color: Color.fromARGB(255, 0, 0, 0),
                  size: 30,
                ),
              ),
            ],
          ),
          const SizedBox(width: 10),
          Padding(
            padding: const EdgeInsets.all(1.0),
            child: Theme(
              data: Theme.of(context).copyWith(
                popupMenuTheme: PopupMenuThemeData(
                  color: Colors.white, // Hintergrundfarbe des Popup-Menüs
                  elevation: 8, // Schattenhöhe des Popup-Menüs
                  shape: RoundedRectangleBorder(
                    borderRadius:
                        BorderRadius.circular(10), // Abgerundete Ecken
                  ),
                ),
              ),
              child: PopupMenuButton<int>(
                onSelected: (value) {
                  // Aktionen je nach Auswahl im Menü
                  if (value == 1) {
                    print("Neues Update ausgewählt");
                  } else if (value == 2) {
                    print("Drohnenbenachrichtigung ausgewählt");
                  }
                },
                offset:
                    const Offset(5, 50), // Popup leicht nach unten verschieben
                icon: Icon(
                  isNotificationFilled
                      // ignore: dead_code
                      ? Icons.notifications
                      : Icons.notifications_none,
                  color: Colors.black,
                  size: 30,
                ),
                itemBuilder: (context) => [
                  const PopupMenuItem(
                    value: 1,
                    child: ListTile(
                      leading: Icon(Icons.update),
                      title: Text('Neues Update verfügbar!'),
                      subtitle: Text('Version 1.2 jetzt herunterladen'),
                    ),
                  ),
                  const PopupMenuItem(
                    value: 2,
                    child: ListTile(
                      leading: Icon(Icons.battery_full),
                      title: Text('Drohne bereit für den Flug!'),
                      subtitle: Text('Bitte überprüfen Sie den Akkustand'),
                    ),
                  ),
                ],
              ),
            ),
          ),
          IconButton(
            icon: const CircleAvatar(
              backgroundColor: Colors.black,
              radius: 15,
              child: Icon(Icons.person, color: Colors.white, size: 20),
            ),
            onPressed: () {
              print("Benutzereinstellungen geöffnet");
              Navigator.push(
                context,
                MaterialPageRoute(
                    builder: (context) => const UserSettingsPage()),
              );
            },
          ),
          const SizedBox(width: 10),
        ],
      ),
      drawer: Drawer(
        child: Container(
          color: Colors.white,
          child: Column(
            children: [
              DrawerHeader(
                padding: EdgeInsets.zero,
                decoration: const BoxDecoration(
                    color: Color.fromARGB(255, 72, 93, 202)),
                child: Center(
                  child: Column(
                    children: [
                      Image.asset(
                        'assets/images/LogoBlack.png',
                        height: 100,
                      ),
                      const SizedBox(height: 8),
                      Text(
                        'Willkommen, ${widget.username}',
                        style:
                            const TextStyle(color: Colors.white, fontSize: 18),
                      ),
                    ],
                  ),
                ),
              ),
              ListTile(
                leading: const Icon(Icons.home),
                title: const Text('Home'),
                iconColor: Colors.black,
                onTap: () {
                  Navigator.pushReplacement(
                    context,
                    MaterialPageRoute(
                      builder: (context) =>
                          HomeScreen(username: widget.username),
                    ),
                  );
                },
              ),
              ListTile(
                leading: const Icon(Icons.navigation),
                title: const Text('Navigation'),
                iconColor: Colors.black,
                onTap: () {
                  Navigator.push(
                    context,
                    MaterialPageRoute(
                      builder: (context) =>
                          NavigationsScreen(username: widget.username),
                    ),
                  );
                },
              ),
              ListTile(
                leading: const Icon(Icons.show_chart),
                title: const Text('Showcase'),
                iconColor: Colors.black,
                onTap: () {
                  Navigator.push(
                    context,
                    MaterialPageRoute(
                      builder: (context) =>
                          ShowcaseScreen(username: widget.username),
                    ),
                  );
                },
              ),
              const Spacer(),
              ListTile(
                leading: const Icon(Icons.settings),
                title: const Text('Settings'),
                iconColor: Colors.black,
                onTap: () {
                  Navigator.push(
                    context,
                    MaterialPageRoute(
                      builder: (context) => const UserSettingsPage(),
                    ),
                  );
                },
              ),
            ],
          ),
        ),
      ),
      body: Padding(
        padding: const EdgeInsets.all(16.0),
        child: Column(
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            Row(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                Expanded(
                  child: GestureDetector(
                    onTap: () {
                      setState(() {
                        isRouteMode = true;
                      });
                    },
                    child: Container(
                      padding: const EdgeInsets.symmetric(vertical: 10),
                      decoration: BoxDecoration(
                        border: Border.all(
                          color: isRouteMode ? Colors.blue : Colors.grey,
                        ),
                        borderRadius: BorderRadius.circular(8),
                        color: isRouteMode ? Colors.blue[50] : Colors.white,
                      ),
                      child: const Center(
                        child: Text(
                          'Route',
                          style: TextStyle(
                              fontSize: 18, fontWeight: FontWeight.bold),
                        ),
                      ),
                    ),
                  ),
                ),
                const SizedBox(width: 10),
                Expanded(
                  child: GestureDetector(
                    onTap: () {
                      setState(() {
                        isRouteMode = false;
                      });
                    },
                    child: Container(
                      padding: const EdgeInsets.symmetric(vertical: 10),
                      decoration: BoxDecoration(
                        border: Border.all(
                          color: !isRouteMode ? Colors.blue : Colors.grey,
                        ),
                        borderRadius: BorderRadius.circular(8),
                        color: !isRouteMode ? Colors.blue[50] : Colors.white,
                      ),
                      child: const Center(
                        child: Text(
                          'Free',
                          style: TextStyle(
                              fontSize: 18, fontWeight: FontWeight.bold),
                        ),
                      ),
                    ),
                  ),
                ),
              ],
            ),
            const SizedBox(height: 20),
            if (isRouteMode) ...[
              const Text(
                'Ziel',
                style: TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                    color: Colors.black),
              ),
              GestureDetector(
                onTap: selectDestination,
                child: Container(
                  margin: const EdgeInsets.symmetric(vertical: 10),
                  padding: const EdgeInsets.all(12),
                  decoration: BoxDecoration(
                    color: Colors.grey[200],
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Row(
                    children: [
                      const Icon(Icons.place, color: Colors.black),
                      const SizedBox(width: 10),
                      Expanded(
                        child: Text(
                          destination,
                          style: const TextStyle(
                              fontSize: 16, color: Colors.black),
                        ),
                      ),
                    ],
                  ),
                ),
              ),
              const Text(
                'Estimated Time',
                style: TextStyle(
                    fontSize: 18,
                    fontWeight: FontWeight.bold,
                    color: Colors.black),
              ),
              const Row(
                children: [
                  Icon(Icons.timer, color: Colors.red),
                  SizedBox(width: 10),
                  Text(
                    '24 Minutes',
                    style: TextStyle(fontSize: 16, color: Colors.grey),
                  ),
                ],
              ),
              const SizedBox(height: 20),
            ],
            const Text(
              'Aktuelles Hindernis',
              style: TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  color: Colors.black),
            ),
            Container(
              margin: const EdgeInsets.symmetric(vertical: 10),
              padding: const EdgeInsets.all(12),
              decoration: BoxDecoration(
                color: Colors.grey[200],
                borderRadius: BorderRadius.circular(8),
              ),
              child: Text(
                currentObstacle,
                style: const TextStyle(fontSize: 16, color: Colors.black),
              ),
            ),
            const Spacer(),
            ElevatedButton(
              style: ElevatedButton.styleFrom(
                padding: const EdgeInsets.symmetric(vertical: 15),
                backgroundColor: Colors.blue,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(8),
                ),
              ),
              onPressed: addExtraStop,
              child: const Center(
                child: Text(
                  'Add extra stop',
                  style: TextStyle(fontSize: 18, color: Colors.white),
                ),
              ),
            ),
            const SizedBox(height: 10),
            ElevatedButton(
              style: ElevatedButton.styleFrom(
                padding: const EdgeInsets.symmetric(vertical: 15),
                backgroundColor: Colors.green,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(8),
                ),
              ),
              onPressed: isRouteMode
                  ? (isNavigating ? null : startNavigation)
                  : startFreeMode,
              child: Center(
                child: Text(
                  isRouteMode ? 'Start Route' : 'Start Free Mode',
                  style: const TextStyle(fontSize: 18, color: Colors.white),
                ),
              ),
            ),
            const SizedBox(height: 10),
            ElevatedButton(
              style: ElevatedButton.styleFrom(
                padding: const EdgeInsets.symmetric(vertical: 15),
                backgroundColor: Colors.red,
                shape: RoundedRectangleBorder(
                  borderRadius: BorderRadius.circular(8),
                ),
              ),
              onPressed:
                  isNavigating || obstacleTimer != null ? endNavigation : null,
              child: const Center(
                child: Text(
                  'End',
                  style: TextStyle(fontSize: 18, color: Colors.white),
                ),
              ),
            ),
          ],
        ),
      ),
      bottomNavigationBar: CustomNavigationBar(
        currentIndex: 0, // Setzt den Standard-Index auf "Home"
        onTap: (index) {
          if (index == 0) {
            Navigator.push(
              context,
              MaterialPageRoute(
                  builder: (context) => HomeScreen(username: widget.username)),
            );
          } else if (index == 1) {
            Navigator.push(
              context,
              MaterialPageRoute(builder: (context) => const UserSettingsPage()),
            );
          }
        },
      ),
    );
  }
}
