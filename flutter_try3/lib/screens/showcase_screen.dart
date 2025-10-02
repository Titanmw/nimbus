import 'package:flutter/material.dart';
import 'package:flutter_try3/reusable_widgets/reusable_widget.dart';
import 'package:flutter_try3/screens/home_screen.dart';
import 'package:flutter_try3/screens/navigations_screen.dart';
import 'package:flutter_try3/screens/user_settings.dart';

class ShowcaseScreen extends StatelessWidget {
  const ShowcaseScreen({super.key, required this.username});

  final String username;

  // Funktion zum Senden von Befehlen an die Drohne
  void sendDroneCommand(String command) {
    print("Befehl an Drohne gesendet: $command");
  }

  @override
  Widget build(BuildContext context) {
    bool isNotificationFilled = false;
    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        title: const Text('Showcase'),
        backgroundColor: Colors.white,
        elevation: 0,
        leading: Builder(
          builder: (BuildContext context) {
            return Semantics(
              label: 'Navigationsmenü öffnen',
              child: IconButton(
                icon: const Icon(Icons.menu, color: Colors.black),
                onPressed: () {
                  Scaffold.of(context).openDrawer();
                },
              ),
            );
          },
        ),
        actions: [
          Row(
            children: [
              Semantics(
                label: 'Batteriestatus anzeigen',
                child: Transform.rotate(
                  angle: 90 * (3.14 / 180),
                  child: const Icon(
                    Icons.battery_charging_full_rounded,
                    color: Color.fromARGB(255, 0, 0, 0),
                    size: 30,
                  ),
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
                  color: Colors.white,
                  elevation: 8,
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(10),
                  ),
                ),
              ),
              child: PopupMenuButton<int>(
                tooltip: 'Benachrichtigungen anzeigen',
                onSelected: (value) {
                  if (value == 1) {
                    print("Neues Update ausgewählt");
                  } else if (value == 2) {
                    print("Drohnenbenachrichtigung ausgewählt");
                  }
                },
                offset: const Offset(5, 50),
                icon: Icon(
                  // ignore: dead_code
                  isNotificationFilled ? Icons.notifications : Icons.notifications_none,
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
          Semantics(
            label: 'Benutzereinstellungen öffnen',
            child: IconButton(
              icon: const CircleAvatar(
                backgroundColor: Colors.black,
                radius: 15,
                child: Icon(Icons.person, color: Colors.white, size: 20),
              ),
              onPressed: () {
                Navigator.push(
                  context,
                  MaterialPageRoute(
                      builder: (context) => const UserSettingsPage()),
                );
              },
            ),
          ),
          const SizedBox(width: 10),
        ],
      ),
      drawer: Drawer(
        child: Semantics(
          label: 'Navigationsmenü',
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
                          'Willkommen, $username',
                          style: const TextStyle(color: Colors.white, fontSize: 18),
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
                        builder: (context) => HomeScreen(username: username),
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
                        builder: (context) => NavigationsScreen(username: username),
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
                        builder: (context) => ShowcaseScreen(username: username),
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
      ),
      body: Center(
        child: Semantics(
          label: 'Drohnenaktionen',
          child: Padding(
            padding: const EdgeInsets.all(20.0),
            child: Column(
              mainAxisAlignment: MainAxisAlignment.center,
              children: [
                DroneActionButton(
                  title: 'Looping',
                  imagePath: 'assets/images/looping.png',
                  onPressed: () => sendDroneCommand('looping'),
                  width: double.infinity,
                ),
                const SizedBox(height: 10),
                DroneActionButton(
                  title: 'Somersault',
                  imagePath: 'assets/images/somersault.png',
                  onPressed: () => sendDroneCommand('somersault'),
                  width: double.infinity,
                ),
                const SizedBox(height: 10),
                DroneActionButton(
                  title: 'Screw',
                  imagePath: 'assets/images/screw.png',
                  onPressed: () => sendDroneCommand('screw'),
                  width: double.infinity,
                ),
              ],
            ),
          ),
        ),
      ),
      bottomNavigationBar: Semantics(
        label: 'Untere Navigationsleiste',
        child: CustomNavigationBar(
          currentIndex: 0,
          onTap: (index) {
            if (index == 0) {
              Navigator.push(
                context,
                MaterialPageRoute(
                    builder: (context) => HomeScreen(username: username)),
              );
            } else if (index == 1) {
              Navigator.push(
                context,
                MaterialPageRoute(
                    builder: (context) => const UserSettingsPage()),
              );
            }
          },
        ),
      ),
    );
  }
}

class DroneActionButton extends StatelessWidget {
  final String title;
  final String imagePath;
  final VoidCallback onPressed;
  final double width;

  const DroneActionButton({
    super.key,
    required this.title,
    required this.imagePath,
    required this.onPressed,
    this.width = double.infinity,
  });

  @override
  Widget build(BuildContext context) {
    return Semantics(
      label: '$title-Aktion ausführen',
      child: SizedBox(
        width: width,
        child: ElevatedButton(
          onPressed: onPressed,
          style: ElevatedButton.styleFrom(
            backgroundColor: Colors.white,
            padding: const EdgeInsets.symmetric(vertical: 30),
            shape: RoundedRectangleBorder(
              borderRadius: BorderRadius.circular(15),
            ),
            elevation: 5,
          ),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.center,
            crossAxisAlignment: CrossAxisAlignment.center,
            children: [
              Text(
                title,
                style: const TextStyle(
                  fontSize: 18,
                  fontWeight: FontWeight.bold,
                  color: Colors.black,
                ),
              ),
              const SizedBox(height: 10),
              Image.asset(
                imagePath,
                height: 80,
              ),
            ],
          ),
        ),
      ),
    );
  }
}
