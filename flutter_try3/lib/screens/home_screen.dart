import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';
import 'package:flutter_try3/reusable_widgets/reusable_widget.dart';
import 'package:flutter_try3/screens/navigations_screen.dart';
import 'package:flutter_try3/screens/showcase_screen.dart';
import 'package:flutter_try3/screens/user_settings.dart';

class HomeScreen extends StatefulWidget {
  final String username; // Speichern des Benutzernamens

  const HomeScreen({super.key, required this.username});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  String? _username;
  bool isNotificationFilled = false; // Für den Wechsel zwischen Umrandung und Füllung

  @override
  void initState() {
    super.initState();
    _username = widget.username; // Den übergebenen Benutzernamen verwenden
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        backgroundColor: Colors.white,
        elevation: 0,
        leading: Builder(
          builder: (BuildContext context) {
            return Semantics(
              label: 'Navigationsmenü öffnen',
              child: IconButton(
                icon: const Icon(Icons.menu, color: Colors.black),
                onPressed: () {
                  Scaffold.of(context).openDrawer(); // Öffnet den Drawer
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
                  color: Colors.white, // Hintergrundfarbe des Popup-Menüs
                  elevation: 8, // Schattenhöhe des Popup-Menüs
                  shape: RoundedRectangleBorder(
                    borderRadius: BorderRadius.circular(10), // Abgerundete Ecken
                  ),
                ),
              ),
              child: PopupMenuButton<int>(
                tooltip: 'Benachrichtigungen anzeigen',
                onSelected: (value) {
                  // Aktionen je nach Auswahl im Menü
                  if (value == 1) {
                    print("Neues Update ausgewählt");
                  } else if (value == 2) {
                    print("Drohnenbenachrichtigung ausgewählt");
                  }
                },
                offset: const Offset(5, 50),
                icon: Icon(
                  isNotificationFilled
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
                          'Willkommen, $_username',
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
                        builder: (context) =>
                            HomeScreen(username: _username ?? 'Guest'),
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
                        builder: (context) => NavigationsScreen(username: _username ?? 'Guest'),
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
                            ShowcaseScreen(username: _username ?? 'Guest'),
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
      body: Semantics(
        label: 'Hauptinhalt',
        child: Container(
          padding: const EdgeInsets.fromLTRB(20, 20, 0, 0),
          child: Column(
            mainAxisAlignment: MainAxisAlignment.start,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              const SizedBox(height: 5),
              Text(
                'Hi ${_username ?? 'Guest'},',
                style: const TextStyle(
                  fontSize: 24,
                  fontWeight: FontWeight.bold,
                  color: Colors.black,
                ),
              ),
              Text(
                'Your drone is ready to fly!',
                style: TextStyle(fontSize: 18, color: Colors.grey[600]),
              ),
              const SizedBox(height: 20),
              Stack(
                children: [
                  Row(
                      mainAxisAlignment: MainAxisAlignment.end,
                      crossAxisAlignment: CrossAxisAlignment.end,
                      children: [
                        Image.asset(
                          'assets/images/Circles.png',
                          height: 300,
                        )
                      ]),
                  Row(
                      mainAxisAlignment: MainAxisAlignment.end,
                      crossAxisAlignment: CrossAxisAlignment.end,
                      children: [
                        Image.asset(
                          'assets/images/drone.png',
                          height: 340,
                        )
                      ]),
                ],
              ),
              const Spacer(),
              Semantics(
                label: 'Navigationsseite öffnen',
                child: Container(
                    width: MediaQuery.of(context).size.width,
                    height: 61,
                    margin: const EdgeInsets.fromLTRB(0, 10, 0, 20),
                    padding: const EdgeInsets.fromLTRB(0, 0, 20, 0),
                    decoration:
                        BoxDecoration(borderRadius: BorderRadius.circular(90)),
                    child: ElevatedButton(
                      onPressed: () {
                        Navigator.push(
                            context,
                            MaterialPageRoute(
                                builder: (context) => NavigationsScreen(username: _username ?? 'Guest')));
                      },
                      style: ButtonStyle(
                        elevation: MaterialStateProperty.all(9),
                        backgroundColor: MaterialStateProperty.resolveWith((states) {
                          if (states.contains(MaterialState.pressed)) {
                            return Colors.black45;
                          }
                          return const Color.fromARGB(255, 255, 255, 255);
                        }),
                        shape: MaterialStateProperty.all<RoundedRectangleBorder>(
                            RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(30))),
                      ),
                      child: const Text("Navigation",
                          style: TextStyle(
                              color: Colors.black87,
                              fontWeight: FontWeight.bold,
                              fontSize: 16)),
                    )),
              ),
              Semantics(
                label: 'Showcase-Seite öffnen',
                child: Container(
                    width: MediaQuery.of(context).size.width,
                    height: 61,
                    margin: const EdgeInsets.fromLTRB(0, 10, 0, 20),
                    padding: const EdgeInsets.fromLTRB(0, 0, 20, 0),
                    decoration:
                        BoxDecoration(borderRadius: BorderRadius.circular(90)),
                    child: ElevatedButton(
                      onPressed: () {
                        Navigator.push(
                            context,
                            MaterialPageRoute(
                                builder: (context) => ShowcaseScreen(
                                    username: _username ?? 'Guest')));
                      },
                      style: ButtonStyle(
                        elevation: MaterialStateProperty.all(9),
                        backgroundColor: MaterialStateProperty.resolveWith((states) {
                          if (states.contains(MaterialState.pressed)) {
                            return Colors.black45;
                          }
                          return Colors.white;
                        }),
                        shape: MaterialStateProperty.all<RoundedRectangleBorder>(
                            RoundedRectangleBorder(
                                borderRadius: BorderRadius.circular(30))),
                      ),
                      child: const Text("Showcase",
                          style: TextStyle(
                              color: Colors.black87,
                              fontWeight: FontWeight.bold,
                              fontSize: 16)),
                    )),
              ),
            ],
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
                    builder: (context) => HomeScreen(
                          username: "$_username",
                        )),
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

// NavigationButton-Komponente
class NavigationButton extends StatelessWidget {
  final String text;
  final VoidCallback onPressed;

  const NavigationButton(
      {super.key, required this.text, required this.onPressed});

  @override
  Widget build(BuildContext context) {
    return Semantics(
      label: '$text-Button',
      child: ElevatedButton(
        style: ElevatedButton.styleFrom(
          foregroundColor: Colors.white,
          backgroundColor: Colors.black,
          side: BorderSide(color: Colors.grey[300]!),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(10),
          ),
          padding: const EdgeInsets.symmetric(vertical: 20, horizontal: 40),
        ),
        onPressed: onPressed,
        child: Text(
          text,
          style: const TextStyle(fontSize: 16),
        ),
      ),
    );
  }
}

final FirebaseAuth _auth = FirebaseAuth.instance;
final FirebaseFirestore _firestore = FirebaseFirestore.instance;

Future<String?> getUserName() async {
  User? user = _auth.currentUser;
  if (user != null) {
    DocumentSnapshot userDoc =
        await _firestore.collection('users').doc(user.uid).get();
    return userDoc.get('username') as String?;
  }
  return null;
}
