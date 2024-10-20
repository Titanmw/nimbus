import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';
import 'package:flutter_try3/screens/navigations_screen.dart';
import 'package:flutter_try3/screens/showcase_screen.dart';
import 'package:flutter_try3/screens/signin_screen.dart';

class HomeScreen extends StatefulWidget {
  final String username; // Speichern des Benutzernamens

  const HomeScreen({super.key, required this.username});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> {
  String? _username;

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
        leading: IconButton(
          icon: const Icon(Icons.menu, color: Colors.black),
          onPressed: () {
            print("Menü-Button gedrückt");
            // Menü-Aktion hier hinzufügen
          },
        ),
        actions: [
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
      body: Container(
        padding: const EdgeInsets.fromLTRB(20, 20, 0, 0),
        child: Column(
          mainAxisAlignment: MainAxisAlignment.start,
          crossAxisAlignment: CrossAxisAlignment.start,
          children: [
            const SizedBox(height: 20),
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
                        height: 350,
                      )
                    ]),
              ],
            ),
            const Spacer(),
            Container(
                width: MediaQuery.of(context).size.width,
                height: 65,
                margin: const EdgeInsets.fromLTRB(0, 10, 0, 20),
                padding: const EdgeInsets.fromLTRB(0, 0, 20, 0),
                decoration:
                    BoxDecoration(borderRadius: BorderRadius.circular(90)),
                child: ElevatedButton(
                  onPressed: () {
                    Navigator.push(
                        context,
                        MaterialPageRoute(
                            builder: (context) => const NavigationsScreen()));
                  },
                  style: ButtonStyle(
                    elevation: WidgetStateProperty.all(9),
                    backgroundColor: WidgetStateProperty.resolveWith((states) {
                      if (states.contains(WidgetState.pressed)) {
                        return Colors.black45;
                      }
                      return const Color.fromARGB(255, 255, 255, 255);
                    }),
                    shape: WidgetStateProperty.all<RoundedRectangleBorder>(
                        RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(30))),
                  ),
                  child: const Text("Navigation",
                      style: TextStyle(
                          color: Colors.black87,
                          fontWeight: FontWeight.bold,
                          fontSize: 16)),
                )),
            Container(
                width: MediaQuery.of(context).size.width,
                height: 70,
                margin: const EdgeInsets.fromLTRB(0, 10, 0, 20),
                padding: const EdgeInsets.fromLTRB(0, 0, 20, 0),
                decoration:
                    BoxDecoration(borderRadius: BorderRadius.circular(90)),
                child: ElevatedButton(
                  onPressed: () {
                    Navigator.push(
                        context,
                        MaterialPageRoute(
                            builder: (context) => const ShowcaseScreen()));
                    // Showcase Aktion hier
                  },
                  style: ButtonStyle(
                    elevation: WidgetStateProperty.all(9),
                    backgroundColor: WidgetStateProperty.resolveWith((states) {
                      if (states.contains(WidgetState.pressed)) {
                        return Colors.black45;
                      }
                      return Colors.white;
                    }),
                    shape: WidgetStateProperty.all<RoundedRectangleBorder>(
                        RoundedRectangleBorder(
                            borderRadius: BorderRadius.circular(30))),
                  ),
                  child: const Text("Showcase",
                      style: TextStyle(
                          color: Colors.black87,
                          fontWeight: FontWeight.bold,
                          fontSize: 16)),
                )),
          ],
        ),
      ),
      bottomNavigationBar: BottomNavigationBar(
        currentIndex: 0,
        items: const [
          BottomNavigationBarItem(
            icon: Icon(Icons.home),
            label: 'Home',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.settings),
            label: 'Settings',
          ),
        ],
        onTap: (index) {
          // Navigation durch die BottomNavigationBar hier implementieren
          print("Navigiere zu: $index");
        },
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
    return ElevatedButton(
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
    );
  }
}

final FirebaseAuth _auth = FirebaseAuth.instance;
final FirebaseFirestore _firestore = FirebaseFirestore.instance;

Future<String?> getUserName() async {
  User? user = _auth.currentUser;
  if (user != null) {
    DocumentSnapshot userDoc = await _firestore.collection('users').doc(user.uid).get();
    print("User ID: ${user.uid}"); // Debugging
    print("User Document: ${userDoc.data()}"); // Debugging
    return userDoc.get('username') as String?;
  }
  return null;
}


// Benutzer-Einstellungen Seite
class UserSettingsPage extends StatelessWidget {
  const UserSettingsPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(
        title: const Text('User Settings'),
        actions: [
          ElevatedButton(
            style: ElevatedButton.styleFrom(
              backgroundColor: Colors.black,
              foregroundColor: Colors.white,
            ),
            child: const Text("Logout"),
            onPressed: () {
              FirebaseAuth.instance.signOut().then((value) {
                print("Signed out");
                Navigator.pushReplacement(
                  context,
                  MaterialPageRoute(builder: (context) => const SignInScreen()),
                );
              });
            },
          ),
        ],
      ),
      body: const Center(
        child: Text('User Settings Page'),
      ),
    );
  }
}
