import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';
import 'package:flutter_try3/screens/signin_screen.dart';

class UserSettingsPage extends StatelessWidget {
  const UserSettingsPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      backgroundColor: Colors.white,
      appBar: AppBar(
        backgroundColor: Colors.white,
        title: const Text(
          'User Settings',
          style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold),
        ),
        actions: [
          Semantics(
            label: 'Abmelden und zurück zur Anmeldeseite',
            child: ElevatedButton(
              style: ElevatedButton.styleFrom(
                backgroundColor: Colors.black,
                foregroundColor: Colors.white,
              ),
              child: const Padding(
                padding: EdgeInsets.symmetric(horizontal: 5),
                child: Text(
                  "Logout",
                  style: TextStyle(fontSize: 18),
                ),
              ),
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
          ),
        ],
      ),
      body: Semantics(
        label: 'Einstellungen für Benutzer',
        child: Padding(
          padding: const EdgeInsets.all(20.0),
          child: ListView(
            children: [
              Semantics(
                label: 'Kontoeinstellungen bearbeiten',
                child: ListTile(
                  leading: const Icon(Icons.person, size: 30),
                  title: const Text(
                    'Edit Profile',
                    style: TextStyle(fontSize: 20),
                  ),
                  onTap: () {
                    print("Edit Profile tapped");
                  },
                ),
              ),
              Semantics(
                label: 'Passwort zurücksetzen',
                child: ListTile(
                  leading: const Icon(Icons.lock_reset, size: 30),
                  title: const Text(
                    'Reset Password',
                    style: TextStyle(fontSize: 20),
                  ),
                  onTap: () {
                    print("Reset Password tapped");
                  },
                ),
              ),
              Semantics(
                label: 'Benachrichtigungseinstellungen ändern',
                child: ListTile(
                  leading: const Icon(Icons.notifications, size: 30),
                  title: const Text(
                    'Notification Settings',
                    style: TextStyle(fontSize: 20),
                  ),
                  onTap: () {
                    print("Notification Settings tapped");
                  },
                ),
              ),
              Semantics(
                label: 'Datenschutzeinstellungen überprüfen',
                child: ListTile(
                  leading: const Icon(Icons.privacy_tip, size: 30),
                  title: const Text(
                    'Privacy Settings',
                    style: TextStyle(fontSize: 20),
                  ),
                  onTap: () {
                    print("Privacy Settings tapped");
                  },
                ),
              ),
              Semantics(
                label: 'Hilfe und Support anzeigen',
                child: ListTile(
                  leading: const Icon(Icons.help, size: 30),
                  title: const Text(
                    'Help & Support',
                    style: TextStyle(fontSize: 20),
                  ),
                  onTap: () {
                    print("Help & Support tapped");
                  },
                ),
              ),
              Semantics(
                label: 'Über die App',
                child: ListTile(
                  leading: const Icon(Icons.info, size: 30),
                  title: const Text(
                    'About App',
                    style: TextStyle(fontSize: 20),
                  ),
                  onTap: () {
                    print("About App tapped");
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}
