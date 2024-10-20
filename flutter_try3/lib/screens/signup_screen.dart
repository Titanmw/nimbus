import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';
import 'package:flutter_try3/reusable_widgets/reusable_widget.dart';
import 'package:flutter_try3/screens/home_screen.dart';
import 'package:flutter_try3/screens/signin_screen.dart';

class SignUpScreen extends StatefulWidget {
  const SignUpScreen({super.key});

  @override
  State<SignUpScreen> createState() => _SignupScreenState();
}

class _SignupScreenState extends State<SignUpScreen> {
  final TextEditingController _passwordTextController = TextEditingController();
  final TextEditingController _emailTextController = TextEditingController();
  final TextEditingController _usernameTextController = TextEditingController();

  final FirebaseAuth _auth = FirebaseAuth.instance;
  final FirebaseFirestore _firestore = FirebaseFirestore.instance;

  void signUpUser(String email, String password, String username) async {
    try {
      // Erstellen des Benutzers mit E-Mail und Passwort
      UserCredential userCredential =
          await _auth.createUserWithEmailAndPassword(
        email: email,
        password: password,
      );

      // Nutzer-ID erhalten
      String uid = userCredential.user!.uid;

      // Benutzername und andere Daten in Firestore speichern
      await _firestore.collection('users').doc(uid).set({
        'username': username,
        'email': email,
        // Weitere Felder können hier hinzugefügt werden
      });

      print("Benutzer erfolgreich registriert und in Firestore gespeichert.");
    } catch (e) {
      print("Fehler beim Sign-Up: $e");
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
        extendBodyBehindAppBar: true,
        appBar: AppBar(
          backgroundColor: Colors.transparent,
          elevation: 0,
          title: const Text(
            "Sign Up",
            style: TextStyle(
                fontSize: 24, fontWeight: FontWeight.bold, color: Colors.black),
          ),
        ),
        body: Container(
          width: MediaQuery.of(context).size.width,
          height: MediaQuery.of(context).size.height,
          decoration: const BoxDecoration(
              gradient: LinearGradient(
                  colors: [Color.fromARGB(255, 72, 93, 202), Colors.white],
                  begin: Alignment.topCenter,
                  end: Alignment.bottomCenter)),
          child: SingleChildScrollView(
            child: Padding(
              padding: EdgeInsets.fromLTRB(
                  20, MediaQuery.of(context).size.height * 0.15, 20, 0),
              child: Column(
                children: <Widget>[
                  const SizedBox(
                    height: 20,
                  ),
                  reusableTextField("Enter Username", Icons.person_outline,
                      false, _usernameTextController),
                  const SizedBox(
                    height: 20,
                  ),
                  reusableTextField("Enter E-Mail", Icons.email_outlined, false,
                      _emailTextController),
                  const SizedBox(
                    height: 20,
                  ),
                  reusableTextField("Enter Password", Icons.lock_outline, false,
                      _passwordTextController),
                  const SizedBox(
                    height: 20,
                  ),
                  firebaseButton(context, "Sign Up", () async {
                    if (_usernameTextController.text.isEmpty ||
                        _emailTextController.text.isEmpty ||
                        _passwordTextController.text.isEmpty) {
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(
                            content: Text("Please fill in all fields")),
                      );
                    } else if (!_emailTextController.text.contains('@')) {
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(
                            content: Text("Please enter a valid email")),
                      );
                    } else if (_passwordTextController.text.length < 6) {
                      ScaffoldMessenger.of(context).showSnackBar(
                        const SnackBar(
                            content: Text(
                                "Password should be at least 6 characters")),
                      );
                    } else {
                      try {
                        // Aufruf der Funktion signUpUser und Warten auf deren Abschluss
                        signUpUser(
                          _emailTextController.text.trim(),
                          _passwordTextController.text.trim(),
                          _usernameTextController.text.trim(),
                        );

                        print("Created new Account");
                        if (mounted) {
                          // Überprüfen, ob der Kontext noch vorhanden ist
                          Navigator.pushReplacement(
                            context,
                            MaterialPageRoute(
                                builder: (context) => HomeScreen(
                                    username: _usernameTextController.text)),
                          );
                        }
                      } catch (error) {
                        ScaffoldMessenger.of(context).showSnackBar(
                          SnackBar(content: Text("Error: ${error.toString()}")),
                        );
                      }
                    }
                  }),
                  signInOption()
                ],
              ),
            ),
          ),
        ));
  }

  Row signInOption() {
    return Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        const Text("Already have account?",
            style: TextStyle(color: Colors.black)),
        GestureDetector(
          onTap: () {
            Navigator.push(context,
                MaterialPageRoute(builder: (context) => const SignInScreen()));
          },
          child: const Text(
            " Sign In ",
            style: TextStyle(
              color: Colors.blue,
              fontWeight: FontWeight.bold,
            ),
          ),
        ),
      ],
    );
  }
}
