//Sign In
import 'package:cloud_firestore/cloud_firestore.dart';
import 'package:firebase_auth/firebase_auth.dart';
import 'package:flutter/material.dart';
import 'package:flutter_try3/reusable_widgets/reusable_widget.dart';
import 'package:flutter_try3/screens/home_screen.dart';
import 'package:flutter_try3/screens/reset_password.dart';
import 'package:flutter_try3/screens/signup_screen.dart';

class SignInScreen extends StatefulWidget {
  const SignInScreen({super.key});

  @override
  State<SignInScreen> createState() => _SignInScreenState();
}

class _SignInScreenState extends State<SignInScreen> {
  final TextEditingController _passwordTextController = TextEditingController();
  final TextEditingController _emailTextController = TextEditingController();

  final FirebaseAuth _auth = FirebaseAuth.instance;
  final FirebaseFirestore _firestore = FirebaseFirestore.instance;

  bool isLoading = false;

  Future<String?> signInUser(String email, String password) async {
    setState(() {
      isLoading = true; // Ladezustand aktivieren
    });

    try {
      // Benutzer anmelden
      UserCredential userCredential = await _auth.signInWithEmailAndPassword(
        email: email,
        password: password,
      );

      String uid = userCredential.user!.uid;
      DocumentSnapshot userDoc =
          await _firestore.collection('users').doc(uid).get();

      if (userDoc.exists) {
        String username = userDoc.get('username') as String;
        print("Benutzer erfolgreich angemeldet. Benutzername: $username");
        return username;
      } else {
        print("Benutzerdokument existiert nicht.");
        return null;
      }
    } catch (e) {
      print("Fehler beim Sign-In: $e");
      return null;
    } finally {
      setState(() {
        isLoading = false; // Ladezustand deaktivieren
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: isLoading // Wenn isLoading true ist, zeige den ProgressIndicator
          ? const Center(child: CircularProgressIndicator())
          : Container(
              // Background
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
                        // Nimbus Logo
                        logoWidget("assets/images/LogoBlack.png"),

                        // Willkommensschreiben
                        RichText(
                          textAlign: TextAlign.center,
                          text: const TextSpan(
                            children: [
                              TextSpan(
                                text: "Welcome to\nNimbus",
                                style: TextStyle(
                                  fontSize: 30,
                                  fontWeight: FontWeight.w900,
                                  color: Colors.black,
                                  fontFamily: 'Trueno',
                                ),
                              ),
                            ],
                          ),
                        ),

                        // Eingabefelder
                        const SizedBox(
                          height:
                              30, // Added some space between the text and the next widget
                        ),
                        reusableTextField("Enter Email", Icons.email_outlined,
                            false, _emailTextController),
                        const SizedBox(
                          height: 20,
                        ),
                        reusableTextField("Enter Password", Icons.lock_outline,
                            true, _passwordTextController),
                        const SizedBox(
                          height: 5,
                        ),
                        forgetPassword(context),

                        firebaseButton(
                            context, isLoading ? "Loading..." : "Sign In",
                            () async {
                          if (_emailTextController.text.isEmpty ||
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
                            String? username = await signInUser(
                                _emailTextController.text,
                                _passwordTextController.text);

                            if (username != null) {
                              // Wenn die Anmeldung erfolgreich war, zur HomeScreen-Navigation
                              if (mounted) {
                                Navigator.pushReplacement(
                                  context,
                                  MaterialPageRoute(
                                    builder: (context) =>
                                        HomeScreen(username: username),
                                  ),
                                );
                              }
                            } else {
                              // Benutzername ist null, das bedeutet, dass die Anmeldung fehlgeschlagen ist
                              ScaffoldMessenger.of(context).showSnackBar(
                                const SnackBar(content: Text("Sign In failed")),
                              );
                            }
                          }
                        }),
                        signUpOption()
                      ],
                    )),
              ),
            ),
    );
  }

  Row signUpOption() {
    return Row(
      mainAxisAlignment: MainAxisAlignment.center,
      children: [
        const Text("Don't have account?",
            style: TextStyle(fontSize: 20, color: Colors.black)),
        GestureDetector(
          onTap: () {
            Navigator.push(
              context,
              MaterialPageRoute(
                builder: (context) =>
                    const SignUpScreen(), // Pass the username here
              ),
            );
          },
          child: const Text(
            " Sign Up ",
            style: TextStyle(
              fontSize: 20,
              color: Colors.blue,
              fontWeight: FontWeight.bold,
            ),
          ),
        ),
      ],
    );
  }

  Widget forgetPassword(BuildContext context) {
    return Container(
        width: MediaQuery.of(context).size.width,
        height: 35,
        alignment: Alignment.bottomRight,
        child: TextButton(
            child: const Text(
              "Forgot Password?",
              style: TextStyle(color: Colors.black38, fontSize: 16),
              textAlign: TextAlign.right,
            ),
            onPressed: () => Navigator.push(
                context,
                MaterialPageRoute(
                    builder: (context) => const ResetPasswordScreen()))));
  }
}
