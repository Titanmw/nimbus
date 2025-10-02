import 'package:flutter/material.dart';
import 'dart:io' show Platform;

Image logoWidget(String imagesrc) {
  return Image.asset(imagesrc,
      fit: BoxFit.fitWidth, width: 230, height: 230, color: Colors.black);
}

TextField reusableTextField(String text, IconData icon, bool isPasswordType,
    TextEditingController controller) {
  return TextField(
      controller: controller,
      obscureText: isPasswordType,
      enableSuggestions: isPasswordType,
      autocorrect: isPasswordType,
      cursorColor: Colors.white,
      style: const TextStyle(color: Colors.white, fontSize: 18),
      decoration: InputDecoration(
        prefixIcon: Icon(icon, color: Colors.white70),
        labelText: text,
        labelStyle: const TextStyle(color: Colors.white),
        filled: true,
        floatingLabelBehavior: FloatingLabelBehavior.never,
        fillColor: Colors.black.withOpacity(0.7),
        border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(30.0),
            borderSide: const BorderSide(width: 0, style: BorderStyle.none)),
      ),
      keyboardType: isPasswordType
          ? TextInputType.visiblePassword
          : TextInputType.emailAddress);
}

Container firebaseButton(
    BuildContext context, String title, Function onTop) {
  return Container(
    width: MediaQuery.of(context).size.width,
    height: 50,
    margin: const EdgeInsets.fromLTRB(0, 10, 0, 20),
    decoration: BoxDecoration(borderRadius: BorderRadius.circular(90)),
    child: ElevatedButton(
        onPressed: () {
          onTop();
        },
      style: ButtonStyle(
        backgroundColor: WidgetStateProperty.resolveWith((states) {
          if(states.contains(WidgetState.pressed)) {
            return Colors.black45;
          }
          return Colors.white;
        }), 
        shape: WidgetStateProperty.all<RoundedRectangleBorder>(
          RoundedRectangleBorder(borderRadius: BorderRadius.circular(30))),
      ),
        child: Text(title,
        style: const TextStyle(color: Colors.black87, fontWeight: FontWeight.bold, fontSize: 20
        )
      ),
  ));
}


class CustomNavigationBar extends StatelessWidget {
  final int currentIndex;
  final ValueChanged<int> onTap;

  const CustomNavigationBar({
    super.key,
    required this.currentIndex,
    required this.onTap,
  });

  @override
  Widget build(BuildContext context) {
    // Spezifische Einstellungen für iOS
    final isIOS = Platform.isIOS;
    final double verticalPadding = isIOS ? 1.24 : 1.0; // Weniger Padding für iOS
    final double iconSize = isIOS ? 20 : 20; // Kleinere Icons für iOS

    return Container(
      padding: EdgeInsets.symmetric(vertical: verticalPadding),
      margin: const EdgeInsets.all(16),
      decoration: BoxDecoration(
        color: Colors.white,
        borderRadius: BorderRadius.circular(30),
        boxShadow: const [
          BoxShadow(
            color: Colors.black12,
            blurRadius: 10,
            spreadRadius: 1,
          ),
        ],
      ),
      child: BottomNavigationBar(
        currentIndex: currentIndex,
        onTap: onTap,
        backgroundColor: Colors.transparent,
        elevation: 0,
        type: BottomNavigationBarType.fixed,
        selectedItemColor: const Color.fromARGB(255, 72, 93, 202),
        unselectedItemColor: Colors.grey,
        showSelectedLabels: true,
        showUnselectedLabels: true,
        items: [
          BottomNavigationBarItem(
            icon: Icon(Icons.home_filled, size: iconSize),
            label: 'Home',
          ),
          BottomNavigationBarItem(
            icon: Icon(Icons.settings_outlined, size: iconSize),
            label: 'Settings',
          ),
        ],
      ),
    );
  }
}
