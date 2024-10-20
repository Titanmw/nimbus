import 'package:flutter/material.dart';

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
      style: TextStyle(color: Colors.white.withOpacity(0.9)),
      decoration: InputDecoration(
        prefixIcon: Icon(icon, color: Colors.white70),
        labelText: text,
        labelStyle: TextStyle(color: Colors.white.withOpacity(0.5)),
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
        style: const TextStyle(color: Colors.black87, fontWeight: FontWeight.bold, fontSize: 16
        )
      ),
  ));
}
