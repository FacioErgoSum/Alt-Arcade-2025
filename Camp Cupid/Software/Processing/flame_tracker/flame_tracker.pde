// Example by Tom Igoe
// Modified for https://www.dfrobot.com by Lumi, Jan. 2014

/*
   This code should show one colored blob for each detected IR source (max four) at the relative position to the camera.
*/

import processing.serial.*;

int lf = 10;    // Linefeed in ASCII
String myString = null;
Serial myPort;  // The serial port

void setup() {
  // List all the available serial ports
  println(Serial.list());
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[0], 19200);
  myPort.clear();
  // Throw out the first reading, in case we started reading
  // in the middle of a string from the sender.
  myString = myPort.readStringUntil(lf);
  myString = null;
  size(800,800);
  //frameRate(30);
}

void draw() {
  background(77);
  myString = myPort.readStringUntil(lf);
  if (myString != null) {
    myString = trim(myString); // Remove any whitespace/newlines
    
    // DEBUG: Show exactly what we received
    println("Raw received: '" + myString + "'");
    println("String length: " + myString.length());
    
    int[] output = int(split(myString, ','));
    println("Split array length: " + output.length);
    
    // Safety check before accessing array elements
    if (output.length >= 8) {
      int xx = output[0];
      int yy = output[1];
      int ww = output[2];
      int zz = output[3];
      int xxx = output[4];
      int yyy = output[5];
      int www = output[6];
      int zzz = output[7];
      int qqq = output[8];

      ellipseMode(RADIUS);
      fill(255, 0, 0);
      ellipse(xx, yy, 20, 20);
      fill(0, 255, 0);
      ellipse(ww, zz, 20, 20);
      fill(0, 0, 255);
      ellipse(xxx, yyy, 20, 20);
      fill(255);
      ellipse(www, zzz, 20, 20);
    } else {
      println("ERROR: Expected 8 values, got " + output.length);
      if (output.length > 0) {
        println("First value: " + output[0]);
      }
    }
  }
}
