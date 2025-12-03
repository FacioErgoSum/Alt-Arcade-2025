// Arrow Target Tracker with Hit Detection
// Modified from Tom Igoe example and DFRobot code
import processing.serial.*;

int lf = 10;    // Linefeed in ASCII
String myString = null;
Serial myPort;  // The serial port

// Target circle parameters
float centerX, centerY;
float targetRadius = 100; // Adjustable target size

// Hit detection state
int hitState = 0;  // 0=none, 1=soft, 2=medium, 3=hard
int lastHitState = 0;
boolean targetHit = false;
String hitMessage = "";
int messageTimer = 0;
int MESSAGE_DURATION = 60; // frames to show message (at 60fps = 1 second)

// IR blob positions
int[] blobX = new int[4];
int[] blobY = new int[4];
boolean[] blobValid = new boolean[4];

void setup() {
  size(1023, 1023);
  centerX = width / 2;
  centerY = height / 2;
  
  // List all the available serial ports
  println("Available serial ports:");
  printArray(Serial.list());
  
  // Open the port you are using at the rate you want:
  myPort = new Serial(this, Serial.list()[0], 19200);
  myPort.clear();
  
  // Throw out the first reading, in case we started reading
  // in the middle of a string from the sender.
  myString = myPort.readStringUntil(lf);
  myString = null;
  
  frameRate(60);
}

void draw() {
  background(77);
  
  // Read serial data
  myString = myPort.readStringUntil(lf);
  if (myString != null) {
    myString = trim(myString);
    
    // Skip debug/info lines that don't contain coordinate data
    if (myString.contains("***") || myString.contains("HIT DETECTED") || 
        myString.contains("Peak") || myString.contains("Scanning") ||
        myString.contains("I2C") || myString.contains("WHO_AM_I") ||
        myString.contains("LSM6DSOX") || myString.contains("===") ||
        myString.contains("Thresholds") || myString.contains("Format") ||
        myString.length() == 0) {
      return;
    }
    
    // Parse the data: X1,Y1,X2,Y2,X3,Y3,X4,Y4,HitState
    String[] parts = split(myString, ',');
    
    if (parts.length >= 9) {
      // Parse blob positions and scale from IR sensor range (0-1023) to screen size
      for (int i = 0; i < 4; i++) {
        int rawX = int(trim(parts[i * 2]));
        int rawY = int(trim(parts[i * 2 + 1]));
        
        // Scale from 0-1023 to screen dimensions
        blobX[i] = int(map(rawX, 0, 1023, 0, width));
        blobY[i] = int(map(rawY, 0, 1023, height, 0));
        
        // Check if blob is valid (IR sensors return 1023 when no detection)
        blobValid[i] = (rawX < 1020 && rawY < 1020 && rawX > 3 && rawY > 3);
        
        // Debug output
        if (blobValid[i]) {
          println("Blob " + i + ": Raw(" + rawX + "," + rawY + ") -> Screen(" + blobX[i] + "," + blobY[i] + ")");
        }
      }
      
      // Get hit state
      hitState = int(trim(parts[8]));
      
      // Detect new hit
      if (hitState > 0 && lastHitState == 0) {
        // New hit detected!
        boolean anyBlobInTarget = checkAnyBlobInTarget();
        
        if (anyBlobInTarget) {
          hitMessage = "HIT!";
          targetHit = true;
        } else {
          hitMessage = "MISS!";
          targetHit = false;
        }
        messageTimer = MESSAGE_DURATION;
      }
      
      lastHitState = hitState;
    }
  }
  
  // Check if any blob is currently in target
  boolean blobInTarget = checkAnyBlobInTarget();
  
  // Draw target circle
  drawTarget(blobInTarget);
  
  // Draw IR blobs
  drawBlobs();
  
  // Draw hit/miss message
  if (messageTimer > 0) {
    drawMessage();
    messageTimer--;
  }
  
  // Draw instructions
  drawInstructions();
}

boolean checkAnyBlobInTarget() {
  for (int i = 0; i < 4; i++) {
    if (blobValid[i]) {
      float distance = dist(blobX[i], blobY[i], centerX, centerY);
      if (distance <= targetRadius) {
        return true;
      }
    }
  }
  return false;
}

void drawTarget(boolean active) {
  noFill();
  strokeWeight(8);
  
  if (active) {
    stroke(0, 255, 0); // Green when blob is inside
  } else {
    stroke(255, 0, 0); // Red when no blob inside
  }
  
  ellipse(centerX, centerY, targetRadius * 2, targetRadius * 2);
  
  // Draw crosshair
  strokeWeight(2);
  stroke(150);
  line(centerX - 20, centerY, centerX + 20, centerY);
  line(centerX, centerY - 20, centerX, centerY + 20);
}

void drawBlobs() {
  ellipseMode(RADIUS);
  strokeWeight(3);
  
  // Different colors for each blob
  color[] blobColors = {
    color(255, 0, 0),    // Red
    color(0, 255, 0),    // Green
    color(0, 0, 255),    // Blue
    color(255, 255, 0)   // Yellow
  };
  
  for (int i = 0; i < 4; i++) {
    if (blobValid[i]) {
      stroke(255);  // White outline for visibility
      fill(blobColors[i]);
      ellipse(blobX[i], blobY[i], 10, 10);  // Larger for better visibility
      
      // Draw crosshair on blob
      stroke(255);
      strokeWeight(2);
      line(blobX[i] - 10, blobY[i], blobX[i] + 10, blobY[i]);
      line(blobX[i], blobY[i] - 10, blobX[i], blobY[i] + 10);
      
      // Draw line to center if in target
      float distance = dist(blobX[i], blobY[i], centerX, centerY);
      if (distance <= targetRadius) {
        stroke(blobColors[i], 150);
        strokeWeight(2);
        line(blobX[i], blobY[i], centerX, centerY);
      }
    }
  }
}

void drawMessage() {
  textAlign(CENTER, CENTER);
  textSize(100);
  
  // Fade out effect
  float alpha = map(messageTimer, 0, MESSAGE_DURATION, 0, 255);
  
  if (targetHit) {
    fill(0, 255, 0, alpha); // Green for hit
    text(hitMessage, centerX, centerY - 250);
  } else {
    fill(255, 0, 0, alpha); // Red for miss
    text(hitMessage, centerX, centerY - 250);
  }
  
  // Show hit intensity
  String intensity = "";
  if (hitState == 1) intensity = "(Soft)";
  else if (hitState == 2) intensity = "(Medium)";
  else if (hitState == 3) intensity = "(Hard)";
  
  textSize(40);
  fill(255, alpha);
  text(intensity, centerX, centerY - 180);
}

void drawInstructions() {
  textAlign(LEFT, TOP);
  textSize(14);
  fill(255);
  
  text("Arrow Target Practice", 10, 10);
  text("Target Radius: " + int(targetRadius) + "px", 10, 30);
  text("Hit State: " + hitState, 10, 50);
  
  // Show blob count and positions
  int validBlobs = 0;
  for (int i = 0; i < 4; i++) {
    if (blobValid[i]) {
      validBlobs++;
      text("Blob " + i + ": (" + blobX[i] + ", " + blobY[i] + ")", 10, 70 + (i * 20));
    }
  }
  text("IR Blobs Detected: " + validBlobs, 10, 150);
  
  // Instructions at bottom
  textAlign(LEFT, BOTTOM);
  text("Press UP/DOWN to adjust target size", 10, height - 40);
  text("Green target = Flame aligned | Red target = Flame not aligned", 10, height - 20);
  
  // Show last serial message for debugging
  if (myString != null) {
    textAlign(RIGHT, BOTTOM);
    textSize(10);
    fill(150);
    text("Last: " + myString, width - 10, height - 10);
  }
}

void keyPressed() {
  if (keyCode == UP) {
    targetRadius += 10;
    targetRadius = min(targetRadius, 400);
  } else if (keyCode == DOWN) {
    targetRadius -= 10;
    targetRadius = max(targetRadius, 50);
  }
}
