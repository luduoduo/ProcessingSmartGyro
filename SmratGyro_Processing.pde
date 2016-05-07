/****************************************************************************************** //<>// //<>// //<>// //<>//
 * Test Sketch for Gyro AHRS v1.4.2
 * 9 Degree of Measurement Attitude and Heading Reference System
 * for Sparkfun "9DOF Gyro IMU" and "9DOF Sensor Stick"
 *
 * Released under GNU GPL (General Public License) v3.0
 * Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
 * Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
 * Written by Peter Bartz (peter-bartz@gmx.de)
 *
 * Infos, updates, bug reports, contributions and feedback:
 *     https://github.com/ptrbrtz/Gyro-9dof-ahrs
 ******************************************************************************************/

/*
  NOTE: There seems to be a bug with the serial library in Processing versions 1.5
 and 1.5.1: "WARNING: RXTX Version mismatch ...".
 Processing 2.0.x seems to work just fine. Later versions may too.
 Alternatively, the older version 1.2.1 also works and is still available on the web.
 */

import processing.opengl.*;
import processing.serial.*;

// IF THE SKETCH CRASHES OR HANGS ON STARTUP, MAKE SURE YOU ARE USING THE RIGHT SERIAL PORT:
// 1. Have a look at the Processing console output of this sketch.
// 2. Look for the serial port list and find the port you need (it's the same as in Arduino).
// 3. Set your port number here:
final static int SERIAL_PORT_NUM = 1;
// 4. Try again.


final static int SERIAL_PORT_BAUD_RATE = 57600;

float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;

float yawOffset = 0.0f;
float pitchOffset = 0.0f;
float rollOffset = 0.0f;

PFont font;
Serial serial;

float lock_angle;
int lock_status;

//0 means YPR, 1 means LOCK
int sensor_mode=1;


void drawArrow(float headWidthFactor, float headLengthFactor) {
  float headWidth = headWidthFactor * 200.0f;
  float headLength = headLengthFactor * 200.0f;

  pushMatrix();

  // Draw base
  translate(0, 0, -100);
  box(100, 100, 200);

  // Draw pointer
  translate(-headWidth/2, -50, -100);
  beginShape(QUAD_STRIP);
  vertex(0, 0, 0);
  vertex(0, 100, 0);
  vertex(headWidth, 0, 0);
  vertex(headWidth, 100, 0);
  vertex(headWidth/2, 0, -headLength);
  vertex(headWidth/2, 100, -headLength);
  vertex(0, 0, 0);
  vertex(0, 100, 0);
  endShape();
  beginShape(TRIANGLES);
  vertex(0, 0, 0);
  vertex(headWidth, 0, 0);
  vertex(headWidth/2, 0, -headLength);
  vertex(0, 100, 0);
  vertex(headWidth, 100, 0);
  vertex(headWidth/2, 100, -headLength);
  endShape();

  popMatrix();
}


void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
  float angle = 0;
  float angleIncrement = TWO_PI / sides;
  beginShape(QUAD_STRIP);
  for (int i = 0; i < sides + 1; ++i) {
    vertex(topRadius*cos(angle), 0, topRadius*sin(angle));
    vertex(bottomRadius*cos(angle), tall, bottomRadius*sin(angle));
    angle += angleIncrement;
  }
  endShape();

  // If it is not a cone, draw the circular top cap
  if (topRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(0, 0, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }

  // If it is not a cone, draw the circular bottom cap
  if (bottomRadius != 0) {
    angle = 0;
    beginShape(TRIANGLE_FAN);

    // Center point
    vertex(0, tall, 0);
    for (int i = 0; i < sides + 1; i++) {
      vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
      angle += angleIncrement;
    }
    endShape();
  }
}

void drawBoard_YPR() {
  pushMatrix();

  //print("yaw=");
  //println(yaw);
  //print("yawOffset=");
  //println(yawOffset);
  //print("rotate X=");
  //println(-radians(yaw - yawOffset));

  rotateY(-radians(yaw - yawOffset));
  rotateX(-radians(pitch - pitchOffset));
  rotateZ(radians(roll-rollOffset)); 

  // Board body
  fill(255, 0, 0);
  //box(300, 20, 400);
  drawCylinder(150, 150, 50, 64);

  // Forward-arrow
  pushMatrix();
  translate(0, 0, -150);
  scale(0.5f, 0.2f, 0.25f);
  fill(0, 255, 0);
  drawArrow(1.0f, 2.0f);
  popMatrix();

  popMatrix();
}

void drawBoard_LOCK() {
  pushMatrix();
  rotateY(-radians(-40));
  rotateX(-radians(90));
  rotateY(-radians(lock_angle)); 

  // Board body
  fill(0, 255, 0);
  //box(300, 20, 400);
  drawCylinder(150, 150, 50, 64);

  // Forward-arrow
  pushMatrix();
  translate(0, 0, -150);
  scale(0.5f, 0.2f, 0.25f);
  fill(255, 255, 0);
  drawArrow(1.0f, 2.0f);

  popMatrix();
  popMatrix();
}

// Skip incoming serial stream data until token is found
boolean readToken(Serial serial, String token) {
  // Wait until enough bytes are available
  if (serial.available() < token.length())
    return false;

  // Check if incoming bytes match token
  for (int i = 0; i < token.length(); i++) {
    if (serial.read() != token.charAt(i))
      return false;
  }

  return true;
}

// Global setup
void setup() {
  // Setup graphics
  //size(1024, 768, OPENGL);
  fullScreen(OPENGL);
  smooth(4);
  noStroke();
  frameRate(60);

  // Load font
  font = loadFont("Univers-66.vlw");
  textFont(font);

  // Setup serial port I/O
  println("AVAILABLE SERIAL PORTS:");
  println(Serial.list());
  String portName = Serial.list()[SERIAL_PORT_NUM];
  println();
  println("HAVE A LOOK AT THE LIST ABOVE AND SET THE RIGHT SERIAL PORT NUMBER IN THE CODE!");
  println("  -> Using port " + SERIAL_PORT_NUM + ": " + portName);
  serial = new Serial(this, portName, SERIAL_PORT_BAUD_RATE);

  if (sensor_mode==0)
    setupGyro_YPRMode();
  else
    setupGyro_LockMode();
}

void setupGyro_LockMode() {
  println("Trying to setup and synch Gyro...");

  // On Mac OSX and Linux (Windows too?) the board will do a reset when we connect, which is really bad.
  // See "Automatic (Software) Reset" on http://www.arduino.cc/en/Main/ArduinoBoardProMini
  // So we have to wait until the bootloader is finished and the Gyro firmware can receive commands.
  // To prevent this, disconnect/cut/unplug the DTR line going to the board. This also has the advantage,
  // that the angles you receive are stable right from the beginning. 
  // delay(3000);  // 3 seconds should be enough

  // Set Gyro output parameters
  serial.write("#l1");
  serial.write("#of");  // Turn on short-style output
  serial.write("#o0");  // Turn on continuous streaming output
  serial.write("#oe0"); // Disable error message output
}


void setupGyro_YPRMode() {
  println("Trying to setup and synch Gyro...");

  // On Mac OSX and Linux (Windows too?) the board will do a reset when we connect, which is really bad.
  // See "Automatic (Software) Reset" on http://www.arduino.cc/en/Main/ArduinoBoardProMini
  // So we have to wait until the bootloader is finished and the Gyro firmware can receive commands.
  // To prevent this, disconnect/cut/unplug the DTR line going to the board. This also has the advantage,
  // that the angles you receive are stable right from the beginning. 
  // delay(3000);  // 3 seconds should be enough

  // Set Gyro output parameters
  serial.write("#l0");
  serial.write("#of");  // Turn on short-style output
  serial.write("#o1");  // Turn on continuous streaming output
  serial.write("#oe0"); // Disable error message output
}


boolean synched = false;
boolean syncRequested = false;

void requestSync()
{
  // Synch with Gyro
  serial.clear();  // Clear input buffer up to here
  serial.write("#s00");  // Request synch token
}


float readFloat(Serial s) {
  // Convert from little endian (Gyro) to big endian (Java) and interpret as float
  return Float.intBitsToFloat(s.read() + (s.read() << 8) + (s.read() << 16) + (s.read() << 24));
}


// Blocks until another byte is available on serial port
char readChar()
{
  while (serial.available() < 1) {
  } // Block
  int c=serial.read();
  return (char)c;
}


float getAngle()
{
  char signMark;
  int id[]={0, 0, 0, 0}; //e.g. id = "$+0062" mean 6.2

  while (serial.available() < 5) {
    delay(10);  //must delay
  }

  signMark=(char)serial.read();
  for (int i = 0; i < 4; i++) {
    id[i]=serial.read()-'0';
  }

  //println((char)id[0], id[1], id[2], id[3], id[4]);

  float angle = (signMark == '+' ? 1.0 : -1.0) * (id[0] * 100.0 + id[1] * 10.0 + id[2] + id[3] * 0.1);
  return angle;
}

void do_parse_command_YPR()
{
  if (serial.available() > 0)
  {
    int val=serial.read();
    if (val == '@')  //Euler angle output
    {
      yaw = getAngle();
      pitch = getAngle();
      roll = getAngle();

      print("Y=");
      print(yaw);
      print("    P=");
      print(pitch);
      print("    R=");
      println(roll);
      println("---------------");
    }
  }
  // Output angles
  pushMatrix();
  translate(10, height - 10);
  textAlign(LEFT);
  text("Yaw: " + ((int) yaw), 0, 0);
  text("Pitch: " + ((int) pitch), 150, 0);
  text("Roll: " + ((int) roll), 300, 0);
  popMatrix();
}


void do_parse_command_LOCK()
{
  if (serial.available() > 0)
  {
    int val=serial.read();
    if (val == '%')  //Lock mode output
    {
      char signMark=(char)serial.read();
      int id[]={0, 0, 0, 0}; //e.g. id = "+0062" mean 6.2
      //while (serial.available() < 4) {
      //  delay(10);  //must delay
      //}
      for (int i = 0; i < 4; i++) {
        id[i]=serial.read()-'0';
      }

      //println((char)id[0], id[1], id[2], id[3], id[4]);
      float angle = (signMark == '+' ? 1.0 : -1.0) * (id[0] * 100.0 + id[1] * 10.0 + id[2] + id[3] * 0.1);

      serial.read();  //separator
      int status=serial.read()-'0';

      lock_angle=angle;
      lock_status=status;

      print("Angle=");
      print(lock_angle);
      print("    Status=");
      print(lock_status);
      println("---------------");
    }
  }
  // Output angles
  pushMatrix();
  translate(10, height - 10);
  textAlign(LEFT);
  text("Angle: " + ((float) lock_angle), 0, 0);
  text("Status: " + ((int) lock_status), 150, 0);
  popMatrix();
}

//Main loop
void draw() {
  // Reset scene
  background(0);

  lights();

  // Sync with Gyro 
  if (!synched) {
    textAlign(CENTER);
    fill(255);
    text("Connecting to Gyro...", width/2, height/2, -200);

    if (!syncRequested)
    {
      requestSync();  // Set ouput params and request synch token
      syncRequested=true;
    }

    synched = readToken(serial, "#SYNCH00\r\n");  // Look for synch token
    if (synched)
      println("synched OK");

    return;
  }

  if (sensor_mode==0)
    do_parse_command_YPR();
  else
    do_parse_command_LOCK();

  // Draw board
  pushMatrix();
  translate(width/2, height/2, -400);
  if (sensor_mode==0)
    drawBoard_YPR();
  else
    drawBoard_LOCK();
  popMatrix();

  textFont(font, 20);
  fill(255);
  textAlign(LEFT);

  // Output info text
  text("Point FTDI connector towards screen and press 'a' to align", 10, 25);
}