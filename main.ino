#include <Arduino.h>
#include <IRremote.h>
#include "IRRemoteDefine.h"

/**
 * Created by Yvan RAJAONARIVONY
 * 01/06/2016
 * ------------------------------
 *
 * Robot Arduino 4WD with features :
 * - Motor left
 * - Motor right
 *
 * Change logs
 * -----------
 *
 */

#define DELTA_TIME 200


// Left motor pins init (enLeft = enable motor, pinB1 = forward, pinB2 = backward)
int leftEnable = 3;
int leftForward = 4;
int leftBackward = 2;

// Right motor pins init (enRight = enable motor, pinA1 = forward, pinA2 = backward)
int rightEnable = 6;
int rightForward = 7;
int rightBackward = 5;

// IR remote init
int pin_recept = 9; // On définit le pin 11
IRrecv ir_recept(pin_recept);
decode_results ir_decode; // stockage données reçues

boolean isRunning;

void setup() {

// Set motor pins to output
pinMode(leftEnable, OUTPUT);
pinMode(leftForward, OUTPUT);
pinMode(leftBackward, OUTPUT);

pinMode(rightEnable, OUTPUT);
pinMode(rightForward, OUTPUT);
pinMode(rightBackward, OUTPUT);

disableMotors();

isRunning = false;

// Init IR remote pins
Serial.begin(9600);
ir_recept.enableIRIn(); // Initialisation de la réception
}

void loop() {

  delay(DELTA_TIME);

  if (ir_recept.decode(&ir_decode))
  {
      int key = ir_decode.value;
      Serial.println(ir_decode.value, HEX); // On affiche le code en hexadecimal
      ir_recept.resume();
      doCommand(key);
  }
}

// interpret IR remote command
void doCommand(int key) {

if (isRunning) {
  brake();
  Serial.println("Coast and return");
  return;
}

  switch (key) {
    case KEY_OK:
      brake();
      Serial.println("Break");
      break;

    case KEY_1:
      enableMotors();
      Serial.println("Motors enabled");
      break;

    case KEY_0:
      disableMotors();
      Serial.println("Motors disabled");
      break;

    case KEY_TOP:
      forward();
      Serial.println("Forward");
      break;

    case KEY_BOTTOM:
      backward();
      Serial.println("Backward");
      break;

    case KEY_LEFT:
      turnLeft();
      Serial.println("Turn left");
      break;

    case KEY_RIGHT:
      turnRight();
      Serial.println("Turn right");
      break;
  }
}

// High level H-bridge commands
void enableMotors() {
  digitalWrite(leftEnable, HIGH);
  digitalWrite(rightEnable, HIGH);
}

void disableMotors() {
  digitalWrite(leftEnable, LOW);
  digitalWrite(rightEnable, LOW);
}

void forward() {
  motorLeftForward();
  motorRightForward();
  isRunning = true;
}

void coast() {
  motorLeftCoast();
  motorRightCoast();
  isRunning = false;
}

void brake() {
  motorLeftBrake();
  motorRightBrake();
  isRunning = false;
}

void backward() {
  motorLeftBackward();
  motorRightBackward();
  isRunning = true;
}

void turnLeft() {
  motorLeftBackward();
  motorRightForward();
  isRunning = true;
}

void turnRight() {
  motorRightBackward();
  motorLeftForward();
  isRunning = true;
}

void motorLeftForward() {
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
}

void motorLeftBackward() {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
}

void motorLeftCoast() {
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
}

void motorLeftBrake() {
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, HIGH);
}

void motorRightForward() {
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
}

void motorRightBackward() {
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
}

void motorRightCoast() {
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
}

void motorRightBrake() {
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, HIGH);
}
