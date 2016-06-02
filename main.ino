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

int speed = 0;

// IR remote init
int pin_recept = 9; // On définit le pin 11
IRrecv ir_recept(pin_recept);
decode_results ir_decode; // stockage données reçues

boolean test;

void setup() {

// Set motor pins to output
pinMode(leftEnable, OUTPUT);
pinMode(leftForward, OUTPUT);
pinMode(leftBackward, OUTPUT);

pinMode(rightEnable, OUTPUT);
pinMode(rightForward, OUTPUT);
pinMode(rightBackward, OUTPUT);

disableMotors();

test = true;
speed = 0;

// Init IR remote pins
Serial.begin(9600);
ir_recept.enableIRIn(); // Initialisation de la réception
}

void loop() {

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
      forward(DELTA_TIME);
      Serial.println("Forward");
      coast();
      break;

    case KEY_BOTTOM:
      backward(DELTA_TIME);
      Serial.println("Backward");
      coast();
      break;

    case KEY_LEFT:
      turnLeft(DELTA_TIME);
      Serial.println("Turn left");
      coast();
      break;

    case KEY_RIGHT:
      turnRight(DELTA_TIME);
      Serial.println("Turn right");
      coast();
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

void forward(int time) {
  motorLeftForward();
  motorRightForward();
  delay(time);
}

void coast() {
  motorLeftCoast();
  motorRightCoast();
}

void brake() {
  motorLeftBrake();
  motorRightBrake();
  speed = 0;
}

void backward(int time) {
  motorLeftBackward();
  motorRightBackward();
  delay(time);
}

void turnLeft(int time) {
  motorLeftBackward();
  motorRightForward();
  delay(time);
}

void turnRight(int time) {
  motorRightBackward();
  motorLeftForward();
  delay(time);
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
