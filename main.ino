#include <Arduino.h>
#include <IRremote.h>
#include <SoftwareSerial.h>
#include "Ultrasonic.h"
#include <Servo.h>
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

#define DELTA_TIME        200
#define MODE_NONE         0
#define MODE_IR_COMMAND   1
#define MODE_ULTRASONIC   2

// Init ultrasonic and servo
#include "Ultrasonic.h"
#include <Servo.h>
Ultrasonic ultrasonic(12,13); // 12->trig, 13->echo
Servo mServo;
const int STEP = 5;
const int SIZE = 37;   // Size of distance array =(180/STEP) +1
int dist[SIZE];
int rDelay;
int mid = 90;

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
boolean useUltrasonic;
int mode;

SoftwareSerial bluetooth(10, 11);

void setup() {

// Set up bluetooth
bluetooth.begin(9600);

// Set motor pins to output
pinMode(leftEnable, OUTPUT);
pinMode(leftForward, OUTPUT);
pinMode(leftBackward, OUTPUT);

pinMode(rightEnable, OUTPUT);
pinMode(rightForward, OUTPUT);
pinMode(rightBackward, OUTPUT);

enableMotors();

isRunning = false;

// Init IR remote pins
Serial.begin(9600);
ir_recept.enableIRIn(); // Initialisation de la réception

// Init ultrasonic and servo
mServo.attach(5, 570, 2320);   // attaches the servo on pin 5 to the servo object
rDelay = 5 * STEP;            // assuming 10ms/degree speed

useUltrasonic = false;

}

void loop() {

  // Ultrasonic
  /*if (useUltrasonic) {
    rangeSweep(mid-90, mid+90, dist);
    disp(dist);
    int angle = getAngle(dist);
    //Serial.println(angle);
  }*/

if (Serial.available()) {
  enableMotors();
  int value = Serial.read();
  //bluetooth.write(value);
  Serial.println(value, HEX);
  useUltrasonic = true;

  if (value == 0x10) {
    forward();
  Serial.println("forward");
  } else if(value == 0x20) {
    backward();
    Serial.println("backward");
  } else if (value == 0x30) {
    turnLeft();
    Serial.println("turnLeft");
  } else if (value == 0x40) {
    turnRight();
    Serial.println("turnRight");
  } else if (value == 0xAA){
    disableMotors();
    Serial.println("brake");
    useUltrasonic = false;
  }
}

  /*if (ir_recept.decode(&ir_decode))
  {
    int key = ir_decode.value;
    Serial.println(ir_decode.value, HEX); // On affiche le code en hexadecimal
    ir_recept.resume();
    doCommand(key);
  }

  if (mode == MODE_ULTRASONIC) {

    rangeSweep(mid-90, mid+90, dist);
    disp(dist);
    int angle = getAngle(dist);
    Serial.println(angle);

  } else if (mode == MODE_IR_COMMAND) {

    delay(DELTA_TIME);

  }*/

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

    case KEY_2:
        mode = MODE_ULTRASONIC;
        enableMotors();
        Serial.println("Mode ULTRASONIC selected");
        break;

    case KEY_1:
      mode = MODE_IR_COMMAND;
      enableMotors();
      Serial.println("Mode IR_COMMAND selected");
      break;

    case KEY_0:
      mode = MODE_NONE;
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


/**
 * Reads distance over 180 degrees twice in left-to-right
 * sweep, then right-to-left sweep and averages the readings
 */
void rangeSweep(int st, int en, int dist[]) {
  int pos = 0;    // variable to store the servo position
  for(pos = st; pos<en; pos+=STEP) {
    mServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(rDelay);               // waits 10ms/degree for the servo to reach the position
    dist[int(pos/STEP)] = ultrasonic.Ranging(CM);
  }

  for(pos = en; pos>st; pos-=STEP) {
    mServo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(rDelay);
    dist[int(pos/STEP)] += ultrasonic.Ranging(CM);
    dist[int(pos/STEP)] /= 2;
  }
}

/**
 * Prints the range readings to the serial monitor
 */
void  disp(int dist[]) {
  for(int i = 0; i < SIZE; i++) {
    Serial.print(i*STEP);
    Serial.print(", ");
    Serial.println(dist[i]);
  }
}

/**
 * Get the angle at which the distance is maximum
 */
int getAngle(int dist[]) {
  int maxDist=0;
  int angle=mid;
  for(int i = 0; i < SIZE; i++) {
    if(maxDist<dist[i]) {
      maxDist = dist[i];
      angle = i * STEP;
    }
  }
  return angle;
}
