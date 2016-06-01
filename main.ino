#include <Arduino.h>

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

void setup() {

// Set motor pins to output
pinMode(leftEnable, OUTPUT);
pinMode(leftForward, OUTPUT);
pinMode(leftBackward, OUTPUT);

pinMode(rightEnable, OUTPUT);
pinMode(rightForward, OUTPUT);
pinMode(rightBackward, OUTPUT);

}

void loop() {


}


// High level H-bridge commands
void enableMotors() {
  digitalWrite(leftEnable, HIGH);
  digitalWrite(rightEnable, HIGH);
}
