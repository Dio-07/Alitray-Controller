#include <Arduino.h>
#include <AccelStepper.h>
#include <FastAccelStepper.h>
#include "stepAlightAPI.hpp"

using namespace alight;

// Motor Pinout
const int stepX = 2;
const int dirX = 5;

const int stepY = 3;
const int dirY = 6;

const int stepZ = 4;
const int dirZ = 7;

const int stepA = 12; // requires jumpers
const int dirA = 13;  // requires jumpers

const int enPin = 8;

// Using TMC 2209 drivers, they take 1/8th steps need to document ughhhhhhhh
const int maxSpeed = 4000;
const int accel = 5000;
const int stepsPerRev = 1600;

int incomingByte;

int toBit(char letter){ // converts letters to bits to control with serial stream
  int bitNumber = letter;
  return bitNumber;
}

// constructor for stepper motor
AccelStepper stepperX(AccelStepper::DRIVER, stepX, dirX); // first number is steps, second number is direction
AccelStepper stepperY(AccelStepper::DRIVER, stepY, dirY);
stepperController base(&stepperX, 20);
stepperController shoulder(&stepperY, 20);

// null pointers if the motor is not connected, up to 6 motors for 6 DOF
motors allMotors(&base,
    &shoulder,
    nullptr,
    nullptr,
    nullptr,
    nullptr
);

void setup() {
  Serial.begin(9600);
  pinMode(enPin, OUTPUT); // enable all motors
  digitalWrite(enPin, LOW);

  base.initialize(maxSpeed, accel, stepsPerRev); // acceleration goes here
  shoulder.initialize(maxSpeed, accel, stepsPerRev);
}

int timer = 0;
void loop() {
  // Read the oldest byte (character) from the serial buffer
  incomingByte = Serial.read(); 

  if (incomingByte == toBit('\n') && !(allMotors.isAnyRunning()) && millis() - timer > 350){ // press enter to run
    timer = millis();
    base.moveByDeg(90);
    base.moveToAngle(90);
    base.moveToAngle(0);
    shoulder.moveToAngle(180);
    shoulder.moveToAngle(0);

  }else if (incomingByte == toBit('s')){ // stop S
    allMotors.stopAll();
  }else if (incomingByte == toBit('\033')){ // disable escape
    digitalWrite(enPin, HIGH);
  }else if (incomingByte == toBit('e')){ // Enable E
    digitalWrite(enPin, LOW);
  }

  allMotors.runAll();
}