#include <Arduino.h>
#include <AccelStepper.h>
#include "stepAlightAPI.hpp"

using namespace alight;

// Serial Communication pinout
#define rx 0
#define tx 1

// Motor Pinout
#define stepX 2
#define dirX 5

#define stepY 3
#define dirY 6

#define stepZ 4
#define dirZ 7

#define stepA 12 // requires jumpers
#define dirA 13  // requires jumpers

#define enPin 8

// speed, accel, and steps per rev calculated before startup
#define maxSpeed 4000
#define accel 3000
#define stepsPerRev 1600
#define gearing 20

// serial monitor keyboard read
#define enter '\n'
#define esc '\033'

int incomingByte;

int toBit(char letter){ // converts letters to bits to control with serial stream
  return letter;
}

// constructor for stepper motor
AccelStepper stepperX(AccelStepper::DRIVER, stepX, dirX);
AccelStepper stepperY(AccelStepper::DRIVER, stepY, dirY);
AccelStepper stepperZ(AccelStepper::DRIVER, stepZ, dirZ);
AccelStepper stepperA(AccelStepper::DRIVER, stepA, dirA);
stepperController base(&stepperX, gearing, stepsPerRev, 89); // base to joint
stepperController shoulder(&stepperY, gearing, stepsPerRev, 149); // joint to joint
stepperController elbow(&stepperZ, gearing, stepsPerRev, 74); // joint to joint
stepperController wrist(&stepperA, gearing, stepsPerRev, 42); // this is lenght from joint to end

// null pointers if the motor is not connected, up to 6 motors for 6 DOF
// motors should be in order from bottom to top

// The first index should be reserved for the base motor
stepperController* motorArray[6] = {&base, &shoulder, &elbow, &wrist, nullptr, nullptr};

void setup() {
  Serial.begin(9600);
  pinMode(enPin, OUTPUT); // enable all motors
  digitalWrite(enPin, LOW);

  base.initialize(maxSpeed, accel, 0); // acceleration goes here
  shoulder.initialize(maxSpeed, accel, 106);
  elbow.initialize(maxSpeed, accel, 164);
  wrist.initialize(maxSpeed, accel, 0);
}

initializeArm arm(motorArray);

int timer = 0;
void loop() {
  // Read the oldest byte (character) from the serial buffer
  incomingByte = Serial.read(); 

  if (incomingByte == toBit(enter) && !(arm.isAnyRunning()) && millis() - timer > 350){ // press enter to run, debounce of 350ms
    timer = millis();
    stepperY.move(50000);

    //arm.moveToPoint(-150, 100, 270);

  }else if (incomingByte == toBit('s')){ // stop S
    arm.stopAll();
  }else if (incomingByte == toBit(esc)){ // disable escape
    digitalWrite(enPin, HIGH);
  }else if (incomingByte == toBit('e')){ // Enable E
    digitalWrite(enPin, LOW);
  }else if (incomingByte == toBit('h')){ // Enable E
    arm.returnHome();
  }

  arm.runAll();
}