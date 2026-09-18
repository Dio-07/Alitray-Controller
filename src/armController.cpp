#include <BasicLinearAlgebra.h>
#include "armController.hpp"
#include "stepperController.hpp"

using namespace alight;

initializeArm::initializeArm(stepperController* m_stepMotors[6]){
// This sets the end-effector position
for(int i = 0; i < 6; i++){
        motorCount ++;
        stepMotors[i] = m_stepMotors[i];
    }
}

bool initializeArm::isAnyRunning(){
    for (int i = 0; i < 6; i++){
        if (stepMotors[i] && stepMotors[i]->isRunning()) return true;
        if (stepMotors[i] && !(stepMotors[i]->isRunning())) longestTime = 0;
    }
    return false;
}

void initializeArm::stopAll(){
    for (int i = 0; i < motorCount; i++){
        if (stepMotors[i]){
            stepMotors[i]->stop();
        }else{
            break;
        }
    }
}

bool initializeArm::isAnyPaused(){
    for (int i = 1; i < 6; i++){ // for testing purposes this begins at 1, ignoring the base For real practice change this to 0
        if (stepMotors[i] && stepMotors[i]->isPaused()) return true;
    }
    return false;
}

float initializeArm::timeToTarget(stepperController* motor){
    // The calculations will be completed based on angular velocity
    // look for the highest time based on the movement of the whole arm
    // set all the motors speed so that they all take that long to finish their movement
    float radToTar = (abs(motor->getError()) * motor->degPerStep) * (PI / 180);
    float timeToMax = motor->speedSet / motor->accelSet;
    float timeToEnd = sqrt((2 * radToTar)/(motor->accelSet * motor->degPerStep * (PI/180.0)));
    float radMovedInAccel =  ((motor->accelSet * motor->degPerStep * (PI/180.0)) * pow(timeToMax, 2)) / 2.0;
    float speedRad = motor->speedSet * motor->degPerStep * (PI/180.0);

    if (timeToEnd >= timeToMax){
        timeToEnd = timeToMax + (radToTar - radMovedInAccel) / speedRad;
    }
    
    //Serial.println(abs(motor->getError()));
    return timeToEnd;
}

void initializeArm::speedCalculation(float time, stepperController* motor){
    float timeToMax = motor->speedSet / motor->accelSet;
    float radToTar = (abs(motor->getError()) * motor->degPerStep) * (PI / 180);
    // (motor->accelSet * motor->degPerStep * (PI/180.0)) this transforms the step based accelerations to radians
    float radMovedInAccel =  ((motor->accelSet * motor->degPerStep * (PI/180.0)) * pow(timeToMax, 2)) / 2.0;

    float remainingDist = radToTar - radMovedInAccel;
    float newSpeedRad = remainingDist / time;     // distance / time
    float stps;

    if (time <= timeToMax){ // meaning it could only accelerate and not reach max speed
        stps = (((2 * radToTar) / pow(time, 2)) / (motor->degPerStep * (PI/180.0)));  // This includes the transformation from rad/s to steps/s

        motor->setAccel(stps);
    }else{
        time = time - timeToMax; // removes the time it spent accelerating
        stps = newSpeedRad / (motor->degPerStep * (PI/180.0)); // includes transformation from rad/s to steps/s
        motor->setMaxSpeed(stps);
       //Serial.println(motor->getSpeed());
    }
}

void initializeArm::runAll(){
    for (int i = 0; i < motorCount; i++){
        if (longestTime < timeToTarget(stepMotors[i])) longestTime = timeToTarget(stepMotors[i]);
    }
    //Serial.println(longestTime);
    for (int i = 0; i < motorCount; i++){
        if (stepMotors[i]){ 
            // find longest time
            // calculate speed based on time
            speedCalculation(4, stepMotors[i]);
            stepMotors[i]->run();

        }else{
            break;
        }
    }
}

thetas initializeArm::calculateIK(int x, int y, int eEAngle){
    thetas angles;

    // calculates what degree the base needs to turn to


    float Xp = x - (stepMotors[3]->getRadius() * cos(eEAngle * PI / 180)); // calculates Xprime disregarding the last joint since the angle for that joint is given
    float Yp = y - stepMotors[0]->getRadius() - (stepMotors[3]->getRadius() * sin(eEAngle * PI/180));

    float cosT2 = (pow(Xp, 2) + pow(Yp, 2) - pow(stepMotors[1]->getRadius(), 2) - pow(stepMotors[2]->getRadius(), 2)) / (2.0 * stepMotors[1]->getRadius() * stepMotors[2]->getRadius());
    // This is the dissolved equation for finding theta 2, this way I can use cosOfTheta2 as a much smaller name

    // Check reachability
    if (cosT2 < -1.0 || cosT2 > 1.0) {
        angles.canReach = false;
        Serial.println("Out of Range");
        return angles;
    }else{
        angles.canReach = true;
    }

    float sinT2 = sqrt(1.0 - pow(cosT2, 2));
    angles.theta2 = atan2(sinT2, cosT2) * 180/PI;

    angles.theta1 = (atan2(Yp, Xp) - atan2(stepMotors[2]->getRadius() * sinT2, stepMotors[1]->getRadius() + (stepMotors[2]->getRadius() * cosT2))) * 180/PI;
    angles.theta3 = eEAngle - angles.theta1 - angles.theta2;

    return angles;
}

void initializeArm::moveToPoint(int x, int y, int eEAngle){
    calcAng = calculateIK(x, y, eEAngle);

    if (calcAng.canReach == false) return;

    stepMotors[1]->moveToAngle(calcAng.theta1);
    stepMotors[2]->moveToAngle(calcAng.theta2);
    stepMotors[3]->moveToAngle(calcAng.theta3);
}

void initializeArm::pause(int time){
    for (int i = 0; i < motorCount; i++){
        if (stepMotors[i]){
            stepMotors[i]->pause(time);
        }
    }
}

void initializeArm::returnHome(){
    for (int i = 0; i < motorCount; i++){
        if (stepMotors[i]){
            stepMotors[i]->moveToAngle(stepMotors[i]->startHeading);
        }
    }
}