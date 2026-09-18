#include "stepperController.hpp"

// custom step controller
using namespace alight;

stepperController::stepperController(AccelStepper* stepMotor, int gearRatio, int stepsPerRevolution, float radius_mm)
:m_stepMotor(stepMotor), m_gearRatio(gearRatio), stepsPerRev(stepsPerRevolution) , radius(radius_mm)
{}

void stepperController::initialize(int speedMax, int accel, int startPosition){
    startHeading = startPosition;
    accelSet = accel;
    speedSet = speedMax;

    stepsPerRev = stepsPerRev * m_gearRatio; // get total steps for 1 revolution
    degPerStep = 360.000 / stepsPerRev;
    startPosition = startPosition / degPerStep;

    m_stepMotor->setMaxSpeed(speedMax);
    m_stepMotor->setAcceleration(accel);
    m_stepMotor->setCurrentPosition(startPosition);
    
}

void stepperController::run(){ // movement handler for move by deg and move to angle

    if(m_stepMotor->distanceToGo() != 0){
        m_stepMotor->run();

    }else if(paused){
        if (millis() - startTime >= pauseDelay){
            Serial.println("unpaused");
            paused = false;
            m_queueCount ++;
        }

    }else if(m_queueCount < m_queueEnd){ // if we have movements left to do and the motor is not running, move
        //m_stepMotor->setMaxSpeed(speedSet);
        //m_stepMotor->setAcceleration(accelSet);
        const movement& move = m_queue[m_queueCount]; // stores the movement and its type as a struct containing its type and degree
        // the array contains both type and degree

        switch(move.type){ // if movement.type = ...
            case (moveType::PAUSE): {
                paused = true;
                Serial.println("paused");
                pauseDelay = move.amount; // in this case steps means time
                startTime = millis();
                break;
            }

            case (moveType::ABSOLUTE): {
                clockWise = move.amount - getPosition(); // calculate clockwise
                CClockWise = (clockWise > 0) ? // calculate counter clockwise
                    clockWise - stepsPerRev:
                    clockWise + stepsPerRev; // the ? is an if else saying if (condition) ? True : False

                steps = (abs(clockWise) <= abs(CClockWise)) ? // compare
                    clockWise:
                    CClockWise;
                    
                m_stepMotor->move(steps);
                m_queueCount ++;
                break;
            }

            case (moveType::RELATIVE): {
                steps = move.amount / degPerStep;
                m_stepMotor->move(steps);
                m_queueCount ++;
                break;
            }

            case (moveType::STEPS): {
                steps = move.amount;
                m_stepMotor->move(steps);
                m_queueCount ++;
                break;
            }
        }
        
    }else{ // meaning no move movements left start back at the beginning of the list
        m_queueCount = 0;
        m_queueEnd = 0;
    }
}

void stepperController::stop(){
    m_stepMotor->stop(); // kinda redundant but wtv
}

// This adds movements to the queue
void stepperController::moveByDeg(float degree){
    if (m_queueEnd >= maxMovements - 1) { // If at movement cap, recall the functin until movement clears up
         
    }
    m_queue[m_queueEnd].type = moveType::RELATIVE; // feeds position type
    m_queue[m_queueEnd].amount = degree; // feeds the degree to move by
    m_queueEnd ++; // add 1 to the queue end controls how long the queue is
}

void stepperController::moveToAngle(float angle){ // adds movement to queue
    if (m_queueEnd >= maxMovements - 1) return; // If at movement cap, return

    // calculate what step target is at
    m_queue[m_queueEnd].type = moveType::ABSOLUTE; // feeds position type
    if(angle < 0 ){
        angle += 360;
    }
    m_queue[m_queueEnd].amount = (angle / 360.0) * (stepsPerRev);; // feeds what angle to move to
    m_queueEnd ++; // add 1 to the queue end controls how long the queue is
}

void stepperController::moveSteps(int steps){
    if (m_queueEnd >= maxMovements - 1) return;

    m_queue[m_queueEnd].type = moveType::STEPS;
    m_queue[m_queueEnd].amount = steps;

    m_queueEnd ++;

}

void stepperController::pause(int time){
    if (m_queueEnd >= maxMovements - 1) return;

    m_queue[m_queueEnd].type = moveType::PAUSE;
    m_queue[m_queueEnd].amount = time;

    m_queueEnd ++;
}

void stepperController::setHeading(float theta){
    theta = theta / degPerStep;
    m_stepMotor->setCurrentPosition(theta);
}

void stepperController::setMaxSpeed(float stepsPerSecond){
    m_stepMotor->setMaxSpeed(stepsPerSecond);
}

void stepperController::setAccel(float stepsPerSec){
    m_stepMotor->setAcceleration(stepsPerSec);
}

float stepperController::getHeading(){
    return (getPosition() * degPerStep);
}

bool stepperController::isRunning(){
    return m_stepMotor->isRunning();
}

long stepperController::getError(){
    return m_stepMotor->distanceToGo();
}

int stepperController::getPosition(){
    uint16_t pos = m_stepMotor->currentPosition() % stepsPerRev; // calculate pos before every movement
    if (pos < 0){ // checks to see if returned answer is negative
        pos += stepsPerRev;
    }

    return pos;
}

float stepperController::getRadius(){
    return radius;
}

int stepperController::getSpeed(){
    return m_stepMotor->maxSpeed();
}

int stepperController::getAccel(){
    return m_stepMotor->acceleration();
}

bool stepperController::isPaused(){
    return paused;
}