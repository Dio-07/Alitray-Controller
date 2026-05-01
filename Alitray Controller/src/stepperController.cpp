#include "stepperController.hpp"

// custom step controller
using namespace alight;

stepperController::stepperController(AccelStepper* stepMotor, int gearRatio)
:m_stepMotor(stepMotor), m_gearRatio(gearRatio) {}

int stepperController::getPosition(){
    int pos = m_stepMotor->currentPosition() % stepsPerRev; // calculate pos before every movement
    if (pos < 0){ // checks to see if returned answer is negative
        pos += stepsPerRev;
    }

    return pos;
}

void stepperController::initialize(int speedMax, int accel, int stepsPerRevolution){
    m_stepMotor->setMaxSpeed(speedMax);
    m_stepMotor->setAcceleration(accel);
    m_stepMotor->setCurrentPosition(0);
    stepsPerRev = stepsPerRevolution * m_gearRatio; // get total steps for 1 revolution
}

void stepperController::run(){ // movement handler for move by deg and move to angle
    if(m_stepMotor->distanceToGo() != 0){
        m_stepMotor->run();
    }else if(m_queueCount < m_queueEnd && !(m_stepMotor->isRunning())){ // if we have movements left to do and the motor is not running, move

        const movement& move = m_queue[m_queueCount]; // stores the movement and its type as a struct containing its type and degree
        // the array contains both type and degree

        switch(move.type){ // if movement.type = ...
            case (moveType::ABSOLUTE): {
                clockWise = move.amount - getPosition();
                CClockWise = (clockWise > 0) ? 
                    clockWise - stepsPerRev:
                    clockWise + stepsPerRev; // the ? is an if else saying if (condition) ? True : False

                steps = (abs(clockWise) <= abs(CClockWise)) ?
                    clockWise:
                    CClockWise;
                    
                m_stepMotor->move(steps);
                m_queueCount ++;
                break;
            }

            case moveType::RELATIVE:
                steps = (move.amount / 360) * stepsPerRev;
                m_stepMotor->move(steps);
                m_queueCount ++;
                break;

            case moveType::STEPS:
                steps = move.amount;
                m_stepMotor->move(steps);
                m_queueCount ++;
                break;
        }
        
    }else{ // meaning no move movements left start back at the beginning of the list
        m_queueCount = 0;
        m_queueEnd = 0;
    }
}

void stepperController::stop(){
    m_stepMotor->stop();
}

// This adds movements to the queue
void stepperController::moveByDeg(float degree){
    if (m_queueEnd >= maxMovements - 1) return; // If at movement cap, return
    m_queue[m_queueEnd].type = moveType::RELATIVE; // feeds position type
    m_queue[m_queueEnd].amount = degree; // feeds the degree to move by
    m_queueEnd ++; // add 1 to the queue end controls how long the queue is
}

void stepperController::moveToAngle(float angle){ // adds movement to queue
    if (m_queueEnd >= maxMovements - 1) return; // If at movement cap, return

    // calculate what step target is at
    m_queue[m_queueEnd].type = moveType::ABSOLUTE; // feeds position type
    m_queue[m_queueEnd].amount = (angle / 360) * (stepsPerRev);; // feeds what angle to move to

    m_queueEnd ++; // add 1 to the queue end controls how long the queue is
}

void stepperController::moveSteps(int steps){
    if (m_queueEnd >= maxMovements - 1) return;

    m_queue[m_queueEnd].type = moveType::STEPS;
    m_queue[m_queueEnd].amount = steps;

    m_queueEnd ++;

}

bool stepperController::isRunning(){
    return m_stepMotor->isRunning();
}

int stepperController::getError(){
    return m_stepMotor->distanceToGo();
}