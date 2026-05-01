#include "motorManager.hpp"

using namespace alight;

motors::motors(stepperController* stepMotor1, stepperController* stepMotor2, stepperController* stepMotor3, stepperController* stepMotor4, stepperController* stepMotor5, stepperController* stepMotor6)
: m_stepMotor1(stepMotor1), m_stepMotor2(stepMotor2), m_stepMotor3(stepMotor3), m_stepMotor4(stepMotor4), m_stepMotor5(stepMotor5) {}

bool motors::isAnyRunning(){
    if (m_stepMotor1 && m_stepMotor1->isRunning()) return true; // if it exists and its running
    if (m_stepMotor2 && m_stepMotor2->isRunning()) return true;
    if (m_stepMotor3 && m_stepMotor3->isRunning()) return true;
    if (m_stepMotor4 && m_stepMotor4->isRunning()) return true;
    if (m_stepMotor5 && m_stepMotor4->isRunning()) return true;
    if (m_stepMotor6 && m_stepMotor4->isRunning()) return true;
    return false;
}

void motors::stopAll(){
    if (m_stepMotor1) m_stepMotor1->stop(); // if it exists stop it
    if (m_stepMotor2) m_stepMotor2->stop();
    if (m_stepMotor3) m_stepMotor3->stop();
    if (m_stepMotor4) m_stepMotor4->stop();
    if (m_stepMotor5) m_stepMotor4->stop();
    if (m_stepMotor6) m_stepMotor4->stop();
}

void motors::runAll(){
    if (m_stepMotor1) m_stepMotor1->run();
    if (m_stepMotor2) m_stepMotor2->run();
    if (m_stepMotor3) m_stepMotor3->run();
    if (m_stepMotor4) m_stepMotor4->run();
    if (m_stepMotor5) m_stepMotor4->run();
    if (m_stepMotor6) m_stepMotor4->run();
}