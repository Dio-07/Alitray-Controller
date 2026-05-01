#pragma once
#include "stepperController.hpp"

namespace alight{
    class motors{
    public:
        motors(stepperController* stepMotor1, stepperController* stepMotor2, stepperController* stepMotor3, stepperController* stepMotor4, stepperController* stepMotor5,stepperController* stepMotor6);

        bool isAnyRunning();

        void stopAll();

        void runAll();

    private:
        stepperController* m_stepMotor1;
        stepperController* m_stepMotor2;
        stepperController* m_stepMotor3;
        stepperController* m_stepMotor4;
        stepperController* m_stepMotor5;
        stepperController* m_stepMotor6;
    };

}