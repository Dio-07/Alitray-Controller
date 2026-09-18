#pragma once
#include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include "stepperController.hpp"

struct thetas{
    float base;
    float theta1;
    float theta2;
    float theta3;
    float theta4;
    float theta5;
    bool canReach = true;
};

namespace alight{
    class initializeArm{

    public:
        initializeArm(stepperController* m_stepMotors[6]);

        bool isAnyRunning();

        void stopAll();

        bool isAnyPaused();

        float timeToTarget(stepperController* motor); // return steps per second

        void speedCalculation(float time, stepperController* motor); // recalculates speed based on time

        void runAll();

        void moveToPoint(int x, int y, int eEAngle);

        void pause(int time);

        void returnHome();

    private:
        thetas calculateIK(int x, int y, int eEAngle);
        BLA::Matrix<3,3> Tmulted;
        BLA::Matrix<3,3> T;
        stepperController* stepMotors[6];
        thetas calcAng;
        int motorCount;
        float eEffectorX = 0;
        float eEffectorY = 0;
        int longestTime = 0;
    };

}