#pragma once
#include <AccelStepper.h>

namespace alight{
    const int maxMovements = 20;
    
    // it is enum class since enum class do not lock names
    // we will use these in switch cases
    enum class moveType { // this sets relative to 0 and absolute to 1
        RELATIVE,   
        ABSOLUTE,
        STEPS
    };

    // creates a new struct that holds 2 values, the type and amount (degrees or steps)
    // we make a list of these that hold information like the type and angle to move to or by
    struct movement{
        moveType type;
        float amount;
    };

    class stepperController{
    public:
        stepperController(AccelStepper* stepMotor, int gearRatio);

        void initialize(int speedMax, int accel, int stepsPerRevolution);

        void run();

        void stop();

        void moveByDeg(float degree);

        void moveToAngle(float angle);

        void moveSteps(int steps);

        bool isRunning();

        int getError();

        int getPosition(); // returns what step the motor is on from starting position (CCW) Can only be positive


    private:
        AccelStepper* m_stepMotor;
        int m_gearRatio;
        movement m_queue[50]; // a queue of movements including their type and angle
        int m_queueEnd = 0; // how long the queue is
        int m_queueCount = 0; // what index in the queue we are in
        int stepsPerRev;
        int steps;
        int clockWise;
        int CClockWise;
    };
}