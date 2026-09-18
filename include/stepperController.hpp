#pragma once
#include <Arduino.h>
#include <AccelStepper.h>

namespace alight{
    const unsigned int maxMovements = 16;
    
    // it is enum class since enum class do not lock names
    // we will use these in switch cases
    enum class moveType { // this sets relative to 0 and absolute to 1
        RELATIVE,   
        ABSOLUTE,
        STEPS,
        PAUSE
    };

    // creates a new struct that holds 2 values, the type and amount (degrees or steps)
    // we make a list of these that hold information like the type and angle to move to or by
    struct movement{
        moveType type;
        float amount;
    };

    class stepperController{
    public:
        stepperController(AccelStepper* stepMotor, int gearRatio, int stepsPerRevolution, float radius_mm);

        void initialize(int speedMax, int accel, int startPosition);

        void run();

        void stop();

        void moveByDeg(float degree);

        void moveToAngle(float angle);

        void moveSteps(int steps);
        
        void pause(int time);

        void setHeading(float theta);

        void setMaxSpeed(float stepsPerSec);

        void setAccel(float stepsPerSec);

        float getHeading();

        bool isRunning();

        long getError();

        int getPosition(); // returns what step the motor is on from starting position (CCW) Can only be positive

        float getRadius();

        int getSpeed(); // returns Max speed in steps per second

        int getAccel(); // returns Steps per second

        bool isPaused();

        float degPerStep;

        bool paused = false;

        int startHeading;

        unsigned long startTime;

        float accelSet;
        float speedSet;

    private:
        AccelStepper* m_stepMotor;
        uint8_t m_gearRatio;
        movement m_queue[maxMovements]; // a queue of movements including their type and angle
        uint8_t m_queueEnd = 0; // how long the queue is
        uint8_t m_queueCount = 0; // what index in the queue we are in
        uint16_t stepsPerRev;
        float radius;
        int steps;
        unsigned long pauseDelay = 0;
        int clockWise;
        int CClockWise;
    };
}