#pragma once

#include <Arduino.h>
#include <TeensyTimerTool.h>

#include "Sensors/Sensor.h"
#include "MotorDrivers/MotorDriver.h"

class AxisController
{
    public:
        AxisController(MotorDriver* Driver = nullptr, Sensor* PositionalSensor = nullptr)
            { driver = Driver; sensor = PositionalSensor; };

        // Set physical limits for this axis. Angular unit degrees.
        void setPhysicalLimits(float MaxVelocityLimit, float MaxAccelerationLimit, float MinimumAngle, float MaximumAngle)
            {maxVelLimit = MaxVelocityLimit; maxAccelLimit = MaxAccelerationLimit; minAngle = MinimumAngle; maxAngle = MaximumAngle;};

        // set our tolerance of error, defined as +- the value given here in degrees.
        void setTolerance(float AllowablePositionError, float AllowableVelocityError)
            {allowedErrorPos = AllowablePositionError; allowedErrorVel = AllowableVelocityError; };

        // set how many times we want the control loop to run per second.
        // actually updating the control loop is handled internally with a timer, 
        //  to ensure accurate loop timing.
        // units are an integer number of Hz
        void setLoopUpdateRate(uint loopsPerSecond)
        {
            updateRate = loopsPerSecond;
            dt = (1.0/loopsPerSecond);
        }

        void setNewGoal(float NewGoalPosition, float NewGoalVelocity){ goalPos=NewGoalPosition; goalVel = NewGoalVelocity; };

        uint8_t begin()
        {
            // motor driver and sensor will be started before this
            driver->setHoldBehavior(HoldBehavior::coastMode); // begin in a non-powered state for any calibration reasons

            // don't start by default
            controlLoopTimer.begin([this](){this->updateLoop();}, (dt*(10^6)), false);

            return 0;
        };

        uint8_t updateLoop()
        {
            // sensor->update(); // sensor updates itself
            // driver.update();

            // get our current error state
            posError = goalPos - sensor->getDistFrom0();
            velError = goalVel - sensor->getVelocity();




            return 0;
        };

        bool isEnabled(){return enabled;};

        void debugPrint(Stream *printInterface)
        {

        };

        void enable()
            { enabled = true; controlLoopTimer.start(); };

        void disable()
            { enabled = false; controlLoopTimer.stop(); };

    private:
        // physical hardware interfaces
        MotorDriver* driver = nullptr;
        Sensor* sensor = nullptr;

        // control loop timer
        TeensyTimerTool::PeriodicTimer controlLoopTimer;

        bool enabled = false;

        // internal storage variables
        float posError, velError; // angular unit degrees
        float maxVelLimit, maxAccelLimit, minAngle, maxAngle = 0.0; // angular unit degrees
        float allowedErrorPos, allowedErrorVel = 0.0; // angular unit degrees
        float goalPos, goalVel = 0.0; // angular unit degrees
        double dt; // seconds (s)
        uint updateRate; // hertz (Hz)

};