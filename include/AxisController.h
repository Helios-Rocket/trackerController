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

        // set how often we want the control loop to run.
        // actually updating the control loop is handled internally with a timer, 
        //  to ensure accurate loop timing.
        // units are microseconds (us)
        void setLoopTimeStep(double timeStepUS)
        {
            timeStep = timeStepUS/(10^6);
            dt = (1.0/timeStep);
        }

        void setNewGoal(float NewGoalPosition, float NewGoalVelocity){ goalPos=NewGoalPosition; goalVel = NewGoalVelocity; };

        uint8_t begin()
        {
            // motor driver and sensor will be started before this
            driver->setHoldBehavior(HoldBehavior::coastMode); // begin in a non-powered state for any calibration reasons

            // don't start by default
            controlLoopTimer.begin([this](){this->updateLoop();}, (timeStep*(10^6)), false);

            return 0;
        }

        uint8_t updateLoop()
        {
            sensor->update();
            // driver.update();




            return 0;
        }

        bool isEnabled(){return enabled;};

        void debugPrint(Stream *printInterface)
        {

        };

    private:
        // physical hardware interfaces
        MotorDriver* driver = nullptr;
        Sensor* sensor = nullptr;

        // control loop timer
        TeensyTimerTool::PeriodicTimer controlLoopTimer;

        bool enabled = false;

        // internal storage variables
        float maxVelLimit, maxAccelLimit, minAngle, maxAngle = 0.0; // angular unit degrees
        float allowedErrorPos, allowedErrorVel = 0.0; // angular unit degrees
        float goalPos, goalVel = 0.0; // angular unit degrees
        double timeStep, dt; // seconds (s)

};