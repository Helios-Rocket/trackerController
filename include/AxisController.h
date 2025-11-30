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

        void setTuning(float PositionP, float PositionI, float PositionD, float VelocityP, float VelocityI, float VelocityD, float PositionStrength, float VelocityStrength)
        {
            posP = PositionP; posI = PositionI; posD = PositionD;
            velP = VelocityP; velI = VelocityI; velD = VelocityD;
            posStrength = PositionStrength; velStrength = VelocityStrength;
        };


        // set how many times we want the control loop to run per second.
        // actually updating the control loop is handled internally with a timer, 
        //  to ensure accurate loop timing.
        // units are an integer number of Hz
        void setLoopUpdateRate(uint loopsPerSecond)
        {
            // undo our performance-optimizing transforms to I & D gains
            posI /= dt; posD *= dt;
            velI /= dt; velD *= dt;

            updateRate = loopsPerSecond;
            dt = (1.0/loopsPerSecond);

            // assume constant loop time, 
            //  so pre-apply constant dt factor to integral and derivative gains
            // this yields performance improvements
            posI *= dt; posD /= dt;
            velI *= dt; velD /= dt;
        }

        void setNewGoal(float NewGoalPosition, float NewGoalVelocity)
        {
            lastGoalPos = goalPos; lastGoalVel = goalVel;
            goalPos=NewGoalPosition; goalVel = NewGoalVelocity; 
        };

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

            // get our current state
            pos = sensor->getDistFrom0();
            vel = sensor->getVelocity();

            // get our current error
            posError = goalPos - pos;
            velError = goalVel - vel;


            // calculate position PID

            // summate integral term only if we wouldn't be saturating
            if(fabs(posIntegralSum) < maxVelLimit){ posIntegralSum += (posError * posI); };
            // calculate position command
            float posU = (posP * posError 
                    + posIntegralSum
                    + posD * ((goalPos-lastGoalPos) - (pos - lastPos))
            );



            // calculate velocity PID

            // summate integral term only if we wouldn't be saturating
            if(fabs(velIntegralSum) < maxAccelLimit){ velIntegralSum += (velError * velI); };
            // calculate position command
            float velU = (velP * velError 
                    + velIntegralSum
                    + velD * ((goalVel-lastGoalVel) - (vel - lastVel))
            );


            // mix both our control values
            output = (posStrength * posU) + (velStrength * velU);

            // limit our output to our max velocity
            if(fabs(output) > maxVelLimit){
                float dir = (output > 0) ? 1 : -1;
                output = dir * maxVelLimit;
            }

            // write our output to the axis
            driver->setVelocityCommand(output);


            // update our "last" values
            lastPos = pos; lastVel = vel;
            lastPosError = posError; lastVelError = velError;
            return 0;
        };

        bool isEnabled(){return enabled;};

        void debugPrint(Stream *printInterface)
        {

        };

        void enable()
        { 
            enabled = true; 
            
            // reset state
            posIntegralSum = 0.0; velIntegralSum = 0.0;
            lastPos = sensor->getDistFrom0(); lastVel = sensor->getVelocity();
            posError, velError, lastPosError, lastVelError = 0.0;
            lastGoalPos = goalPos; lastGoalVel = goalVel;
            
            controlLoopTimer.start(); 
        };

        void disable()
            { enabled = false; controlLoopTimer.stop(); driver->stop(); };

    private:
        // physical hardware interfaces
        MotorDriver* driver = nullptr;
        Sensor* sensor = nullptr;

        // control loop timer
        TeensyTimerTool::PeriodicTimer controlLoopTimer;

        bool enabled = false;
        

        // internal storage variables
        float output = 0.0;
        float posIntegralSum, velIntegralSum = 0.0;
        float posP, posI, posD, velP, velI, velD, posStrength, velStrength = 0.0;
        float posError, velError, lastPosError, lastVelError; // angular unit degrees
        float maxVelLimit, maxAccelLimit, minAngle, maxAngle = 0.0; // angular unit degrees
        float allowedErrorPos, allowedErrorVel = 0.0; // angular unit degrees
        float goalPos, goalVel, lastGoalPos, lastGoalVel = 0.0; // angular unit degrees
        float pos, vel, lastPos, lastVel = 0.0;
        double dt; // seconds (s)
        uint updateRate; // hertz (Hz)

};