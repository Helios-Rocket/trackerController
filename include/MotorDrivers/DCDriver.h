#pragma once

#include <Arduino.h>

#include "MotorDriver.h"

class DCDriver : public MotorDriver
{
    public:
        DCDriver() = default;

        void setPins(uint8_t IN1, uint8_t IN2, uint8_t EN) override
            { dir1 = IN1; dir2 = IN2; pwm = EN; };

        void setPhysicalConstants(float maxSpeedTheoretical, uint32_t UNUSED) override
            { theoreticalMaxSpeed = maxSpeedTheoretical; };

        uint8_t begin() override
        {
            // configure our pins
            pinMode(dir1, OUTPUT);
            pinMode(dir2, OUTPUT);
            pinMode(pwm, OUTPUT);
            digitalWrite(pwm, LOW);

            analogWriteResolution(12);
            analogWriteFrequency(pwm, 30000);

            return 0;
        };

        // this normally should not be used, just use setVelocityCommand() instead.
        void setDirection(bool forwards) override
        { 
            digitalWrite(dir1, forwards);
            digitalWrite(dir2, !forwards);
        };
        
        // if you wish to set the velocity but not start running the motor, set the second argument to false
        // THIS IS VELOCITY AT THE MOTOR
        void setVelocityCommand(float setVelocity, bool startAutomatically = true) override
        {
            if(setVelocity == 0.0){stop(); return; };

            currentVelocityCommand = setVelocity;            

            if(startAutomatically){ start(); };
        };

        void start()
        {
            setDirection(getDir(currentVelocityCommand));
            SerialUSB.println(currentVelocityCommand);
            analogWrite(pwm, abs(currentVelocityCommand));
            running = true;
        }

        void stop()
        {
            analogWrite(pwm, 0);
            running = false;
        };


    private:
        uint8_t dir1 = -1;
        uint8_t dir2 = -1;
        uint8_t pwm = -1;

        float theoreticalMaxSpeed;
        
        uint16_t getPulseWidth(float desiredVel)
        {
            // convert to a PWM pulse frequency
            // we have the resolution set at 12 bits above, which is 0-4095
            return 4095.0 * (fabs(desiredVel) / theoreticalMaxSpeed);
        };

        bool getDir(float desiredVel)
        {
            return desiredVel >= 0;
        };

};