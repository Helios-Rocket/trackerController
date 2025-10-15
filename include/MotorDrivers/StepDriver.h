#pragma once

#include <Arduino.h>
#include <TeensyTimerTool.h>

#include "MotorDriver.h"

class StepDriver : public MotorDriver
{
    public:
        StepDriver() = default;

        void setPins(uint8_t DirectionPin, uint8_t StepPin, uint8_t EnablePin) override
            { dirPin = DirectionPin; stepPin = StepPin; enablePin = EnablePin; };

        void setPhysicalConstants(float degreesPerStep, float microstepResolution) override
            { degPerStep = degreesPerStep; microstepRes = microstepResolution; };

        uint8_t begin() override
        {
            // configure our pins
            pinMode(dirPin, OUTPUT);
            pinMode(stepPin, OUTPUT);
            pinMode(enablePin, OUTPUT);
            digitalWrite(enablePin, LOW);

            return 0;
        };

        // this normally should not be used, just use setVelocityCommand() instead.
        void setDirection(bool forwards) override
            { digitalWrite(dirPin, forwards); };
        
        // if you wish to set the velocity but not start running the motor, set the second argument to false
        // THIS IS VELOCITY AT THE MOTOR
        void setVelocityCommand(float setVelocity, bool startAutomatically = true) override
        {
            if(setVelocity == 0.0){stop(); return; };

            currentVelocityCommand = setVelocity;

            // calculate our required values for output
            double stepFreq = abs(setVelocity) / (degPerStep / microstepRes);
            bool dir = setVelocity >= 0;

            setDirection(dir);
            updateFrequency(stepFreq);
            

            if(startAutomatically){ start(); };
        };

        void start() { timer.start(); running = true; };

        void stop() { timer.stop(); running = false; };


    private:
        TeensyTimerTool::PeriodicTimer timer = TeensyTimerTool::PeriodicTimer(TeensyTimerTool::TCK);

        uint8_t dirPin = -1;
        uint8_t stepPin = -1;
        uint8_t enablePin = -1;

        float degPerStep;
        float microstepRes;

        void updateFrequency(double frequency)
        {
            if(frequency < 1.0){
                stop();
                return;
            }
            // Convert frequency to period in microseconds
            double periodMicros = 1e6 / frequency / 2; // half period (toggle HIGH/LOW)

            /*
            digitalWriteFast can change a pin's state far faster, 
            at the expense of not doing some safety checking/perfect cross-platform compatibility.
            given this code is targeting a specific platform ( Polaris / Teensy 4.0(micromod) ),
            this is an acceptable tradeoff for better performance
            */
            // ^ this is a lie back from when i was dumb.
            // I leave the comment in as it's a good explanation of a theoretical tradeoff

            timer.begin([this]() {
                digitalToggle(stepPin);
            }, periodMicros, false);
        }
};