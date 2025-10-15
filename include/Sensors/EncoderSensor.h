#pragma once

#include <Arduino.h>
#include <Encoder.h>

#include "Sensor.h"

class EncoderSensor : public Sensor
{
    public:
        EncoderSensor() = default;
        void setSensorPins(uint8_t PinA, uint8_t PinB) override
            { pinA = PinA; pinB = PinB; };

        uint8_t begin() override
        {
            encoder.begin(pinA, pinB);
            zeroed = true;
            return 0;
        };

        // returns 1 if zeroed during loop cycle, 0 otherwise
        uint8_t update() override
        {
            // encoder.update(&encoder.encoder);
            currentPos = encoder.read() - zeroPos;

            // updateVelocity();


            return 0;
        }

        void debugPrint(Stream *printInterface)
        {
            printInterface->print("Current Position: "); printInterface->print(currentPos); printInterface->print(", ");
            printInterface->print("Current Velocity: "); printInterface->print(getVelocity(), 5); printInterface->print(", ");
            printInterface->print("Zero Position: "); printInterface->print(zeroPos); printInterface->print(", ");
            printInterface->print("Encoder Ticks: "); printInterface->print(encoder.read()); printInterface->print(", ");
        }


    private:
        uint8_t pinA;
        uint8_t pinB;

        Encoder encoder = Encoder();

};