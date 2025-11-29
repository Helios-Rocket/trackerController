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

            updateTimer.begin([this](){this->update();}, (dt*(10^6)));
        };

        
        uint8_t update() override
        {
            // encoder.update(&encoder.encoder);
            int32_t currentPosRaw = encoder.read();
            currentPos = (encoder.read()*conversionConstant)-zeroPos;

            currentVel = ((currentPosRaw - lastPosRaw) / dt) * conversionConstant;

            // update our last changes
            lastPosRaw = currentPosRaw;
            lastPos = currentPos;

            return 0;
        }

        void debugPrint(Stream *printInterface)
        {
            printInterface->print("Current Position: "); printInterface->print(currentPos); printInterface->print(", ");
            printInterface->print("Current Velocity: "); printInterface->print(currentVel, 5); printInterface->print(", ");
            // printInterface->print("Zero Position: "); printInterface->print(zeroPos); printInterface->print(", ");
            printInterface->print("Encoder Ticks: "); printInterface->print(encoder.read()); printInterface->print(", ");
            printInterface->println();
        }


    private:
        uint8_t pinA;
        uint8_t pinB;

        int32_t lastPosRaw = 0;

        Encoder encoder = Encoder();

};