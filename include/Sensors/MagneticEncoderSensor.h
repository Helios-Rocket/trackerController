#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <AS5600.h>


#include "Sensor.h"

class MagneticEncoderSensor : public Sensor
{
    public:
        MagneticEncoderSensor() = default;
        void setSensorPins(uint8_t PinCLK, uint8_t PinDATA) override
            { scl = PinCLK; sda = PinDATA; };

        uint8_t begin() override
        {
            // start I2C
            Wire.setSCL(scl);
            Wire.setSDA(sda);
            Wire.begin();

            // start our actual sensor
            as5600.begin();
            as5600.setDirection(AS5600_COUNTERCLOCK_WISE);

            zeroed = true;
            return as5600.isConnected();
        };

        uint8_t update() override
        {
            currentPos = as5600.readAngle()*conversionConstant;

            currentVel = as5600.getAngularSpeed(AS5600_MODE_DEGREES, false);

            return 0;
        }

        void debugPrint(Stream *printInterface)
        {
            printInterface->printf("Encoder Angle: %.3f, ",currentPos*AS5600_RAW_TO_DEGREES);
            printInterface->printf("Axis Angle: %.3f \n",currentPos*conversionConstant);
        }


    private:
        uint8_t scl, sda;
        AS5600 as5600;
        

};