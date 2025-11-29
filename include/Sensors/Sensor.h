#pragma once

#include <Arduino.h>
#include <TeensyTimerTool.h>

// think of this as an interface
class Sensor
{
    public:
        Sensor(){};
        virtual void setSensorPins(uint8_t, uint8_t){};

        void setUpdateRate(float updatesPerSecond)
            { dt = 1.0/updatesPerSecond; }

        void setPhysicalConversionConstant(double ConversionConstant)
            { conversionConstant = ConversionConstant; };

        void setZero(int64_t ZeroPos)
            {zeroPos = ZeroPos; zeroed = true; };

        void unsetZero()
            { zeroed = false; }

        virtual uint8_t begin();
        virtual uint8_t update();

        bool isZeroed() { return zeroed; };
        
        // returns in the unit specified by your conversion constant
        virtual float getDistFrom0()
            {return currentPos; };

        virtual float getVelocity()
            {return currentVel; };

        virtual void debugPrint(Stream* printInterface);

        virtual ~Sensor() {};

        virtual void updateVelocity(){};

    protected:
        TeensyTimerTool::PeriodicTimer updateTimer;
        double currentPos, lastPos, zeroPos, currentVel = 0.0;

        unsigned long lastTime;

        float conversionConstant;

        double dt;

        bool zeroed = false;
};
