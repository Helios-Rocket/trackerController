#pragma once

#include <Arduino.h>

enum HoldBehavior {coastMode, brakeMode};

// think of this as an interface
class MotorDriver
{
    public:
        MotorDriver(){};

        virtual void setPins(uint8_t, uint8_t, uint8_t); // used for both types

        virtual void setPhysicalConstants(float , uint32_t);

        virtual uint8_t begin();
        // virtual uint8_t update();

        void setHoldBehavior(HoldBehavior HoldMode){holdMode = HoldMode;};

        // if you wish to set the velocity but not start running the motor, set the second argument to false
        // THIS IS VELOCITY AT THE MOTOR
        virtual void setVelocityCommand(float setVelocity, bool startAutomatically = true);

        // this normally should not be used, just use setVelocityCommand() instead.
        virtual void setDirection(bool forwards);    

        // start/stop motor
        virtual void start(){};
        virtual void stop(){};

        void debugPrint(Stream* printInterface);

        virtual ~MotorDriver() {};

        float getCurrentVelocityCommand(){return currentVelocityCommand;};
        bool isRunning(){return running;};
        HoldBehavior getHoldBehavior(){return holdMode;};

        
    protected:
        float currentVelocityCommand = 0;
        bool running = false;
        HoldBehavior holdMode;
};