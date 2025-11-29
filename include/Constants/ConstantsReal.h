#pragma once

#include <Arduino.h> // removes errors on uints
// #include <chrono>

////////////////////////////////////////////////////////////////////// Tuning Parameters //////////////////////////////////////////////////////////////////////

constexpr float homingVelocity = 0.5; // degrees per second

// THESE ARE AT AXIS
constexpr float azimuthMaxVelocity      = 1.0; // degrees per second
constexpr float azimuthMaxAcceleration  = 100.0; // degrees per second^2

// THESE ARE AT AXIS
constexpr float elevationMaxVelocity      = 100.0; // degrees per second
constexpr float elevationMaxAcceleration  = 100.0; // degrees per second^2


constexpr float azimuthAcceptableError         = 0.0; // degrees
constexpr float azimuthAcceptableVelocityError = 0.0; // degrees per second

constexpr float elevationAcceptableError         = 0.0; // degrees
constexpr float elevationAcceptableVelocityError = 0.0; // degrees per second


constexpr float azimuthFF   = 0.0; // technically units of velocity
constexpr float azimuthkP   = 0.0; // unitless
constexpr float azimuthkI   = 0.0; // unitless
constexpr float azimuthkD   = 0.0; // unitless

constexpr float elevationFF   = 0.0; // technically units of velocity
constexpr float elevationkP   = 0.0; // unitless
constexpr float elevationkI   = 0.0; // unitless
constexpr float elevationkD   = 0.0; // unitless

// constexpr float elevationGravityCompFactor = 0.5; // technically not unitless, but determined emperically, not by calculation


constexpr float controlLoopTimeStep = 10000; // microseconds - 10000us = 10ms = 100Hz control loop update rate

////////////////////////////////////////////////////////////////////// Physical Constants //////////////////////////////////////////////////////////////////////

constexpr float azimuthMainGearTeeth = 120; // gear teeth
constexpr float azimuthMotorPinionTeeth = 24; // gear teeth
constexpr float azimuthGearboxRatio = 520; // gear ratio, NEED TO VERIFY

constexpr float azimuthGearRatio = ((azimuthMotorPinionTeeth/azimuthMainGearTeeth) * (1/azimuthGearboxRatio));

constexpr uint azimuthActualTicksPerMotorRev = 4; // NEED TO VERIFY
constexpr uint azimuthTicksPerMotorRev = 4 * azimuthActualTicksPerMotorRev;

constexpr double azimuthDegreesPerTick = azimuthGearRatio * azimuthTicksPerMotorRev / 360.0;



constexpr float elevationMainGearTeeth = 84; // gear teeth
constexpr float elevationMotorPinionTeeth = 13; // gear teeth

constexpr float elevationDriveGearRatio = (elevationMotorPinionTeeth/elevationMainGearTeeth);

constexpr float elevationEncoderGearTeeth = 42;
constexpr float elevationEncoderPinionTeeth = 36;

constexpr float elevationEncoderGearRatio (elevationEncoderPinionTeeth / elevationEncoderGearTeeth);

constexpr int microStepResolutionElevation = 40000;
constexpr float DegreesPerStepElevation = 1.8f;

////////////////////////////////////////////////////////////////////// Sensor Calibration Values //////////////////////////////////////////////////////////////////////

// must be determined / defined
constexpr float azimuthMinimumAngle = -120.0f; // degrees
constexpr float azimuthMaximumAngle = 120.0f; // degrees

constexpr int64_t azimuthZeroReading = 0; // units

// must be determined/defined
constexpr float elevationMinimumAngle = 0.0f; // degrees
constexpr float elevationMaximumAngle = 100.0f; // degrees

constexpr int64_t elevationZeroReading = 0; // units

////////////////////////////////////////////////////////////////////// Pins //////////////////////////////////////////////////////////////////////

constexpr uint8_t azimuthEncoderA = 22;
constexpr uint8_t azimuthEncoderB = 23;

constexpr uint8_t azimuthIN1 = 1;
constexpr uint8_t azimuthIN2 = 2;
constexpr uint8_t azimuthPWM = 3;

constexpr uint8_t elevationStep = 6;
constexpr uint8_t elevationDirection = 5;
constexpr uint8_t elevationEnable = 4;
constexpr uint8_t elevationAlarm = 0;

constexpr uint8_t elevationSDA = 18;
constexpr uint8_t elevationSCL = 19;
