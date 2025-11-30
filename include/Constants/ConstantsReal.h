#pragma once

#include <Arduino.h> // removes errors on uints

////////////////////////////////////////////////////////////////////// Tuning Parameters //////////////////////////////////////////////////////////////////////

// All measurements in this section are on the output axis.


//////////////////// AZIMUTH
const float azimuthMaxVelocity      = 1.0; // degrees per second
const float azimuthMaxAcceleration  = 100.0; // degrees per second^2

// currently unused
const float azimuthAcceptableError         = 0.0; // degrees
const float azimuthAcceptableVelocityError = 0.0; // degrees per second

const float azimuthPositionKP = 0.0;
const float azimuthPositionKI = 0.0;
const float azimuthPositionKD = 0.0;

const float azimuthVelocityKP = 0.0;
const float azimuthVelocityKI = 0.0;
const float azimuthVelocityKD = 0.0;

const float azimuthPositionStrength = 0.0;
const float azimuthVelocityStrength = 0.0;

//////////////////// ELEVATION
const float elevationMaxVelocity      = 1.0; // degrees per second
const float elevationMaxAcceleration  = 100.0; // degrees per second^2

// currently unused
const float elevationAcceptableError         = 0.0; // degrees
const float elevationAcceptableVelocityError = 0.0; // degrees per second

const float elevationPositionKP = 0.0;
const float elevationPositionKI = 0.0;
const float elevationPositionKD = 0.0;

const float elevationVelocityKP = 0.0;
const float elevationVelocityKI = 0.0;
const float elevationVelocityKD = 0.0;

const float elevationPositionStrength = 0.0;
const float elevationVelocityStrength = 0.0;




// how quickly do we run the loop?
const float controlLoopUpdateRate = 100; // Hertz

////////////////////////////////////////////////////////////////////// Physical Constants //////////////////////////////////////////////////////////////////////

//////////////////// AZIMUTH
const uint azimuthMainGearTeeth = 120; // gear teeth
const uint azimuthMotorPinionTeeth = 24; // gear teeth
const uint azimuthGearboxRatio = 520; // gear ratio, NEED TO VERIFY

constexpr float azimuthGearRatio = (((float)azimuthMotorPinionTeeth/(float)azimuthMainGearTeeth) * (1.0/(float)azimuthGearboxRatio));

const uint azimuthActualTicksPerMotorRev = 4; // NEED TO VERIFY
const uint azimuthTicksPerMotorRev = 4 * azimuthActualTicksPerMotorRev;

constexpr double azimuthDegreesPerTick = azimuthGearRatio * azimuthTicksPerMotorRev / 360.0;


//////////////////// ELEVATION
const uint elevationMainGearTeeth = 84; // gear teeth
const uint elevationMotorPinionTeeth = 13; // gear teeth

constexpr float elevationDriveGearRatio = ((float)elevationMotorPinionTeeth / (float)elevationMainGearTeeth);

const uint elevationEncoderGearTeeth = 42;
const uint elevationEncoderPinionTeeth = 36;

constexpr float elevationEncoderGearRatio ((float)elevationEncoderPinionTeeth / (float)elevationEncoderGearTeeth);

const uint32_t microStepResolutionElevation = 40000;
const float DegreesPerStepElevation = 1.8f;

////////////////////////////////////////////////////////////////////// Sensor Calibration Values //////////////////////////////////////////////////////////////////////


// must be determined / defined
const float azimuthMinimumAngle = -120.0f; // degrees
const float azimuthMaximumAngle = 120.0f; // degrees

const float azimuthZeroReading = 0.0f; // degrees


// must be determined/defined
const float elevationMinimumAngle = 0.0f; // degrees
const float elevationMaximumAngle = 100.0f; // degrees

const float elevationZeroReading = 0.0f; // degrees


////////////////////////////////////////////////////////////////////// Pins //////////////////////////////////////////////////////////////////////

const uint8_t azimuthEncoderA = 22;
const uint8_t azimuthEncoderB = 23;

const uint8_t azimuthIN1 = 1;
const uint8_t azimuthIN2 = 2;
const uint8_t azimuthPWM = 3;

const uint8_t elevationStep = 6;
const uint8_t elevationDirection = 5;
const uint8_t elevationEnable = 4;
const uint8_t elevationAlarm = 0;

const uint8_t elevationSDA = 18;
const uint8_t elevationSCL = 19;
