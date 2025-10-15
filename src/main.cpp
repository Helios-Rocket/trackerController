#include <Arduino.h>

// CHOOSE DEPLOY PLATFORM BY SELECTING ENVIRONMENT IN PLATFORMIO

////////////////////////////////////////////////////////////////////// Includes //////////////////////////////////////////////////////////////////////

// platform specific defines
#ifdef REAL
  #include "Constants/ConstantsReal.h"
#endif
#ifdef SUBSCALE
  #include "Constants/ConstantsSubscale.h"
#endif
#ifdef SIM
  #include "Constants/ConstantsSim.h"
#endif // platform specific defines

// axis controller code
#include "AxisController.h"

// stepper driver code
#include "StepDriver.h"

// positional feedback code
#include "EncoderSensor.h"
#include "PotSensor.h"

// this handles our interfacing to the outside world
#include "StreamInterface.h"

// temporary serial interface code for tuning / bringup
// #include "TunerInterface.h"

// #define DEBUG

////////////////////////////////////////////////////////////////////// Global Objects //////////////////////////////////////////////////////////////////////

// hardware declerations for azimuth axis
Sensor* azimuthSensor = new EncoderSensor();
StepDriver azimuthMotorDriver;

// motion controller defs for the azimuth axis
AxisController azimuthController(&azimuthMotorDriver, azimuthEnable, azimuthSensor);

// hardware declerations for elevation axis
Sensor* elevationSensor = new PotSensor();
StepDriver elevationMotorDriver;

// motion controller defs for the elevation axis
AxisController elevationController(&elevationMotorDriver, elevationEnable, elevationSensor);

// serial interface
StreamInterface serialInterface(&SerialUSB, azimuthSensor, elevationSensor, &azimuthController, &elevationController);

// tuning interface
// TunerInterface tuner(&SerialUSB);

TeensyTimerTool::PeriodicTimer interfaceTimer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer debugPrintTimer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer blinkTimer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer sensorVelocityTimer(TeensyTimerTool::TCK);


////////////////////////////////////////////////////////////////////// Local Function Declarations //////////////////////////////////////////////////////////////////////

void interfaceLoop();

void debugPrint();
void interface();

void configureHardware(); // provides pin mappings and tuning parameters to objects

// tuner application related functions
void runTuner();
void calculateVelAccel(float actualPos);
void sendTunerData(float desiredPos, float actualPos, float desiredVel, float actualVelocity, float desiredAcc, float actualAccel);

////////////////////////////////////////////////////////////////////// setup() //////////////////////////////////////////////////////////////////////

void setup() 
{
  SerialUSB.begin(115200);
  while(!SerialUSB){} // wait for connection

  SerialUSB.setTimeout(5);

  configureHardware(); // setup pins and tuning parameters for controllers

  debugPrintTimer.begin(debugPrint, 100ms); // thank you std::chrono for readable units
  interfaceTimer.begin(interfaceLoop, 10ms); // 100000 in µs = 100ms = 0.1s

  blinkTimer.begin([]{digitalToggle(LED_BUILTIN);}, 1s); // thank you std::chrono for readable units

  sensorVelocityTimer.begin([]{azimuthSensor->updateVelocity();elevationSensor->updateVelocity();}, 100ms); // thank you std::chrono for readable units

  // start actual things
  azimuthController.begin();  
  elevationController.begin();

  // start LEDS
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // actual begin code
  delay(10);
#ifdef DEBUG

#endif





}

////////////////////////////////////////////////////////////////////// loop() //////////////////////////////////////////////////////////////////////

// as everything is run through timers, loop goes unused
void loop() 
{
  // this is called implicitly
  // yield(); 
}

////////////////////////////////////////////////////////////////////// Local Function Definitions //////////////////////////////////////////////////////////////////////

void interfaceLoop()
{
  #ifndef DEBUG
  interface();
  #endif
  // debugPrint();
}

void debugPrint(){
  #ifdef DEBUG
  // azimuthController.debugPrint(&SerialUSB);
  // elevationController.debugPrint(&SerialUSB);
  // azimuthSensor->debugPrint(&SerialUSB);
  // elevationSensor->update();
  // elevationSensor->debugPrint(&SerialUSB);
  #endif
}

void interface(){
  serialInterface.read();
}

void configureHardware()
{
  // configure azimuth hardware
  // azimuthSensor->setSensorPins(azimuthEncoderA, azimuthEncoderB, azimuthLimitSwitch);
  // azimuthSensor->setPhysicalConversionConstant(azimuthConversionRatio);
  // azimuthSensor->setZeroOffset(azimuthZeroOffsetDegrees);
  // azimuthMotorDriver.setPins(azimuthDirection, azimuthStep);
  // azimuthMotorDriver.setPhysicalConstants(DegreesPerStepAzimuth, microStepResolutionAzimuth);

  // configure azimuth motion controller
  // azimuthController.setPhysicalLimits(azimuthMaxVelocity, azimuthMaxAcceleration, azimuthGearRatio);
  // azimuthController.setTuningParameters(azimuthFF, azimuthkP, azimuthkI, azimuthkD, 0.0, azimuthAcceptableError, azimuthAcceptableVelocityError, homingVelocity);
  // azimuthController.setLoopTimeStep(controlLoopTimeStep);

  // configure elevation hardware
  // elevationSensor->setSensorPins(elevationPotentiometer);
  // elevationSensor->setPhysicalConversionConstant(elevationConversionRatio);
  // elevationSensor->setZero(elevationMinimumValue); // we only do this for the elevation, azimuth zeroes itself
  // elevationMotorDriver.setPins(elevationDirection, elevationStep, elevationDirection2, elevationStep2);
  // elevationMotorDriver.setPhysicalConstants(DegreesPerStepElevation, microStepResolutionElevation);

  // configure elevation motion controller
  // elevationController.setPhysicalLimits(elevationMaxVelocity, elevationMaxAcceleration, elevationGearRatio);
  // elevationController.setTuningParameters(elevationFF, elevationkP, elevationkI, elevationkD, elevationGravityCompFactor, elevationAcceptableError, elevationAcceptableVelocityError, homingVelocity);
  // elevationController.setLoopTimeStep(controlLoopTimeStep);
  // elevationController.setLimits(elevationMinimumAngle, elevationMaximumAngle);
}