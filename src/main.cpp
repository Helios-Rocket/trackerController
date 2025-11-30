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

// motor driver code
#include "MotorDrivers/MotorDriver.h"
#include "MotorDrivers/DCDriver.h"
#include "MotorDrivers/StepDriver.h"

// positional feedback code
#include "Sensors/Sensor.h"
#include "Sensors/EncoderSensor.h"
#include "Sensors/MagneticEncoderSensor.h"

// this handles our interfacing to the outside world
// #include "StreamInterface.h"

// temporary serial interface code for tuning / bringup
// #include "TunerInterface.h"

#define DEBUG

////////////////////////////////////////////////////////////////////// Global Objects //////////////////////////////////////////////////////////////////////

// hardware declerations for azimuth axis
Sensor* azimuthSensor = new EncoderSensor();
MotorDriver* azimuthMotorDriver = new DCDriver();

// hardware declerations for elevation axis
Sensor* elevationSensor = new MagneticEncoderSensor();
MotorDriver* elevationMotorDriver = new StepDriver();

// motion controller defs
AxisController azimuthController(azimuthMotorDriver, azimuthSensor);
AxisController elevationController(elevationMotorDriver, elevationSensor);

// serial interface
// StreamInterface serialInterface(&SerialUSB, azimuthSensor, elevationSensor, &azimuthController, &elevationController);


TeensyTimerTool::PeriodicTimer interfaceTimer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer debugPrintTimer(TeensyTimerTool::TCK);
TeensyTimerTool::PeriodicTimer blinkTimer(TeensyTimerTool::TCK);

////////////////////////////////////////////////////////////////////// Local Function Declarations //////////////////////////////////////////////////////////////////////

void debugPrint();

void configureHardware(); // provides pin mappings and tuning parameters to objects

////////////////////////////////////////////////////////////////////// setup() //////////////////////////////////////////////////////////////////////

void setup() 
{
  SerialUSB.begin(115200);
  while(!SerialUSB){} // wait for connection

  SerialUSB.setTimeout(5);

  configureHardware(); // setup pins and tuning parameters for controllers

  // start timers
  debugPrintTimer.begin(debugPrint, 100ms); 
  // interfaceTimer.begin(interfaceLoop, 10ms); 
  blinkTimer.begin([]{digitalToggle(LED_BUILTIN);}, 1s); 
  // sensorVelocityTimer.begin([]{azimuthSensor->updateVelocity();elevationSensor->updateVelocity();}, 100ms);

  // start actual things
  // azimuthController.begin();  
  // elevationController.begin();

  // start LEDS
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // actual begin code
  delay(10);
#ifdef DEBUG
  elevationSensor->begin();
  azimuthSensor->begin();
  elevationMotorDriver->begin();
  azimuthMotorDriver->begin();

  // azimuthMotorDriver->setVelocityCommand(3500);
  // min, approx 2300
  // max, approx 3500

  elevationMotorDriver->setVelocityCommand(0.6);

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

void debugPrint(){
  #ifdef DEBUG
  // azimuthSensor->update();
  // azimuthSensor->debugPrint(&SerialUSB);
  // elevationSensor->update();
  // elevationSensor->debugPrint(&SerialUSB);
  
  #endif
}


void configureHardware()
{
  // configure azimuth hardware
  azimuthSensor->setSensorPins(azimuthEncoderA, azimuthEncoderB);
  azimuthSensor->setPhysicalConversionConstant(azimuthDegreesPerTick);
  azimuthSensor->setZero(azimuthZeroReading);
  azimuthMotorDriver->setPins(azimuthIN1, azimuthIN2, azimuthPWM);
  azimuthMotorDriver->setPhysicalConstants(azimuthMaxVelocity/azimuthGearRatio, 0);

  // configure elevation hardware
  elevationSensor->setSensorPins(elevationSCL, elevationSDA);
  elevationSensor->setPhysicalConversionConstant(AS5600_RAW_TO_DEGREES * elevationEncoderGearRatio);
  elevationSensor->setZero(elevationZeroReading); // we only do this for the elevation, azimuth zeroes itself
  elevationMotorDriver->setPins(elevationDirection, elevationStep, elevationEnable);
  elevationMotorDriver->setPhysicalConstants(DegreesPerStepElevation, microStepResolutionElevation);

  // configure azimuth motion controller
  azimuthController.setPhysicalLimits(azimuthMaxVelocity, azimuthMaxAcceleration, azimuthMinimumAngle, azimuthMaximumAngle);
  azimuthController.setTolerance(azimuthAcceptableError, azimuthAcceptableVelocityError);
  azimuthController.setTuning(azimuthPositionKP, azimuthPositionKI, azimuthPositionKD, azimuthVelocityKP, azimuthVelocityKI, azimuthVelocityKD, azimuthPositionStrength, azimuthVelocityStrength);
  azimuthController.setLoopUpdateRate(controlLoopUpdateRate);
  

  // configure elevation motion controller
  elevationController.setPhysicalLimits(elevationMaxVelocity, elevationMaxAcceleration, elevationMinimumAngle, elevationMaximumAngle);
  elevationController.setTolerance(elevationAcceptableError, elevationAcceptableVelocityError);
  elevationController.setTuning(elevationPositionKP, elevationPositionKI, elevationPositionKD, elevationVelocityKP, elevationVelocityKI, elevationVelocityKD, elevationPositionStrength, elevationVelocityStrength);
  elevationController.setLoopUpdateRate(controlLoopUpdateRate);

}