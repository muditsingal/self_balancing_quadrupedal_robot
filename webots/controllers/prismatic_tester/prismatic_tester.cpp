// File:          prismatic_tester.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Motor.hpp>
#include <webots/Device.hpp>
#include <webots/Robot.hpp>
#include <windows.h>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>

#define TIME_STEP 32

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  Motor *FL_slider = robot->getMotor("FL_slider");
  Motor *FR_slider = robot->getMotor("FR_slider");
  Motor *BL_slider = robot->getMotor("BL_slider");
  Motor *BR_slider = robot->getMotor("BR_slider");
  // get the time step of the current world.
  //int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  FL_slider->setPosition(0.05);
  FR_slider->setPosition(0.05);
  BL_slider->setPosition(0.05);
  BR_slider->setPosition(0.05);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    //FL_slider->setPosition(5.5);
    FL_slider->setPosition(0.08);
    FR_slider->setPosition(0.08);
    BL_slider->setPosition(0.05);
    BR_slider->setPosition(0.05);
  

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
