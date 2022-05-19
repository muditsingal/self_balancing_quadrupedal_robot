// File:          tilt_mechanism.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <windows.h>
#include <webots/Keyboard.hpp>
#include <iostream>
#include <fstream>

#define TIME_STEP 32
//#define TIME_STEP 128
#define to_rad 3.14159/180
#define to_deg 180/3.14159

// All the webots classes are defined in the "webots" namespace
using namespace webots;
//using namespace std;
//using namespace std::chrono; 

std::ofstream platform_roll;
std::ofstream platform_pitch;

int tot_steps;
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
  Keyboard *k = new Keyboard();
  // get the time step of the current world.
  //int timeStep = (int)robot->getBasicTimeStep();
  Motor *roll_m = robot->getMotor("roll_motor");
  Motor *pitch_m = robot->getMotor("pitch_motor");
  k->enable(10);
  
  platform_roll.open("platform_roll_1.csv");
  platform_pitch.open("platform_pitch_1.csv");
  
  int key;
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  roll_m->setPosition(0*to_rad);
  pitch_m->setPosition(0*to_rad);
  int go = 1;
  float fn;
  float x=0;
  float roll_rad, pitch_rad;
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    
    fn = sin(x);
    roll_rad = 10*fn*to_rad;
    pitch_rad = 10*fn*to_rad;
    roll_m->setPosition(roll_rad);
    pitch_m->setPosition(pitch_rad);
    // Process sensor data here.
    
    //write to csv file 
    platform_roll << tot_steps << "," << roll_rad*to_deg << "\n";
    platform_pitch << tot_steps << "," << pitch_rad*to_deg << "\n";
    
    go = - go;
    x = x + 0.04;
    key = k->getKey();
    //printf("Key is %d \n" ,key);
    /*
    if(key == 314)  //left key
      roll_m->setPosition(6*to_rad);
    else if(key == 315)  //up key
      roll_m->setPosition(12*to_rad);
    else if(key == 316)  //right key
      roll_m->setPosition(-6*to_rad);
    else if(key == 317)  //down key
      roll_m->setPosition(-12*to_rad);
    */
    
    if(key == 314)  //left key
      pitch_m->setPosition(4*to_rad);
    else if(key == 315)  //up key
      pitch_m->setPosition(8*to_rad);
    else if(key == 316)  //right key
      pitch_m->setPosition(-4*to_rad);
    else if(key == 317)  //down key
      pitch_m->setPosition(-8*to_rad);
    
    
    if(key == 81)
      break;
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    //robot->step(TIME_STEP);
    tot_steps++;
  }

  // Enter here exit cleanup code.
  platform_roll.close();
  platform_pitch.close();
  delete robot;
  return 0;
}
