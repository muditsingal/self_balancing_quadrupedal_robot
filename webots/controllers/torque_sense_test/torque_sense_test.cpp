// File:          torque_sense_test.cpp
// Date:          11-07-2020
// Description:
// Author:        Mudit
// Modifications: Torque_sense

#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include <webots/TouchSensor.hpp>
#include <windows.h>
#include <webots/InertialUnit.hpp>
#include <webots/Keyboard.hpp>
#include <iostream>
#include <fstream>
#include <algorithm> 
#include <chrono> 

#define TIME_STEP 32
//#define TIME_STEP 128
#define to_rad 3.14159/180
#define to_deg 180/3.14159

// All the webots classes are defined in the "webots" namespace

using namespace webots;
using namespace std;
using namespace std::chrono; 

int tot_steps=0;

float l1 = 10;//9.35;
float l2 = 10;//12.4;

void calculate_angle_3d(float x, float y, float z, float *hip_deg,float *theta1deg,float *theta2deg);
void robot_feet_set(float x_fl, float x_fr, float x_bl, float x_br, float y_fl, float y_fr, float y_bl, float y_br, float z_fl, float z_fr, float z_bl, float z_br);
void self_balance();
void self_balance_diag();
void self_balance_ind();
void self_balance_force_sense();
void calc_forces();
void walk_trot();
void walk_trot_smooth();
void walk();
void walk_dog();

void set_pid_hips(double kp, double ki, double kd);
void set_pid_lower(double kp, double ki, double kd);
void set_pid_upper(double kp, double ki, double kd);

Keyboard *k = new Keyboard();
Robot *robot = new Robot();
 
Motor *FL[3];
Motor *FR[3];
Motor *BL[3];
Motor *BR[3];

InertialUnit *imu;
TouchSensor *FL_force;
TouchSensor *FR_force;
TouchSensor *BL_force;
TouchSensor *BR_force;


float x_fl,x_fr,x_bl,x_br;
float y_fl,y_fr,y_bl,y_br;
float z_fl,z_fr,z_bl,z_br;
float y_fl_walk,y_fr_walk,y_bl_walk,y_br_walk;

float y1_fl,y1_fr,y1_bl,y1_br;  //for roll
float y2_fl,y2_fr,y2_bl,y2_br;  //for pitch

double kp_hips, ki_hips, kd_hips;
double kp_upper, ki_upper, kd_upper;
double kp_lower, ki_lower, kd_lower;

double torques[4][3];  //3 is for 3 motors in each leg, 4 is for each leg
double forces_perp[4];  //forces perpendicular to the body
double err_FL, err_FR, err_BL, err_BR;
double prev_FL, prev_FR, prev_BL, prev_BR;
std::ofstream myfile;
std::ofstream rotations;

float y_fl_roll_off, y_fr_roll_off, y_bl_roll_off, y_br_roll_off;
float y_fl_pitch_off, y_fr_pitch_off, y_bl_pitch_off, y_br_pitch_off;
float y_fl_off, y_fr_off, y_bl_off, y_br_off;
float y_fl_off_p, y_fr_off_p, y_bl_off_p, y_br_off_p;

float FL_hip_pos = 0*to_rad;
float FL_1_pos = 0*to_rad;
float FL_2_pos = 0*to_rad;

float FR_hip_pos = 0*to_rad;
float FR_1_pos = 0*to_rad;
float FR_2_pos = 0*to_rad;

float BL_hip_pos = 0*to_rad;
float BL_1_pos = 0*to_rad;
float BL_2_pos = 0*to_rad;

float BR_hip_pos = 0*to_rad;
float BR_1_pos = 0*to_rad;
float BR_2_pos = 0*to_rad;

float pitch = 0;
float pitch_prev = 0;

int diag;
const double *rot;
const double *force_3d_FL;
const double *force_3d_FR;
const double *force_3d_BL;
const double *force_3d_BR;

//time keeping
int time_durs[10000];
long int durs = 0;

int main(int argc, char **argv) 
{  
  char FL_names[3][8] = {"FL_hip", "FL_1", "FL_2"};
  char FR_names[3][8] = {"FR_hip", "FR_1", "FR_2"};
  char BL_names[3][8] = {"BL_hip", "BL_1", "BL_2"};
  char BR_names[3][8] = {"BR_hip", "BR_1", "BR_2"};
  
  int tilt=1;
  int cnt=0;
  
  k->enable(10);
  int key;
  imu = robot->getInertialUnit("body_ori");
  FL_force = robot->getTouchSensor("Force_FL");
  FR_force = robot->getTouchSensor("Force_FR");
  BL_force = robot->getTouchSensor("Force_BL");
  BR_force = robot->getTouchSensor("Force_BR");
  //BL_force = robot->getTouchSensor(name);
  
  for (int i = 0; i < 3; i++) 
  {
    FL[i] = robot->getMotor(FL_names[i]);
    FR[i] = robot->getMotor(FR_names[i]);
    BL[i] = robot->getMotor(BL_names[i]);
    BR[i] = robot->getMotor(BR_names[i]);
    
    FL[i]->enableTorqueFeedback(20);
    FR[i]->enableTorqueFeedback(20);
    BL[i]->enableTorqueFeedback(20);
    BR[i]->enableTorqueFeedback(20);
    //BL[i]->setPosition(INFINITY);
    //BL[i]->setVelocity(0.0);
  }  
  
  //printf("Is anyone home? \n");
  //FL[2]->setControlPID(10,0,0);
  
  
  myfile.open ("fl_trajectory_1.csv");
  rotations.open("bot_rotations_1.csv");
  
  //x_fl = 0, x_fr = 0, x_bl = 0, x_br = 0;
  //y_fl = -15, y_fr = -15, y_bl = -15, y_br = -15;
  z_fl = 0, z_fr = 0, z_bl = 0, z_br = 0;
  
  //robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl, y_fr, y_bl, y_br, z_fl, z_fr, z_bl, z_br);
  
  
  y_fl_off_p = 0, y_fr_off_p = 0, y_bl_off_p = 0, y_br_off_p = 0;
  //robot_feet_set(5,5,5,5,-15,-15,-15,-15,0,0,0,0);
  
  //another readings that work=> p=3.5, i=0, d=0.35
  
  /*
  kp_hips = 7.4, ki_hips = 0, kd_hips = 0.45;
  kp_upper = 7.4, ki_upper = 0, kd_upper = 0.45;
  kp_lower = 7.4, ki_lower = 0, kd_lower = 0.45;
  
  set_pid_hips(kp_hips,ki_hips,kd_hips);
  set_pid_upper(kp_upper,ki_upper,kd_upper);
  set_pid_lower(kp_lower,ki_lower,kd_lower);
  */
  
  imu->enable(20);
  FL_force->enable(100);
  FR_force->enable(100);
  BL_force->enable(100);
  BR_force->enable(100);
  
  y_fl_walk = -15;
  y_fr_walk = -15;
  y_bl_walk = -15;
  y_br_walk = -15;
  
  //auto start = high_resolution_clock::now(); 
  

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
  //while (1) {  //bad experience
    
    key = k->getKey();
    //printf("Is anyone home? ");
    torques[0][1] = FL[1]->getTorqueFeedback();
    //printf("The torque is %f \n" ,torques[0][1]);
    
    tilt = - tilt;
    //rot = imu->getRollPitchYaw();
    
    force_3d_FL = FL_force->getValues();
    force_3d_FR = FR_force->getValues();
    force_3d_BL = BL_force->getValues();
    force_3d_BR = BR_force->getValues();
    
    forces_perp[0] = sqrt(force_3d_FL[0]*force_3d_FL[0] + force_3d_FL[1]*force_3d_FL[1] + force_3d_FL[2]*force_3d_FL[2]);
    forces_perp[1] = sqrt(force_3d_FR[0]*force_3d_FR[0] + force_3d_FR[1]*force_3d_FR[1] + force_3d_FR[2]*force_3d_FR[2]);
    forces_perp[2] = sqrt(force_3d_BL[0]*force_3d_BL[0] + force_3d_BL[1]*force_3d_BL[1] + force_3d_BL[2]*force_3d_BL[2]);
    forces_perp[3] = sqrt(force_3d_BR[0]*force_3d_BR[0] + force_3d_BR[1]*force_3d_BR[1] + force_3d_BR[2]*force_3d_BR[2]);
    
    
    //self_balance_diag();
    //self_balance_ind();
    
    if(cnt > 5)
      walk_dog();
    
      //walk_trot();
      //walk();
      //walk_trot_smooth();
      //self_balance_ind();
      //self_balance_force_sense();
    //calc_forces();
      
    //printf("The rotation is roll %f, pitch %f, yaw %f \n" ,rot[0]*to_deg,rot[1]*to_deg,rot[2]*to_deg);
    //printf("The forces are FL %f, FR %f, BL %f, BR %f \n" , forces_perp[0], forces_perp[1], forces_perp[2], forces_perp[3]);
    
    //printf("theta1 = %f, theta2 = %f  \n" , (BR_1_pos)*to_deg, (BR_2_pos)*to_deg);
    /*
    printf("FL forces are %f, %f, %f \n" , force_3d_FL[0], force_3d_FL[1], force_3d_FL[2]);
    printf("FR forces are %f, %f, %f \n" , force_3d_FR[0], force_3d_FR[1], force_3d_FR[2]);
    printf("BL forces are %f, %f, %f \n" , force_3d_BL[0], force_3d_BL[1], force_3d_BL[2]);
    printf("BR forces are %f, %f, %f \n" , force_3d_BR[0], force_3d_BR[1], force_3d_BR[2]);
    */
    //robot->step(TIME_STEP);
    
    //myfile << x_fl << "," << y_fl_walk << "\n";
    
    //printf("key pressed is %d \n" ,key);
    printf("the pitch is %f \n" ,pitch);
    if(key == 81)  //key = q
      break;
      
    //tot_steps++;
    cnt++;
  }
  myfile.close();
  rotations.close();
  // Enter here exit cleanup code.
  
  /*
  auto stop = high_resolution_clock::now();
  auto duration = duration_cast<milliseconds>(stop - start);   
  
  cout << "Time taken by function: " << duration.count() << " milliseconds" << "\n"; 
  */
  /*
  printf("The durations are:  \n");
  for(int i = 0; i < durs; i++)
  {
     printf("%d \n" ,time_durs[i]);
  }
  */
  delete robot;
  
  return 0;
}

void calculate_angle_3d(float x, float y, float z, float *hip_deg,float *theta1deg,float *theta2deg)
{
  float theta2;
  int flag = 0;
  
  *hip_deg = atan(z/(-y));
  
  y = -sqrt(y*y + z*z);
  if(x<0)
  {
    x = -x;
    y = -y;
    flag = 1;
  }
  
  theta2 = -acos((x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2));
  *theta1deg = (atan(y/x) + atan((l2*sin(theta2))/(l1 + l2*cos(theta2))));
  
  if(flag == 0)
    *theta1deg = *theta1deg;
  else 
    *theta1deg = -180*to_rad + *theta1deg;// - 90*to_rad;
  *theta2deg = acos((x*x + y*y - l1*l1 - l2*l2)/(2*l1*l2));
  
  //printf("theta1 = %f, theta2 = %f  \n" , (*theta1deg)*to_deg, (*theta2deg)*to_deg);
}


void robot_feet_set(float x_fl, float x_fr, float x_bl, float x_br, float y_fl, float y_fr, float y_bl, float y_br, float z_fl, float z_fr, float z_bl, float z_br)
{
  calculate_angle_3d(x_fl,y_fl,z_fl,&FL_hip_pos,&FL_1_pos,&FL_2_pos);
  FL_1_pos = -FL_1_pos -90*to_rad;
  FL_2_pos = -FL_2_pos;
  
  FL[0]->setPosition(FL_hip_pos);
  FL[1]->setPosition(FL_1_pos);
  FL[2]->setPosition(FL_2_pos);
  
  
  calculate_angle_3d(x_bl,y_bl,z_bl,&BL_hip_pos,&BL_1_pos,&BL_2_pos);
  BL_1_pos = -BL_1_pos -90*to_rad;
  BL_2_pos = -BL_2_pos;
  
  BL[0]->setPosition(BL_hip_pos);
  BL[1]->setPosition(BL_1_pos);
  BL[2]->setPosition(BL_2_pos);
  
  
  calculate_angle_3d(x_fr,y_fr,z_fr,&FR_hip_pos,&FR_1_pos,&FR_2_pos);
  FR_1_pos = -FR_1_pos -90*to_rad;
  FR_2_pos = -FR_2_pos;
  
  FR[0]->setPosition(FR_hip_pos);
  FR[1]->setPosition(FR_1_pos);
  FR[2]->setPosition(FR_2_pos);
  
  
  calculate_angle_3d(x_br,y_br,z_br,&BR_hip_pos,&BR_1_pos,&BR_2_pos);
  BR_1_pos = -BR_1_pos -90*to_rad;
  BR_2_pos = -BR_2_pos;
  
  BR[0]->setPosition(BR_hip_pos);
  BR[1]->setPosition(BR_1_pos);
  BR[2]->setPosition(BR_2_pos);
  //printf("In legs set \n");
  //printf("theta1 = %f, theta2 = %f  \n" , (BR_1_pos)*to_deg, (BR_1_pos)*to_deg);
}

void self_balance()
{
  rot = imu->getRollPitchYaw();
  
  y_fl_roll_off = (z_fl + 9.5)*tan(rot[0]) - (y1_fr - y1_fl)/2;
  y_fr_roll_off = -(-z_fr + 9.5)*tan(rot[0]) + (y1_fr - y1_fl)/2;
  y_bl_roll_off = (z_bl + 9.5)*tan(rot[0]) - (y1_br - y1_bl)/2;
  y_br_roll_off = -(-z_br + 9.5)*tan(rot[0]) + (y1_br - y1_bl)/2;
    
  y1_fl = -15 + y_fl_roll_off;
  y1_fr = -15 + y_fr_roll_off;
  y1_bl = -15 + y_bl_roll_off;
  y1_br = -15 + y_br_roll_off;

  if(y1_fl < -20)
    y1_fl = -20;
  if(y1_fl > 0)
    y1_fl = 0;
  
  if(y1_fr < -20)
    y1_fr = -20;
  if(y1_fr > 0)
    y1_fr = 0;
  
  if(y1_bl < -20)
    y1_bl = -20;
  if(y1_bl > 0)
    y1_bl = 0;
    
  if(y1_br < -20)
    y1_br = -20;
  if(y1_br > 0)
    y1_br = 0;
    
  y_fl_pitch_off = (x_fl + 15.5)*tan(rot[1]) - (y2_bl - y2_fl)/2;
  y_fr_pitch_off = (x_fr + 15.5)*tan(rot[1]) - (y2_br - y2_fr)/2;
  y_bl_pitch_off = -(-x_bl + 15.5)*tan(rot[1]) + (y2_bl - y2_fl)/2;
  y_br_pitch_off = -(-x_br + 15.5)*tan(rot[1]) + (y2_br - y2_fr)/2;
  
  
  y2_fl = -15 + y_fl_pitch_off;
  y2_fr = -15 + y_fr_pitch_off;
  y2_bl = -15 + y_bl_pitch_off;
  y2_br = -15 + y_br_pitch_off;
  
  if(y2_fl < -20)
    y2_fl = -20;
  if(y2_fl > 0)
    y2_fl = 0;
  
  if(y2_fr < -20)
    y2_fr = -20;
  if(y2_fr > 0)
    y2_fr = 0;
  
  if(y2_bl < -20)
    y2_bl = -20;
  if(y2_bl > 0)
    y2_bl = 0;
    
  if(y2_br < -20)
    y2_br = -20;
  if(y2_br > 0)
    y2_br = 0;
  
  y_fl = -15 + y_fl_roll_off + y_fl_pitch_off;
  y_fr = -15 + y_fr_roll_off + y_fr_pitch_off;
  y_bl = -15 + y_bl_roll_off + y_bl_pitch_off;
  y_br = -15 + y_br_roll_off + y_br_pitch_off;
  
  if(y_fl < -20)
    y_fl = -20;
  if(y_fl > 0)
    y_fl = 0;
  
  if(y_fr < -20)
    y_fr = -20;
  if(y_fr > 0)
    y_fr = 0;
  
  if(y_bl < -20)
    y_bl = -20;
  if(y_bl > 0)
    y_bl = 0;
    
  if(y_br < -20)
    y_br = -20;
  if(y_br > 0)
    y_br = 0;
  
  printf("New y's %f %f %f %f \n" ,y_fl,y_fr,y_bl,y_br);
  robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl, y_fr, y_bl, y_br, z_fl, z_fr, z_bl, z_br); 
}

void self_balance_diag()
{
  rot = imu->getRollPitchYaw();

  y_fl_off = (z_fl + 9.5)*tan(rot[0]) + (x_fl + 15.5)*tan(rot[1]) + (y_fl - y_br)/2;
  y_fr_off = -(-z_fr + 9.5)*tan(rot[0]) + (x_fr + 15.5)*tan(rot[1]) + (y_fr - y_bl)/2;
  y_bl_off = (z_bl + 9.5)*tan(rot[0]) -(-x_bl + 15.5)*tan(rot[1]) - (y_fr - y_bl)/2;
  y_br_off = -(-z_br + 9.5)*tan(rot[0]) -(-x_br + 15.5)*tan(rot[1]) - (y_fl - y_br)/2;
  
  /*
  y_fl = -15 + y_fl_off;
  y_fr = -15 + y_fr_off;
  y_bl = -15 + y_bl_off;
  y_br = -15 + y_br_off;
  */
  
  y_fl = y_fl_walk + y_fl_off;
  y_fr = y_fr_walk + y_fr_off;
  y_bl = y_bl_walk + y_bl_off;
  y_br = y_br_walk + y_br_off;
  
  if(y_fl < -20)
    y_fl = -20;
  if(y_fl > 0)
    y_fl = 0;
  
  if(y_fr < -20)
    y_fr = -20;
  if(y_fr > 0)
    y_fr = 0;
  
  if(y_bl < -20)
    y_bl = -20;
  if(y_bl > 0)
    y_bl = 0;
    
  if(y_br < -20)
    y_br = -20;
  if(y_br > 0)
    y_br = 0;
  
  //rotations << rot[0] << "," << rot[1] << "," << rot[2] << "\n";
  
  //printf("New y's %f %f %f %f \n" ,y_fl,y_fr,y_bl,y_br);
  robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl, y_fr, y_bl, y_br, z_fl, z_fr, z_bl, z_br); 
}

void self_balance_ind()
{
  rot = imu->getRollPitchYaw();
  
  float r0_off = 0, r1_off = 0;

  
  if(rot[0] < 2*to_rad && rot[0] > -2*to_rad)
    r0_off = rot[0];
 
  if(rot[1] < 2*to_rad && rot[1] > -2*to_rad)
    r1_off = rot[1];
  
  //int to_deg and to_rad have been done to discretize the angles
  /*
  if(diag == 0)
  {
    y_fl_off = (z_fl + 9.5)*tan(to_rad*((int)((rot[0] - r0_off)*to_deg))) + (x_fl + 15.5)*tan(to_rad*((int)((rot[1] - r1_off)*to_deg))) + y_fl_off_p;
    y_fr_off = 0;
    y_bl_off = 0;
    y_br_off = -(-z_br + 9.5)*tan(to_rad*((int)((rot[0] - r0_off)*to_deg))) -(-x_br + 15.5)*tan(to_rad*((int)(to_deg*(rot[1] - r1_off)))) + y_br_off_p;
    
    y_fl_off_p = y_fl_off;
    y_fr_off_p = y_fr_off;
    y_bl_off_p = y_bl_off;
    y_br_off_p = y_br_off;
    
    y_fl = y_fl_walk + y_fl_off;
    y_fr = y_fr_walk + y_fr_off;
    y_bl = y_bl_walk + y_bl_off;
    y_br = y_br_walk + y_br_off;
  }
  
  else
  {
    y_fl_off = 0;
    y_fr_off = -(-z_fr + 9.5)*tan(to_rad*((int)((rot[0] - r0_off)*to_deg))) + (x_fr + 15.5)*tan(to_rad*((int)((rot[1] - r1_off)*to_deg))) + y_fr_off_p;
    y_bl_off = (z_bl + 9.5)*tan(to_rad*((int)((rot[0] - r0_off)*to_deg))) -(-x_bl + 15.5)*tan(to_rad*((int)((rot[1] - r1_off)*to_deg))) + y_bl_off_p;
    y_br_off = 0;
    
    y_fl_off_p = y_fl_off;
    y_fr_off_p = y_fr_off;
    y_bl_off_p = y_bl_off;
    y_br_off_p = y_br_off;
    
    y_fl = y_fl_walk + y_fl_off;
    y_fr = y_fr_walk + y_fr_off;
    y_bl = y_bl_walk + y_bl_off;
    y_br = y_br_walk + y_br_off;
  }
  */
    
  /*
  y_fl_off = (z_fl + 9.5)*tan(rot[0] - r0_off) + (x_fl + 15.5)*tan(rot[1] - r1_off) + y_fl_off_p;
  y_fr_off = -(-z_fr + 9.5)*tan(rot[0] - r0_off) + (x_fr + 15.5)*tan(rot[1] - r1_off) + y_fr_off_p;
  y_bl_off = (z_bl + 9.5)*tan(rot[0] - r0_off) -(-x_bl + 15.5)*tan(rot[1] - r1_off) + y_bl_off_p;
  y_br_off = -(-z_br + 9.5)*tan(rot[0] - r0_off) -(-x_br + 15.5)*tan(rot[1] - r1_off) + y_br_off_p;
  */
  
  /*
  y_fl_off = (z_fl + 9.5)*tan(rot[0]) + (x_fl + 15.5)*tan(rot[1]) + y_fl_off_p;
  y_fr_off = -(-z_fr + 9.5)*tan(rot[0]) + (x_fr + 15.5)*tan(rot[1]) + y_fr_off_p;
  y_bl_off = (z_bl + 9.5)*tan(rot[0]) -(-x_bl + 15.5)*tan(rot[1]) + y_bl_off_p;
  y_br_off = -(-z_br + 9.5)*tan(rot[0]) -(-x_br + 15.5)*tan(rot[1]) + y_br_off_p;
  */
  
  pitch = 0;//rot[1] - r1_off + pitch_prev;
  
  y_fl_off = (z_fl + 9.5)*tan(to_rad*((int)((rot[0] - r0_off)*to_deg))) + (x_fl + 15.5)*tan(to_rad*((int)((rot[1] - r1_off)*to_deg))) + y_fl_off_p;
  y_fr_off = -(-z_fr + 9.5)*tan(to_rad*((int)((rot[0] - r0_off)*to_deg))) + (x_fr + 15.5)*tan(to_rad*((int)((rot[1] - r1_off)*to_deg))) + y_fr_off_p;
  y_bl_off = (z_bl + 9.5)*tan(to_rad*((int)((rot[0] - r0_off)*to_deg))) -(-x_bl + 15.5)*tan(to_rad*((int)((rot[1] - r1_off)*to_deg))) + y_bl_off_p;
  y_br_off = -(-z_br + 9.5)*tan(to_rad*((int)((rot[0] - r0_off)*to_deg))) -(-x_br + 15.5)*tan(to_rad*((int)(to_deg*(rot[1] - r1_off)))) + y_br_off_p;
  
  pitch_prev = pitch;
  
  
  y_fl_off_p = y_fl_off;
  y_fr_off_p = y_fr_off;
  y_bl_off_p = y_bl_off;
  y_br_off_p = y_br_off;
  
  y_fl = y_fl_walk + y_fl_off;
  y_fr = y_fr_walk + y_fr_off;
  y_bl = y_bl_walk + y_bl_off;
  y_br = y_br_walk + y_br_off;
  
  
  if(y_fl < -20)
    y_fl = -20;
  else if(y_fl > 0)
    y_fl = 0;
  
  if(y_fr < -20)
    y_fr = -20;
  else if(y_fr > 0)
    y_fr = 0;
  
  if(y_bl < -20)
    y_bl = -20;
  else if(y_bl > 0)
    y_bl = 0;
    
  if(y_br < -20)
    y_br = -20;
  else if(y_br > 0)
    y_br = 0;
  
  rotations << tot_steps << "," << to_deg*rot[0] << "," << to_deg*rot[1] << "," << to_deg*rot[2] << "\n";//roll pitch yaw
  //printf("New y's %f %f %f %f \n" ,y_fl,y_fr,y_bl,y_br);
  robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl, y_fr, y_bl, y_br, z_fl, z_fr, z_bl, z_br);
}

void set_pid_hips(double kp, double ki, double kd)
{
  FL[0]->setControlPID(kp,ki,kd);
  FR[0]->setControlPID(kp,ki,kd);
  BL[0]->setControlPID(kp,ki,kd);
  BR[0]->setControlPID(kp,ki,kd);
}

void set_pid_lower(double kp, double ki, double kd)
{
  FL[1]->setControlPID(kp,ki,kd);
  FR[1]->setControlPID(kp,ki,kd);
  BL[1]->setControlPID(kp,ki,kd);
  BR[1]->setControlPID(kp,ki,kd);  
}

void set_pid_upper(double kp, double ki, double kd)
{
  FL[2]->setControlPID(kp,ki,kd);
  FR[2]->setControlPID(kp,ki,kd);
  BL[2]->setControlPID(kp,ki,kd);
  BR[2]->setControlPID(kp,ki,kd);  
}

void calc_forces()
{
  int i;
  for(i=0; i<3; i++)
  {
    torques[0][i] = FL[i]->getTorqueFeedback();
    torques[1][i] = FR[i]->getTorqueFeedback();
    torques[2][i] = BL[i]->getTorqueFeedback();
    torques[3][i] = BR[i]->getTorqueFeedback();
  }
  forces_perp[0] = (torques[0][0]*abs(z_fl) - torques[0][1]*l1*sin(FL_1_pos));
  forces_perp[1] = (torques[1][0]*abs(z_fr) - torques[1][1]*l1*sin(FR_1_pos));
  forces_perp[2] = (torques[2][0]*abs(z_bl) - torques[2][1]*l1*sin(BL_1_pos));
  forces_perp[3] = (torques[3][0]*abs(z_br) - torques[3][1]*l1*sin(BR_1_pos));
  
}

void self_balance_force_sense()
{
  kp_hips = +0.008, ki_hips = 0, kd_hips = 0.00;//.08/TIME_STEP;
  
  for(int i=0; i<4; i++)
  {
    if(forces_perp[i] > 1000)
      forces_perp[i] = 1000;
  }
  
  err_FL = 242 - forces_perp[0];
  err_FR = 228 - forces_perp[1];
  err_BL = 226 - forces_perp[2];
  err_BR = 220 - forces_perp[3];
  
  y_fl_off = kp_hips*(err_FL) + ki_hips*0 + kd_hips*(err_FL - prev_FL) - y_fl_off_p;
  y_fr_off = kp_hips*(err_FR) + ki_hips*0 + kd_hips*(err_FR - prev_FR) - y_fr_off_p;
  y_bl_off = kp_hips*(err_BL) + ki_hips*0 + kd_hips*(err_BL - prev_BL) - y_bl_off_p;
  y_br_off = kp_hips*(err_BR) + ki_hips*0 + kd_hips*(err_BR - prev_BR) - y_br_off_p;
  
  y_fl = -15 + y_fl_off;
  y_fr = -15 + y_fr_off;
  y_bl = -15 + y_bl_off;
  y_br = -15 + y_br_off;
  
  y_fl_off_p = y_fl_off;
  y_fr_off_p = y_fr_off;
  y_bl_off_p = y_bl_off;
  y_br_off_p = y_br_off;
  
  if(y_fl < -20)
    y_fl = -20;
  else if(y_fl > 0)
    y_fl = 0;
  
  if(y_fr < -20)
    y_fr = -20;
  else if(y_fr > 0)
    y_fr = 0;
  
  if(y_bl < -20)
    y_bl = -20;
  else if(y_bl > 0)
    y_bl = 0;
    
  if(y_br < -20)
    y_br = -20;
  else if(y_br > 0)
    y_br = 0;
  
  prev_FL = err_FL;
  prev_FR = err_FR;
  prev_BL = err_BL;
  prev_BR = err_BR;
    
  printf("New y's %f %f %f %f \n" ,y_fl,y_fr,y_bl,y_br);
  robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl, y_fr, y_bl, y_br, z_fl, z_fr, z_bl, z_br);
}

void walk_trot()
{
  int i;

  x_fl = 4;
  x_br = 4;
  
  x_fr = -4;
  x_bl = -4;
  
  for(i = 0;i<8;i++)  //FL BR contact
  {
    diag = 0;
    y_fl_walk = -16 - sqrt(16 - x_fl*x_fl)/6;
    //z_fl_walk = -(4 - x_fl)/2;
    y_br_walk = -16 - sqrt(16 - x_br*x_br)/6;
    //z_br = (4 - x_br)/2;
    
    y_fr_walk = -16 + 2*sqrt(16 - x_fr*x_fr)/3;
    //z_fr = 0;
    y_bl_walk = -16 + 2*sqrt(16 - x_bl*x_bl)/3;
    //z_bl = 0;
    
    self_balance_ind();
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";
    //robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    x_fl--;
    x_br--;
    x_fr++;
    x_bl++;
    
    robot->step(TIME_STEP);
  }
  
  //x_fl = -4;
  //x_fr = +4;
  
  for(i = 0;i<8;i++)  //FR BL contact
  {
    diag = 1;
    y_fl_walk = -16 + 2*sqrt(16 - x_fl*x_fl)/3;
    //z_fl = 0;
    y_br_walk = -16 + 2*sqrt(16 - x_br*x_br)/3;
    //z_br = 0;
    
    y_fr_walk = -16 - sqrt(16 - x_fr*x_fr)/6;
    //z_fr = (-4 + x_fr)/2;
    y_bl_walk = -16 - sqrt(16 - x_bl*x_bl)/6;
    //z_bl = -(-4 + x_bl)/2;
    
    self_balance_ind();
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";

    //robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    x_fl++;
    x_br++;
    
    x_fr--;
    x_bl--;
    robot->step(TIME_STEP);
  }  
}

void walk_trot_smooth()
{
  int i;

  x_fl = 4;
  x_br = 4;
  
  x_fr = -4;
  x_bl = -4;
  
  for(i = 0;i<41;i++)  //FL BR contact
  {
    diag = 0;
    y_fl_walk = -16 - sqrt(16 - x_fl*x_fl)/6;
    //z_fl_walk = -(4 - x_fl)/2;
    y_br_walk = -16 - sqrt(16 - x_br*x_br)/6;
    //z_br = (4 - x_br)/2;
    
    y_fr_walk = -16 + 2*sqrt(16 - x_fr*x_fr)/3;
    //z_fr = 0;
    y_bl_walk = -16 + 2*sqrt(16 - x_bl*x_bl)/3;
    //z_bl = 0;
    
    //self_balance_ind();
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";
    robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    x_fl -= 0.2;
    x_br -= 0.2;
    x_fr += 0.2;
    x_bl += 0.2;
    
    robot->step(TIME_STEP);
  }
  
  x_fl = -4;
  x_fr = +4;
  
  for(i = 0;i<41;i++)  //FR BL contact
  {
    diag = 1;
    y_fl_walk = -16 + 2*sqrt(16 - x_fl*x_fl)/3;
    //z_fl = 0;
    y_br_walk = -16 + 2*sqrt(16 - x_br*x_br)/3;
    //z_br = 0;
    
    y_fr_walk = -16 - sqrt(16 - x_fr*x_fr)/6;
    //z_fr = (-4 + x_fr)/2;
    y_bl_walk = -16 - sqrt(16 - x_bl*x_bl)/6;
    //z_bl = -(-4 + x_bl)/2;
    
    //self_balance_ind();
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";

    robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    x_fl += 0.2;;
    x_br += 0.2;
    
    x_fr -= 0.2;
    x_bl -= 0.2;
    robot->step(TIME_STEP);
  }  
}

void walk()
{
  int i;

  x_fl = -3;
  x_br = -1;
  
  x_fr = 1;
  x_bl = 3;
  
  for(i = 0;i<6;i++)  //FL swing
  {
    //diag = 0;
    y_fl_walk = -16 + 2*sqrt(9 - x_fl*x_fl)/3;
    //z_fl_walk = -(4 - x_fl)/2;
    y_br_walk = -16;// - sqrt(16 - x_br*x_br)/6;
    //z_br = (4 - x_br)/2;
    
    y_fr_walk = -16;// - sqrt(16 - x_fr*x_fr)/6;
    //z_fr = 0;
    y_bl_walk = -16;// -sqrt(16 - x_bl*x_bl)/6;
    //z_bl = 0;
    
    //self_balance_ind();
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";
    robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    x_fl += 1;
    x_br -= 0.3333;
    x_fr -= 0.3333;
    x_bl -= 0.3333;
    
    robot->step(TIME_STEP);
  }
  
  //x_fl = -4;
  //x_fr = +4;
  for(i = 0;i<6;i++)  //BR swing
  {
    //diag = 0;
    y_fl_walk = -16;// - sqrt(16 - x_br*x_br)/6;
    //z_fl_walk = -(4 - x_fl)/2;
    y_br_walk = -16 + 2*sqrt(9 - x_br*x_br)/3;
    //z_br = (4 - x_br)/2;
    
    y_fr_walk = -16;// - sqrt(16 - x_fr*x_fr)/6;
    //z_fr = 0;
    y_bl_walk = -16;// -sqrt(16 - x_bl*x_bl)/6;
    //z_bl = 0;
    
    //self_balance_ind();
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";
    robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    x_fl -= 0.3333;
    x_br += 1;
    x_fr -= 0.3333;
    x_bl -= 0.3333;
    
    robot->step(TIME_STEP);
  }  
  
  for(i = 0;i<6;i++)  //fr swing
  {
    //diag = 0;
    y_fl_walk = -16;// - sqrt(16 - x_br*x_br)/6;
    //z_fl_walk = -(4 - x_fl)/2;
    y_br_walk = -16;// - sqrt(16 - x_br*x_br)/6;
    //z_br = (4 - x_br)/2;
    
    y_fr_walk = -16 + 2*sqrt(9 - x_fr*x_fr)/3;
    //z_fr = 0;
    y_bl_walk = -16;// -sqrt(16 - x_bl*x_bl)/6;
    //z_bl = 0;
    
    //self_balance_ind();
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";
    robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    x_fl -= 0.3333;
    x_br -= 0.3333;
    x_fr += 1;
    x_bl -= 0.3333;
    
    robot->step(TIME_STEP);
  }
    
  for(i = 0;i<6;i++)  //bl swing
  {
    //diag = 0;
    y_fl_walk = -16;// - sqrt(16 - x_br*x_br)/6;
    //z_fl_walk = -(4 - x_fl)/2;
    y_br_walk = -16;// - sqrt(16 - x_br*x_br)/6;
    //z_br = (4 - x_br)/2;
    
    y_fr_walk = -16;// - sqrt(16 - x_fr*x_fr)/6;
    //z_fr = 0;
    y_bl_walk = -16 + 2*sqrt(9 - x_bl*x_bl)/3;
    //z_bl = 0;
    
    //self_balance_ind();
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";
    robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    x_fl -= 0.3333;
    x_br -= 0.3333;
    x_fr -= 0.3333;
    x_bl += 1;
    
    robot->step(TIME_STEP);
  }     
}

void walk_dog()  //order => BR FR BL FL
{
  int i;

  x_fl = 3;
  x_br = -3;
  
  x_fr = -1;
  x_bl = 1;
  //+ tan(pitch)*x_
  //pitch = - pitch;
  //for(i = 0;i<6;i++)  //BR swing
  for(i = 0;i<12;i++)  //BR swing
  {
    //diag = 0;
    y_fl_walk = -16+1 - tan(pitch)*x_fl;// - sqrt(16 - x_br*x_br)/6;
    //z_fl_walk = -(4 - x_fl)/2;
    //z_fl=0;
    y_br_walk = -16 + 2*sqrt(9 - x_br*x_br)/3 - tan(pitch)*x_br;
    //y_fl=0;
    //z_br = (4 - x_br)/2;
    
    y_fr_walk = -16 - tan(pitch)*x_fr;// - sqrt(16 - x_fr*x_fr)/6;
    //z_fr = 1;
    y_bl_walk = -16 - tan(pitch)*x_bl;// -sqrt(16 - x_bl*x_bl)/6;
    //z_bl = 1; //positive z is RHS of bot
    
    //with time keeping
    auto start = high_resolution_clock::now(); 
    self_balance_ind();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<nanoseconds>(stop - start);   
    time_durs[durs] = duration.count();
    durs++;
    
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";
    //robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    /*
    x_fl -= 0.3333*TIME_STEP/64;
    x_br += 1*TIME_STEP/64;
    x_fr -= 0.3333*TIME_STEP/64;
    x_bl -= 0.3333*TIME_STEP/64;
    */
    
    x_fl -= 0.1666;//0.3333/2;
    x_br += 0.5;//1/2;
    x_fr -= 0.1666;//0.3333/2;
    x_bl -= 0.1666;//0.3333/2;
    
    
    robot->step(TIME_STEP);
    tot_steps++;
  }
  
  //for(i = 0;i < 6;i++)  //fr swing
  for(i = 0;i < 12;i++)  //fr swing
  {
    //diag = 0;
    y_fl_walk = -16 - tan(pitch)*x_fl;// - sqrt(16 - x_br*x_br)/6;
    //z_fl_walk = -(4 - x_fl)/2;
    //z_fl = 1;
    y_br_walk = -16 - tan(pitch)*x_br;// - sqrt(16 - x_br*x_br)/6;
    //z_br = (4 - x_br)/2;
    //z_br = 1;
    
    y_fr_walk = -16 + 2*sqrt(9 - x_fr*x_fr)/3 - tan(pitch)*x_fr;
    //z_fr = 0;
    y_bl_walk = -16+1 - tan(pitch)*x_bl;// -sqrt(16 - x_bl*x_bl)/6;
    //z_bl = 0;
    
    //with time keeping/2
    auto start = high_resolution_clock::now(); 
    self_balance_ind();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<nanoseconds>(stop - start);   
    time_durs[durs] = duration.count();
    durs++;
    
    
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";
    //robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    /*
    x_fl -= 0.3333*TIME_STEP/64;
    x_br -= 0.3333*TIME_STEP/64;
    x_fr += 1*TIME_STEP/64;
    x_bl -= 0.3333*TIME_STEP/64;
    */
    
    x_fl -= 0.1666;//0.3333/2;
    x_br -= 0.1666;//0.3333/2;
    x_fr += 0.5;//1/2;
    x_bl -= 0.1666;//0.3333/2;
    
    robot->step(TIME_STEP);
    tot_steps++;
  }
  
 //for(i = 0;i<6*2;i++)  //bl swing
  for(i = 0;i<12;i++)  //bl swing
  {
    //diag = 0;
    y_fl_walk = -16 - tan(pitch)*x_fl;// - sqrt(16 - x_br*x_br)/6;
    //z_fl_walk = -(4 - x_fl)/2;
    //z_fl = -1;
    y_br_walk = -16 - tan(pitch)*x_br;// - sqrt(16 - x_br*x_br)/6;
    //z_br = (4 - x_br)/2;
    //z_br = -1;
    
    y_fr_walk = -16+1 - tan(pitch)*x_fr;// - sqrt(16 - x_fr*x_fr)/6;
    //z_fr = 0;
    y_bl_walk = -16 + 2*sqrt(9 - x_bl*x_bl)/3 - tan(pitch)*x_bl;
    //z_bl = 0;
    
    //with time keeping
    auto start = high_resolution_clock::now(); 
    self_balance_ind();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<nanoseconds>(stop - start);   
    time_durs[durs] = duration.count();
    durs++;    //self_balance_diag();
    
    myfile << x_fl << "," << y_fl_walk << "\n";
    //robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    /*
    x_fl -= 0.3333*TIME_STEP/64;
    x_br -= 0.3333*TIME_STEP/64;
    x_fr -= 0.3333*TIME_STEP/64;
    x_bl += 1*TIME_STEP/64;
    */
    
    x_fl -= 0.1666;//0.3333/2;
    x_br -= 0.1666;//0.3333/2;
    x_fr -= 0.1666;//0.3333/2;
    x_bl += 0.5;//1/2;
    
    
    robot->step(TIME_STEP);
    tot_steps++;
  }
  
  //for(i = 0;i<6*64/TIME_STEP;i++)  //FL swing
  for(i = 0;i<12;i++)  //FL swing
  {
    //diag = 0;
    y_fl_walk = -16 + 2*sqrt(9 - x_fl*x_fl)/3 - tan(pitch)*x_fl;
    //z_fl_walk = -(4 - x_fl)/2;
    //z_fl = 0;
    y_br_walk = -16+1 - tan(pitch)*x_br;// - sqrt(16 - x_br*x_br)/6;
    //z_br = (4 - x_br)/2;
    //z_br = 0;
    
    y_fr_walk = -16 - tan(pitch)*x_fr;// - sqrt(16 - x_fr*x_fr)/6;
    //z_fr = -1;
    y_bl_walk = -16 - tan(pitch)*x_bl;// -sqrt(16 - x_bl*x_bl)/6;
    //z_bl = -1;
    
    //with time keeping
    auto start = high_resolution_clock::now(); 
    self_balance_ind();
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<nanoseconds>(stop - start);   
    time_durs[durs] = duration.count();
    durs++;
    
    //self_balance_diag();
    myfile << x_fl << "," << y_fl_walk << "\n";
    //robot_feet_set(x_fl, x_fr, x_bl, x_br, y_fl_walk, y_fr_walk, y_bl_walk, y_br_walk, z_fl, z_fr, z_bl, z_br);
    
    /*
    x_fl += 1*TIME_STEP/64;
    x_br -= 0.3333*TIME_STEP/64;
    x_fr -= 0.3333*TIME_STEP/64;
    x_bl -= 0.3333*TIME_STEP/64;
    */
    
    x_fl += 0.5;//1/2;
    x_br -= 0.1666;//0.3333/2;
    x_fr -= 0.1666;//0.3333/2;
    x_bl -= 0.1666;//0.3333/2;
    
    
    robot->step(TIME_STEP);
    tot_steps++;
  }
  //pitch = - pitch;
}
 