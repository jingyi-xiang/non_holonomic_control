/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\13918                                            */
/*    Created:      Mon Sep 06 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// inertial20           inertial      11              
// encoderRight         encoder       A, B            
// leftFront            motor         1               
// leftBack             motor         2               
// rightFront           motor         3               
// rightBack            motor         4               
// encoderMiddle        rotation      5               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "iostream"
#include "vector"
#include "motion.h"
#include "odom.h"
#include "screen_print.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // wait (2500, msec);

  inertial20.calibrate();
  while(inertial20.isCalibrating()){
    task::sleep(100);
  }

  inertial20.setHeading(270, deg);
  inertial20.setRotation(0, deg);

  thread odom = thread(position_update);

  // screen graphing initialization
  draw_background();

  // moveToRefPose (double targetX, double targetY, double targetHeading, double linMax, double turnMax, double Kp_lin, double Kp_turn, double Kp_predict, double r)
  // moveToRefPose (0.5, 3, 90, 100, 100, 70, 1.6, 8, 0.5);
  std::vector<std::vector<double>> path 
  {
    {1, 1},
    {1, 2},
    {1, 3},
    {1, 4},
    {3, 4},
    {3, 3},
    {3, 2},
    {3, 1},
    {1, 1},
  };

  draw_path (path);

  // void followRefPath (const std::vector<std::vector<double>> &path, double targetAngle, double maxTotalVel, double linMax, double turnMax, double Kp_lin, double Kp_turn, double tune_turn, int numOfSeg)
  followRefPath (path, 180, 100, 100, 100, 75, 1.5, 1.2, 0);
  draw_path (path);

  while (1)
  {
    wait (10, msec);
  }
  
}
