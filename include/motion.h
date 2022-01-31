#ifndef MOTION_H
#define MOTION_H

#include "vector"

extern int last_found_index;

void driverControl ();
void moveToRefPose (double targetX, double targetY, double targetHeading, double linMax, double turnMax, double Kp_lin, double Kp_turn, double Kp_predict, double r);
void followRefPath (const std::vector<std::vector<double>> &path, double targetAngle, double maxTotalVel, double linMax, double turnMax, double Kp_lin, double Kp_turn, double tune_turn, int numOfSeg);

#endif