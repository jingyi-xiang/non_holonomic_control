#include "vex.h"
#include "mathFunc.h"
#include "iostream"
#include "vector"
#include "odom.h"

using namespace vex;

void driverControl (void)
{
  leftFront.spin (directionType::fwd, Controller1.Axis3.value() + Controller1.Axis1.value(), velocityUnits::pct);
  leftBack.spin (directionType::fwd, Controller1.Axis3.value() + Controller1.Axis1.value(), velocityUnits::pct);
  rightFront.spin (directionType::fwd, Controller1.Axis3.value() - Controller1.Axis1.value(), velocityUnits::pct);
  rightBack.spin (directionType::fwd, Controller1.Axis3.value() - Controller1.Axis1.value(), velocityUnits::pct);
}

void moveWithAssignedSpeed (double linVel, double turnVel)
{
  leftFront.spin(directionType::fwd, linVel-turnVel, velocityUnits::pct);
  leftBack.spin(directionType::fwd, linVel-turnVel, velocityUnits::pct);
  rightFront.spin(directionType::fwd, linVel+turnVel, velocityUnits::pct);
  rightBack.spin(directionType::fwd, linVel+turnVel, velocityUnits::pct);
}

void groupStopHold (void)
{
  leftFront.stop(hold);
  leftBack.stop(hold);
  rightFront.stop(hold);
  rightBack.stop(hold);
}

void groupStopCoast (void)
{
  leftFront.stop(coast);
  leftBack.stop(coast);
  rightFront.stop(coast);
  rightBack.stop(coast);
}

double assignMinPower (double vel, double minVel)
{
  if (vel < 0)
  {
    return -minVel;
  }
  else 
  {
    return minVel;
  }
}

double atan_vel_func (double total_distance, double total_remaining, double maxVel, double tuning_constant)
{
  double correspondingVel = 0;
  // vel = A * atan(tuning_const * (x - total distance))
  double A = maxVel / atan(-tuning_constant * total_distance);  // x = 0
  correspondingVel = A * atan(-tuning_constant * total_remaining);

  return correspondingVel;
}

void moveToRefPose (double targetX, double targetY, double targetHeading, double linMax, double turnMax, double Kp_lin, double Kp_turn, double Kd_turn, double r)
{
  // initialization
  double startingVel = 0;
  double tolerance_end = 0.15;
  double maxTotalVel = 50;
  bool targetReached = false;
  bool nearEnd = false;

  // looking ahead
  double Kp_predict = 4;
  double prevVel;

  double prevTurnError = 0;
  // double total_turnAngle = 90;  // temp
  // bool nearEnd = false;
  // bool intermediatePtReached = false; 
  // double intermediateR = 1;
  // double tolerance = 0.5;
  // double intermediateX = 0.75; // targetX - intermediateR*cos(targetHeading *M_PI/180);
  // double intermediateY = 1; // targetY - intermediateR*sin(targetHeading *M_PI/180);

  do 
  {
    // get lin error
    double linearError = sqrt(pow((targetX-currentX), 2) + pow((targetY-currentY), 2));

    // intermediate direction ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // trying to look ahead
    //                     rpm   /s     cir        ft
    double maxLinVelFeet = 600 / 60 * M_PI * 2.75 /12;
    double stepDis = prevVel * maxLinVelFeet;  // percentage * maxSpeed
    double predictedX = currentX + stepDis * cos (currentHeading * M_PI/180);
    double predictedY = currentY + stepDis * sin (currentHeading * M_PI/180);

    // get abs target angle
    // double absTargetAngle = atan2 ((targetY-currentY), (targetX-currentX)) *180/M_PI;
    double absTargetAngle = atan2 ((targetY-predictedY), (targetX-predictedX)) *180/M_PI;
    if (absTargetAngle < 0)
    {
      absTargetAngle += 360;
    }

    double D = sqrt(pow((targetX-currentX), 2) + pow((targetY-currentY), 2));
    double alpha = findMinAngle(absTargetAngle, targetHeading);
    double beta = atan(r/D) *180/M_PI;

    if (alpha < 0)
    {
      beta = -beta;
    }

    // get turn error
    double turnError = 0;
    double errorTerm1 = findMinAngle(absTargetAngle, currentHeading);
    if (fabs(alpha) < fabs(beta))
    {
      // turnError = findMinAngle(errorTerm1, -alpha)
      turnError = errorTerm1 + alpha;
    }
    else
    {
      // turnError = findMinAngle(errorTerm1, -beta)
      turnError = errorTerm1 + beta;
    }
    // intermediate direction ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // intermidiate point ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // if ((sqrt(pow((currentX-intermediateX), 2) + pow((currentY-intermediateY), 2))) < tolerance)
    // {
    //   intermediatePtReached = true;
    // }
    // // get abs target angle
    // double absTargetAngle = 0;
    // if (intermediatePtReached)
    // {
    //   absTargetAngle = atan2 ((targetY-currentY), (targetX-currentX)) *180/M_PI;
    //   if (absTargetAngle < 0)
    //   {
    //     absTargetAngle += 360;
    //   }
    // }
    // else
    // {
    //   absTargetAngle = atan2 ((intermediateY-currentY), (intermediateX-currentX)) *180/M_PI;
    //   if (absTargetAngle < 0)
    //   {
    //     absTargetAngle += 360;
    //   }
    // }
    // // get heading error
    // double turnError = findMinAngle(absTargetAngle, currentHeading);
    // intermediate point ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // vel calculations
    startingVel += 0.2;
    double linVel = Kp_lin * linearError; // * sgn(cos(turnError *M_PI/180));  // in pct
    if (linVel > startingVel)
    {
      linVel = startingVel;
    }
    
    // where did this shit come from?
    // turnVel = Kp_turn * math.atan(math.tan(turnError *pi/180)) *180/pi  # in pct
    double turnVel = Kp_turn * turnError + Kd_turn * (turnError - prevTurnError);
    // double turnVel = atan_vel_func(total_turnAngle, turnError, turnMax, Kp_turn);

    // if near end
    if ((sqrt(pow((currentX-targetX), 2) + pow((currentY-targetY), 2)) < 0.5) && (fabs(errorTerm1) < 45))
    {
      nearEnd = true;
    }
    if (nearEnd)
    {
      linVel = Kp_lin * linearError * sgn(cos(turnError *M_PI/180));
      turnError = findMinAngle(targetHeading, currentHeading);
      turnVel = Kp_turn * atan(tan(turnError *M_PI/180)) *180/M_PI;
      // turnVel = atan_vel_func(total_turnAngle, turnError, turnMax, Kp_turn);
    }

    if ((sqrt(pow((currentX-targetX), 2) + pow((currentY-targetY), 2)) < tolerance_end) && (fabs(turnError)<1))
    {
      targetReached = true;
    }

    // limit max vel
    if (linVel > linMax)
    {
      linVel = linMax;
    }
    if (linVel < -linMax)
    {
      linVel = -linMax;
    }
    if (turnVel > turnMax)
    {
      turnVel = turnMax;
    }
    if (turnVel < -turnMax)
    {
      turnVel = -turnMax;
    }

    // prioritize turning
    if ((linVel + fabs(turnVel)) > maxTotalVel)
    {
      linVel = maxTotalVel - fabs(turnVel);
    }

    // execute
    moveWithAssignedSpeed(linVel, turnVel);

    // update
    prevTurnError = turnError;
    prevVel = linVel;
  }
  while (!targetReached);
  
  groupStopHold();
  wait(200, msec);
  groupStopCoast();
}

void followRefPath (const std::vector<std::vector<double>> &path, double targetAngle, double maxTotalVel, double linMax, double turnMax, double Kp_lin, double Kp_turn, double tune_turn, int numOfSeg)
{
  double robotWidth = 13.5/12; // feet
  int lastFoundIndex = 0;
  double lookAheadDis = 1;
  double endTol = 0.15;
  bool endNear = false;
  bool targetReached = false;
  do
  {
    bool foundIntersect = false;
    int startIndex = lastFoundIndex;
    double nextToFollow_x = path[lastFoundIndex][0];
    double nextToFollow_y = path[lastFoundIndex][1];

    // calculate remaining distance
    double remainingDis = sqrt (pow(currentX-path[path.size()-numOfSeg-2][0], 2) + pow(currentY-path[path.size()-numOfSeg-2][1], 2));
    if (lastFoundIndex < path.size()-numOfSeg-2)
    {
      remainingDis = sqrt (pow(currentX-path[lastFoundIndex+1][0], 2) + pow(currentY-path[lastFoundIndex+1][1], 2));
      for (int i = lastFoundIndex+1; i < path.size()-numOfSeg-2; i ++)
      {
        remainingDis += sqrt (pow(path[i][0]-path[i+1][0], 2) + pow(path[i][1]-path[i+1][1], 2));
      }
    }

    // there will be a 'buffer point' that is lookAheadDis away from the actual last point in path
    // when the last found index equals the index of the actual last point in path, the robot can still find intersection
    // but it's time to enter stopping condition
    if (lastFoundIndex == (path.size()-numOfSeg-3))
    {
      std::cout<<"end near"<<std::endl;
      endNear = true;
    }

    std::cout<<lastFoundIndex<<" "<<path.size()-numOfSeg-2<<std::endl;

    // initialize lin error
    double linVel = Kp_lin * remainingDis;
    if (linVel > linMax)
    {
      linVel = linMax;
    }

    // initialize turn error
    double absTargetAngle = targetAngle;
    double turnError = findMinAngle (absTargetAngle, currentHeading);
    double turnVel = Kp_turn * turnError;

    // if close enough to the target point
    if (endNear)
    {
      std::cout<<"end near"<<std::endl;
      moveToRefPose (path[path.size()-numOfSeg-2][0], path[path.size()-numOfSeg-2][1], targetAngle, linMax, turnMax, Kp_lin, Kp_turn, 1, 0.8);
      targetReached = true;
    }
    // when not near the end, follow pure pursuit procedure
    else 
    {
      // loop through path to find intersection
      for (int i = startIndex; i < (path.size()-numOfSeg-1); i ++)
      {
        double x1 = path[i][0] - currentX;
        double y1 = path[i][1] - currentY;
        double x2 = path[i+1][0] - currentX;
        double y2 = path[i+1][1] - currentY;
        double dx = x2 - x1;
        double dy = y2 - y1;
        double dr = sqrt (pow(dx, 2) + pow(dy, 2));
        double D = x1*y2 - x2*y1;
        double discriminant = pow(lookAheadDis, 2) * pow(dr, 2) - pow(D, 2);

        if (discriminant >= 0)
        {
          double sol_x1 = (D * dy + sgn(dy) * dx * sqrt(discriminant)) / pow(dr, 2);
          double sol_x2 = (D * dy - sgn(dy) * dx * sqrt(discriminant)) / pow(dr, 2);
          double sol_y1 = (- D * dx + fabs(dy) * sqrt(discriminant)) / pow(dr, 2);
          double sol_y2 = (- D * dx - fabs(dy) * sqrt(discriminant)) / pow(dr, 2);

          double sol_pt1_x = sol_x1 + currentX; 
          double sol_pt1_y = sol_y1 + currentY;
          double sol_pt2_x = sol_x2 + currentX; 
          double sol_pt2_y = sol_y2 + currentY;

          double minX = fmin(path[i][0], path[i+1][0]) - 0.01;
          double minY = fmin(path[i][1], path[i+1][1]) - 0.01;
          double maxX = fmax(path[i][0], path[i+1][0]) + 0.01;
          double maxY = fmax(path[i][1], path[i+1][1]) + 0.01;

          // check if solutions are in range
          // if (((minX <= sol_pt1_x <= maxX) && (minY <= sol_pt1_y <= maxY)) || ((minX <= sol_pt2_x <= maxX) && (minY <= sol_pt2_y <= maxY)))
          if ((((minX <= sol_pt1_x) && (sol_pt1_x <= maxX)) && ((minY <= sol_pt1_y) && (sol_pt1_y <= maxY)))
          || (((minX <= sol_pt2_x) && (sol_pt2_x <= maxX)) && ((minY <= sol_pt2_y) && (sol_pt2_y <= maxY))))
          {
            foundIntersect = true;
            // if the point is too far away from the last point found, it's probably cutting corners
            if (i - lastFoundIndex > 1)
            {
              lastFoundIndex += 1;
              nextToFollow_x = path[lastFoundIndex][0];
              nextToFollow_y = path[lastFoundIndex][1];
            }
            else
            {
              // if both solutions are in range, determine which one is better
              // if (((minX <= sol_pt1_x <= maxX) && (minY <= sol_pt1_y <= maxY)) && ((minX <= sol_pt2_x <= maxX) && (minY <= sol_pt2_y <= maxY)))
              if ((((minX <= sol_pt1_x) && (sol_pt1_x <= maxX)) && ((minY <= sol_pt1_y) && (sol_pt1_y <= maxY)))
              && (((minX <= sol_pt2_x) && (sol_pt2_x <= maxX)) && ((minY <= sol_pt2_y) && (sol_pt2_y <= maxY))))
              {
                lastFoundIndex = i;
                // take the intersection closer to the next point in path
                if (sqrt(pow(sol_pt1_x-path[i+1][0], 2) + pow(sol_pt1_y-path[i+1][1], 2)) < sqrt(pow(sol_pt2_x-path[i+1][0], 2) + pow(sol_pt2_y-path[i+1][1], 2)))
                {
                  nextToFollow_x = sol_pt1_x;
                  nextToFollow_y = sol_pt1_y;
                }
                else 
                {
                  nextToFollow_x = sol_pt2_x;
                  nextToFollow_y = sol_pt2_y;
                }
              }
              // if only one is in range, take that one
              else 
              {
                // if ((minX <= sol_pt1_x <= maxX) && (minY <= sol_pt1_y <=maxY))
                if (((minX <= sol_pt1_x) && (sol_pt1_x <= maxX)) && ((minY <= sol_pt1_y) && (sol_pt1_y <= maxY)))
                {
                  nextToFollow_x = sol_pt1_x;
                  nextToFollow_y = sol_pt1_y;
                }
                else 
                {
                  nextToFollow_x = sol_pt2_x;
                  nextToFollow_y = sol_pt2_y;
                }
              }
              // only exit the loop if the sol pt found is closer to the next point in path than the current pos
              if (sqrt(pow(nextToFollow_x-path[i+1][0], 2) + pow(nextToFollow_y-path[i+1][1], 2)) < sqrt(pow(currentX-path[i+1][0], 2) + pow(currentY-path[i+1][1], 2)))
              {
                break;
              }
              else 
              {
                lastFoundIndex = i + 1;
              }
            }
          }
          else 
          {
            foundIntersect = false;
            nextToFollow_x = path[lastFoundIndex][0];
            nextToFollow_y = path[lastFoundIndex][1];
          }
        }
      }

      absTargetAngle = atan2 ((nextToFollow_y-currentY), (nextToFollow_x-currentX)) *180/M_PI;
      if (absTargetAngle < 0)
      {
        absTargetAngle += 360;
      }
      // calculate heading error
      turnError = findMinAngle (absTargetAngle, currentHeading);

      // // old method
      // turnVel = Kp_turn * turnError;
      // if (fabs(turnVel) > turnMax)
      // {
      //   if (turnVel > 0)
      //   {
      //     turnVel = turnMax;
      //   }
      //   else
      //   {
      //     turnVel = -turnMax;
      //   }
      // }
      // // prioritize turning
      // if ((linVel + fabs(turnVel)) > maxTotalVel)
      // {
      //   linVel = maxTotalVel - fabs(turnVel);
      // }

      // new method
      double R = lookAheadDis / (2*sin(turnError *M_PI/180));
      turnVel = robotWidth/(2*R) * linVel * tune_turn;
      if (fabs(turnVel) > turnMax)
      {
        if (turnVel > turnMax) {turnVel = turnMax;}
        if (turnVel < -turnMax) {turnVel = -turnMax;}
      }
      // physical constraint
      if ((linVel + fabs(turnVel)) > maxTotalVel)
      {
        linVel = maxTotalVel / (fabs(robotWidth / (2*R))*tune_turn + 1);
        if (turnVel >= 0)
        {
          turnVel = maxTotalVel - linVel;
        }
        else
        {
          turnVel = - (maxTotalVel - linVel);
        }
      }

      moveWithAssignedSpeed(linVel, turnVel);
    }

    Brain.Screen.printAt (10, 20, "current x = %1.2f", currentX);
    Brain.Screen.printAt (10, 40, "current Y = %1.2f", currentY);
    Brain.Screen.printAt (10, 60, "current heading = %1.2f", currentHeading);
    Brain.Screen.printAt (10, 100, "next x to follow = %1.2f", nextToFollow_x);
    Brain.Screen.printAt (10, 120, "next y to follow = %1.2f", nextToFollow_y);
    Brain.Screen.printAt (10, 140, "turnError = %1.2f", turnError);
    Brain.Screen.printAt (10, 160, "remaining dis = %1.2f", remainingDis);
  }
  while (!targetReached);
}