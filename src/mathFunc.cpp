#include "vex.h"

using namespace vex;

//math functions
double findMinAngle (double firstAngle, double secondAngle){
  double minimunAngle = firstAngle - secondAngle;
  if(minimunAngle > 180){
    minimunAngle = firstAngle - 360 - secondAngle;
  }

  if(minimunAngle < -180){
    minimunAngle = firstAngle + (360 - secondAngle);
  }
  return minimunAngle;
}

double point2pointDis (double pt1[2], double pt2[2]){
  return sqrt( pow((pt2[0] - pt1[0]), 2) + pow((pt2[1] - pt1[1]), 2));
}

int sgn (double num)
{
  if (num < 0)
  {
    return -1;
  }
  else
  {
    return 1;
  }
}
