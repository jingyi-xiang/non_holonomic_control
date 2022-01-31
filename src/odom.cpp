#include "vex.h"
#include "mathFunc.h"

using namespace vex;

//position update
const double pi = M_PI;
double currentHeading = 90;
double previousHeading = 90;
double currentHeading_rad = currentHeading *pi/180;
double changeInHeading = 0;
double changeInHeading_rad = 0;
double scaleFactor_heading = 1.009;
// 0.98901098901098

double wheelDiameter = 2.75;
double scaleFactor = 1; //0.9987378640776699;
double backToCenterDistance = 0.0;

double currentLeft = 0;
double currentRight = 0;
double currentBack = 0;
double changeInLeft = 0;
double changeInRight = 0;
double changeInBack = 0;
double previousLeft = 0;
double previousRight = 0;
double previousBack = 0;

double localChangeInX = 0;
double localChangeInY = 0;
double globalChangeInX = 0;
double globalChangeInY = 0;

double currentX = 0;
double currentY = 0;
double previousX = currentX;
double previousY = currentY;

double initHeading = 90;

double sideToCenterDistance = 4.0/12.0;

// the inertial sensor reads slightly less than 1 for each degree
double heading_correction (double currentRotation){
  double correctedHeading = fmod((currentRotation * scaleFactor_heading), 360) + initHeading;

  if (correctedHeading > 360){
    correctedHeading = fmod(correctedHeading, 360);
  }

  if (correctedHeading < 0){
    correctedHeading = 360 + correctedHeading;
  }

  return correctedHeading;
}

// odom code (position update)
void position_update (){

  wheelDiameter = wheelDiameter * scaleFactor;

  // encoderLeft.resetRotation();
  encoderMiddle.resetPosition();
  encoderRight.resetRotation();

  currentHeading = heading_correction(-1 * inertial20.rotation());
  previousHeading = currentHeading;

  while(1){

    // correct heading
    currentHeading = heading_correction(-1 * inertial20.rotation());

    // local coordinate
    // heading correction. turning from 1 to 359 causes problem without correction because (359-1)>180 which is non sense
    changeInHeading = findMinAngle(currentHeading, previousHeading);
      
    // change in heading convert to radian. all trig functions use radian
    changeInHeading_rad = changeInHeading * pi / 180;
      
    // get current tracking wheel locations (in degrees)
    // currentLeft = encoderLeft.rotation(deg);
    currentRight = encoderRight.rotation(deg);
    currentBack = encoderMiddle.position(deg);
      
    // convert to distance unit (feet)
    // changeInLeft = ( currentLeft - previousLeft ) / 360 * pi * wheelDiameter / 12;
    changeInRight = ( currentRight - previousRight ) / 360 * pi * wheelDiameter / 12;
    changeInBack = ( currentBack - previousBack ) / 360 * pi * wheelDiameter / 12;
      
    // prevent divide by zero. if heading doesn't change, local x and y changes are simply encoder readings without trig
    if ( changeInHeading_rad == 0 ) {
      localChangeInX = changeInBack;
      localChangeInY = ( changeInRight + changeInLeft ) / 2;
    }
    // compute local changes (local coordinate system)
    else {
      localChangeInY = 2 * sin ( changeInHeading_rad / 2 ) * ( changeInRight / changeInHeading_rad - sideToCenterDistance );
      // since back tracking wheel is not at center, need to offset it
      localChangeInX = 2 * sin ( changeInHeading_rad / 2 ) * ( changeInBack / changeInHeading_rad - backToCenterDistance );
    }
      
    // convert to global coordinate
    currentHeading_rad = currentHeading * pi / 180;
    globalChangeInX = localChangeInY * cos ( currentHeading_rad - changeInHeading_rad / 2 ) + localChangeInX * sin ( currentHeading_rad - changeInHeading_rad / 2 );
    globalChangeInY = localChangeInY * sin ( currentHeading_rad - changeInHeading_rad / 2 ) - localChangeInX * cos ( currentHeading_rad - changeInHeading_rad / 2 );
    currentX = previousX + globalChangeInX;
    currentY = previousY + globalChangeInY;

    // not using this method anymore
    // offset the tracking center so that it's at the center of the robot
    // the new tracking center should have coordinate (currentX, currentY + 0.5) in local coordinate
    /***
    local to global conversion:
    x = y'*cos(theta) + x'*sin(theta)
    y = y'*sin(theta) - x'*cos(theta)
    ***/
    // therefore, when covert to global, the coordinate should be (currentX + 0.5*cos(heading), currentY + 0.5*sin(heading))
      
    // store values
    previousHeading = currentHeading;
    // previousLeft = currentLeft;
    previousRight = currentRight;
    previousBack = currentBack;
    previousX = currentX;
    previousY = currentY;

    // if(display){
    //   
    // }
    task::sleep(10);
  }
}

void manual_tune ( void ){
  Brain.Screen.setFont(fontType::mono20);
  Brain.Screen.printAt(10, 40, "x = %1.2f", currentX);
  Brain.Screen.printAt(10, 60, "y = %1.2f", currentY);

  Brain.Screen.printAt(10, 100,"heading = %1.2f", currentHeading);
  Brain.Screen.printAt(10, 140, "inertial.heading() = %1.2f", inertial20.heading());
  Brain.Screen.printAt(10, 160, "inertial.rotation() = %1.2f", inertial20.rotation());
  Brain.Screen.printAt(10, 180, "heading not corrected = %1.2f", 360 - inertial20.heading());
}