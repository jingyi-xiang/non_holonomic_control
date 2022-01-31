using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern inertial inertial20;
extern encoder encoderRight;
extern motor leftFront;
extern motor leftBack;
extern motor rightFront;
extern motor rightBack;
extern rotation encoderMiddle;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );