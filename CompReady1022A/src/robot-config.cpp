#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LeftMotorA = motor(PORT2, ratio18_1, true);
motor RightMotorA = motor(PORT11, ratio18_1, false);
motor ClawMotor = motor(PORT10, ratio36_1, false);
motor ArmMotor = motor(PORT9, ratio36_1, false);
motor LeftMotorB = motor(PORT20, ratio18_1, true);
motor RightMotorB = motor(PORT12, ratio18_1, false);
inertial Inertial15 = inertial(PORT15);
motor TowerIntake = motor(PORT7, ratio36_1, false);
motor Motor3 = motor(PORT3, ratio18_1, false);
motor backtower = motor(PORT5, ratio18_1, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}