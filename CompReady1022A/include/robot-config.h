using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LeftMotorA;
extern motor RightMotorA;
extern motor ClawMotor;
extern motor ArmMotor;
extern motor LeftMotorB;
extern motor RightMotorB;
extern inertial Inertial15;
extern motor TowerIntake;
extern motor Motor3;
extern motor backtower;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );