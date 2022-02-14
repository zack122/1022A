/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LeftMotorA           motor         2               
// RightMotorA          motor         11              
// ClawMotor            motor         10              
// ArmMotor             motor         9               
// LeftMotorB           motor         20              
// RightMotorB          motor         12              
// Inertial15           inertial      15              
// TowerIntake          motor         7               
// Motor3               motor         3               
// backtower            motor         5               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath> //std::abs
using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
    Inertial15.calibrate();
  // waits for the Inertial Sensor to calibrate
  while (Inertial15.isCalibrating()) {
    wait(100, msec);
  };
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

//Settings
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;
int maxTurnIntegral = 300; // These cap the integrals
int maxIntegral = 300;
int integralBound = 3; //If error is outside the bounds, then apply the integral. This is a buffer with +-integralBound degrees

//Autonomous Settings
int desiredValue = 200;
int desiredTurnValue = 0;

int error; //SensorValue - DesiredValue : Position
int prevError = 0; //Position 20 miliseconds ago
int derivative; // error - prevError : Speed
int totalError = 0; //totalError = totalError + error

int turnError; //SensorValue - DesiredValue : Position
int turnPrevError = 0; //Position 20 miliseconds ago
int turnDerivative; // error - prevError : Speed
int turnTotalError = 0; //totalError = totalError + error

bool resetDriveSensors = false;


//Variables modified for use
bool enableDrivePID = true;

//Pasted from a C++ resource
double signnum_c(double x) {
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return x;
}


int drivePID(){
  
  while(enableDrivePID){

    if (resetDriveSensors) {
      resetDriveSensors = false;
      LeftMotorA.setPosition(0,degrees);
      RightMotorA.setPosition(0,degrees);
      LeftMotorB.setPosition(0,degrees);
      RightMotorB.setPosition(0,degrees);
    }


    //Get the position of both motors
    int leftMotorPosition = LeftMotorA.position(degrees);
    int rightMotorPosition = RightMotorA.position(degrees);
    int leftMotorBPosition = LeftMotorB.position(degrees);
    int rightMotorBPosition = RightMotorB.position(degrees);

    ///////////////////////////////////////////
    //Lateral movement PID
    /////////////////////////////////////////////////////////////////////
    //Get average of the motors
    int averagePosition = (leftMotorPosition + rightMotorPosition + rightMotorBPosition + leftMotorBPosition)/2;

    //Potential
    error = averagePosition - desiredValue;

    //Derivative
    derivative = error - prevError;

      //Integral
    if(abs(error) < integralBound){
    totalError+=error; 
    }  else {
    totalError = 0; 
    }
    //totalError += error;

    //This would cap the integral
    totalError = abs(totalError) > maxIntegral ? signnum_c(totalError) * maxIntegral : totalError;

    double lateralMotorPower = error * kP + derivative * kD + totalError * kI;
    /////////////////////////////////////////////////////////////////////


    ///////////////////////////////////////////
    //Turning movement PID
    /////////////////////////////////////////////////////////////////////
    //Get average of the two motors
    int turnDifference = leftMotorPosition + leftMotorBPosition - rightMotorPosition + rightMotorBPosition;

    //Potential
    turnError = turnDifference - desiredTurnValue;

    //Derivative
    turnDerivative = turnError - turnPrevError;

     //Integral
    if(abs(error) < integralBound){
    turnTotalError+=turnError; 
    }  else {
    turnTotalError = 0; 
    }
    //turnTotalError += turnError;

    //This would cap the integral
    turnTotalError = abs(turnTotalError) > maxIntegral ? signnum_c(turnTotalError) * maxIntegral : turnTotalError;

    double turnMotorPower = turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI;
    /////////////////////////////////////////////////////////////////////

    LeftMotorA.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    LeftMotorB.spin(forward, lateralMotorPower + turnMotorPower, voltageUnits::volt);
    RightMotorB.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);
    RightMotorA.spin(forward, lateralMotorPower - turnMotorPower, voltageUnits::volt);

    

    prevError = error;
    turnPrevError = turnError;
    vex::task::sleep(20);

  }

  return 1;
}


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {

//  vex::task billWiTheScienceFi(drivePID);

//   resetDriveSensors = true;
//   desiredValue = 300;
//   desiredTurnValue = 600;

//   vex::task::sleep(1000);

//   resetDriveSensors = true;
//   desiredValue = 300;
//   desiredTurnValue = 300;


  LeftMotorA.setVelocity(100, velocityUnits::pct);
  RightMotorA.setVelocity(100, velocityUnits::pct);
  LeftMotorB.setVelocity(100, velocityUnits::pct);
  RightMotorB.setVelocity(100, velocityUnits::pct);

  ClawMotor.spinToPosition(-1000, rotationUnits::deg, false);
 
  LeftMotorA.spin(directionType::fwd);
  RightMotorA.spin(directionType::fwd);
  LeftMotorB.spin(directionType::fwd);
  RightMotorB.spin(directionType::fwd);

  task::sleep(800);
 
  LeftMotorB.stop();
  LeftMotorA.stop();
  RightMotorB.stop();
  RightMotorA.stop();
 

  wait(1,seconds);

  ClawMotor.spinToPosition(0, rotationUnits::deg, false);

  LeftMotorA.spin(directionType::rev);
  RightMotorA.spin(directionType::rev);
  LeftMotorB.spin(directionType::rev);
  RightMotorB.spin(directionType::rev);

  task::sleep(500);

  LeftMotorB.stop();
  LeftMotorA.stop();
  RightMotorB.stop();
  RightMotorA.stop();




  LeftMotorA.spin(directionType::fwd);
  RightMotorA.spin(directionType::rev);
  LeftMotorB.spin(directionType::fwd);
  RightMotorB.spin(directionType::rev);
  
  // Waits until the motor reaches a 90 degree turn and stops the Left and
  // Right Motors.
  waitUntil((Inertial15.rotation(degrees) >= 90));
  LeftMotorB.stop();
  LeftMotorA.stop();
  RightMotorB.stop();
  RightMotorA.stop();

  ClawMotor.spinToPosition(-1000, rotationUnits::deg, false);

  LeftMotorA.spin(directionType::fwd);
  RightMotorA.spin(directionType::rev);
  LeftMotorB.spin(directionType::fwd);
  RightMotorB.spin(directionType::rev);
  
  // Waits until the motor reaches a 90 degree turn and stops the Left and
  // Right Motors.
  waitUntil((Inertial15.rotation(degrees) >= 312));
  LeftMotorB.stop();
  LeftMotorA.stop();
  RightMotorB.stop();
  RightMotorA.stop();


LeftMotorA.spin(directionType::fwd);
  RightMotorA.spin(directionType::fwd);
  LeftMotorB.spin(directionType::fwd);
  RightMotorB.spin(directionType::fwd);

  task::sleep(1000);
 
  LeftMotorB.stop();
  LeftMotorA.stop();
  RightMotorB.stop();
  RightMotorA.stop();
 

  wait(1,seconds);

  ClawMotor.spinToPosition(0, rotationUnits::deg, false);
  
 LeftMotorA.spin(directionType::rev);
  RightMotorA.spin(directionType::rev);
  LeftMotorB.spin(directionType::rev);
  RightMotorB.spin(directionType::rev);

  task::sleep(1200);

  LeftMotorB.stop();
  LeftMotorA.stop();
  RightMotorB.stop();
  RightMotorA.stop();




  // LeftMotorA.spin(directionType::fwd);
  // RightMotorA.spin(directionType::rev);
  // LeftMotorB.spin(directionType::fwd);
  // RightMotorB.spin(directionType::rev);
  
  // // Waits until the motor reaches a 90 degree turn and stops the Left and
  // // Right Motors.
  // waitUntil((Inertial15.rotation(degrees) >= 90));
  // LeftMotorB.stop();
  // LeftMotorA.stop();
  // RightMotorB.stop();
  // RightMotorA.stop();

  
  // LeftMotorA.spin(directionType::fwd);
  // RightMotorA.spin(directionType::fwd);
  // LeftMotorB.spin(directionType::fwd);
  // RightMotorB.spin(directionType::fwd);

  // task::sleep(600);
 
  // LeftMotorB.stop();
  // LeftMotorA.stop();
  // RightMotorB.stop();
  // RightMotorA.stop();
  

  // LeftMotorA.spin(directionType::fwd);
  // RightMotorA.spin(directionType::fwd);
  // LeftMotorB.spin(directionType::fwd);
  // RightMotorB.spin(directionType::fwd);

//   LeftMotorA.setVelocity(40, velocityUnits::pct);
//   RightMotorA.setVelocity(40, velocityUnits::pct);
//   LeftMotorB.setVelocity(40, velocityUnits::pct);
//   RightMotorB.setVelocity(40, velocityUnits::pct);




//   task::sleep(250);
  
//  wait(2, seconds);
 
//  ClawMotor.spinToPosition(0, rotationUnits::deg, false);

//   LeftMotorA.spin(directionType::rev);
//   RightMotorA.spin(directionType::rev);
//   LeftMotorB.spin(directionType::rev);
//   RightMotorB.spin(directionType::rev);

























// LeftMotorA.spin(directionType::fwd);
//   RightMotorA.spin(directionType::fwd);
//   LeftMotorB.spin(directionType::fwd);
//   RightMotorB.spin(directionType::fwd);


//   task::sleep(3000);

  
//   LeftMotorA.spin(directionType::fwd);
//   RightMotorA.spin(directionType::fwd);
//   LeftMotorB.spin(directionType::rev);
//   RightMotorB.spin(directionType::rev);
//   // Waits until the motor reaches a 90 degree turn and stops the Left and
//   // Right Motors.
//   waitUntil((Inertial15.rotation(degrees) >= 70.0));
//   LeftMotorB.stop();
//   LeftMotorA.stop();
//   RightMotorB.stop();
//   RightMotorA.stop();


//   LeftMotorA.spin(directionType::fwd);
//   RightMotorA.spin(directionType::fwd);
//   LeftMotorB.spin(directionType::fwd);
//   RightMotorB.spin(directionType::fwd);


//    task::sleep(3000);

  // task::sleep(3000);

  // LeftMotorA.stop();
  // RightMotorA.stop();
  // LeftMotorB.stop();
  // RightMotorB.stop();

    

  // LeftMotorA.setVelocity(30, velocityUnits::pct);
  // RightMotorA.setVelocity(30, velocityUnits::pct);
  // LeftMotorB.setVelocity(30, velocityUnits::pct);
  // RightMotorB.setVelocity(30, velocityUnits::pct);

  
  // TowerIntake.spin(directionType::fwd);





  // task::sleep(500);


  // LeftMotorA.stop();
  // RightMotorA.stop();
  // LeftMotorB.stop();
  // RightMotorB.stop();

  // TowerIntake.spin(directionType::rev);


  // wait(2, seconds);

//  LeftMotorA.spin(directionType::fwd);
//   RightMotorA.spin(directionType::fwd);
//   LeftMotorB.spin(directionType::rev);
//   RightMotorB.spin(directionType::rev);
//   // Waits until the motor reaches a 90 degree turn and stops the Left and
//   // Right Motors.
//   waitUntil((Inertial15.rotation(degrees) >= 70.0));
//   LeftMotorB.stop();
//   LeftMotorA.stop();
//   RightMotorB.stop();
//   RightMotorA.stop();



  // RightMotorB.spin(forward);
  // RightMotorA.spin(forward);
  // LeftMotorA.spin(reverse);
  // LeftMotorB.spin(reverse);
  // // Waits until the motor reaches a 90 degree turn and stops the Left and
  // // Right Motors.
  // waitUntil((Inertial15.rotation(degrees) >= 70.0));
  // LeftMotorB.stop();
  // LeftMotorA.stop();
  // RightMotorB.stop();
  // RightMotorA.stop();

// ClawMotor.spinToPosition(-360, rotationUnits::deg, false);

//   LeftMotorA.spin(directionType::fwd);
//   RightMotorA.spin(directionType::fwd);
//   LeftMotorB.spin(directionType::fwd);
//   RightMotorB.spin(directionType::fwd);

//   task::sleep(1500);


  // ClawMotor.spinToPosition(0, rotationUnits::deg, false);
  


  
  // LeftMotorA.spin(directionType::fwd);
  // RightMotorA.spin(directionType::fwd);
  // LeftMotorB.spin(directionType::fwd);
  // RightMotorB.spin(directionType::fwd);

  // task::sleep(3000);

  // LeftMotorA.stop();
  // RightMotorA.stop();
  // LeftMotorB.stop();
  // RightMotorB.stop();

  







  Controller1.Screen.print("Autonomous has Ended.");
  Controller1.rumble(".-.-");
  

  

}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

  enableDrivePID = false;

  ///////////////////////////
  //Settings
  ///////////////////////////////////////////////////////////////////////////

  //Drivetrain
  double turnImportance = 0.5;
  ArmMotor.setBrake(hold);
  TowerIntake.setBrake(hold);
  ClawMotor.setBrake(hold);
 
  
  while (1) {
  
    ///////////////////////////
    //Driver Control
    ///////////////////////////////////////////////////////////////////////////
    double turnVal = Controller1.Axis1.position(percent);
    double forwardVal = Controller1.Axis3.position(percent);

    double turnVolts = turnVal * 0.12;
    double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);

    //0 - 12 = -12
    //0 + 12 = 12(due to cap)


    LeftMotorA.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
        LeftMotorB.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);

    RightMotorA.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
        RightMotorB.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);

    ///////////////////////////////////////////////////////////////////////////


    ///////////////////////////
    //Arm Control
    ///////////////////////////////////////////////////////////////////////////
    bool topRightButton = Controller1.ButtonR1.pressing();
    bool bottomRightButton = Controller1.ButtonR2.pressing();
 
    if (topRightButton){
      ArmMotor.spin(forward, 12.0, voltageUnits::volt);
    }
    else if (bottomRightButton){
      ArmMotor.spin(forward, -12.0, voltageUnits::volt);
    }
    else{
      ArmMotor.spin(forward, 0, voltageUnits::volt);
    }
    ///////////////////////////////////////////////////////////////////////////

    ///////////////////////////
    //Claw Control
    ///////////////////////////////////////////////////////////////////////////
    bool topLeftButton = Controller1.ButtonL1.pressing();
    bool bottomLeftButton = Controller1.ButtonL2.pressing();
       

    if (topLeftButton){
      ClawMotor.spin(forward, 12.0, voltageUnits::volt);
    }
    else if (bottomLeftButton){
      ClawMotor.spin(forward, -12.0, voltageUnits::volt);
    }
    else{
      ClawMotor.spin(forward, 0, voltageUnits::volt);
    }
    ///////////////////////////////////////////////////////////////////////////


    ///////////////////////////
    //Tower Intake Control
    ///////////////////////////////////////////////////////////////////////////
    bool Buttonb = Controller1.ButtonB.pressing();
    bool Buttony = Controller1.ButtonY.pressing();

    if (Buttonb){
      TowerIntake.spin(forward, 12.0, voltageUnits::volt);
    }
    else if (Buttony){
      TowerIntake.spin(forward, -12.0, voltageUnits::volt);
    }
    else{
      TowerIntake.spin(forward, 0, voltageUnits::volt);
    }
    ///////////////////////////////////////////////////////////////////////////

     ///////////////////////////
    //Back Tower Intake Control
    ///////////////////////////////////////////////////////////////////////////
    bool Buttonup = Controller1.ButtonUp.pressing();
    bool Buttondown = Controller1.ButtonDown.pressing();

    if (Buttonup){
      backtower.spin(forward, 12.0, voltageUnits::volt);
    }
    else if (Buttondown){
      backtower.spin(forward, -12.0, voltageUnits::volt);
    }
    else{
      backtower.spin(forward, 0, voltageUnits::volt);
    }
    ///////////////////////////////////////////////////////////////////////////



    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
