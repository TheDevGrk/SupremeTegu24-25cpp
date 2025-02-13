/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       GrkDev                                                     */
/*    Created:      2/9/2025, 11:39:06 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here
brain Brain;

// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller           controller                    
// RightFront           motor         3               
// RightBack            motor         4              
// LeftFront            motor         2               
// LeftBack             motor         14   
// Intake               motor         10
// Conveyor             motor         8
// Grabber              piston        A (Three wire port)

controller Controller;

motor RightFront = motor(PORT3, ratio18_1, false);
motor RightBack = motor(PORT4, ratio18_1, false);
motor LeftFront = motor(PORT2, ratio18_1, false);
motor LeftBack = motor(PORT14, ratio18_1, false);

motor Intake = motor(PORT10, ratio18_1, true);
motor Conveyor = motor(PORT8, ratio18_1, true);

motor_group RightSide = motor_group(RightFront, RightBack);
motor_group LeftSide = motor_group(LeftFront, LeftBack);

pneumatics Grabber = pneumatics(Brain.ThreeWirePort.A);

optical ColorSensor = optical(PORT16);



/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void openGrabber(){
  Grabber.open();
}
void closeGrabber(){
  Grabber.close();
}

void intakeForward(){
  Intake.spin(forward, 12, volt);
  Conveyor.spin(forward, 12, volt);
}
void intakeReverse(){
  Intake.spin(reverse, 12, volt);
  Conveyor.spin(reverse, 12, volt);
}
void intakeStop(){
  Intake.stop();
  Conveyor.stop();
}

void detectColor(){
  ColorSensor.setLightPower(100, percent);
  Brain.Screen.print(ColorSensor.color());

}

void pre_auton(void) {
  // TODO (idea) add way to change controls by adding a program to the controller that allows you
  // TODO to change the controls by displaying the control and then letting you press the button
  // TODO that you want to be mapped to that control
  // TODO also would allow you to move the joystick on the axis you want to be drive and turn
  // TODO and let you set a profile name (maybe)
  // TODO would save to SD card (have to figure this out still)
  // TODO all of the programs would use the "active" profile from the SD card
  // TODO so it would still just be the 3 programs and then 1 at the end to change the controls

  // Set callback functions for all buttons to control parts of the robot
  Controller.ButtonR1.pressed(openGrabber);
  Controller.ButtonR2.pressed(closeGrabber);

  Controller.ButtonL2.pressed(intakeForward);
  Controller.ButtonL2.released(intakeStop);
  Controller.ButtonL1.pressed(intakeReverse);
  Controller.ButtonL1.released(intakeStop);

  Controller.ButtonA.pressed(detectColor);
}

// Global variables the modify drivePID
bool resetDriveSenors = false;
int desiredDistance;
int desiredTurn;
bool enableDrivePID = true;

// Essentially returns the "sign" (not to be confused with sine) of x, if it's negative -1, positive 1 (+1), 0 returns 0
// In order to preserve the sign of a number
double signFunction(double x) {
  if (x > 0.0) return 1.0;
  if (x < 0.0) return -1.0;
  return x;
}

// Function to use PID when driving tasks are needed
void drivePID(){
  // Define constants for use in PID
  double kP = 0.5;
  double kI = 0.05;
  double kD = 0.05;

  double turnkP = 0.5;
  double turnkI = 0.05;
  double turnkD = 0.05;

  // Integral caps
  int maxIntegral = 300;
  int maxTurnIntegral = 300;
  int integralBound = 3;

  // Define variable that can change
  int error;
  int previousError = 0;
  int totalError = 0;
  int derivative;

  int turnError;
  int turnPreviousError = 0;
  int turnTotalError = 0;
  int turnDerivative;

  while (enableDrivePID){
    if (resetDriveSenors){
      resetDriveSenors = false;
      LeftSide.setPosition(0.0, degrees);
      RightSide.setPosition(0.0, degrees);
    }

    // Finds the current rotations of all the drivetrain motors
    int leftPosition = LeftSide.position(degrees);
    int rightPosition = RightSide.position(degrees);

    // -----------------------------------------
    // Driving (lateral) PID
    // -----------------------------------------

    int averagePosition = (leftPosition + rightPosition) / 2;

    // Potential (P)
    error = averagePosition - desiredDistance;

    // Integral (I)
    if (abs(error) < integralBound){
      totalError += error;
    }
    else{
      totalError = 0;
    }

    totalError = abs(totalError) > maxIntegral ? signFunction(totalError) * maxIntegral : totalError;

    // Derrivative (D)
    derivative = error - previousError;

    // Calculate volts based on constants and PID
    double lateralVolts = (error * kP) + (derivative * kD) + (totalError * kI);

    // -----------------------------------------
    // Turning PID
    // -----------------------------------------
  
    int turnDifference = leftPosition - rightPosition;

    // TODO add inertial sensor and make turnDifference the value of the inertial sensor
    
    // Potential (P)
    turnError = turnDifference - desiredTurn;
    
    // Integral (I)
    if (abs(turnError) < integralBound){
      turnTotalError += turnError;
    }
    else{
      turnTotalError = 0;
    }

    turnTotalError = abs(turnTotalError) > maxTurnIntegral ? signFunction(turnTotalError) * maxTurnIntegral : turnTotalError;

    // Derrivative (D)
    turnDerivative = turnError - turnPreviousError;

    // Calculate volts based on constants and PID
    double turnVolts = (turnError * turnkP) + (turnDerivative * turnkD) + (turnTotalError * turnkI);

    // Apply calculated voltage to drive train
    LeftSide.spin(forward, lateralVolts + turnVolts, volt);
    RightSide.spin(forward, lateralVolts - turnVolts, volt);

    previousError = error;
    turnPreviousError = turnError;
    task::sleep(20);
  }

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
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Actions on the robot are executed depending on input
    // Such as the position of the joysticks and buttons that are pressed

    // how much priority turns take
    int turnImportance = 0.5;
    int drivePosition = Controller.Axis1.position();
    int turnPosition = Controller.Axis3.position();

    int turnVolts = turnPosition * 0.12;
    int driveVolts = drivePosition * 0.12 * (1 - (std::abs(turnVolts / 12) * turnImportance));

    LeftSide.spin(forward, driveVolts + turnVolts, volt);
    RightSide.spin(forward, driveVolts - turnVolts, volt);

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
