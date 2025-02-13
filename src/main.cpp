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
