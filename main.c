#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  encoder,        sensorQuadEncoder)
#pragma config(Motor,  port2,           backl,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           frontl,        tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           backr,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port9,           frontr,        tmotorVex393_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/**
 * @file    main.c
 * @brief   This file contains the base for competiton code.
 * @details The three sections include Pre-Auton, Auton, and User
 *          Control.
 *
 * @author		Sean Kelley      sgtkode01@gmail.com
 * @author		Bernard Suwirjo  bsuwirjo@gmail.com
 * @author 		Spencer Couture  spence.couture@gmail.com
 */


#pragma platform(VEX)

//Competition Control and Duration Settings
#pragma competitionControl(Competition)
#pragma autonomousDuration(15)
#pragma userControlDuration(120)

//Main competition background code...do not modify!
#include "Vex_Competition_Includes.c"

// all functions for competition code
#include "functions.c"



/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Pre-Autonomous
//
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * Period before autonomous when bot cannot move, but minimal code can run
 */
void pre_auton(){
	bStopTasksBetweenModes = true;

}



/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Autonomous
//
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * Period of match when bot is using only code to operate.
 */
task autonomous(){
		forwardSeconds(2, 50);
		turnRightDegrees(60, 30);
}



/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 User Control Task
//
/////////////////////////////////////////////////////////////////////////////////////////

/**
 * Period of match when driver controls the bot
 */
task usercontrol(){


  while (true)
	{
    if(bVEXNETActive){


      /////////////////////////////////////////////////////////////////////////////////////////
      //
      //                                      Drive
      //
      /////////////////////////////////////////////////////////////////////////////////////////
  		motor[backr] = vexRT[Ch2];
  		motor[frontr] = vexRT[Ch2];
  		motor[backl] = vexRT[Ch3];
  		motor[frontl] = vexRT[Ch3];

      wait1Msec(10);
    }
	}
}
