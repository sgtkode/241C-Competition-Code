#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  encoderL,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  encoderR,       sensorQuadEncoder)
#pragma config(Motor,  port1,           flyR1,         tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           flyR2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           flyL1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           flyL2,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           frontl,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           frontr,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           backl,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           backr,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           topIntake,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          bottomIntake,  tmotorVex393_HBridge, openLoop)
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
void pre_auton()
{
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

	// allow motors to be controller by variables
	startTask(runMotors);

	// position of bot on field
	int position = 2;

	if(position == 1){ // blue, net side

		spin_flywheel(0, 92, 300);
		wait1Msec(1500);
		driveByEncoder(1000, 6, 100);
		turnRightTicks(50, 90);
		wait1Msec(250);
		motor[bottomIntake] = 75;
		wait1Msec(500);
		motor[bottomIntake] = 0;
		wait1Msec(250);
		spin_flywheel(0, 0, 300);

		//forwardSeconds(1);
	} else if(position == 2){ // blue, enemy side

		spin_flywheel(0, 92, 300);
		wait1Msec(1500);
		driveByEncoder(1000, 6, 100);
		turnLeftTicks(62, 90);
		wait1Msec(250);
		motor[bottomIntake] = 75;
		wait1Msec(1000);
		motor[bottomIntake] = 0;
		wait1Msec(250);
		spin_flywheel(0, 0, 300);

	} else if(position == 3){ // red, net side

		spin_flywheel(0, 92, 300);
		wait1Msec(1500);
		driveByEncoder(1000, 6, 100);
		turnLeftTicks(50, 90);
		wait1Msec(250);
		motor[bottomIntake] = 75;
		wait1Msec(500);
		motor[bottomIntake] = 0;
		wait1Msec(250);
		spin_flywheel(0, 0, 300);

	} else if(position == 4){ // red, enemy side

		spin_flywheel(0, 92, 300);
		wait1Msec(1500);
		driveByEncoder(1000, 6, 100);
		turnRightTicks(50, 90);
		wait1Msec(250);
		motor[bottomIntake] = 75;
		wait1Msec(500);
		motor[bottomIntake] = 0;
		wait1Msec(250);
		spin_flywheel(0, 0, 300);

	}
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

	bool flywheelHalf = false;
	float initial = 0;

  while (true)
	{
    if(bVEXNETActive){


      /////////////////////////////////////////////////////////////////////////////////////////
      //
      //                                      Drive
      //
      /////////////////////////////////////////////////////////////////////////////////////////
  		if((SensorValue[encoderL] - SensorValue[encoderR]) > 5){
				motor[backr] = vexRT[Ch2];
	  		motor[frontr] = vexRT[Ch2];
	  		motor[backl] = vexRT[Ch3]*0.5;
	  		motor[frontl] = vexRT[Ch3]*0.5;
			} else if ((SensorValue[encoderR] - SensorValue[encoderL]) > 5){
				motor[backr] = vexRT[Ch2]*0.5;
	  		motor[frontr] = vexRT[Ch2]*0.5;
	  		motor[backl] = vexRT[Ch3];
	  		motor[frontl] = vexRT[Ch3];
			} else {
				motor[backr] = vexRT[Ch2];
	  		motor[frontr] = vexRT[Ch2];
	  		motor[backl] = vexRT[Ch3];
	  		motor[frontl] = vexRT[Ch3];
			}

			/////////////////////////////////////////////////////////////////////////////////////////
      //
      //                                      Flywheel
      //
      /////////////////////////////////////////////////////////////////////////////////////////
  		if(vexRT[Btn5U] == 1){
  			if(flywheelHalf){
  				spin_flywheel(40, 40, 300);
  				motor[topIntake] = 50;
  				flywheelHalf = false;
  			} else {
  				spin_flywheel(initial, 75, 300);
  				motor[topIntake] = 100;
  				flywheelHalf = true;
  				initial = 40;
  			}
  		}

  		if(vexRT[Btn5D] == 1){
  			motor[flyR1] = 0;
        motor[flyR2] = 0;
        motor[flyL1] = 0;
        motor[flyL2] = 0;
        motor[topIntake] = 0;
  		}

      if(vexRT[Btn6U] == 1){
  			motor[bottomIntake] = 100;
  		} else if (vexRT[Btn6D] == 1){
        motor[bottomIntake] = -100;
      } else {
        motor[bottomIntake] = 0;
      }

      wait1Msec(10);
    }
	}
}
