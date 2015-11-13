/**
 * @file functions.c
 * @brief This file contains all functions to be used by
 * the competition code.
 *
 * @author	 Sean Kelley      sgtkode01@gmail.com
 *
 */

#include "main.h"

 /////////////////////////////////////////////////////////////////////////////////////////
//
//                                 CONSTANTS
//
/////////////////////////////////////////////////////////////////////////////////////////

#define TIMEOUT_CNT_PER_SEC    10   /*!< amount of timeout counts per second */
#define MOTOR_SPEED        		 118  /*!< default motor speed */

int frontLeftVal  = 0; /*!< value of the front left  motor */
int backLeftVal   = 0; /*!< value of the back  left  motor */
int frontRightVal = 0; /*!< value of the front right motor */
int backRightVal  = 0; /*!< value of the back  right motor */


/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 DRIVETRAIN
//
/////////////////////////////////////////////////////////////////////////////////////////

/**
* Clears all the motors
*
* @author  Bernard Suwirjo  bsuwirjo@gmail.com
* @author	 Sean Kelley      sgtkode01@gmail.com
*/
void clearMotors(){
	//Set all motor values to 0
  motorSet(frontl,  0);
	motorSet(backl,  0);
	motorSet(frontr,  0);
	motorSet(backr,  0);
}

/**
* Sets all motors to a certain value
*
* @author Sean Kelley      sgtkode01@gmail.com
*
* @param  speed   the speed of the motors
*/
void setMotors(int speed){
	//Set all motor values to power value
	motorSet(frontl,  speed);
	motorSet(backl,  speed);
	motorSet(frontr,  speed);
	motorSet(backr,  speed);
}

/**
* Sets the left motors to a given speed
*
* @author Spencer Couture  spence.couture@gmail.com
* @author Sean Kelley  sgtkode01@gmail.com
*
* @param  speed   the speed that the left motors will be set to
*
*/
void setMotorsLeft(int speed){
	motorSet(frontl,  speed);
	motorSet(backl,  speed);
}

/**
* Sets the right motors to a given speed
*
* @author Spencer Couture  spence.couture@gmail.com
* @author Sean Kelley  sgtkode01@gmail.com
*
* @param  speed   the speed that the right motors will be set to
*
*/
void setMotorsRight(int speed){
	motorSet(frontr,  speed);
	motorSet(backr,  speed);
}

/**
* Runs each motor for 1.5 seconds
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
*/
void testMotors()
{
	motorSet(frontr,  118);//Set individual motor
	wait1Msec(1500); //Wait 1.5 seconds
	clearMotors(); //clear motor(s)
	motorSet(backr,  118);
	wait1Msec(1500);
	clearMotors();
	motorSet(frontl,  118);
	wait1Msec(1500);
	clearMotors();
	motorSet(backl,  118);
	wait1Msec(1500);
	clearMotors();
}

/**
* Moves bot forward for a given amount of seconds
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
*
* @param  seconds  amount of seconds to move forward
*	@param  speed    speed of motors
*
*/
void forwardSeconds(float seconds, int speed=MOTOR_SPEED)
{
	//Set all motors to target value
	setMotors(speed);
	wait1Msec(seconds * 1000);//Wait given amount of time
	clearMotors();
}

/**
* Moves bot backward for a given amount of seconds
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
*
* @param  seconds  amount of seconds to move backward
*	@param  speed    speed of motors
*
*/
void backwardSeconds(float seconds, int speed=MOTOR_SPEED)
{
	//Set all motors to negative target value
	setMotors(-speed);
	wait1Msec(seconds * 1000);//Wait given amount of time
	clearMotors();
}

/**
* Drive until an encoder value is reached
*
* @author Sean Kelley		sgtkode01@gmail.com
*
* @param  encoder_count         encoder ticks to drive forward
*	@param  timeout_in_seconds    timeout for motors if encoder value is surpassed
* @param  speed									speed of motors
*
*/
int driveByEncoder( int encoder_count, int timeout_in_seconds = 5 , int speed=MOTOR_SPEED){
	int  timeout;

	// Drive motor until encoder has moved a number counts or
	// timeout_in_seconds seconds have passed

	// Zero the encoder
	encoderReset(encoder);

	// Run the motor forwards or backwards
	if( encoder_count > 0 ){
		setMotors(speed);
	} else {
		setMotors(-speed);
	}

	// run loop until encoder hits encoder_count counts or timeout reached

	for( timeout=(timeout_in_seconds*TIMEOUT_CNT_PER_SEC); timeout > 0; timeout-- ){
		// check encoder
		if( encoder_count > 0 ){
			// going forwards
			if( encoderGet(encoder) >= encoder_count ){
				break;
			} else {
			// going backwards
				if( encoderGet(encoder) <= encoder_count ){
					break;
				}
			}
		}

		// wait 1/10 second
		wait1Msec( 100 );
	}

	// Stop the motor
	clearMotors();

	// See if we sucessfully found the right encoder value
	if( timeout <= 0 ){
		// there was an error - perhaps do something
		// return error
		return (-1);
	} else {
		// return success
		return 0;
	}
}

/**
*
* Locks left side motors with PI loop
*
* @warning function does not work
*
* @author Sean Kelley  sgtkode01@gmail.com
*
*//*
task lockLeftSide()
{
	//float target = 0;
	//float pGain = .3;
	//float iGain = .02;
	//float error = target-SensorValue[encoderLeft];
	//float errorSum=0;
	while(1==1){
/*		error=target-SensorValue[encoderLeft];
		errorSum+=error;
		motor[FL] = error*pGain+errorSum*iGain;
		  motor[BL] = error*pGain+errorSum*iGain;
	}
}*/

/**
*
* Locks right side motors with PI loop
*
* @warning function does not work
*
* @author Sean Kelley  sgtkode01@gmail.com
*
*//*
task lockRightSide()
{
	//float target = 0;
	//float pGain = .3;
	//float iGain = .02;
	//float error = target-SensorValue[encoderRight];
	//float errorSum=0;
	while(true){
	/*error=target-SensorValue[encoderRight];
		errorSum+=error;
		motor[FR] = error*pGain+errorSum*iGain;
	  motor[BR] = error*pGain+errorSum*iGain;
	}
}*/

/**
* Turns bot right only using left side motors
*
* @warning requires gyro
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
* @author Sean Kelley  sgtkode01@gmail.com
*
* @param   degrees   amount of degrees to turn right
* @param   forward   boolean if bot is turning forward or backward
* @param   speed     speed of motors
*
*/
void fancyTurnRightDegrees(int degrees, bool forward=true, int speed = MOTOR_SPEED){

	// reset encoders
	degrees=degrees*10;
	// reset gyro
	//gyro takes degrees from 0-3600, so we multiply by 10 to get a gyro processable number
	gyroReset(gyro);
	// turn forwards or backwards based on forward boolean
	if(forward){
		while(abs(gyroGet(gyro)) < degrees){ //While the gyro value is less than the target perform code below
			//Set only the left side motors to the target value
			motorSet(frontl,  speed);
		  motorSet(backl,  speed);
		}
		// stop motors
		clearMotors();
	} else {
		while(abs(gyroGet(gyro)) < degrees){
			//Set only the left side motors to the negative target value
			motorSet(frontl,  -speed);
      motorSet(backl,  -speed);
		}
		// stop motors
		clearMotors();
	}
}

/**
* Turns bot left only using left side motors
*
* @warning requires gyro
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
* @author Sean Kelley  sgtkode01@gmail.com
*
* @param   degrees   amount of degrees to turn left
* @param   forward   boolean if bot is turning forward or backward
* @param   speed     speed of motors
*
*/
void fancyTurnLeftDegrees(int degrees, bool forward=true, int speed = MOTOR_SPEED){
	// reset encoders
	degrees=degrees*10;
	// reset gyro
	//gyro takes degrees from 0-3600, so we multiply by 10 to get a gyro processable number
	gyroReset(gyro);
	// turn forwards or backwards based on forward boolean
	if(forward){
		while(abs(gyroGet(gyro)) < degrees){ //While the gyro value is less than the target perform code below
			//Set only the left side motors to the target value
			motorSet(frontr,  speed);
		  motorSet(backr,  speed);
		}
		// stop motors
		clearMotors();
	} else {
		while(abs(gyroGet(gyro)) < degrees){
			//Set only the left side motors to the negative target value
			motorSet(frontr,  -speed);
      motorSet(backr,  -speed);
		}
		// stop motors
		clearMotors();
	}
}

/**
* Turns bot right a given amount of degrees
*
* @warning requires gyro
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
* @author Sean Kelley  sgtkode01@gmail.com
*
* @param  degree  amount of degrees to turn right
*	@param	 speed       speed of motors
*
*/
void turnRightDegrees(float degree, float speed=90)
{
	//Reset gyro
	gyroReset(gyro);
	//gyro takes degrees from 0-3600, so we multiply by 10 to get a gyro processable number
	degree=degree*10;
	//We want to slow down when we approach the target, so we calculate a first turn segment as 60% of the total
	float first=degree*.6;
	while(abs(gyroGet(gyro)) < first){ //Turn the first 60%
			//Since it's turn right, we want to set right motors backwards and left motors forward.
			motorSet(frontl,  speed);
    	motorSet(frontr,  -speed);
    	motorSet(backl,  speed);
    	motorSet(backr,  -speed);
	}
	while(abs(gyroGet(gyro)) <degree){ //Turn the remainin amount.
		//We don't want the motors to run too slow, so we set a a safety net. The motor can't have a power less than 40.
		if(speed*.35<40)//If 35% of the motor power is less than 40, set the power to 40.
		{
      motorSet(frontl,  40);
    	motorSet(frontr,  -40);
    	motorSet(backl,  40);
    	motorSet(backr,  -40);
		} else { //If not set it to 35%
      motorSet(frontl,  speed*.35);
    	motorSet(frontr,  -speed*.35);
    	motorSet(backl,  speed*.35);
    	motorSet(backr,  -speed*.35);
    }
	}
	clearMotors();
}

/**
* Turns bot left a given amount of degrees
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
* @author Sean Kelley  sgtkode01@gmail.com
*
* @param  degree  amount of degrees to turn left
*	@param	speed   speed of motors
*
*/
void turnLeftDegrees(float degree, float speed=90)
{
	//Reset gyro
	gyroReset(gyro);
	//gyro takes degrees from 0-3600, so we multiply by 10 to get a gyro processable number
	degree=degree*10;
	//We want to slow down when we approach the target, so we calculate a first turn segment as 60% of the total
	float first=degree*.6;
	while(abs(gyroGet(gyro)) < first){
			//Since it's turn left, we want to set right motors forwards and left motors backwards.
				motorSet(frontl,  -speed);
  	    motorSet(frontr,  speed);
  	    motorSet(backl,  -speed);
  	    motorSet(backr,  speed);
	}
	while(abs(gyroGet(gyro)) < degree){
		//We don't want the motors to run too slow, so we set a a safety net. The motor can't have a power less than 40.
		if(speed*.35<40)//If 35% of the motor power is less than 40, set the power to 40.
		{
				motorSet(frontl, -40);
		    motorSet(frontr, 40);
		    motorSet(backl,  -40);
		    motorSet(backr, 40);
		} else { //If not set it to 35%
				motorSet(frontr, -speed*.35);
  	    motorSet(frontr,  speed*.35);
  	    motorSet(backl, -speed*.35);
  	    motorSet(backr, speed*.35);
    }
	}
	clearMotors();
}


/**
* Turns bot right a given amount of seconds
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
*
* @param   seconds   amount of seconds to turn right
* @param	 speed     speed of motors
*
*/
void turnRightSeconds(float seconds, float speed=118)
{
	//Since turn right, we want to set left motors forwards and right motors backwards.
  motorSet(frontl,  speed);
  motorSet(backl,  speed);
  motorSet(frontr,  -speed);
  motorSet(backr,  -speed);
	wait1Msec(seconds*1000); //Wait desired amount of time
	clearMotors(); //Stop
}

/**
* Turns bot left a given amount of seconds
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
*
* @param  seconds   amount of seconds to turn left
* @param	speed    speed of motors
*
*/
void turnLeftSeconds(float seconds, float speed=118)
{
	//Since turn left, we want to set the right motors forward and the left motors backwards
	motorSet(frontl,  -speed);
	motorSet(backl,  -speed);
	motorSet(frontr,  speed);
	motorSet(backr,  speed);
	wait1Msec(seconds*1000); //Wait desired amount of time
	clearMotors(); //Stop
}

/**
*	Spins up the fly wheel to a certain power
* value over a certain amount of seconds
*
* @author Sean Kelley  sgtkode01@gmail.com
*
* @param   speed   		speed of motors to reach
* @param	 seconds    seconds to spin up motors
*
*/
void spin_flywheel(float speed, int seconds){
  int power = 0;
  if (motorGet(flyR1) == 0 && motorGet(flyR2) == 0 && motorGet(flyL1) == 0 && motorGet(flyL2) == 0){
    for (int i = 0; i < 21; i++){
      motorSet(flyR1, power);
      motorSet(flyR2, power);
      motorSet(flyL1, power);
      motorSet(flyL2, power);
      power = power + (speed/20);
      wait1Msec(seconds/20);
    }
  }
}
