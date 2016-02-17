#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, in1,    gyro,           sensorGyro)
#pragma config(Sensor, dgtl1,  encoderL,       sensorQuadEncoder)
#pragma config(Sensor, dgtl3,  encoderR,       sensorQuadEncoder)
#pragma config(Sensor, dgtl7,  ledMed,         sensorLEDtoVCC)
#pragma config(Sensor, dgtl8,  ledHigh,        sensorLEDtoVCC)
#pragma config(Sensor, I2C_1,  flyR2IEM,       sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Sensor, I2C_2,  flyL2IEM,       sensorQuadEncoderOnI2CPort,    , AutoAssign)
#pragma config(Motor,  port1,           flyR1,         tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port2,           flyR2,         tmotorVex393_MC29, openLoop, encoderPort, I2C_1)
#pragma config(Motor,  port3,           flyL1,         tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port4,           flyL2,         tmotorVex393_MC29, openLoop, encoderPort, I2C_2)
#pragma config(Motor,  port5,           frontl,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port6,           frontr,        tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port7,           backl,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port8,           backr,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port9,           topIntake,     tmotorVex393_MC29, openLoop)
#pragma config(Motor,  port10,          bottomIntake,  tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

/**
 * @file functions.c
 * @brief This file contains all functions to be used by
 * the competition code.
 *
 * by itself, it will not work. Instead, save it
 * @warning DO NOT compile this file
 * and compile main.c
 *
 * Also, you must include your motor and sensor setup somewhere in this file.
 *
 * @author	 Sean Kelley      sgtkode01@gmail.com
 * @author   Bernard Suwirjo  bsuwirjo@gmail.com
 * @author	 Josh Asari				josh.asari@gmail.com
 *
 */

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 VARIABLES
//
/////////////////////////////////////////////////////////////////////////////////////////

#define TIMEOUT_CNT_PER_SEC    10   /*!< amount of timeout counts per second */
#define MOTOR_SPEED        		 118  /*!< default motor speed */

int frontLeftVal  = 0; /*!< value of the front left  motor */
int backLeftVal   = 0; /*!< value of the back  left  motor */
int frontRightVal = 0; /*!< value of the front right motor */
int backRightVal  = 0; /*!< value of the back  right motor */

int FW_highSpeed = 80;
int FW_highSpeedDefault = 80;
int FW_medSpeed = 4;
int FW_medSpeedDefault = 4;
int FW_loopCount = 0; /*!< loop count for flywheel */
float FW_ticksPassed = 0; /*!< amount of ticks passed by flywheel in 5 loop counts */
bool FW_half = false; /*!< boolean that determines if flywheel is at half */
bool FW_running = false; /*!< boolean that determines if flywheel is running */
int FW_power = 0; /*!< power value for the flywheel */

#define PID_SENSOR_SCALE    1

#define PID_MOTOR_SCALE     -1

#define PID_DRIVE_MAX       127
#define PID_DRIVE_MIN     (-127)

#define PID_INTEGRAL_LIMIT  50

float  pidSensorCurrentValue;

float  pidError;
float  pidLastError;
float  pidIntegral;
float  pidDerivative;
float  pidDrive;



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
	frontLeftVal  = 0;
	backLeftVal   = 0;
	frontRightVal = 0;
	backRightVal  = 0;
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
	frontLeftVal  = speed;
	backLeftVal   = speed;
	frontRightVal = speed;
	backRightVal  = speed;
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
	frontLeftVal = speed;
	backLeftVal  = speed;
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
	frontRightVal = speed;
	backRightVal  = speed;
}

/**
* Runs each motor for 1.5 seconds
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
*/
void testMotors()
{
	frontRightVal=118;//Set individual motor
	wait1Msec(1500); //Wait 1.5 seconds
	clearMotors(); //clear motor(s)
	backRightVal=118;
	wait1Msec(1500);
	clearMotors();
	frontLeftVal=118;
	wait1Msec(1500);
	clearMotors();
	backLeftVal=118;
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
	SensorValue[ encoderL ] = 0;
	SensorValue[ encoderR ] = 0;

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

			if((SensorValue[encoderL] + SensorValue[encoderR])/2 >= encoder_count){
				break;
			} else {
				if((SensorValue[encoderL] - SensorValue[encoderR]) > 5){
					frontLeftVal  = speed*0.5;
					backLeftVal   = speed*0.5;
					frontRightVal = speed;
					backRightVal  = speed;
				} else if ((SensorValue[encoderR] - SensorValue[encoderL]) > 5){
					frontLeftVal  = speed;
					backLeftVal   = speed;
					frontRightVal = speed*0.5;
					backRightVal  = speed*0.5;
				} else {
					frontLeftVal  = speed;
					backLeftVal   = speed;
					frontRightVal = speed;
					backRightVal  = speed;
				}
			}
		} else {

			if((SensorValue[encoderL] + SensorValue[encoderR])/2 <= encoder_count){
				break;
			} else {
				if((SensorValue[encoderL] - SensorValue[encoderR]) > 5){
					frontLeftVal  = -speed*0.5;
					backLeftVal   = -speed*0.5;
					frontRightVal = -speed;
					backRightVal  = -speed;
				} else if ((SensorValue[encoderR] - SensorValue[encoderL]) > 5){
					frontLeftVal  = -speed;
					backLeftVal   = -speed;
					frontRightVal = -speed*0.5;
					backRightVal  = -speed*0.5;
				} else {
					frontLeftVal  = -speed;
					backLeftVal   = -speed;
					frontRightVal = -speed;
					backRightVal  = -speed;
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
*/
task lockLeftSide()
{
	//float target = 0;
	//float pGain = .3;
	//float iGain = .02;
	//float error = target-SensorValue[encoderLeft];
	//float errorSum=0;
	while(true){
/*		error=target-SensorValue[encoderLeft];
		errorSum+=error;
		motor[FL] = error*pGain+errorSum*iGain;
		  motor[BL] = error*pGain+errorSum*iGain;*/
	}
}

/**
*
* Locks right side motors with PI loop
*
* @warning function does not work
*
* @author Sean Kelley  sgtkode01@gmail.com
*
*/
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
	  motor[BR] = error*pGain+errorSum*iGain;*/
	}
}

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
	SensorValue[gyro]=0;
	// turn forwards or backwards based on forward boolean
	if(forward){
		while(abs(SensorValue[gyro]) < degrees){ //While the gyro value is less than the target perform code below
			//Set only the left side motors to the target value
			frontLeftVal = speed;
		  backLeftVal  = speed;
		}
		// stop motors
		clearMotors();
	} else {
		while(abs(SensorValue[gyro]) < degrees){
			//Set only the left side motors to the negative target value
			frontLeftVal = -speed;
		  backLeftVal  = -speed;
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
	SensorValue[gyro]=0;
	// turn forwards or backwards based on forward boolean
	if(forward){
		while(abs(SensorValue[gyro]) < degrees){ //While the gyro value is less than the target perform code below
			//Set only the left side motors to the target value
			frontRightVal = speed;
		  backRightVal  = speed;
		}
		// stop motors
		clearMotors();
	} else {
		while(abs(SensorValue[gyro]) < degrees){
			//Set only the left side motors to the negative target value
			frontRightVal = -speed;
		  backRightVal  = -speed;
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
	SensorValue[gyro]=0;
	//gyro takes degrees from 0-3600, so we multiply by 10 to get a gyro processable number
	degree=degree*10;
	//We want to slow down when we approach the target, so we calculate a first turn segment as 60% of the total
	float first=degree*.6;
	while(abs(SensorValue[gyro]) < first){ //Turn the first 60%
			//Since it's turn right, we want to set right motors backwards and left motors forward.
			frontLeftVal = speed;
    	frontRightVal = -speed;
    	backLeftVal = speed;
    	backRightVal = -speed;
	}
	while(abs(SensorValue[gyro]) <degree){ //Turn the remainin amount.
		//We don't want the motors to run too slow, so we set a a safety net. The motor can't have a power less than 40.
		if(speed*.35<40)//If 35% of the motor power is less than 40, set the power to 40.
		{
			frontLeftVal = 40;
    	frontRightVal = -40;
    	backLeftVal = 40;
    	backRightVal = -40;
		} else { //If not set it to 35%
				frontLeftVal = speed*.35;
	    	frontRightVal = -speed*.35;
	    	backLeftVal = speed*.35;
	    	backRightVal = -speed*.35;
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
	SensorValue[gyro]=0;
	//gyro takes degrees from 0-3600, so we multiply by 10 to get a gyro processable number
	degree=degree*10;
	//We want to slow down when we approach the target, so we calculate a first turn segment as 60% of the total
	float first=degree*.6;
	while(abs(SensorValue[gyro]) < first){
			//Since it's turn left, we want to set right motors forwards and left motors backwards.
				frontLeftVal = -speed;
  	    frontRightVal = speed;
  	    backLeftVal = -speed;
  	    backRightVal = speed;
	}
	while(abs(SensorValue[gyro]) < degree){
		//We don't want the motors to run too slow, so we set a a safety net. The motor can't have a power less than 40.
		if(speed*.35<40)//If 35% of the motor power is less than 40, set the power to 40.
		{
				frontLeftVal = -40;
		    frontRightVal = 40;
		    backLeftVal = -40;
		    backRightVal = 40;
		} else { //If not set it to 35%
				frontLeftVal = -speed*.35;
  	    frontRightVal = speed*.35;
  	    backLeftVal = -speed*.35;
  	    backRightVal = speed*.35;
    }
	}
	clearMotors();
}

/**
* Turns bot right a given amount of ticks
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
* @author Sean Kelley  sgtkode01@gmail.com
*
* @param  ticks  amount of degrees to turn right
*	@param	speed  speed of motors
*
*/
void turnRightTicks(int ticks, float speed=118)
{
	SensorValue[encoderL]=0;
	while(SensorValue[encoderL]<ticks)
	{
		motor[frontl] = speed;
		motor[backl] = speed;
		motor[frontr] = -speed;
		motor[backr] = -speed;
	}
}

/**
* Turns bot left a given amount of ticks
*
* @author Bernard Suwirjo  bsuwirjo@gmail.com
* @author Sean Kelley  sgtkode01@gmail.com
*
* @param  ticks  amount of degrees to turn left
*	@param	speed  speed of motors
*
*/
void turnLeftTicks(int ticks, float speed=118)
{
	SensorValue[encoderR]=0;
	while(SensorValue[encoderR]<ticks)
	{
		motor[frontl] = -speed;
		motor[backl] = -speed;
		motor[frontr] = speed;
		motor[backr] = speed;
	}
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
	frontLeftVal=speed;
	backLeftVal=speed;
	frontRightVal=-speed;
	backRightVal=-speed;
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
	frontLeftVal = -speed;
	backLeftVal = -speed;
	frontRightVal = speed;
	backRightVal = speed;
	wait1Msec(seconds*1000); //Wait desired amount of time
	clearMotors(); //Stop
}

/**
*	Set the motors to the current values
*
* @author Sean Kelley  sgtkode01@gmail.com
*
*/
task runMotors(){
	while(true){
		motor[backr]  = backRightVal;
		motor[backl]  = backLeftVal;
		motor[frontl] = frontLeftVal;
		motor[frontr] = backRightVal;
	}
}

/////////////////////////////////////////////////////////////////////////////////////////
//
//                                 Flywheel
//
/////////////////////////////////////////////////////////////////////////////////////////

/**
*	Spins up the fly wheel to a certain speed
*
* @author Sean Kelley  sgtkode01@gmail.com
*
*/
task spin_flywheel(){
	while(true){
		if(FW_loopCount >= 25){
			if(FW_running){
				FW_ticksPassed = (abs(SensorValue[flyR2IEM]) + abs(SensorValue[flyL2IEM])) / 2;

				SensorValue[flyR2IEM] = 0;
				SensorValue[flyL2IEM] = 0;

				if(FW_half == false){
					if(FW_ticksPassed == FW_highSpeed){
						SensorValue[ledMed] = 1;
						SensorValue[ledHigh] = 1;
						motor[flyR1] = FW_power;
				    motor[flyR2] = FW_power;
				    motor[flyL1] = FW_power;
				    motor[flyL2] = FW_power;
				    motor[topIntake] = FW_power - (FW_power/5);
					} else if(FW_ticksPassed >= FW_highSpeed+5){
						SensorValue[ledMed] = 1;
						SensorValue[ledHigh] = 1;
						motor[flyR1] = FW_power;
				    motor[flyR2] = FW_power;
				    motor[flyL1] = FW_power;
				    motor[flyL2] = FW_power;
				    motor[topIntake] = FW_power - (FW_power/5);
						FW_power = FW_power - 1;
					} else {
						SensorValue[ledMed] = 0;
						SensorValue[ledHigh] = 0;
						if(FW_power <= 127){
							if(FW_ticksPassed < FW_highSpeed*0.5){
								FW_power = FW_power + 5;
							} else if(FW_ticksPassed < FW_highSpeed*0.6) {
								FW_power = FW_power + 5;
							} else {
								FW_power = FW_power + 1;
							}
						}
						motor[flyR1] = FW_power;
				    motor[flyR2] = FW_power;
				    motor[flyL1] = FW_power;
				    motor[flyL2] = FW_power;
				    motor[topIntake] = FW_power - (FW_power/5);
					}
				} else {
					if(FW_ticksPassed == FW_medSpeed){
						SensorValue[ledMed] = 1;
						SensorValue[ledHigh] = 0;
						motor[flyR1] = FW_power;
				    motor[flyR2] = FW_power;
				    motor[flyL1] = FW_power;
				    motor[flyL2] = FW_power;
				    motor[topIntake] = FW_power - (FW_power/5);
					} else if(FW_ticksPassed >= FW_medSpeed+5){
						SensorValue[ledMed] = 1;
						SensorValue[ledHigh] = 0;
						motor[flyR1] = FW_power;
				    motor[flyR2] = FW_power;
				    motor[flyL1] = FW_power;
				    motor[flyL2] = FW_power;
				    motor[topIntake] = FW_power - (FW_power/5);
						FW_power = FW_power - 1;
					} else {
						SensorValue[ledMed] = 0;
						SensorValue[ledHigh] = 0;
						if(FW_power <= 127){
							if(FW_ticksPassed < FW_highSpeed*0.5){
								FW_power = FW_power + 5;
							} else if(FW_ticksPassed < FW_highSpeed*0.6) {
								FW_power = FW_power + 5;
							} else {
								FW_power = FW_power + 1;
							}
						}
						motor[flyR1] = FW_power;
				    motor[flyR2] = FW_power;
				    motor[flyL1] = FW_power;
				    motor[flyL2] = FW_power;
				    motor[topIntake] = FW_power - (FW_power/5);
					}
				}
			} else {
				motor[flyR1] = 0;
		    motor[flyR2] = 0;
		    motor[flyL1] = 0;
		    motor[flyL2] = 0;
		    motor[topIntake] = 0;
		    FW_power = 0;
			}
			FW_loopCount = 0;
	  } else {
	  	FW_loopCount += 1;
	  }
	  wait1Msec(10);
	}
}

// These could be constants but leaving
// as variables allows them to be modified in the debugger "live"
float  pid_Kp = 1.0;
float  pid_Ki = 0.04;
float  pid_Kd = 1.0;
int swag = 0;

/*-----------------------------------------------------------------------------*/
/*                                                                             */
/*  pid control task                                                           */
/*                                                                             */
/*-----------------------------------------------------------------------------*/

task FW_pidController()
{

  // Init the variables - thanks Glenn :)
  //pidLastError  = 0;
  //pidIntegral   = 0;

  while( true ) {
    // Is PID control active ?
  	swag += 1;
    if( FW_running ) {
      // Read the sensor value and scale
      pidSensorCurrentValue = ((abs(SensorValue[flyR2IEM]) + abs(SensorValue[flyL2IEM])) / 2) * PID_SENSOR_SCALE;

      // since flywheel is continuously spinning, reset encoders
      SensorValue[flyR2IEM] = 0;
			SensorValue[flyL2IEM] = 0;

      // calculate error
      if(FW_half){
      	pidError = pidSensorCurrentValue - FW_medSpeed;
      	if(pidError <= FW_medSpeed+5){
      		SensorValue[ledMed] = 1;
        } else if(pidError >= FW_medSpeed-5) {
        	SensorValue[ledMed] = 1;
        } else {
					SensorValue[ledMed] = 0;
        }
      } else {
      	pidError = pidSensorCurrentValue - FW_highSpeed;
      	if(pidError <= FW_highSpeed+5){
      		SensorValue[ledHigh] = 1;
        } else if(pidError >= FW_highSpeed-5) {
        	SensorValue[ledHigh] = 1;
        } else {
					SensorValue[ledHigh] = 0;
        }
      }

      // integral - if Ki is not 0
      if( pid_Ki != 0 )
          {
          // If we are inside controlable window then integrate the error
          if( abs(pidError) < PID_INTEGRAL_LIMIT )
              pidIntegral = pidIntegral + pidError;
          else
              pidIntegral = 0;
          }
      else
          pidIntegral = 0;

      // calculate the derivative
      pidDerivative = pidError - pidLastError;
      pidLastError  = pidError;

      // calculate drive
      pidDrive = (pid_Kp * pidError) + (pid_Ki * pidIntegral) + (pid_Kd * pidDerivative);

      // limit drive
      if( pidDrive > PID_DRIVE_MAX )
          pidDrive = PID_DRIVE_MAX;
      if( pidDrive < PID_DRIVE_MIN )
          pidDrive = PID_DRIVE_MIN;

      // send to flywheel
      motor[flyR1] = pidDrive * PID_MOTOR_SCALE;
	    motor[flyR2] = pidDrive * PID_MOTOR_SCALE;
	    motor[flyL1] = pidDrive * PID_MOTOR_SCALE;
	    motor[flyL2] = pidDrive * PID_MOTOR_SCALE;
	    motor[topIntake] = (pidDrive * PID_MOTOR_SCALE) - ((pidDrive * PID_MOTOR_SCALE)/5);
    } else {
      // clear all
      pidError      = 0;
      pidLastError  = 0;
      pidIntegral   = 0;
      pidDerivative = 0;
      pidDrive = 0;
      motor[flyR1] = 0;
	    motor[flyR2] = 0;
	    motor[flyL1] = 0;
	    motor[flyL2] = 0;
	    motor[topIntake] = 0;
    }
	  // Run at 4Hz
	  wait1Msec( 250 );
	}
}

void spin_flywheel_old(float initial=0, float speed, int seconds){
  int power = initial;
  for (int i = 0; i < 21; i++){
    motor[flyR1] = power;
    motor[flyR2] = power;
    motor[flyL1] = power;
    motor[flyL2] = power;
    motor[topIntake] = power - (speed/5);
    power = power + (speed/20);
    wait1Msec(seconds/20);
  }

  motor[flyR1] = speed;
  motor[flyR2] = speed;
  motor[flyL1] = speed;
  motor[flyL2] = speed;
  motor[topIntake] = speed - (speed/5);
}
