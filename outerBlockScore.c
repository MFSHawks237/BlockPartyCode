#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S2,     IR,             sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S3,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S4,     eopd,           sensorAnalogActive)
#pragma config(Motor,  motorA,          Spin1,         tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          Spin2,         tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     Left,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     Lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     Flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     motorH,        tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    aarm,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    wrist,                tServoStandard)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "utils.sweg.h"

int processedDist;
bool basket = true;
int spd = 15;

task main()
{
	getJoystickSettings(joystick);
	waitForStart();
	motor[Lift] = 57;
	wait1Msec(600);
	motor[Lift] = 0;
	HTEOPDsetLongRange(eopd);
	initializeRobot();
	while(basket)
	{
		processedDist = HTEOPDreadProcessed(eopd);
		wait1Msec(50);
		if (processedDist <= 5)
		{
			motor[Left] = spd;
			motor[Right] = spd;
		}
		else
		{
			motor[Left] = -spd;
			motor[Right] = -spd;
			wait1Msec(700);
			motor[Left] = 0;
			motor[Right] = 0;
			AtonScore();
			setDistance(900);
			while(g_driveEnabled) // Drive to ramp
			{
				driveForDistance();
			}
			SetTurnTarget(-65); //-85 265
			while(g_turnEnabled) // Turn the robot toward ramp
			{
				GyroTask(g_Gyro);
				TurnTask();
				wait1Msec(10);
			}
			setDistance(3400.0);
			while(g_driveEnabled) // Drive on ramp
			{
				driveForDistance();
			}
			basket = false;
		}
	}
}