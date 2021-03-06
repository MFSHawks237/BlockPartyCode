#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  none)
#pragma config(Sensor, S2,     IR,             sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S3,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Sensor, S4,     eopd,           sensorAnalogActive)
#pragma config(Motor,  motorA,          Spinner,       tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
#pragma config(Motor,  mtr_S1_C1_1,     Left,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     Wrist,         tmotorTetrix, openLoop, reversed)
#pragma config(Motor,  mtr_S1_C2_2,     Lift,          tmotorTetrix, openLoop, reversed)
#pragma config(Servo,  srvo_S1_C3_1,    Flag,                 tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C3_2,    Flag2,                tServoContinuousRotation)
#pragma config(Servo,  srvo_S1_C3_3,    aarm,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//Last Edit: Armon 1-16-14

#include "utils.h"

//#define eopd        msensor_s4_1
//#define accel       msensor_s4_2
//#define color       msensor_s4_3

task main()
{
	waitForStart();
	getJoystickSettings(joystick);
	initializeRobot();
	setDistance(500); //520
	while(g_driveEnabled) // Drive closer to baskets
	{
		driveForDistance();
	}
	SetTurnTarget(85.0);
	while(g_turnEnabled) // Turn the robot parallel to the baskets
	{
		GyroTask(g_Gyro);
		TurnTask();
		wait1Msec(10);
	}
	wait1Msec(500);
	IRseeker(); // Seek the IR
	motor[Right] = 0;
	motor[Left] = 0;
	AtonScore(); // Swings out the servo arm
	RampGo(); // turn around and drive up to the ramp and on
}
