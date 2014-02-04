#pragma config(Hubs,  S1, HTMotor,  HTMotor,  HTServo,  HTMotor)
#pragma config(Sensor, S1,     ,               sensorI2CMuxController)
#pragma config(Sensor, S2,     IR,             sensorHiTechnicIRSeeker1200)
#pragma config(Sensor, S3,     gyro,           sensorI2CHiTechnicGyro)
#pragma config(Motor,  motorA,          Spin1,         tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          Spin2,         tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  mtr_S1_C1_1,     Left,          tmotorTetrix, openLoop, encoder)
#pragma config(Motor,  mtr_S1_C1_2,     Right,         tmotorTetrix, openLoop, reversed, encoder)
#pragma config(Motor,  mtr_S1_C2_1,     Lift,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C2_2,     Flag,          tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_1,     Wrist,         tmotorTetrix, openLoop)
#pragma config(Motor,  mtr_S1_C4_2,     motorI,        tmotorTetrix, openLoop)
#pragma config(Servo,  srvo_S1_C3_1,    aarm,                 tServoStandard)
#pragma config(Servo,  srvo_S1_C3_2,    servo2,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_3,    servo3,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_4,    servo4,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_5,    servo5,               tServoNone)
#pragma config(Servo,  srvo_S1_C3_6,    servo6,               tServoNone)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//Last Edit: Armon 1-30-14

#include "utils.sweg.h"

//#define eopd        msensor_s4_1
//#define accel       msensor_s4_2
//#define color       msensor_s4_3

task main()
{
	waitForStart();
	getJoystickSettings(joystick);
	motor[Lift] = 57;
	wait1Msec(700);
	motor[Lift] = 0;
	motor[Wrist] = -25;
	wait1Msec(300);
	motor[Wrist] = 0;
	initializeRobot();
	setDistance(500); //500
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
