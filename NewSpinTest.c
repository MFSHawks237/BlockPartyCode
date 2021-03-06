#pragma config(Motor,  motorA,          A,             tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorB,          B,             tmotorNXT, PIDControl, encoder)
#pragma config(Motor,  motorC,           ,             tmotorNXT, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "JoystickDriver.c"

task main()
{
	while(true)
	{
		getJoystickSettings(joystick);
		if(joy1Btn(8))
		{
			motor[A] = 100;
			motor[B] = 100;
		}
		else if(joy1Btn(7))
		{
			motor[A] = -100;
			motor[B] = -100;
		}
		else
		{
			motor[A] = 0;
			motor[B] = 0;
		}
	}
}
