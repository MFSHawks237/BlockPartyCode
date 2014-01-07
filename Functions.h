#include "JoystickDriver.c"
#include "drivers/gyro.h"
#define numButtons  11
bool state[2][numButtons+1];
bool change[2][numButtons+1];


void forward(int power)
{
	motor[Left] = power;
	motor[Right] = power;
}

void backward(int power)
{
	motor[Left] = -power;
	motor[Right] = -power;
}

void stopdrive()
{
	motor[Left] = 0;
	motor[Right] = 0;
}

float switcher = 1;
void drive(int driveBuffer, tMotor Left, tMotor Right)
{
	if(joy1Btn(4))
	{
		switcher = 1;
	}
	if(joy1Btn(3))
	{
		switcher = 0.5;
	}
	if(joy1Btn(2))
	{
		switcher = 0.25;
	}
	if(joy1Btn(1))
	{
		switcher = 0.13;
	}

	if(abs(joystick.joy1_y1) > driveBuffer)
	{
		motor[Left] = joystick.joy1_y1*switcher;
	}
	else
	{
		motor[Left] = 0;
	}
	if(abs(joystick.joy1_y2) > driveBuffer)
	{
		motor[Right] = joystick.joy1_y2*switcher;
	}
	else
	{
		motor[Right] = 0;
	}
}


int joy2BtnChange(int btn)
{
	if((bool)change[1][btn-1])
	{
		if((bool)state[1][btn-1])
		{
			return 1;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		return 0;
	}
}

int joy1BtnChange(int btn)
{
	if((bool)change[0][btn-1])
	{
		if((bool)state[0][btn-1])
		{
			return 1;
		}
		else
		{
			return -1;
		}
	}
	else
	{
		return 0;
	}
}


void advancedJoy()
{
	for(int i = 1;i <= numButtons+1;i++)
	{
		if((bool)joy1Btn(i) != (bool)state[0][i-1])
		{
			state[0][i-1] = !state[0][i-1];
			change[0][i-1] = true;
		}
		else
		{
			change[0][i-1] = false;
		}

		if((bool)joy2Btn(i) != (bool)state[1][i-1])
		{
			state[1][i-1] = !state[1][i-1];
			change[1][i-1] = true;
		}
		else
		{
			change[1][i-1] = false;
		}
	}
}

bool joy1BtnPressed(int btn)
{
	return (joy1BtnChange(btn) == 1);
}

bool joy2BtnPressed(int btn)
{
	return (joy2BtnChange(btn) == 1);
}
