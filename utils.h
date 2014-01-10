#include "drivers/hitechnic-sensormux.h" //HiTechnic Sensor Multiplexer Driver
#include "drivers/hitechnic-protoboard.h" //HiTechnic Prototype Board Driver
#include "drivers/hitechnic-accelerometer.h" //HiTechnic Accelerometer Sensor Driver
#include "drivers/hitechnic-eopd.h" //HiTechnic EOPD Sensor Driver
#include "drivers/hitechnic-irseeker-v2.h" //HiTechnic IR Seeker V2 Sensor Driver
#include "JoystickDriver.c" //Driver to handle joystick input
#include "drivers/gyro.h" //Gyroscopic Sensor Driver
#include "Functions.h"

#define flagSpinFWD 255
#define flagSpinBKWD 0
#define flagSpinSTP 128
#define motorSTP 0
#define motorMaxFWD 100
#define motorMaxBKWD -100
#define SrvoArmOUT 60
#define SrvoArmIN 0

#define irpower 20
#define dslope 30/720
#define maxPwr 10
#define dminPwr 7
#define minPwr 35
#define BOUND(n, l, h) (((n) < (l))? (l): ((n) > (h))? (h): (n))
#define drivePower 100

GYRO  g_Gyro;
float g_turnTarget = 0.0;
bool  g_turnEnabled = false;
float g_tolerance = 0.5;  //needs to be tuned
float Kp = 0.1;           //proportion gain constant needs to be tuned
bool g_driveEnabled = false;
float g_driveTarget;
float buffer = 5;
bool irseek = true;
int irval;
int i;
float driveError;

void Drive(tMotor Left, tMotor Right)
{
	motor[Left] = joystick.joy1_y1;		//Left Drive
	motor[Right] = joystick.joy1_y2;		//Right Drive
}

void Arm(tMotor Lift ,tMotor Wrist)
{
	motor[Lift] = joystick.joy2_y1;		//Both Lift Motors
	motor[Wrist] = joystick.joy2_y2*0.5;		//Wrist Motor at half power
}

/*void flagSpinUP(tServo Flag, tServo Flag2)
{
	servo[Flag] = flagSpinFWD;		//Spins Flag
	servo[Flag2] = flagSpinFWD;
}*/

void SetTurnTarget(float angle)
{
	g_turnTarget = GyroGetHeading(g_Gyro) + angle;
	g_turnEnabled = true;
}

void TurnTask()
{
	if (g_turnEnabled)
	{
		float error = g_turnTarget - GyroGetHeading(g_Gyro);
		if (abs(error) > g_tolerance)
		{
			//
			// Simple proportional PID control.
			// Limit the outpout to the range of -100 to 100.
			//
			int turnPower = maxPwr*error*Kp + abs(error)/error*minPwr;
			motor[Left] = turnPower;
			motor[Right] = -turnPower;
		}
		else
		{
			motor[Left] = 0;
			motor[Right] = 0;
			g_turnEnabled = false;
		}
	}
}

void initializeRobot()
{
	GyroInit(g_Gyro, gyro, 0);
	return;
}

void setDistance(float target)
{
	g_driveTarget = target + nMotorEncoder[Right];
	g_driveEnabled = true;
}

void driveForDistance()
{
	driveError = g_driveTarget - nMotorEncoder[Right];
	if(abs(driveError) > buffer)
	{
		motor[Left] = driveError*dslope + abs(driveError)/driveError*dminPwr;
		motor[Right] = driveError*dslope + abs(driveError)/driveError*dminPwr;
		if (motor[Left] <= 5)
		{
			g_driveEnabled = false;
		}
	}
	else
	{
		motor[Left] = 0;
		motor[Right] = 0;
		g_driveEnabled = false;
	}
}

void IRseeker()
{
	while(irseek)
	{
		irval = SensorValue[IR];
		if(irval != 5)
		{
			motor[Left] = irpower;
			motor[Right] = irpower;
		}
		else
		{
			i = 0;
			while(irseek)
			{
				irval = SensorValue[IR];
				if(irval != 6 || irval != 4)
				{
					i++;
					motor[Left] = irpower/2;
					motor[Right] = irpower/2;
					if (i <= 0)
					{
						i = i / 2;
						irseek = false;
					}
				}
				else
				{
					motor[Left] = 0;
					motor[Right] = 0;
					i = i / 2;
					irseek = false;
				}
			}
			irseek = false;
		}
	}
}

void AtonScore()
{
	servo[aarm] = 200;
	wait1Msec(2000);
	servo[aarm] = 60;
	wait1Msec(2000);
}
