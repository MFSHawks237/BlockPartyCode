#include "JoystickDriver.c";

task main()
{

	int i;
	eraseDisplay();
	while (true)
	{
		getJoystickSettings(joystick);
		eraseDisplay();
		i = joystick.joy1_y1;
		if (i<-50)
		{
			i = -50;
		}
		nxtDisplayString( 1, "%d", i);
	}
}
