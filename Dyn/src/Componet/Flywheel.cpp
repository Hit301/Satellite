#include"Componet/Flywheel.h"

flywheel::flywheel()
{
	InstallVet << 1, 0, 0;
	Speed= 0;
	Torque = 0;
	LastRenewTime = 0;
	SamplePeriod = 1;
	Kp = 1;
	Ki = 1;
	tao = 1;
	J = 0.064;
}

void flywheel::StateRenew(int64_t NowTime, double Tref)
{
	static double lastspeed;
	static double speedRef;
	static double lastspeedRef=0;
	if (abs(Tref) > 0.5)
	{
		if(Tref>0)
		{
			Tref = 0.5;
		}
		else
		{
			Tref = -0.5;
		}
	}
	speedRef = Tref * 0.01 / J+lastspeedRef;
	
	lastspeed = Speed;
	Speed = (1-Kp/tao-Ki*0.01/tao)*lastspeed+(Kp/tao+Ki*0.01/tao)*lastspeedRef;
	lastspeedRef = speedRef;
	if (abs(Speed)>628)
	{
		if (Speed > 0)
		{
			Speed = 628;
		}
		if (Speed < 0)
		{
			Speed = -628;
		}
	}
	Torque = J * (Speed - lastspeed) / 0.01;
	std::cout << "SPEED" << Speed;
}

void flywheel::Init(double speed, int64_t timestamp)
{
	
		LastRenewTime = timestamp;
		Speed = speed;
	}


